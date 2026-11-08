#include <catch2/catch.hpp>

#include "libslic3r/Model.hpp"
#include "libslic3r/TriangleSelector.hpp"
#include "libslic3r/PaintReproject.hpp"
#include "libslic3r/CutUtils.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"
#include <cereal/archives/binary.hpp>
#include <cereal/types/utility.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/polymorphic.hpp>
#include <boost/filesystem.hpp>
#include <sstream>

using namespace Slic3r;

namespace {
using SlopeDirection = TriangleSelector::SlopeDirection;

enum class BasePaintTool { Sphere, Circle, SmartFill };

// An open triangle isolates the direction filter from unrelated closed-model
// topology. Its center is the cursor hit and its outward normal is prescribed.
TriangleMesh slope_triangle(const Vec3f &normal)
{
    const Vec3f u(normal.z(), 0.f, -normal.x());
    const Vec3f v(0.f, 1.f, 0.f);
    indexed_triangle_set mesh;
    mesh.vertices = {-2.f * u - 2.f * v, 2.f * u - 2.f * v, 4.f * v};
    mesh.indices = {Vec3i(0, 1, 2)};
    return TriangleMesh(mesh);
}

void paint_slope(TriangleSelector &selector, const TriangleMesh &mesh, BasePaintTool tool,
                 float angle, SlopeDirection direction, const Transform3d &transform = Transform3d::Identity(),
                 EnforcerBlockerType state = EnforcerBlockerType::ENFORCER, float radius = 40.f,
                 bool explicit_direction = true)
{
    const auto &face = mesh.its.indices.front();
    const auto &vertices = mesh.its.vertices;
    const Vec3f center = (vertices[face[0]] + vertices[face[1]] + vertices[face[2]]) / 3.f;
    const Vec3f normal = (vertices[face[1]] - vertices[face[0]]).cross(vertices[face[2]] - vertices[face[0]]).normalized();
    if (tool == BasePaintTool::SmartFill) {
        if (explicit_direction)
            selector.seed_fill_select_triangles(center, 0, transform, {}, 180.f, angle, true, direction);
        else
            selector.seed_fill_select_triangles(center, 0, transform, {}, 180.f, angle, true);
        selector.seed_fill_apply_on_triangles(state);
    } else {
        auto cursor = TriangleSelector::SinglePointCursor::cursor_factory(
            center, center + 20.f * normal, radius,
            tool == BasePaintTool::Sphere ? TriangleSelector::SPHERE : TriangleSelector::CIRCLE,
            transform, {});
        if (explicit_direction)
            selector.select_patch(0, std::move(cursor), state, transform, true, angle, direction);
        else
            selector.select_patch(0, std::move(cursor), state, transform, true, angle);
    }
}
} // namespace

TEST_CASE("Support base brushes and smart fill mirror the stock slope filter upward", "[SupportBase][Underhang]")
{
    const auto tool = GENERATE(BasePaintTool::Sphere, BasePaintTool::Circle, BasePaintTool::SmartFill);
    const float angle = GENERATE(0.f, 33.f, 65.f, 90.f);
    const auto direction = GENERATE(SlopeDirection::Downward, SlopeDirection::Upward);
    struct Surface { Vec3f normal; float from_up; };
    const std::vector<Surface> surfaces = {
        {Vec3f(0.f, 0.f, 1.f), 0.f}, {Vec3f(.5f, 0.f, std::sqrt(.75f)), 30.f},
        {Vec3f(std::sqrt(.75f), 0.f, .5f), 60.f}, {Vec3f(1.f, 0.f, 0.f), 90.f},
        {Vec3f(std::sqrt(.75f), 0.f, -.5f), 120.f}, {Vec3f(.5f, 0.f, -std::sqrt(.75f)), 150.f},
        {Vec3f(0.f, 0.f, -1.f), 180.f}
    };
    for (const auto &surface : surfaces) {
        CAPTURE(int(tool), angle, int(direction), surface.from_up);
        const TriangleMesh mesh = slope_triangle(surface.normal);
        TriangleSelector selector(mesh);
        paint_slope(selector, mesh, tool, angle, direction);
        const float from_direction = direction == SlopeDirection::Upward ? surface.from_up : 180.f - surface.from_up;
        // Downward retains the stock floating point boundary at exactly 90°;
        // upward must never admit a vertical surface, including at its maximum.
        if (!(direction == SlopeDirection::Downward && angle == 90.f && surface.from_up == 90.f))
            CHECK(selector.has_facets(EnforcerBlockerType::ENFORCER) == (angle == 0.f || from_direction < angle));
        if (direction == SlopeDirection::Downward) {
            TriangleSelector stock(mesh);
            paint_slope(stock, mesh, tool, angle, direction, Transform3d::Identity(), EnforcerBlockerType::ENFORCER, 40.f, false);
            CHECK(stock.serialize() == selector.serialize());
        }
    }
}

TEST_CASE("Support base slope filters use world normals after rotation reflection and scaling", "[SupportBase][Underhang]")
{
    const auto tool = GENERATE(BasePaintTool::Sphere, BasePaintTool::Circle, BasePaintTool::SmartFill);
    struct SurfaceTransform { const char *name; Vec3f normal; Transform3d transform; bool paintable; };
    Transform3d tilt = Transform3d::Identity();
    tilt.rotate(Eigen::AngleAxisd(PI / 3., Vec3d::UnitY()));
    Transform3d turn_upright = Transform3d::Identity();
    turn_upright.rotate(Eigen::AngleAxisd(-PI / 2., Vec3d::UnitY()));
    Transform3d mirror_xy = Transform3d::Identity(); mirror_xy.scale(Vec3d(-1., 1., 1.));
    Transform3d mirror_z = Transform3d::Identity(); mirror_z.scale(Vec3d(1., 1., -1.));
    Transform3d stretch_x = Transform3d::Identity(); stretch_x.scale(Vec3d(3., 1., 1.));
    Transform3d stretch_z = Transform3d::Identity(); stretch_z.scale(Vec3d(1., 1., 3.));
    Transform3d mixed = Transform3d::Identity();
    mixed.rotate(Eigen::AngleAxisd(PI / 6., Vec3d::UnitY())); mixed.scale(Vec3d(-3., 2., 1.));
    const std::vector<SurfaceTransform> cases = {
        {"flat top tilted beyond threshold", Vec3f::UnitZ(), tilt, false},
        {"vertical facet rotated upward", Vec3f::UnitX(), turn_upright, true},
        {"top reflected horizontally", Vec3f::UnitZ(), mirror_xy, true},
        {"top reflected downward", Vec3f::UnitZ(), mirror_z, false},
        {"bottom reflected upward", -Vec3f::UnitZ(), mirror_z, true},
        {"steep slope flattened by X scale", Vec3f(std::sqrt(.75f), 0.f, .5f), stretch_x, true},
        {"shallow slope steepened by Z scale", Vec3f(.5f, 0.f, std::sqrt(.75f)), stretch_z, false},
        {"rotated reflected and nonuniformly scaled", Vec3f(std::sqrt(.75f), 0.f, .5f), mixed, true}
    };
    for (const auto &test : cases) {
        CAPTURE(int(tool), test.name);
        const TriangleMesh mesh = slope_triangle(test.normal);
        TriangleSelector selector(mesh);
        paint_slope(selector, mesh, tool, 45.f, SlopeDirection::Upward, test.transform);
        CHECK(selector.has_facets(EnforcerBlockerType::ENFORCER) == test.paintable);
    }
}

TEST_CASE("Support base direction filtering remains local while fill traverses neighboring faces", "[SupportBase][Underhang]")
{
    const auto tool = GENERATE(BasePaintTool::Sphere, BasePaintTool::Circle, BasePaintTool::SmartFill);
    const TriangleMesh mesh(its_make_cube(10., 10., 10.));
    TriangleSelector selector(mesh);
    int top = -1;
    for (size_t i = 0; i < mesh.its.indices.size(); ++i) {
        const auto &face = mesh.its.indices[i];
        const auto &v = mesh.its.vertices;
        if ((v[face[1]] - v[face[0]]).cross(v[face[2]] - v[face[0]]).z() > 0.f) { top = int(i); break; }
    }
    REQUIRE(top >= 0);
    const auto &face = mesh.its.indices[top];
    const Vec3f hit = (mesh.its.vertices[face[0]] + mesh.its.vertices[face[1]] + mesh.its.vertices[face[2]]) / 3.f;
    if (tool == BasePaintTool::SmartFill) {
        selector.seed_fill_select_triangles(hit, top, Transform3d::Identity(), {}, 180.f, 90.f, true, SlopeDirection::Upward);
        selector.seed_fill_apply_on_triangles(EnforcerBlockerType::ENFORCER);
    } else {
        selector.select_patch(top, TriangleSelector::SinglePointCursor::cursor_factory(
            hit, hit + Vec3f(0.f, 0.f, 20.f), 100.f,
            tool == BasePaintTool::Sphere ? TriangleSelector::SPHERE : TriangleSelector::CIRCLE, Transform3d::Identity(), {}),
            EnforcerBlockerType::ENFORCER, Transform3d::Identity(), true, 90.f, SlopeDirection::Upward);
    }
    const auto selected = selector.get_facets(EnforcerBlockerType::ENFORCER);
    REQUIRE(selected.indices.size() == 2);
    for (const auto &selected_face : selected.indices)
        for (int vertex : selected_face)
            CHECK(selected.vertices[vertex].z() == Approx(10.f));
}

TEST_CASE("Support base filters retain split facets and apply to painting and erasing", "[SupportBase][Underhang]")
{
    const auto tool = GENERATE(BasePaintTool::Sphere, BasePaintTool::Circle, BasePaintTool::SmartFill);
    const TriangleMesh mesh = slope_triangle(Vec3f::UnitZ());
    TriangleSelector selector(mesh);
    paint_slope(selector, mesh, BasePaintTool::Sphere, 0.f, SlopeDirection::Downward,
                Transform3d::Identity(), EnforcerBlockerType::ENFORCER, .5f);
    REQUIRE(selector.get_triangles().size() > mesh.its.indices.size());
    REQUIRE(selector.has_facets(EnforcerBlockerType::ENFORCER));
    const auto partial = selector.serialize();
    paint_slope(selector, mesh, tool, 45.f, SlopeDirection::Downward,
                Transform3d::Identity(), EnforcerBlockerType::NONE);
    CHECK(selector.serialize() == partial);
    paint_slope(selector, mesh, tool, 45.f, SlopeDirection::Upward,
                Transform3d::Identity(), EnforcerBlockerType::NONE);
    CHECK_FALSE(selector.has_facets(EnforcerBlockerType::ENFORCER));
    paint_slope(selector, mesh, tool, 45.f, SlopeDirection::Upward);
    CHECK(selector.has_facets(EnforcerBlockerType::ENFORCER));
}

TEST_CASE("Support base paint is independent, copied, and erased independently", "[SupportBase][Model]")
{
    Model model;
    auto *object = model.add_object();
    auto *volume = object->add_volume(TriangleMesh(its_make_cube(10., 10., 10.)));
    volume->support_base_facets.set_triangle_from_string(0, "4");
    REQUIRE(volume->is_fdm_support_painted());
    REQUIRE(volume->supported_facets.empty());
    volume->supported_facets.set_triangle_from_string(0, "8");
    Model copy(model);
    auto *copied = copy.objects.front()->volumes.front();
    REQUIRE(copied->support_base_facets.equals(volume->support_base_facets));
    REQUIRE_FALSE(model_custom_supports_data_changed(*object, *copy.objects.front()));
    copied->support_base_facets.reset();
    REQUIRE(model_custom_supports_data_changed(*object, *copy.objects.front()));
    REQUIRE(copied->supported_facets.equals(volume->supported_facets));
    REQUIRE_FALSE(volume->support_base_facets.empty());
    volume->reset_extra_facets();
    REQUIRE(volume->support_base_facets.empty());
}

TEST_CASE("Support bases retain partial triangle paint in the undo annotation format", "[SupportBase][Model]")
{
    Model model;
    auto *volume = model.add_object()->add_volume(TriangleMesh(its_make_cube(10., 10., 10.)));
    TriangleSelector selector(volume->mesh());
    selector.set_facet(0, EnforcerBlockerType::ENFORCER);
    const auto &face = volume->mesh().its.indices.front();
    const auto &vertices = volume->mesh().its.vertices;
    const Vec3f center = (vertices[face[0]] + vertices[face[1]] + vertices[face[2]]) / 3.f;
    selector.select_patch(0, TriangleSelector::SinglePointCursor::cursor_factory(
        center, center + Vec3f(0, 0, 20), 1.f, TriangleSelector::CursorType::SPHERE,
        Transform3d::Identity(), TriangleSelector::ClippingPlane()),
        EnforcerBlockerType::NONE, Transform3d::Identity(), true);
    volume->support_base_facets.set(selector);
    REQUIRE(volume->support_base_facets.get_triangle_as_string(0).size() > 1);
    std::ostringstream buffer;
    {
        cereal::BinaryOutputArchive archive(buffer);
        archive(volume->support_base_facets);
    }
    auto *restored = model.add_object()->add_volume(TriangleMesh(volume->mesh()));
    std::istringstream input(buffer.str());
    {
        cereal::BinaryInputArchive archive(input);
        archive(restored->support_base_facets);
    }
    REQUIRE(restored->support_base_facets.equals(volume->support_base_facets));
    REQUIRE(restored->supported_facets.empty());
}

TEST_CASE("Bambu projects retain base painting without conflating support requirements", "[SupportBase][3mf]")
{
    const auto workspace = boost::filesystem::temp_directory_path() / boost::filesystem::unique_path("support-base-%%%%-%%%%");
    boost::filesystem::create_directories(workspace);
    struct Cleanup {
        boost::filesystem::path path;
        std::string data = data_dir(), temporary = temporary_dir();
        ~Cleanup() {
            set_data_dir(data); set_temporary_dir(temporary);
            boost::system::error_code error; boost::filesystem::remove_all(path, error);
        }
    } cleanup{workspace};
    set_data_dir(workspace.string()); set_temporary_dir(workspace.string());
    Model source;
    auto *object = source.add_object();
    object->add_instance();
    auto *volume = object->add_volume(TriangleMesh(its_make_cube(10., 10., 10.)));
    volume->support_base_facets.set_triangle_from_string(0, "4");
    volume->supported_facets.set_triangle_from_string(0, "8");
    auto *shared = object->add_volume(*volume);
    shared->set_offset(Vec3d(20., 0., 0.));
    shared->support_base_facets.reset();
    shared->support_base_facets.set_triangle_from_string(2, "4");
    // A third part has exactly the legacy annotations and no base paint.
    auto *legacy = object->add_volume(*volume);
    legacy->set_offset(Vec3d(40., 0., 0.));
    legacy->support_base_facets.reset();
    const std::string filename = (workspace / "bases.3mf").string();
    DynamicPrintConfig config;
    StoreParams params;
    params.path = filename.c_str(); params.model = &source; params.config = &config;
    REQUIRE(store_bbs_3mf(params));
    Model loaded;
    DynamicPrintConfig loaded_config;
    ConfigSubstitutionContext substitutions(ForwardCompatibilitySubstitutionRule::Disable);
    bool is_bambu = false; Semver version;
    PlateDataPtrs plates; std::vector<Preset*> presets;
    REQUIRE(load_bbs_3mf(filename.c_str(), &loaded_config, &substitutions, &loaded, &plates, &presets, &is_bambu, &version,
                        nullptr, LoadStrategy::LoadModel | LoadStrategy::LoadConfig));
    REQUIRE(loaded.objects.size() == 1);
    REQUIRE(loaded.objects[0]->volumes.size() == 3);
    for (size_t i = 0; i < 3; ++i) {
        REQUIRE(loaded.objects[0]->volumes[i]->supported_facets.equals(object->volumes[i]->supported_facets));
        REQUIRE(loaded.objects[0]->volumes[i]->support_base_facets.equals(object->volumes[i]->support_base_facets));
    }
    for (auto *plate : plates) delete plate;
}

TEST_CASE("Support base masks survive split merge and mesh replacement", "[SupportBase][Model]")
{
    Model model; auto *object = model.add_object();
    auto first = TriangleMesh(its_make_cube(10., 10., 10.));
    auto second = TriangleMesh(its_make_cube(10., 10., 10.)); second.translate(20., 0., 0.);
    const size_t first_faces = first.its.indices.size(); first.merge(second);
    auto *volume = object->add_volume(std::move(first));
    TriangleSelector selector(volume->mesh());
    for (size_t i = 0; i < first_faces; ++i) selector.set_facet(i, EnforcerBlockerType::ENFORCER);
    volume->support_base_facets.set(selector);
    REQUIRE(volume->split(1) == 2);
    REQUIRE(object->volumes.size() == 2);
    size_t painted_parts = 0;
    for (const auto *part : object->volumes) painted_parts += !part->support_base_facets.empty();
    REQUIRE(painted_parts == 1);
    const bool mirrored = GENERATE(false, true);
    if (mirrored) object->volumes.front()->set_mirror(X, -1.);
    object->merge();
    REQUIRE(object->volumes.size() == 1);
    volume = object->volumes.front();
    REQUIRE(volume->support_base_facets.get_facets(*volume, EnforcerBlockerType::ENFORCER).indices.size() == first_faces);
    REQUIRE(volume->set_mesh_keep_paint(TriangleMesh(volume->mesh())));
    REQUIRE(volume->support_base_facets.get_facets(*volume, EnforcerBlockerType::ENFORCER).indices.size() == first_faces);
    REQUIRE(volume->supported_facets.empty());
}

TEST_CASE("Changing support bases invalidates support generation and preserves model slicing", "[SupportBase][Invalidate]")
{
    Model model; auto *object = model.add_object();
    auto *volume = object->add_volume(TriangleMesh(its_make_cube(10., 10., 2.)));
    object->add_instance(); object->ensure_on_bed();
    DynamicPrintConfig config;
    config.apply(static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    config.set_deserialize_strict({{"layer_height", "0.2"}, {"enable_support", "1"}, {"support_type", "tree(auto)"},
                                  {"support_style", "tree_organic"}, {"support_on_build_plate_only", "1"}});
    Print print; print.set_status_silent(); print.auto_assign_extruders(object); print.apply(model, config);
    print.validate(); print.process();
    auto *printed = print.get_object(size_t(0));
    REQUIRE(printed->is_step_done(posSupportMaterial));
    volume->support_base_facets.set_triangle_from_string(0, "4");
    print.apply(model, config);
    REQUIRE(printed->is_step_done(posSlice));
    REQUIRE(printed->is_step_done(posPerimeters));
    REQUIRE_FALSE(printed->is_step_done(posSupportMaterial));
    REQUIRE(printed->model_object()->volumes[0]->support_base_facets.equals(volume->support_base_facets));
}

TEST_CASE("Contour cut assembly retains support bases on each reconstructed part", "[SupportBase][Model]")
{
    Model model; auto *object = model.add_object();
    object->add_instance();
    for (int i = 0; i < 2; ++i) {
        auto *volume = object->add_volume(TriangleMesh(its_make_cube(10., 10., 10.)));
        volume->set_offset(Vec3d(20. * i, 0., 0.));
        volume->support_base_facets.set_triangle_from_string(i, "4");
        volume->supported_facets.set_triangle_from_string(2 + i, "8");
    }
    Cut cut(object, 0, Transform3d::Identity(), ModelObjectCutAttribute::KeepUpper | ModelObjectCutAttribute::KeepLower);
    const auto &halves = cut.perform_by_contour({{true, false}, {false, false}}, 0);
    REQUIRE(halves.size() == 2);
    for (const auto *half : halves) {
        REQUIRE(half->volumes.size() == 1);
        const auto *volume = half->volumes.front();
        REQUIRE(volume->support_base_facets.get_facets(*volume, EnforcerBlockerType::ENFORCER).indices.size() == 1);
        REQUIRE(volume->supported_facets.get_facets(*volume, EnforcerBlockerType::BLOCKER).indices.size() == 1);
    }
}
