#include <catch2/catch.hpp>

#include "bridge_aware_cubic_fixtures.hpp"
#include "bridge_aware_cubic_observer.hpp"

#include "libslic3r/Fill/BridgeAwareCubicPlan.hpp"

#include <algorithm>
#include <string>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::BridgeAwareCubic;
using namespace BACFixtures;
using namespace BACObserver;

// The courses of section 3.5: seam walls (W1) and rails (R1). Read from the plan through the object, and from the export.

namespace {

// The 48 x 28 x 18 mm cube of the modifier fixture whose right half (24 x 28 mm from x = 24, the whole height) is a parameter
// modifier with the given settings; sliced with modifier_config() the left half prints the pattern at 5 %, 45 degrees, gap 8.
Model right_half_modifier(std::initializer_list<std::pair<const std::string, std::string>> settings)
{
    Model model = single_part("Bridge aware cubic right half", TriangleMesh(its_make_cube(48., 28., 18.)));
    TriangleMesh mesh(its_make_cube(24., 28., 18.));
    mesh.translate(24., 0., 0.);
    ModelVolume *right = model.objects.front()->add_volume(std::move(mesh));
    right->set_type(ModelVolumeType::PARAMETER_MODIFIER);
    right->name = "Right half modifier";
    DynamicPrintConfig config;
    for (const auto &[key, value] : settings)
        config.set_deserialize_strict(key, value);
    right->config.apply(config);
    return model;
}

constexpr double SEAM_X = 80. + 24.;    // the seam plane of the modifier fixtures on the bed, mm

// A sliced print and its exported paths: the plan is read through the print's object, the paths through the observer.
struct Sliced
{
    Print                    print;
    std::vector<EmittedPath> paths;
    const PrintObject&       object() const { return *print.get_object(size_t(0)); }
    const ObjectPlan*        plan() const { return object().bridge_aware_cubic_plan(); }
};

void slice_and_export(Sliced &out, const Workspace &workspace, Model model, const DynamicPrintConfig &settings, const std::string &name)
{
    const DynamicPrintConfig config = observed(settings);
    slice(out.print, model, config);
    const std::string filename = out.print.export_gcode((workspace.path / name).string(), nullptr);
    out.paths = read_extrusions(filename, config);
}

// The index of the object's region whose unsupported gap is `gap`, or −1.
int region_with_gap(const PrintObject &object, double gap)
{
    for (size_t i = 0; i < object.num_printing_regions(); ++i)
        if (std::abs(object.printing_region(i).config().bridge_aware_cubic_max_gap.value - gap) < 1e-9)
            return int(i);
    return -1;
}

// Whether the region has a sparse (stInternal) fill surface on the layer.
bool has_sparse(const Layer &layer, size_t region_id)
{
    if (region_id >= layer.regions().size())
        return false;
    for (const Surface &surface : layer.regions()[region_id]->fill_surfaces.surfaces)
        if (surface.surface_type == stInternal)
            return true;
    return false;
}

// The layer whose print_z is the plane key, or −1.
int layer_of_key(const PrintObject &object, int key)
{
    for (size_t l = 0; l < object.layers().size(); ++l)
        if (plane_key(object.layers()[l]->print_z) == key)
            return int(l);
    return -1;
}

// The width of the sparse infill bead on the export.
double sparse_width(const std::vector<EmittedPath> &paths)
{
    for (const EmittedPath &path : paths)
        if (path.role == erInternalInfill)
            return path.width;
    return 0.;
}

// The length (mm) of the near-vertical sparse segments of a layer whose x lies within [x0, x1]: a seam wall of the modifier
// fixtures runs along y; the lattice lines (45, 105 and 165 degrees) and the connector's hooks lie elsewhere or are not vertical.
double vertical_sparse_length(const EmittedLayer &layer, double x0, double x1)
{
    double total = 0.;
    for (const EmittedPath *path : layer) {
        if (path->role != erInternalInfill)
            continue;
        for (size_t i = 1; i < path->path.points.size(); ++i) {
            const Vec2d a = unscale(path->path.points[i - 1]), b = unscale(path->path.points[i]);
            if (std::abs(a.x() - b.x()) > 0.02 || std::abs(a.y() - b.y()) < 0.5)
                continue;
            if (a.x() < x0 || a.x() > x1 || b.x() < x0 || b.x() > x1)
                continue;
            total += std::abs(a.y() - b.y());
        }
    }
    return total;
}

// The y extent (mm) of the sparse paths of a layer.
double sparse_y_extent(const EmittedLayer &layer)
{
    double y0 = std::numeric_limits<double>::max(), y1 = std::numeric_limits<double>::lowest();
    for (const EmittedPath *path : layer)
        if (path->role == erInternalInfill)
            for (const Point &p : path->path.points) {
                y0 = std::min(y0, unscale<double>(p.y()));
                y1 = std::max(y1, unscale<double>(p.y()));
            }
    return y1 > y0 ? y1 - y0 : 0.;
}

// The largest x of any sparse path vertex of the export.
double sparse_max_x(const std::vector<EmittedPath> &paths)
{
    double x = std::numeric_limits<double>::lowest();
    for (const EmittedPath &path : paths)
        if (path.role == erInternalInfill)
            for (const Point &p : path.path.points)
                x = std::max(x, unscale<double>(p.x()));
    return x;
}

// Every layer and region of the plan has no course.
bool no_courses(const ObjectPlan &plan, const PrintObject &object)
{
    for (size_t l = 0; l < object.layers().size(); ++l)
        for (size_t r = 0; r < object.num_printing_regions(); ++r)
            if (!plan.courses_at(l, r).empty())
                return false;
    return true;
}

} // namespace

// W1: seam walls. Where two lattices meet, the lower-index region lays one bead of its sparse infill just inside its boundary
// (3.5a), on every layer where the seam exists; each half then holds its own gap; a floating modifier's first course is a roof
// for the plane beneath it and K2 holds across the seam; a 0 % neighbour gets a wall at half a bead and stays empty; no wall
// is laid against material or between regions of one group.
TEST_CASE("Bridge aware cubic lays one seam wall where two lattices meet", "[BridgeAwareCubic][W1]")
{
    Workspace workspace;

    SECTION("the modifier seam carries one bead of the lower-index region on every sparse layer, inset as 3.5a") {
        Sliced sliced;
        slice_and_export(sliced, workspace, modifier(), modifier_config("bridgeawarecubic"), "w1-modifier.gcode");
        const PrintObject &object = sliced.object();
        const ObjectPlan *plan = sliced.plan();
        REQUIRE(plan != nullptr);
        const int left = region_with_gap(object, 8.), right = region_with_gap(object, 3.);
        REQUIRE(left >= 0);
        REQUIRE(right >= 0);
        const size_t lower = size_t(std::min(left, right)), higher = size_t(std::max(left, right));
        REQUIRE(plan->group_of_region(lower) != nullptr);
        REQUIRE(plan->group_of_region(higher) != nullptr);
        CHECK(plan->group_of_region(lower) != plan->group_of_region(higher));

        // The plan: walls of the lower-index region on every layer where both regions have sparse fill, none of the other region.
        size_t layers_with_walls = 0;
        for (size_t l = 0; l < object.layers().size(); ++l) {
            INFO("layer " << l << " at z = " << object.layers()[l]->print_z);
            CHECK(plan->courses_at(l, higher).empty());
            const bool seam = has_sparse(*object.layers()[l], lower) && has_sparse(*object.layers()[l], higher);
            CHECK(plan->courses_at(l, lower).empty() == !seam);
            if (!plan->courses_at(l, lower).empty())
                ++layers_with_walls;
        }
        CHECK(layers_with_walls >= 80);

        // The export: on every layer with a wall, a near-vertical sparse bead at w/2 − 0.05 from the seam on the lower-index
        // region's side (the user's 0.05 mm overlap into the neighbour's butt ends), covering at least 90 % of the sparse extent.
        const double w = sparse_width(sliced.paths);
        REQUIRE(w > 0.3);
        const double side = lower == size_t(left) ? -1. : +1.;
        const double x_wall = SEAM_X + side * (0.5 * w - 0.05), x_other = SEAM_X - side * (0.5 * w - 0.05);
        size_t layers_checked = 0;
        for (const auto &[key, layer] : emitted_layers(sliced.paths)) {
            const int l = layer_of_key(object, key);
            if (l < 0 || plan->courses_at(size_t(l), lower).empty())
                continue;
            INFO("z = " << key / 1000.);
            const double extent = sparse_y_extent(layer);
            REQUIRE(extent > 20.);
            CHECK(vertical_sparse_length(layer, x_wall - 0.015, x_wall + 0.015) >= 0.9 * extent);
            CHECK(vertical_sparse_length(layer, x_other - 0.015, x_other + 0.015) < 0.1 * extent);
            ++layers_checked;
        }
        CHECK(layers_checked == layers_with_walls);

        // Each half holds its own gap on the export (K1 with the seam closed): no opening spans the seam, and every opening and
        // run is within the gap of its side.
        const std::vector<PlaneMeasure> planes = measure_planes(sliced.paths, 0.4, 8.);
        REQUIRE_FALSE(planes.empty());
        for (const PlaneMeasure &plane : planes) {
            INFO("plane below z = " << plane.lower / 1000. << " mm");
            for (const ExPolygon &component : plane.components) {
                const BoundingBox box = get_extents(component);
                const double x0 = unscale<double>(box.min.x()), x1 = unscale<double>(box.max.x());
                CHECK_FALSE((x0 < SEAM_X - 0.3 && x1 > SEAM_X + 0.3));
                const double span = std::max(diameter_mm(Geometry::convex_hull(component.contour.points).points), strip_length_mm(component));
                CHECK(span <= (0.5 * (x0 + x1) < SEAM_X ? 8. : 3.));
            }
            CHECK(plane.longest_run <= (plane.where_longest.x() < SEAM_X ? 8. : 3.));
        }
    }

    SECTION("a floating modifier's first course is a roof for the plane beneath it and K2 holds across the seam") {
        Sliced sliced;
        slice_and_export(sliced, workspace, floating_modifier(), modifier_config("bridgeawarecubic"), "w1-floating.gcode");
        const PrintObject &object = sliced.object();
        const ObjectPlan *plan = sliced.plan();
        REQUIRE(plan != nullptr);
        size_t first_course = size_t(-1);
        for (size_t l = 0; l < object.layers().size() && first_course == size_t(-1); ++l)
            for (size_t r = 0; r < object.num_printing_regions(); ++r)
                if (!plan->courses_at(l, r).empty()) { first_course = l; break; }
        REQUIRE(first_course != size_t(-1));
        REQUIRE(first_course > 0);
        CHECK(object.layers()[first_course]->print_z == Approx(6.2));
        const std::vector<PlaneSummary> planes = plan->planes();
        CHECK(std::any_of(planes.begin(), planes.end(), [first_course](const PlaneSummary &p) { return p.roof_layer == first_course && p.plane_layer == first_course - 1; }));
        // Below the modifier the whole part is one region at gap 8; above, the right half (x from the seam) is at gap 3.
        const double z_first = object.layers()[first_course]->print_z;
        const std::vector<EmittedPath> control = exported_extrusions(workspace, floating_modifier(), modifier_config("cubic"), "w1-floating-cubic.gcode");
        const K2Measure measure = measure_k2(sliced.paths, control,
            [z_first](const Vec2d &at, double z) { return (z >= z_first - 1e-6 && at.x() >= SEAM_X ? 3. : 8.) - 0.1; }, 45.);
        CHECK(measure.added_pieces > 0);
        CHECK(measure.free_ends == 0);
        CHECK(measure.long_stretches == 0);
    }

    SECTION("a 0 % neighbour gets a wall at half a bead less the overlap and stays empty") {
        Sliced sliced;
        slice_and_export(sliced, workspace, right_half_modifier({{"sparse_infill_density", "0%"}}), modifier_config("bridgeawarecubic"), "w1-empty.gcode");
        const PrintObject &object = sliced.object();
        const ObjectPlan *plan = sliced.plan();
        REQUIRE(plan != nullptr);
        const double w = sparse_width(sliced.paths);
        REQUIRE(w > 0.3);
        // Inset as beside beads (w/2 - 0.05): at a convex corner the round join of a bead at w/2 leaves a pocket joining the 0 %
        // void to the region's surface; the 0.05 mm overlap into a region that prints nothing there is harmless.
        const double x_wall = SEAM_X - (0.5 * w - 0.05);
        size_t layers_checked = 0;
        for (const auto &[key, layer] : emitted_layers(sliced.paths)) {
            const int l = layer_of_key(object, key);
            if (l < 0)
                continue;
            bool courses = false;
            for (size_t r = 0; r < object.num_printing_regions(); ++r)
                if (!plan->courses_at(size_t(l), r).empty())
                    courses = true;
            if (!courses)
                continue;
            INFO("z = " << key / 1000.);
            const double extent = sparse_y_extent(layer);
            REQUIRE(extent > 20.);
            CHECK(vertical_sparse_length(layer, x_wall - 0.015, x_wall + 0.015) >= 0.9 * extent);
            ++layers_checked;
        }
        CHECK(layers_checked >= 80);
        CHECK(sparse_max_x(sliced.paths) <= SEAM_X + 0.01);
        // The 0 % region's surfaces never meet the pattern's void: the wall seals the seam at the corners too.
        const int zero_region = object.printing_region(0).config().sparse_infill_density.value <= 0. ? 0 : 1;
        size_t sealed_layers = 0;
        for (size_t l = 0; l < object.layers().size(); ++l) {
            const Layer &layer = *object.layers()[l];
            if (size_t(zero_region) >= layer.regions().size())
                continue;
            ExPolygons zero;
            for (const Surface &surface : layer.regions()[zero_region]->fill_surfaces.surfaces)
                if (surface.surface_type == stInternal)
                    zero.push_back(surface.expolygon);
            if (zero.empty())
                continue;
            INFO("layer " << l << " at z = " << layer.print_z);
            for (const ExPolygons &group_void : plan->material(l).void_of_group)
                CHECK(intersection_ex(group_void, zero).empty());
            ++sealed_layers;
        }
        CHECK(sealed_layers >= 80);
    }

    SECTION("no wall against material or between regions of one group") {
        {
            Print print;
            Model model = right_half_modifier({{"sparse_infill_density", "100%"}});
            slice(print, model, modifier_config("bridgeawarecubic"));
            const PrintObject &object = *print.get_object(size_t(0));
            REQUIRE(object.bridge_aware_cubic_plan() != nullptr);
            CHECK(no_courses(*object.bridge_aware_cubic_plan(), object));
        }
        {
            Print print;
            Model model = right_half_modifier({{gap_key, "3"}});
            slice(print, model, modifier_config("bridgeawarecubic"));
            const PrintObject &object = *print.get_object(size_t(0));
            const ObjectPlan *plan = object.bridge_aware_cubic_plan();
            REQUIRE(plan != nullptr);
            REQUIRE(object.num_printing_regions() == 2);
            CHECK(plan->group_of_region(0) == plan->group_of_region(1));
            CHECK(no_courses(*plan, object));
        }
    }
}
