#include <catch2/catch.hpp>

#include "bridge_aware_cubic_fixtures.hpp"

#include "libslic3r/BridgeAwareCubicMaterial.hpp"
#include "libslic3r/BridgeAwareCubicVerify.hpp"
#include "libslic3r/Fill/BridgeAwareCubicPlan.hpp"
#include "libslic3r/PrintBase.hpp"

#include <algorithm>
#include <random>
#include <set>

using namespace Slic3r;
using namespace Slic3r::BridgeAwareCubic;
using namespace BACFixtures;

namespace {

// A 40 x 30 x 12 mm block with a 0.3 mm wide strip through its middle set to 100% rectilinear infill: a solid surface narrower
// than 1.1 solid spacings, which may print but is never counted on.
Model narrow_strip_block()
{
    Model model = single_part("Bridge aware cubic narrow strip", TriangleMesh(its_make_cube(40., 30., 12.)));
    TriangleMesh strip(its_make_cube(0.3, 30., 12.));
    strip.translate(20., 0., 0.);
    ModelVolume *modifier = model.objects.front()->add_volume(std::move(strip));
    modifier->set_type(ModelVolumeType::PARAMETER_MODIFIER);
    modifier->config.set_key_value("sparse_infill_density", new ConfigOptionPercent(100.));
    modifier->config.set_key_value("sparse_infill_pattern", new ConfigOptionEnum<InfillPattern>(ipRectilinear));
    return model;
}

// A prism whose cross-section is the trapezoid with the base [0, width] at z = 0 and the top [top_from, width] at z = height,
// extruded over `length` in y: one side vertical, the other a slope that steps in by height / (top_from) per unit of height.
Model sloped_block(double width, double length, double height, double top_from)
{
    indexed_triangle_set its;
    its.vertices = {
        { 0.f, 0.f, 0.f }, { float(width), 0.f, 0.f }, { float(width), float(length), 0.f }, { 0.f, float(length), 0.f },
        { float(top_from), 0.f, float(height) }, { float(width), 0.f, float(height) }, { float(width), float(length), float(height) }, { float(top_from), float(length), float(height) }
    };
    its.indices = {
        { 0, 2, 1 }, { 0, 3, 2 },   // bottom
        { 4, 5, 6 }, { 4, 6, 7 },   // top
        { 0, 1, 5 }, { 0, 5, 4 },   // front (y = 0)
        { 1, 2, 6 }, { 1, 6, 5 },   // right (x = width), vertical
        { 2, 3, 7 }, { 2, 7, 6 },   // back (y = length)
        { 3, 0, 4 }, { 3, 4, 7 }    // left: the slope
    };
    return single_part("Bridge aware cubic sloped block", TriangleMesh(its));
}

// The regions that print the pattern, one group each (enough for the material of one object).
std::vector<std::vector<size_t>> pattern_groups(const PrintObject &object)
{
    std::vector<std::vector<size_t>> groups;
    for (size_t i = 0; i < object.num_printing_regions(); ++i) {
        const PrintRegionConfig &config = object.printing_region(i).config();
        if (config.sparse_infill_pattern.value == ipBridgeAwareCubic && config.sparse_infill_density.value > 0. && config.sparse_infill_density.value < 100.)
            groups.push_back({ i });
    }
    return groups;
}

ExPolygons surfaces_of(const Layer &layer, const std::function<bool(const LayerRegion &, const Surface &)> &take)
{
    ExPolygons out;
    for (const LayerRegion *region : layer.regions())
        for (const Surface &surface : region->fill_surfaces.surfaces)
            if (take(*region, surface))
                out.push_back(surface.expolygon);
    return out;
}

bool is_sparse(const LayerRegion &region, const Surface &surface)
{
    const double density = region.region().config().sparse_infill_density.value;
    return surface.surface_type == stInternal && density > 0. && density < 100.;
}

// The lowest layer whose non-sparse fill lies over sparse fill of the layer below, from the final surfaces alone. Surfaces are
// shrunk by 0.3 mm so that the one-spacing ensuring strips of the bridge generator do not count as roofs.
size_t first_roof_layer(const PrintObject &object)
{
    const auto &layers = object.layers();
    for (size_t n = 1; n < layers.size(); ++n) {
        const ExPolygons sparse_below = surfaces_of(*layers[n - 1], is_sparse);
        if (sparse_below.empty()) continue;
        const ExPolygons roof = surfaces_of(*layers[n], [](const LayerRegion &region, const Surface &surface) { return !is_sparse(region, surface) && surface.surface_type != stInternalVoid; });
        if (!intersection_ex(offset_ex(roof, -scale_(0.3)), sparse_below).empty())
            return n;
    }
    return size_t(-1);
}

std::set<size_t> plane_layers_of(const std::vector<PlaneSummary> &planes, size_t roof)
{
    std::set<size_t> out;
    for (const PlaneSummary &plane : planes)
        if (plane.roof_layer == roof)
            out.insert(plane.plane_layer);
    return out;
}

} // namespace

// M1: printed material. A solid strip narrower than 1.1 spacings may print (roof side) but surely does not (support side);
// the wall band is material on both sides; the void meets the sparse surfaces; the fill boundary is FillCubic's inset, less the
// courses' beads at the perimeter overlap; the area index agrees with the polygons.
TEST_CASE("Bridge aware cubic material has one definition with two sides", "[BridgeAwareCubic][M1]")
{
    SECTION("a narrow solid strip is a roof above and void below") {
        Print print;
        Model model = narrow_strip_block();
        slice(print, model, fixture_config(5., "bridgeawarecubic"));
        const PrintObject &object = *print.get_object(size_t(0));
        const Layer &layer = *object.layers()[object.layers().size() / 2];
        const LayerMaterial material = printed_material(layer, pattern_groups(object));
        const ExPolygons strip = surfaces_of(layer, [](const LayerRegion &region, const Surface &surface) {
            return region.region().config().sparse_infill_density.value >= 100. || surface.surface_type == stInternalSolid; });
        REQUIRE_FALSE(strip.empty());
        CHECK(area(strip) * SCALING_FACTOR * SCALING_FACTOR < 0.4 * 30.);
        CHECK(diff_ex(offset_ex(strip, -scale_(0.001)), material.may_print).empty());
        // Outside the wall band (the strip's fill surface reaches into the walls by the infill overlap, where it is material).
        CHECK(intersection_ex(offset_ex(diff_ex(strip, material.walls), -scale_(0.001)), material.surely_prints).empty());
        CHECK(diff_ex(offset_ex(material.walls, -scale_(0.001)), material.surely_prints).empty());
        CHECK(diff_ex(offset_ex(material.walls, -scale_(0.001)), material.may_print).empty());
        const ExPolygons sparse = diff_ex(surfaces_of(layer, is_sparse), material.walls);
        CHECK_FALSE(intersection_ex(material.void_area, sparse).empty());
        CHECK(diff_ex(offset_ex(sparse, -scale_(0.001)), material.void_area).empty());
        REQUIRE(material.void_of_group.size() == 1);
        CHECK_FALSE(material.void_of_group.front().empty());
    }

    SECTION("a seam wall separates the voids of two groups on every layer") {
        Print print;
        Model model = modifier();
        slice(print, model, modifier_config("bridgeawarecubic"));
        const PrintObject &object = *print.get_object(size_t(0));
        const ObjectPlan *plan = object.bridge_aware_cubic_plan();
        REQUIRE(plan != nullptr);
        REQUIRE(plan->groups().size() == 2);
        size_t layers_with_both = 0;
        for (size_t l = 0; l < object.layers().size(); ++l) {
            const LayerMaterial &material = plan->material(l);
            REQUIRE(material.void_of_group.size() == 2);
            if (material.void_of_group[0].empty() || material.void_of_group[1].empty())
                continue;
            INFO("layer " << l << " at z = " << object.layers()[l]->print_z);
            CHECK(intersection_ex(material.void_of_group[0], material.void_of_group[1]).empty());
            ++layers_with_both;
        }
        CHECK(layers_with_both >= 80);
    }

    SECTION("the fill boundary is the island inset by half a spacing, less the courses at the perimeter overlap") {
        const double s_f = 0.357, w = 0.4;
        const ExPolygon island(Polygon({ Point::new_scale(0., 0.), Point::new_scale(30., 0.), Point::new_scale(30., 20.), Point::new_scale(0., 20.) }));
        const ExPolygons plain = fill_boundary(island, {}, s_f, w, 15.);
        const ExPolygons expected = offset_ex(island, float(coord_t(float(scale_(-0.5 * s_f)))), ClipperLib::jtMiter, 3.);
        CHECK(diff_ex(plain, expected).empty());
        CHECK(diff_ex(expected, plain).empty());

        const Polylines courses{ Polyline(Point::new_scale(-1., 10.), Point::new_scale(31., 10.)) };
        const ExPolygons cut = fill_boundary(island, courses, s_f, w, 15.);
        const double keep = 0.5 * w - 0.05;    // a line ends inside the course's bead (3.8 as amended): the filler's nub fuses it
        const AreaIndex index(cut);
        CHECK_FALSE(index.contains(Point::new_scale(15., 10. + keep - 0.01)));
        CHECK(index.contains(Point::new_scale(15., 10. + keep + 0.01)));
        CHECK_FALSE(index.contains(Point::new_scale(15., 10. - keep + 0.01)));
        CHECK(index.contains(Point::new_scale(15., 10. - keep - 0.01)));

        std::mt19937 rng(7);
        std::uniform_real_distribution<double> x(-1., 31.), y(-1., 21.);
        for (int i = 0; i < 1000; ++i) {
            const Point p = Point::new_scale(x(rng), y(rng));
            const bool expected_inside = std::any_of(cut.begin(), cut.end(), [&p](const ExPolygon &part) { return part.contains(p); });
            CHECK(index.contains(p) == expected_inside);
        }
        const auto intervals = index.segment_inside(Point::new_scale(15., -2.), Point::new_scale(15., 22.));
        REQUIRE(intervals.size() == 2);
        const double inset = unscale<double>(coord_t(float(scale_(0.5 * s_f))));
        // Along y from −2 to 22 (24 mm): the boundary runs from the inset edge to the course band and from the band to the edge.
        CHECK(-2. + intervals[0].first * 24. == Approx(inset).margin(1e-5));
        CHECK(-2. + intervals[0].second * 24. == Approx(10. - keep).margin(1e-5));
        CHECK(-2. + intervals[1].first * 24. == Approx(10. + keep).margin(1e-5));
        CHECK(-2. + intervals[1].second * 24. == Approx(20. - inset).margin(1e-5));
        CHECK(index.segment_inside(Point::new_scale(-5., 10.), Point::new_scale(-3., 10.)).empty());
    }
}

// P1: demand. The cubes have planes only under the roof, on B + 1 layers below it; a vertical wall and a steep slope create none;
// a step wider than half the nozzle does; a solid block has none and slices; planes stop at layer 0; a cancel propagates.
TEST_CASE("Bridge aware cubic plans demand planes under roofs only", "[BridgeAwareCubic][P1]")
{
    SECTION("the small cube has planes under its roof on B + 1 layers") {
        // The roof from the surfaces of the one-bridge-layer slice: with more internal bridge layers the stock step projects the
        // bridge down as thick bases after the plan, which the final surfaces then show as roofs too.
        size_t roof = size_t(-1);
        for (const int bridge_layers : {1, 3}) {
            CAPTURE(bridge_layers);
            Print print;
            Model model = small_cube();
            slice(print, model, with(fixture_config(5., "bridgeawarecubic"), {{"internal_bridge_layers", std::to_string(bridge_layers)}}));
            const PrintObject &object = *print.get_object(size_t(0));
            REQUIRE(object.bridge_aware_cubic_plan() != nullptr);
            const std::vector<PlaneSummary> planes = object.bridge_aware_cubic_plan()->planes();
            REQUIRE_FALSE(planes.empty());
            if (bridge_layers == 1)
                roof = first_roof_layer(object);
            REQUIRE(roof != size_t(-1));
            for (const PlaneSummary &plane : planes) {
                CHECK(plane.roof_layer == roof);
                CHECK(plane.gap == 5.);
                CHECK(plane.footprint_area_mm2 > 100.);
            }
            std::set<size_t> expected;
            for (int j = 0; j <= bridge_layers; ++j)
                expected.insert(roof - 1 - j);
            CHECK(plane_layers_of(planes, roof) == expected);
            CHECK(planes.size() == size_t(bridge_layers + 1));
        }
    }

    SECTION("a vertical wall, a steep slope and a 45 degree slope create no plane") {
        struct Case { const char *name; Model model; } cases[] = {
            { "thin plate of walls only", single_part("Bridge aware cubic plate", TriangleMesh(its_make_cube(20., 1.6, 20.))) },
            { "slope steeper than 45 degrees", sloped_block(20., 20., 20., 10.) },
            { "slope of 45 degrees", sloped_block(20., 20., 10., 10.) },
        };
        for (Case &c : cases) {
            INFO(c.name);
            Print print;
            slice(print, c.model, with(fixture_config(5., "bridgeawarecubic"), {{"top_shell_layers", "0"}, {"bottom_shell_layers", "0"}}));
            const PrintObject &object = *print.get_object(size_t(0));
            if (object.bridge_aware_cubic_plan() != nullptr)
                CHECK(object.bridge_aware_cubic_plan()->planes().empty());
        }
    }

    SECTION("a roof that ends over sparse fill creates planes at that height and at the top") {
        Print print;
        Model model = partial_roof();
        slice(print, model, fixture_config(5., "bridgeawarecubic"));
        const PrintObject &object = *print.get_object(size_t(0));
        REQUIRE(object.bridge_aware_cubic_plan() != nullptr);
        std::set<size_t> roofs;
        for (const PlaneSummary &plane : object.bridge_aware_cubic_plan()->planes())
            roofs.insert(plane.roof_layer);
        REQUIRE(roofs.size() >= 2);
        // The left half's roof lies at 10 mm (its first solid layer at 9.4 mm with three top layers), the top at 14 mm.
        CHECK(std::any_of(roofs.begin(), roofs.end(), [&](size_t n) { return std::abs(object.layers()[n]->print_z - 9.6) < 0.3; }));
        CHECK(std::any_of(roofs.begin(), roofs.end(), [&](size_t n) { return std::abs(object.layers()[n]->print_z - 13.6) < 0.3; }));
    }

    SECTION("a solid block has no plane and slices") {
        Print print;
        Model model = single_part("Bridge aware cubic solid block", TriangleMesh(its_make_cube(10., 10., 1.)));    // five layers: three bottom and three top shells cover them all
        REQUIRE_NOTHROW(slice(print, model, fixture_config(5., "bridgeawarecubic")));
        const PrintObject &object = *print.get_object(size_t(0));
        if (object.bridge_aware_cubic_plan() != nullptr)
            CHECK(object.bridge_aware_cubic_plan()->planes().empty());
    }

    SECTION("planes stop at the first layer") {
        Print print;
        Model model = single_part("Bridge aware cubic low roof", TriangleMesh(its_make_cube(20., 20., 1.2)));
        slice(print, model, with(fixture_config(5., "bridgeawarecubic"), {{"bottom_shell_layers", "0"}, {"internal_bridge_layers", "3"}}));
        const PrintObject &object = *print.get_object(size_t(0));
        REQUIRE(object.bridge_aware_cubic_plan() != nullptr);
        const std::vector<PlaneSummary> planes = object.bridge_aware_cubic_plan()->planes();
        REQUIRE_FALSE(planes.empty());
        size_t lowest = size_t(-1);
        for (const PlaneSummary &plane : planes)
            lowest = std::min(lowest, plane.plane_layer);
        CHECK(lowest == 0);
        CHECK(planes.size() < 4);
    }

    SECTION("a cancel while the plan builds propagates") {
        Print print;
        Model model = small_cube();
        slice(print, model, fixture_config(5., "bridgeawarecubic"));
        const PrintObject &object = *print.get_object(size_t(0));
        size_t calls = 0;
        REQUIRE_THROWS_AS(ObjectPlan::build(object, [&calls]() { if (++calls == 1) throw CanceledException(); }), CanceledException);
        REQUIRE_NOTHROW(ObjectPlan::build(object, []() {}));
    }
}


// V1: the development verifier measures K1 on the final paths. The small cube sliced as Cubic, measured with the planes its own
// surfaces define, has a failing plane at gap 5 (its cell is 17.5 mm); the pattern's small cube passes on every plane, its plan has
// nothing to warn about, and this build (the flag off) attaches no Bridge aware cubic warning to the slice.
TEST_CASE("Bridge aware cubic verifier fails Cubic and passes the pattern", "[BridgeAwareCubic][V1]")
{
    using namespace Slic3r;
    using namespace BACFixtures;
    {
        Print print;
        Model model = small_cube();
        slice(print, model, fixture_config(5., "cubic"));
        const PrintObject &object = *print.get_object(size_t(0));
        REQUIRE(object.bridge_aware_cubic_plan() == nullptr);
        const auto plan = BridgeAwareCubic::ObjectPlan::build_for_verification(object, [] {});
        REQUIRE_FALSE(plan->planes().empty());
        const BridgeAwareCubic::VerificationReport report = BridgeAwareCubic::verify(object, *plan, [] {});
        REQUIRE_FALSE(report.planes.empty());
        CHECK_FALSE(report.passed());
        CHECK(std::any_of(report.planes.begin(), report.planes.end(), [](const BridgeAwareCubic::PlaneReport &p) { return p.max_opening > 5.; }));
    }
    {
        Print print;
        Model model = small_cube();
        slice(print, model, fixture_config(5., "bridgeawarecubic"));
        const PrintObject &object = *print.get_object(size_t(0));
        REQUIRE(object.bridge_aware_cubic_plan() != nullptr);
        const BridgeAwareCubic::VerificationReport report = BridgeAwareCubic::verify(object, [] {});
        REQUIRE(report.planes.size() >= 2);
        for (const BridgeAwareCubic::PlaneReport &plane : report.planes) {
            INFO("plane at " << plane.z << " mm: " << plane.failure);
            CHECK(plane.passed);
            CHECK(plane.max_opening <= 5.);
            CHECK(plane.max_run <= 5.);
            CHECK(plane.bridge_free_ends == 0);
        }
        CHECK(BridgeAwareCubic::plan_warnings(object).empty());
        for (const auto &warning : object.step_state_with_warnings(posSimplifyInfill).warnings)
            CHECK(warning.message.find("Bridge aware cubic") == std::string::npos);
    }
}
