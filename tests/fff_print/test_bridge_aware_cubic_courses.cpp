#include <catch2/catch.hpp>

#include "bridge_aware_cubic_fixtures.hpp"
#include "bridge_aware_cubic_observer.hpp"

#include <algorithm>
#include <sstream>
#include <string>
#include <vector>

using namespace Slic3r;
using namespace BACFixtures;
using namespace BACObserver;

// The courses of section 3.5: seam walls (W1) and rails (R1), read from the export and the final surfaces.

namespace {

constexpr double SEAM_X = 80. + 24.;    // the seam plane of the modifier fixtures on the bed, mm

// A sliced print and its exported paths, read through the observer.
struct Sliced
{
    Print                    print;
    std::vector<EmittedPath> paths;
    const PrintObject&       object() const { return *print.get_object(size_t(0)); }
};

void slice_and_export(Sliced &out, const Workspace &workspace, Model model, const DynamicPrintConfig &settings, const std::string &name)
{
    const DynamicPrintConfig config = observed(settings);
    slice(out.print, model, config);
    const std::string filename = out.print.export_gcode((workspace.path / name).string(), nullptr);
    out.paths = read_extrusions(filename, config);
}

// The width of the sparse infill bead on the export.
double sparse_width(const std::vector<EmittedPath> &paths)
{
    for (const EmittedPath &path : paths)
        if (path.role == erInternalInfill)
            return path.width;
    return 0.;
}

// A 3-step staircase: a 40 x 30 x 6 mm body with 30, 20 and 10 mm deep blocks of 4 mm on it from x = 10, 20 and 30, so that four
// roofs (at 6, 10, 14 and 18 mm) end over the sparse fill of the step beneath.
Model steps()
{
    TriangleMesh body(its_make_cube(40., 30., 6.));
    for (int i = 1; i <= 3; ++i) {
        TriangleMesh block(its_make_cube(40. - 10. * i, 30., 4.));
        block.translate(10. * i, 0., 6. + 4. * (i - 1));
        body.merge(block);
    }
    return single_part("Bridge aware cubic steps", std::move(body));
}

// A fuselage top: a half cylinder of radius 15 mm, 40 mm long along y, on a 30 x 40 x 10 mm base (the convex hull of both).
Model fuselage_top()
{
    std::vector<Vec3f> points;
    for (const float x : { 0.f, 30.f })
        for (const float y : { 0.f, 40.f })
            for (const float z : { 0.f, 10.f })
                points.emplace_back(x, y, z);
    for (int k = 0; k <= 32; ++k) {
        const double theta = M_PI * k / 32.;
        for (const float y : { 0.f, 40.f })
            points.emplace_back(float(15. + 15. * std::cos(theta)), y, float(10. + 15. * std::sin(theta)));
    }
    return single_part("Bridge aware cubic fuselage top", TriangleMesh(its_convex_hull(points)));
}

// The bridge footprint of every layer beyond `limit_mm` of the roof's solid footprint (what the layers above it print, `bridge_layers`
// of them: a thick base lies under the roof's own layer), where it lies over the sparse void of the plane below: the generator's
// growth past the roof's inner edge beyond what the rails allow. Returns the area (mm²) of that overgrowth, summed over the layers.
double bridge_overgrowth_mm2(const std::vector<EmittedPath> &paths, double limit_mm, int bridge_layers)
{
    double total = 0.;
    const std::map<int, EmittedLayer> layers = emitted_layers(paths);
    for (auto it = layers.begin(); it != layers.end() && std::next(it) != layers.end(); ++it) {
        const EmittedLayer &lower = it->second, &upper = std::next(it)->second;
        EmittedLayer bridges, sparse;
        for (const EmittedPath *path : upper)
            if (path->role == erBridgeInfill) bridges.push_back(path);
        for (const EmittedPath *path : lower)
            if (path->role == erInternalInfill) sparse.push_back(path);
        if (bridges.empty() || sparse.empty())
            continue;
        EmittedLayer roof_paths;
        auto above = std::next(it);
        for (int k = 0; k < bridge_layers && std::next(above) != layers.end(); ++k) {
            ++above;
            for (const EmittedPath *path : above->second)
                if (path->role != erInternalInfill) roof_paths.push_back(path);
        }
        const ExPolygons allowed = offset_ex(footprint(roof_paths), float(scale_(limit_mm)), ClipperLib::jtRound, scale_(.001));
        // The sparse void of the plane below, as the observer measures it: inside its outer wall, off what it prints, beside sparse fill.
        ExPolygons domain;
        for (const ExPolygon &wall : footprint(lower, erExternalPerimeter)) domain.emplace_back(wall.contour);
        const ExPolygons near_sparse = offset_ex(footprint(sparse), float(scale_(.01)));
        ExPolygons sparse_void;
        for (ExPolygon &part : diff_ex(domain, footprint(lower)))
            if (!intersection_ex(ExPolygons{ part }, near_sparse).empty()) sparse_void.push_back(std::move(part));
        total += area(intersection_ex(diff_ex(footprint(bridges), allowed), sparse_void)) * SCALING_FACTOR * SCALING_FACTOR;
    }
    return total;
}

} // namespace

// R1: rails, judged by what they are for. At a bridged roof's inner edge the bridge reaches at most d_r + w_b + 0.05 past the edge
// over the void (Cubic's grows further: the control, where it does on the fixture; the roof is what the layers above the bridge
// print); no bridge end over the pattern's void hangs (an end inside the wall overlap is anchored; the control: Cubic hangs a bridge
// end on the rib); the user's wall, bridge, anchor and density settings included. Thick bases (two and three internal bridge
// layers) and the generator bridging a layer above the predicted roof are recorded limitations (docs/bridge-aware-cubic.md).
struct RailCase {
    std::string          name;
    Model              (*model)();
    DynamicPrintConfig   settings;
    double               gap = 5.;
    DynamicPrintConfig (*base)(const char *) = nullptr;
    bool                 cubic_overgrows = false;   // the control: Cubic's bridge grows past the limit on this fixture
    bool                 cubic_hangs = false;       // the control of the hanging-end measure: Cubic hangs a bridge end here
};

TEST_CASE("Bridge aware cubic lays rails along the roof edges the bridges grow past", "[BridgeAwareCubic][R1]")
{
    Workspace workspace;
    const std::vector<RailCase> cases = {
        { "partial roof", partial_roof, {}, 5., nullptr, true },
        { "steps", steps, {}, 5., nullptr, true },
        // Cubic's bridge here reaches a wall within the limit: no control.
        { "fuselage top", fuselage_top },
        // The user's settings: one Arachne wall at 75 % overlap, thick bridges, 1 mm anchors, 3 % density.
        { "fuselage top, the user's settings", fuselage_top,
          with({}, {{"wall_loops", "1"}, {"wall_generator", "arachne"}, {"infill_wall_overlap", "75%"}, {"thick_bridges", "1"},
                    {"sparse_infill_anchor", "1"}, {"sparse_infill_anchor_max", "1"}, {"sparse_infill_density", "3%"}}) },
        { "small cube", small_cube },
        { "large cube", large_cube },
        { "rib", rib, {}, 5., rib_config, true, true },
        { "small cube, two internal bridge layers", small_cube, with({}, {{"internal_bridge_layers", "2"}}) },
        { "small cube, three internal bridge layers", small_cube, with({}, {{"internal_bridge_layers", "3"}}) },
    };
    for (const RailCase &c : cases) {
        INFO(c.name);
        DynamicPrintConfig settings = c.base != nullptr ? c.base("bridgeawarecubic") : fixture_config(c.gap, "bridgeawarecubic");
        settings.apply(c.settings, true);
        DynamicPrintConfig control_settings = c.base != nullptr ? c.base("cubic") : fixture_config(c.gap, "cubic");
        control_settings.apply(c.settings, true);
        Sliced sliced;
        slice_and_export(sliced, workspace, c.model(), settings, c.name + "-r1.gcode");
        const PrintObject &object = sliced.object();
        const std::vector<EmittedPath> control = exported_extrusions(workspace, c.model(), control_settings, c.name + "-r1-cubic.gcode");
        const BACObserver::BridgeEnds ends = count_bridge_ends(sliced.paths);
        REQUIRE(ends.paths > 0);
        std::ostringstream hanging;
        for (const auto &[z, at] : ends.where) hanging << " z " << z << " (" << at.x() << ", " << at.y() << ")";
        INFO("hanging ends:" << hanging.str());
        CHECK(ends.hanging == 0);
        if (c.cubic_hangs)
            CHECK(count_bridge_ends(control).hanging > 0);
        // d_r = s_b + w/2 + 0.05 with the thick bridge flow's spacing s_b and the sparse bead w; the bridge footprint may reach w_b past
        // the rail's centre line and the touching measure further.
        const double w = sparse_width(sliced.paths);
        REQUIRE(w > 0.3);
        const Flow bridge_flow = object.layers().front()->regions().front()->bridging_flow(frSolidInfill, true);
        const double d_r = bridge_flow.spacing() + 0.5 * w + 0.05, limit = d_r + bridge_flow.width() + 0.05;
        const int B = std::max(1, std::min(3, int(object.printing_region(0).config().internal_bridge_layers.value)));
        CHECK(bridge_overgrowth_mm2(sliced.paths, limit, B) <= 0.05);
        if (c.cubic_overgrows)
            CHECK(bridge_overgrowth_mm2(control, limit, B) > 0.05);
    }
}

// W1: a seam between two lattices (3.5a). Each half of the modifier holds its own gap on the export (K1 with the seam closed): no
// opening spans the seam, and every opening and run is within the gap of its side. K2 across the seam, of the modifier and of a
// floating modifier, is K2seam's: the lines that end at the seam are held only by its wall.
TEST_CASE("Bridge aware cubic holds each region's gap across a seam", "[BridgeAwareCubic][W1]")
{
    Workspace workspace;
    const std::vector<EmittedPath> paths = exported_extrusions(workspace, modifier(), modifier_config("bridgeawarecubic"), "w1-modifier.gcode");
    const std::vector<PlaneMeasure> planes = measure_planes(paths, 0.4, 8.);
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
