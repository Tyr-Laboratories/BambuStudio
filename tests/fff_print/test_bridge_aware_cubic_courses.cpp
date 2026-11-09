#include <catch2/catch.hpp>

#include "bridge_aware_cubic_fixtures.hpp"
#include "bridge_aware_cubic_observer.hpp"

#include "libslic3r/Fill/BridgeAwareCubicPlan.hpp"

#include <algorithm>
#include <sstream>
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

// Every layer and region of the plan has no course.
bool no_courses(const ObjectPlan &plan, const PrintObject &object)
{
    for (size_t l = 0; l < object.layers().size(); ++l)
        for (size_t r = 0; r < object.num_printing_regions(); ++r)
            if (!plan.courses_at(l, r).empty())
                return false;
    return true;
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

// The rail-like beads of a layer: sparse segments at least a bead long that run off the lattice directions, lie farther than a bead
// from the walls' beads and off the lines of the Cubic control's sparse segments of that layer (the connector's hooks along an
// island edge, which Cubic lays too).
std::vector<Line> rail_like_beads(const EmittedLayer &layer, const std::vector<Line> &cubic_segments, double w, const std::vector<double> &directions)
{
    ExPolygons walls = footprint(layer, erExternalPerimeter);
    append(walls, footprint(layer, erPerimeter));
    const ExPolygons wall_band = offset_ex(union_ex(walls), float(scale_(w)), ClipperLib::jtRound, scale_(.001));
    std::vector<Line> out;
    for (const EmittedPath *path : layer) {
        if (path->role != erInternalInfill) continue;
        for (size_t i = 1; i < path->path.points.size(); ++i) {
            const Point &a = path->path.points[i - 1], &b = path->path.points[i];
            if (unscale<double>((b - a).cast<double>().norm()) < w - 1e-4 || along_lattice(a, b, directions) || on_control_line(a, b, cubic_segments)) continue;
            const Point mid((a.x() + b.x()) / 2, (a.y() + b.y()) / 2);
            if (std::none_of(wall_band.begin(), wall_band.end(), [&mid](const ExPolygon &e) { return e.contains(mid); }))
                out.emplace_back(a, b);
        }
    }
    return out;
}

// The inner edges of a layer's bridges: the outline of their footprint (closed by half a bridge bead: neighbouring beads leave
// slivers between them) over the sparse area of the layer below (inside its outer wall, off the beads of its other roles),
// farther than a bead from its walls.
Polylines bridge_inner_edges(const EmittedLayer &upper, const EmittedLayer &lower, double w)
{
    EmittedLayer bridges, other;
    double half_bead = 0.;
    for (const EmittedPath *path : upper)
        if (path->role == erBridgeInfill) {
            bridges.push_back(path);
            half_bead = std::max(half_bead, 0.5 * path->width);
        }
    for (const EmittedPath *path : lower)
        if (path->role != erInternalInfill) other.push_back(path);
    if (bridges.empty())
        return {};
    ExPolygons domain;
    for (const ExPolygon &wall : footprint(lower, erExternalPerimeter)) domain.emplace_back(wall.contour);
    ExPolygons walls = footprint(lower, erExternalPerimeter);
    append(walls, footprint(lower, erPerimeter));
    const ExPolygons sparse_area = diff_ex(domain, offset_ex(union_ex(walls), float(scale_(w)), ClipperLib::jtRound, scale_(.001)));
    const ExPolygons closed = offset_ex(offset_ex(footprint(bridges), float(scale_(half_bead)), ClipperLib::jtRound, scale_(.001)),
                                        -float(scale_(half_bead)), ClipperLib::jtRound, scale_(.001));
    return intersection_pl(to_polylines(closed), diff_ex(sparse_area, footprint(other)));
}

} // namespace

// R1: rails. At a bridged roof's inner edge the bridge reaches at most d_r + w_b + 0.05 past the edge over the void (Cubic's grows
// further: the control, where it does on the fixture; the roof is what the layers above the bridge print); no bridge end over the
// pattern's void hangs (an end inside the wall overlap is anchored); rail beads lie along the inner edges of every bridge layer
// whose roof edges run over the void, and no rail-like bead lies anywhere where the roof edges are at the walls; the user's wall,
// bridge, anchor and density settings included. Thick bases (two and three internal bridge layers) and the generator bridging a layer above the predicted roof are
// recorded limitations (docs/bridge-aware-cubic.md).
struct RailCase {
    std::string          name;
    Model              (*model)();
    DynamicPrintConfig   settings;
    double               gap = 5.;
    DynamicPrintConfig (*base)(const char *) = nullptr;
    bool                 edge_over_void = false;    // a roof edge over the pattern's void: a rail is expected
    double               direction = 45.;
    bool                 cubic_overgrows = false;   // the control: Cubic's bridge grows past the limit on this fixture
    bool                 rails_per_layer = true;    // every bridge layer with inner edges over the void has its rails (the
                                                    // fuselage top's apex is bridged a layer above the prediction: recorded)
};

void check_rails(Workspace &workspace, const std::vector<RailCase> &cases);

TEST_CASE("Bridge aware cubic lays rails along the roof edges the bridges grow past", "[BridgeAwareCubic][R1]")
{
    Workspace workspace;
    check_rails(workspace, {
        { "partial roof", partial_roof, {}, 5., nullptr, true, 45., true },
        { "steps", steps, {}, 5., nullptr, true, 45., true },
        // Cubic's bridge here reaches a wall within the limit: no control.
        { "fuselage top", fuselage_top, {}, 5., nullptr, true, 45., false, false },
        // The user's settings: one Arachne wall at 75 % overlap, thick bridges, 1 mm anchors, 3 % density.
        { "fuselage top, the user's settings", fuselage_top,
          with({}, {{"wall_loops", "1"}, {"wall_generator", "arachne"}, {"infill_wall_overlap", "75%"}, {"thick_bridges", "1"},
                    {"sparse_infill_anchor", "1"}, {"sparse_infill_anchor_max", "1"}, {"sparse_infill_density", "3%"}}), 5., nullptr, true, 45., false, false },
        { "small cube", small_cube, {}, 5., nullptr, false },
        { "large cube", large_cube, {}, 5., nullptr, false },
        { "rib", rib, {}, 5., rib_config, true, 17., true },
        { "small cube, two internal bridge layers", small_cube, with({}, {{"internal_bridge_layers", "2"}}), 5., nullptr, false },
        { "small cube, three internal bridge layers", small_cube, with({}, {{"internal_bridge_layers", "3"}}), 5., nullptr, false },
    });
}

void check_rails(Workspace &workspace, const std::vector<RailCase> &cases)
{
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
        // d_r = s_b + w/2 + 0.05 with the thick bridge flow's spacing s_b and the sparse bead w; the bridge footprint may reach w_b past
        // the rail's centre line and the touching measure further.
        const PrintRegion &region = object.printing_region(0);
        const double w = sparse_width(sliced.paths);
        REQUIRE(w > 0.3);
        const Flow bridge_flow = object.layers().front()->regions().front()->bridging_flow(frSolidInfill, true);
        const double d_r = bridge_flow.spacing() + 0.5 * w + 0.05, limit = d_r + bridge_flow.width() + 0.05;
        const int B = std::max(1, std::min(3, int(region.config().internal_bridge_layers.value)));
        CHECK(bridge_overgrowth_mm2(sliced.paths, limit, B) <= 0.05);
        if (c.cubic_overgrows)
            CHECK(bridge_overgrowth_mm2(control, limit, B) > 0.05);
        // The rails, from the export: rail-like beads along the inner edges (longer than the gap) of the bridges of the layers
        // above them. A fixture whose roofs have edges over the void has them; one whose roofs end at the walls has none.
        std::vector<double> directions;
        for (int k = 0; k < 3; ++k)
            directions.push_back(std::fmod(c.direction + 60. * k + 360., 180.));
        const std::map<int, EmittedLayer> layers = emitted_layers(sliced.paths), control_layers = emitted_layers(control);
        double rail_length = 0.;
        size_t edge_layers = 0, railed_layers = 0;
        std::ostringstream unrailed;
        for (auto it = layers.begin(); it != layers.end() && std::next(it) != layers.end(); ++it) {
            const Polylines edges = bridge_inner_edges(std::next(it)->second, it->second, w);
            double edge_length = 0.;
            for (const Polyline &edge : edges) edge_length += unscale<double>(edge.length());
            if (edge_length <= c.gap)
                continue;
            ++edge_layers;
            const ExPolygons near = union_ex(offset(edges, float(scale_(limit)), ClipperLib::jtRound, scale_(.001), ClipperLib::etOpenRound));
            double here = 0.;
            for (auto below = it;; --below) {
                for (const Line &bead : rail_like_beads(below->second, control_sparse_segments(control_layers, below->first), w, directions)) {
                    const Point mid = (bead.a + bead.b) / 2;
                    if (std::any_of(near.begin(), near.end(), [&mid](const ExPolygon &e) { return e.contains(mid); }))
                        here += unscale<double>(bead.length());
                }
                if (below == layers.begin() || std::distance(below, it) + 1 >= B)
                    break;
            }
            rail_length += here;
            if (here > w)
                ++railed_layers;
            else
                unrailed << " " << std::next(it)->first / 1000.;
        }
        if (c.edge_over_void) {
            CHECK(rail_length > w);
            if (c.rails_per_layer) {
                INFO("bridge layers with inner edges over the void and no rail below:" << unrailed.str());
                CHECK(railed_layers == edge_layers);
            }
        } else {
            // The roofs end at the walls: no rail-like bead on any layer.
            double stray = 0.;
            for (const auto &[key, layer] : layers)
                for (const Line &bead : rail_like_beads(layer, control_sparse_segments(control_layers, key), w, directions))
                    stray += unscale<double>(bead.length());
            CHECK(stray == 0.);
        }
    }
}

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
            [z_first](const Vec2d &at, double z) { return (z >= z_first - 1e-6 && at.x() >= SEAM_X ? 3. : 8.) - 0.1; }, std::vector<double>{ 45., 17. });
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
