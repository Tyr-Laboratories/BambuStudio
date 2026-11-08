// Standalone sanitizer probe: compile the production translation unit here so
// its private conversion function and vector mutations are instrumented too.
// See scripts/check_floating_polyline_lifetime.py; this is not linked into the app.
#include "../../src/libslic3r/EdgeGrid.hpp"
#include <sanitizer/asan_interface.h>
#include <iostream>
#include <cstdlib>

// The native archive can supply uninstrumented weak Eigen inline functions.
// Check borrowed storage explicitly at the production query boundary so that
// sanitizer coverage does not depend on which inline definition the linker uses.
namespace Slic3r::EdgeGrid {
class LifetimeCheckedGrid : public Grid {
public:
    bool has_intersecting_edges() const {
        for (const auto &contour : contours())
            for (const Point *point = contour.begin(); point != contour.end(); ++point)
                if (__asan_address_is_poisoned(point)) {
                    std::cerr << "Floating self-intersection grid references expired point storage.\n";
                    std::abort();
                }
        return Grid::has_intersecting_edges();
    }
};
}
#define Grid LifetimeCheckedGrid
#include "../../src/libslic3r/Fill/FillFloatingConcentric.cpp"
#undef Grid

int main()
{
    using namespace Slic3r;
    for (double trailing_width : {0., 600000., 800000.}) {
        const bool trailing_nonfloating = trailing_width > 0;
        FloatingThickPolyline polyline;
        polyline.points = {Point(0, 0), Point(10000000, 0)};
        polyline.width = {200000., 600000.};
        polyline.is_floating = {true, true};
        if (trailing_nonfloating) {
            polyline.points.emplace_back(20000000, 0);
            polyline.width.insert(polyline.width.end(), {trailing_width, trailing_width});
            polyline.is_floating.push_back(false);
        }
        auto paths = floating_thick_polyline_to_extrusion_paths(
            polyline, erInternalInfill, Flow(0.4f, 0.2f, 0.4f), 100000.f);
        if (paths.empty()) return 1;
        double total_length = 0;
        for (const auto& path : paths) {
            total_length += path.polyline.length();
            const bool expected_floating = path.polyline.points.front().x() < 10000000;
            if ((path.get_customize_flag() == CustomizeFlag::cfFloatingVerticalShell) != expected_floating)
                return 2;
        }
        if (std::abs(total_length - (trailing_nonfloating ? 20000000. : 10000000.)) > 1.) return 3;
    }
    ThickPolyline source;
    source.points = {Point(0, 0), Point(4000000, 0)};
    source.width = {300000., 500000.};
    const ZPath missing_before = {{0, 0, -1}, {2000000, 0, -1}, {4000000, 0, 0}};
    const ZPath missing_after = {{0, 0, 0}, {2000000, 0, -1}, {4000000, 0, -1}};
    const ZPath missing_both = {{0, 0, -1}, {2000000, 0, -1}, {4000000, 0, -1}};
    if (std::abs(interpolate_width(missing_before, source, 2, 600000, 0) - 600000.) > 1.) return 4;
    if (std::abs(interpolate_width(missing_before, source, 2, 600000, 1) - 450000.) > 1.) return 5;
    if (std::abs(interpolate_width(missing_after, source, 2, 600000, 1) - 450000.) > 1.) return 6;
    if (std::abs(interpolate_width(missing_after, source, 2, 600000, 2) - 600000.) > 1.) return 7;
    if (std::abs(interpolate_width(missing_both, source, 2, 600000, 1) - 600000.) > 1.) return 8;
    const ZPath single_unknown = {{0, 0, -1}};
    const ZPath single_known = {{0, 0, 0}};
    if (interpolate_width(single_unknown, source, 2, 600000, 0) != 600000.) return 9;
    if (interpolate_width(single_known, source, 2, 600000, 0) != 300000.) return 10;
    const ZPath both_known = {{0, 0, 0}, {2000000, 0, -1}, {4000000, 0, 1}};
    if (interpolate_width(both_known, source, 2, 600000, 1) != 400000.) return 13;
    FloatingPolyline loop;
    loop.points = {Point(0, 0), Point(1000000, 0), Point(0, 1000000), Point(0, 0)};
    loop.is_floating = {false, true, false, false};
    const auto rebased = loop.rebase_at(1);
    if (rebased.is_floating.size() != rebased.points.size() ||
        rebased.is_floating != std::vector<bool>({true, false, false, true})) return 11;
    FloatingThickPolyline thick_loop;
    thick_loop.points = loop.points;
    thick_loop.width = {400000., 400000., 400000., 400000., 400000., 400000.};
    thick_loop.is_floating = loop.is_floating;
    const auto thick_rebased = thick_loop.rebase_at(1);
    if (thick_rebased.is_floating.size() != thick_rebased.points.size() ||
        thick_rebased.is_floating != std::vector<bool>({true, false, false, true})) return 12;
    struct FillerProbe : FillFloatingConcentric { using FillFloatingConcentric::resplit_order_loops; using FillConcentric::print_object_config; };
    PrintObjectConfig object_config;
    object_config.detect_floating_vertical_shell.value = true;
    FillerProbe filler;
    filler.print_object_config = &object_config;
    Arachne::ExtrusionLine open_path(0, true, false);
    for(const auto &point : Points{Point(0,0),Point(100000,0),Point(100000,100000),Point(0,100000)})
        open_path.junctions.emplace_back(point,40000,0);
    const auto checked = filler.resplit_order_loops(Point(0,0),{&open_path},{},{},40000);
    if(checked.size()!=1 || checked.front().points.size()!=4) return 14;
    const ExPolygons floating_area{ExPolygon(Polygon(Points{Point(-100000,-100000),Point(200000,-100000),Point(200000,200000),Point(-100000,200000)}))};
    const auto floating = filler.resplit_order_loops(Point(0,0),{&open_path},floating_area,{},40000);
    if(floating.size()!=1 || !std::all_of(floating.front().is_floating.begin(),floating.front().is_floating.end(),[](bool flag){return flag;})) return 15;
    Arachne::ExtrusionLine crossing_path(0,true,false);
    for(const auto &point : Points{Point(0,0),Point(100000,100000),Point(0,100000),Point(100000,0)})
        crossing_path.junctions.emplace_back(point,40000,0);
    const auto crossing = filler.resplit_order_loops(Point(0,0),{&crossing_path},floating_area,{},40000);
    if(crossing.size()!=1 || std::any_of(crossing.front().is_floating.begin(),crossing.front().is_floating.end(),[](bool flag){return flag;})) return 16;
    std::cout << "Floating flags, interpolation, grid lifetime and intersection classifications passed.\n";
}
