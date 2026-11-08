// Standalone sanitizer probe: compile the production translation unit here so
// its private conversion function and vector mutations are instrumented too.
// See scripts/check_floating_polyline_lifetime.py; this is not linked into the app.
#include "../../src/libslic3r/Fill/FillFloatingConcentric.cpp"
#include <iostream>

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
    std::cout << "Floating segment flags and boundary interpolation passed.\n";
}
