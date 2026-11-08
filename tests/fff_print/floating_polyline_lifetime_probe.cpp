// Standalone sanitizer probe: compile the production translation unit here so
// its private conversion function and vector mutations are instrumented too.
// See scripts/check_floating_polyline_lifetime.py; this is not linked into the app.
#include "../../src/libslic3r/Fill/FillFloatingConcentric.cpp"
#include <iostream>

int main()
{
    using namespace Slic3r;
    for (bool trailing_nonfloating : {false, true}) {
        FloatingThickPolyline polyline;
        polyline.points = {Point(0, 0), Point(10000000, 0)};
        polyline.width = {200000., 600000.};
        polyline.is_floating = {true, true};
        if (trailing_nonfloating) {
            polyline.points.emplace_back(20000000, 0);
            polyline.width.insert(polyline.width.end(), {600000., 600000.});
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
    std::cout << "Floating segment growth and following-segment flags passed.\n";
}
