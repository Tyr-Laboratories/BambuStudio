#include <catch2/catch.hpp>

#include "libslic3r/Arachne/utils/ExtrusionLine.hpp"
#include "libslic3r/Arachne/WallToolPaths.hpp"
#include "libslic3r/OverhangDetector.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/TriangleSelector.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Algorithm/LineSegmentation/LineSegmentation.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <iomanip>

using namespace Slic3r;

namespace {
ZPoint junction(double x, double y, double width)
{
    return ZPoint(scaled<coord_t>(x), scaled<coord_t>(y), scaled<coord_t>(width));
}

ZPaths window(double left, double right)
{
    return {{junction(left, -2., 0.), junction(right, -2., 0.),
             junction(right, 2., 0.), junction(left, 2., 0.)}};
}

double planar_length(const ZPaths &paths)
{
    double length = 0.;
    for (const auto &path : paths)
        for (size_t i = 1; i < path.size(); ++i)
            length += (path[i].head<2>() - path[i - 1].head<2>()).cast<double>().norm();
    return length;
}

struct WallToolPathsAccess : Arachne::WallToolPaths {
    using Arachne::WallToolPaths::removeSmallLines;
};

Arachne::ExtrusionLine odd_line(const Points &points, coord_t width = 40000)
{
    Arachne::ExtrusionLine line(0, true, false);
    for (const Point &point : points)
        line.emplace_back(point, width, 0);
    return line;
}
}

TEST_CASE("Overhang boundaries interpolate Arachne bead width in the layer plane", "[Overhang][Arachne]")
{
    const double length = GENERATE(0.05, 0.2, 1.);
    const bool falling = GENERATE(false, true);
    const bool reversed = GENERATE(false, true);
    const auto operation = GENERATE(ClipperLib_Z::ctIntersection, ClipperLib_Z::ctDifference);
    CAPTURE(length, falling, reversed, operation);

    ZPath subject{junction(0., 0., falling ? 0.5 : 0.4),
                  junction(length, 0., falling ? 0.4 : 0.5)};
    if (reversed)
        std::reverse(subject.begin(), subject.end());

    const auto clipped = clip_extrusion(subject, window(length / 2., 2.), operation);
    REQUIRE(clipped.size() == 1);
    REQUIRE(clipped.front().size() == 2);
    bool found_boundary = false;
    for (const auto &point : clipped.front()) {
        // Z stores bead spacing, not a third spatial coordinate. At the
        // halfway point of the XY segment, the spacing must also be halfway.
        if (std::abs(point.x() - scaled<coord_t>(length / 2.)) <= 1) {
            CHECK(std::abs(point.z() - scaled<coord_t>(0.45)) <= 2);
            found_boundary = true;
        }
        CHECK(point.y() == 0);
        CHECK(point.z() >= scaled<coord_t>(0.4) - 1);
        CHECK(point.z() <= scaled<coord_t>(0.5) + 1);
    }
    REQUIRE(found_boundary);
}

TEST_CASE("Curved overhang splitting preserves centerlines and bounded extrusion widths", "[Overhang][Arachne]")
{
    const bool reversed = GENERATE(false, true);
    ZPath subject{junction(-0.2, 0., 0.4), junction(0., 0.05, 0.45), junction(0.2, 0., 0.5)};
    if (reversed)
        std::reverse(subject.begin(), subject.end());
    const auto clipping_window = window(-0.1, 0.1);
    const auto supported = clip_extrusion(subject, clipping_window, ClipperLib_Z::ctIntersection);
    const auto unsupported = clip_extrusion(subject, clipping_window, ClipperLib_Z::ctDifference);
    REQUIRE(supported.size() == 1);
    REQUIRE(unsupported.size() == 2);
    CHECK(std::abs(planar_length(supported) + planar_length(unsupported) - planar_length({subject})) <= 4.);

    size_t cut_endpoints = 0;
    for (const auto &paths : {supported, unsupported})
        for (const auto &path : paths)
            for (const auto &point : path) {
                if (std::abs(point.x() - scaled<coord_t>(-0.1)) <= 1) {
                    CHECK(std::abs(point.z() - scaled<coord_t>(0.425)) <= 2);
                    CHECK(std::abs(point.y() - scaled<coord_t>(0.025)) <= 2);
                    ++cut_endpoints;
                } else if (std::abs(point.x() - scaled<coord_t>(0.1)) <= 1) {
                    CHECK(std::abs(point.z() - scaled<coord_t>(0.475)) <= 2);
                    CHECK(std::abs(point.y() - scaled<coord_t>(0.025)) <= 2);
                    ++cut_endpoints;
                }
                CHECK(point.z() >= scaled<coord_t>(0.4) - 1);
                CHECK(point.z() <= scaled<coord_t>(0.5) + 1);
            }
    REQUIRE(cut_endpoints == 4);

    // Exercise the conversion used by the perimeter generator, since corrupt
    // intersection widths otherwise create extra subdivisions and wide beads.
    const Flow flow = Flow::new_from_config_width(frPerimeter, ConfigOptionFloatOrPercent(0.45, false), 0.4, 0.2);
    ExtrusionPaths extrusion_paths;
    extrusion_paths_append(extrusion_paths, supported, erExternalPerimeter, flow);
    extrusion_paths_append(extrusion_paths, unsupported, erOverhangPerimeter, flow);
    REQUIRE_FALSE(extrusion_paths.empty());
    double extrusion_length = 0.;
    for (const auto &path : extrusion_paths) {
        CHECK(path.length() > 0.);
        CHECK(path.mm3_per_mm > 0.);
        CHECK(path.width <= 0.5 + flow.height() * (1. - 0.25 * PI) + 1e-4);
        extrusion_length += path.length();
    }
    CHECK(std::abs(extrusion_length - planar_length({subject})) <= 10.);
}

TEST_CASE("Overhang clipping retains constant widths and drops zero-length input", "[Overhang][Arachne]")
{
    const auto operation = GENERATE(ClipperLib_Z::ctIntersection, ClipperLib_Z::ctDifference);
    const ZPath subject{junction(-0.1, -0.1, 0.42), junction(0.1, 0.1, 0.42)};
    const auto clipped = clip_extrusion(subject, window(0., 2.), operation);
    REQUIRE(clipped.size() == 1);
    for (const auto &point : clipped.front())
        CHECK(point.z() == scaled<coord_t>(0.42));
    const ZPoint point = junction(0., 0., 0.42);
    CHECK(clip_extrusion({point, point}, window(-1., 1.), operation).empty());
}

TEST_CASE("Arachne removes short open odd walls using their actual extrusion length", "[Overhang][Arachne]")
{
    // A 0.4 mm bead uses the existing half-width threshold of 0.2 mm.
    // Integer coordinates keep the exact-boundary cases independent of
    // floating-point scaling and rounding.
    const coord_t length = GENERATE(5000, 10000, 15000, 19999, 20000, 20001);
    const bool reversed = GENERATE(false, true);
    auto line = odd_line({Point(0, 0), Point(length, 0)});
    if (reversed)
        line.reverse();
    std::vector<Arachne::VariableWidthLines> paths{{line}};
    WallToolPathsAccess::removeSmallLines(paths);
    CAPTURE(length, reversed);
    CHECK(paths.front().empty() == (length < 20000));
    if (!paths.front().empty())
        CHECK(paths.front().front().junctions == line.junctions);
}

TEST_CASE("Arachne short-wall filtering handles curves and degenerate open lines", "[Overhang][Arachne]")
{
    auto short_curve = odd_line({Point(0, 0), Point(0, 6000), Point(8000, 6000)});
    auto threshold_curve = odd_line({Point(0, 0), Point(0, 12000), Point(8000, 12000)});
    auto long_curve = odd_line({Point(0, 0), Point(0, 12000), Point(12000, 12000)});
    std::vector<Arachne::VariableWidthLines> paths{{
        short_curve, threshold_curve, long_curve, odd_line({}),
        odd_line({Point(0, 0)}), odd_line({Point(0, 0), Point(0, 0)})}};
    WallToolPathsAccess::removeSmallLines(paths);
    REQUIRE(paths.front().size() == 2);
    for (const auto &line : paths.front())
        CHECK((line.junctions == threshold_curve.junctions || line.junctions == long_curve.junctions));
}

TEST_CASE("Arachne preserves closed and paired walls during short-wall filtering", "[Overhang][Arachne]")
{
    auto closed = odd_line({Point(0, 0), Point(1000, 0), Point(1000, 1000), Point(0, 0)});
    closed.is_closed = true;
    auto paired = odd_line({Point(0, 0), Point(1000, 0)});
    paired.is_odd = false;
    std::vector<Arachne::VariableWidthLines> paths{{closed, paired}};
    WallToolPathsAccess::removeSmallLines(paths);
    REQUIRE(paths.front().size() == 2);
    CHECK(paths.front()[0].junctions == closed.junctions);
    CHECK(paths.front()[1].junctions == paired.junctions);
}

namespace {
struct SlopeWallLayer {
    double z;
    std::vector<double> across_x, across_y;
    std::vector<std::string> fill_paths;
};

std::vector<SlopeWallLayer> slice_sloped_box(bool arachne, double slope, double threshold,
                                           int wall_count = 1, bool single_top = false,
                                           bool single_first = false, bool variable_height = false,
                                           int painted_count = -1)
{
    Model model;
    auto *object = model.add_object();
    TriangleMesh mesh(its_make_cube(30., 20., 6.));
    for (auto &vertex : mesh.its.vertices)
        vertex.x() += slope * vertex.z();
    auto *volume=object->add_volume(mesh);
    if (painted_count>=0) {
        TriangleSelector selector(volume->mesh());
        for (size_t i=0;i<mesh.its.indices.size();++i)
            selector.set_facet(int(i),EnforcerBlockerType::ENFORCER);
        volume->wall_painting_facets.set(selector);
    }
    object->add_instance()->set_offset(Vec3d(80., 80., 0.));
    if (variable_height)
        object->layer_height_profile.set(std::vector<coordf_t>{0., 0.2, 2., 0.2, 3., 0.1, 5., 0.1, 6., 0.25});

    DynamicPrintConfig config;
    config.apply(static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    config.set_deserialize_strict({{"layer_height", "0.2"}, {"initial_layer_print_height", "0.2"},
        {"wall_generator", arachne ? "arachne" : "classic"}, {"wall_loops", std::to_string(wall_count)},
        {"extra_wall_slope_angle", std::to_string(threshold)}, {"infill_wall_overlap", "75%"},
        {"top_one_wall_type", single_top ? "all top" : "not apply"},
        {"only_one_wall_first_layer", single_first ? "1" : "0"}, {"brim_type", "no_brim"}});
    if (painted_count>=0) config.set_deserialize_strict({{"painted_wall_loops",std::to_string(painted_count)}});
    Print print;
    print.set_status_silent();
    print.auto_assign_extruders(object);
    print.apply(model, config);
    REQUIRE(print.objects().size() == 1);
    auto *printed = print.get_object(size_t(0));
    print.process();
    std::vector<SlopeWallLayer> result;
    for (const Layer *layer : printed->layers()) {
        result.push_back({layer->print_z, {}, {}, {}});
        for (const auto *region : layer->regions()) {
            const auto flattened = region->perimeters.flatten();
            for (const auto *entity : flattened.entities) {
                const auto points = entity->as_polyline().points;
                for (size_t i = 1; i < points.size(); ++i) {
                    const auto &a = points[i - 1], &b = points[i];
                    for (int axis = 0; axis < 2; ++axis)
                        if ((a[1 - axis] <= 0 && b[1 - axis] > 0) || (b[1 - axis] <= 0 && a[1 - axis] > 0)) {
                            const double along = -double(a[1 - axis]) / double(b[1 - axis] - a[1 - axis]);
                            const double position = unscaled<double>(a[axis] + along * (b[axis] - a[axis]));
                            (axis == 0 ? result.back().across_x : result.back().across_y).push_back(position);
                        }
                }
            }
            const auto fills=region->fills.flatten();
            for (const auto *entity:fills.entities) {
                std::ostringstream signature;
                signature << int(entity->role());
                for (const auto &point:entity->as_polyline().points)
                    signature << ':' << point.x() << ',' << point.y();
                result.back().fill_paths.push_back(signature.str());
            }
        }
        std::sort(result.back().across_x.begin(), result.back().across_x.end());
        std::sort(result.back().across_y.begin(), result.back().across_y.end());
    }
    return result;
}
}

TEST_CASE("Slope threshold adds an inner wall on both leaning faces only", "[SlopeWalls][Perimeters]")
{
    const bool arachne = GENERATE(false, true);
    const bool variable_height = GENERATE(false, true);
    const auto layers = slice_sloped_box(arachne, 0.7, 30., 1, false, false, variable_height);
    REQUIRE(layers.size() > 10);
    for (const auto &layer : layers) {
        CAPTURE(arachne, variable_height, layer.z);
        // The two X faces lean ~35 degrees from vertical, one inward and one
        // outward. The Y faces stay vertical and must retain just one wall.
        REQUIRE(layer.across_x.size() == 4);
        CHECK(layer.across_y.size() == 2);
    }
}

TEST_CASE("Disabled or unmet slope thresholds preserve ordinary wall counts", "[SlopeWalls][Perimeters]")
{
    const bool arachne = GENERATE(false, true);
    const int wall_count = GENERATE(0, 1, 2, 3);
    const auto disabled = slice_sloped_box(arachne, 0.7, 0., wall_count);
    const auto unmet = slice_sloped_box(arachne, 0.7, 40., wall_count);
    const auto enabled = slice_sloped_box(arachne, 0.7, 30., wall_count);
    REQUIRE(disabled.size() == unmet.size());
    REQUIRE(disabled.size() == enabled.size());
    for (size_t i = 0; i < disabled.size(); ++i) {
        CAPTURE(arachne, wall_count, i);
        CHECK(disabled[i].across_x.size() == size_t(2 * wall_count));
        CHECK(disabled[i].across_y.size() == size_t(2 * wall_count));
        CHECK(unmet[i].across_x == disabled[i].across_x);
        CHECK(unmet[i].across_y == disabled[i].across_y);
        CHECK(unmet[i].fill_paths == disabled[i].fill_paths);
        if (wall_count != 1) {
            CHECK(enabled[i].across_x == disabled[i].across_x);
            CHECK(enabled[i].across_y == disabled[i].across_y);
            CHECK(enabled[i].fill_paths == disabled[i].fill_paths);
        }
    }
}

TEST_CASE("Slope reinforcement takes precedence locally over one-wall top and first layers", "[SlopeWalls][Perimeters]")
{
    const bool arachne = GENERATE(false, true);
    const auto layers = slice_sloped_box(arachne, 0.7, 30., 1, true, true);
    REQUIRE(layers.size() > 10);
    for (const auto &layer : layers) {
        CAPTURE(arachne, layer.z);
        CHECK(layer.across_x.size() == 4);
        CHECK(layer.across_y.size() == 2);
    }
}

TEST_CASE("Slope reinforcement uses the effective painted wall count", "[SlopeWalls][WallPainting]")
{
    const bool arachne=GENERATE(false,true);
    const int count=GENERATE(0,1,2,3);
    const auto layers=slice_sloped_box(arachne,0.7,30.,1,false,false,false,count);
    for (const auto &layer:layers) {
        CAPTURE(arachne,count,layer.z);
        CHECK(layer.across_x.size()==size_t(2*(count==1?2:count)));
        CHECK(layer.across_y.size()==size_t(2*count));
    }
}

TEST_CASE("Slope walls leave existing multi-wall top and first-layer reductions unchanged", "[SlopeWalls][Perimeters]")
{
    const bool arachne=GENERATE(false,true);
    const int count=GENERATE(2,3);
    const auto before=slice_sloped_box(arachne,0.7,0.,count,true,true);
    const auto after=slice_sloped_box(arachne,0.7,30.,count,true,true);
    REQUIRE(before.size()==after.size());
    for(size_t i=0;i<before.size();++i) {
        CAPTURE(arachne,count,i);
        CHECK(before[i].across_x==after[i].across_x);
        CHECK(before[i].across_y==after[i].across_y);
        CHECK(before[i].fill_paths==after[i].fill_paths);
    }
}

TEST_CASE("Slope boundary labels retain transformed inclination and nearest ownership", "[SlopeWalls][Slice]")
{
    const double scale_z=GENERATE(0.5,1.,2.);
    const bool mirrored=GENERATE(false,true);
    Model model;
    auto *object=model.add_object();
    TriangleMesh mesh(its_make_cube(30.,20.,6.));
    for (auto &v:mesh.its.vertices) v.x()+=0.7*v.z();
    object->add_volume(mesh);
    auto *instance=object->add_instance();
    instance->set_rotation(Vec3d(0.,0.,0.37));
    instance->set_mirror(Vec3d(mirrored?-1.:1.,1.,1.));
    instance->set_scaling_factor(Vec3d(1.,1.,scale_z));
    object->ensure_on_bed();
    DynamicPrintConfig config;
    config.apply(static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    config.set_deserialize_strict({{"extra_wall_slope_angle","30"},{"layer_height","0.2"},{"wall_loops","1"}});
    Print print;print.set_status_silent();print.auto_assign_extruders(object);print.apply(model,config);
    auto *printed=print.get_object(size_t(0));printed->slice();
    const auto labels=WallPainting::slice_paint(*printed,[]{});
    REQUIRE(labels.size()>5);
    const double expected=std::atan2(0.7,scale_z)*180./PI;
    for (size_t i=0;i<labels.size();++i) {
        size_t inclined=0,vertical=0;
        for (const auto &line:labels[i]) {
            CAPTURE(scale_z,mirrored,i,line.slope_angle);
            if (line.slope_angle<1.e-5) ++vertical;
            else {CHECK(line.slope_angle==Approx(expected).margin(1.e-4));++inclined;}
        }
        REQUIRE(inclined>0);
        REQUIRE(vertical>0);
    }
}

TEST_CASE("Slope modifier thresholds and existing extra walls are resolved before reinforcement", "[SlopeWalls][WallPainting]")
{
    const Polygon square({Point(0,0),Point(4000000,0),Point(4000000,4000000),Point(0,4000000)});
    WallPainting::LayerPaint labels;
    for(size_t i=0;i<4;++i) labels.push_back({Line(square[i],square[(i+1)%4]),0,false,-1,45.});
    const ExPolygon right(Polygon({Point(2000000,-100),Point(4000100,-100),Point(4000100,4000100),Point(2000000,4000100)}));
    const auto threshold=GENERATE(0.,30.,45.,60.);
    const std::vector<WallPainting::CountRegion> regions{{{right},1,threshold}};
    const auto policy=WallPainting::BoundaryPolicy::bind({square},labels,1,2,scaled<double>(0.03),regions);
    const auto reinforced=policy.reinforce_slopes(30.);
    CHECK(reinforced.count_at(Point(0,2000000))==2);
    CHECK(reinforced.count_at(Point(4000000,2000000))==(threshold>0.&&threshold<45.?2:1));
    const auto alternate=policy.adjusted(1).reinforce_slopes(30.);
    CHECK(alternate.uniform_count()==2);
    CHECK(std::none_of(alternate.spans().begin(),alternate.spans().end(),[](const auto &span){return span.slope_reinforced;}));
}

TEST_CASE("A slope-only modifier retains one material outline and complete shell fill", "[SlopeWalls][Modifiers]")
{
    const bool arachne=GENERATE(false,true);
    Model model;
    auto *object=model.add_object();
    TriangleMesh mesh(its_make_cube(30.,20.,6.));
    for(auto &v:mesh.its.vertices) v.x()+=0.7*v.z();
    object->add_volume(mesh);
    auto modifier_mesh=TriangleMesh(its_make_cube(21.,30.,8.));
    modifier_mesh.translate(-5.,-5.,-1.);
    auto *modifier=object->add_volume(modifier_mesh);
    modifier->set_type(ModelVolumeType::PARAMETER_MODIFIER);
    modifier->config.set("extra_wall_slope_angle",30.);
    object->add_instance();
    DynamicPrintConfig config;
    config.apply(static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    config.set_deserialize_strict({{"wall_generator",arachne?"arachne":"classic"},{"wall_loops","1"},
        {"layer_height","0.2"},{"initial_layer_print_height","0.2"},{"extra_wall_slope_angle","0"},
        {"top_one_wall_type","not apply"},{"only_one_wall_first_layer","0"},{"brim_type","no_brim"},
        {"detect_overhang_wall","1"},{"enable_overhang_speed","1"},{"override_process_overhang_speed","0"}});
    Print print;print.set_status_silent();print.auto_assign_extruders(object);print.apply(model,config);print.process();
    const auto *printed=print.get_object(size_t(0));
    REQUIRE(printed->layers().size()>10);
    double max_outer_overhang_degree=0.;
    for (const Layer *layer:printed->layers()) {
        size_t crossings=0;
        std::vector<double> crossing_positions;
        std::vector<std::string> crossing_segments;
        bool has_reinforcement=false;
        for(const auto *region:layer->regions()) {
            has_reinforcement |= region->has_slope_wall_reinforcement;
            const auto walls=region->perimeters.flatten();
            for(const auto *entity:walls.entities) {
                auto record_overhang=[&](const ExtrusionPath &path) {
                    if(path.role()==erExternalPerimeter)
                        max_outer_overhang_degree=std::max(max_outer_overhang_degree,double(path.overhang_degree));
                };
                if(const auto *path=dynamic_cast<const ExtrusionPath *>(entity)) record_overhang(*path);
                else if(const auto *loop=dynamic_cast<const ExtrusionLoop *>(entity))
                    for(const auto &path:loop->paths) record_overhang(path);
                else if(const auto *multi=dynamic_cast<const ExtrusionMultiPath *>(entity))
                    for(const auto &path:multi->paths) record_overhang(path);
                const auto points=entity->as_polyline().points;
                for(size_t i=1;i<points.size();++i)
                    if((points[i-1].y()<=0&&points[i].y()>0)||(points[i].y()<=0&&points[i-1].y()>0)) {
                        ++crossings;
                        const auto &a=points[i-1], &b=points[i];
                        crossing_positions.push_back(unscaled<double>(a.x()-double(a.y())*(b.x()-a.x())/(b.y()-a.y())));
                        std::ostringstream segment;segment<<a.x()<<','<<a.y()<<" -> "<<b.x()<<','<<b.y();crossing_segments.push_back(segment.str());
                    }
            }
        }
        CAPTURE(arachne,layer->print_z,crossing_positions,crossing_segments);
        REQUIRE(has_reinforcement);
        CHECK(crossings==3); // Two walls left, one right, none on the modifier border.
    }
    // A count-only modifier must not masquerade as fuzzy skin and disable
    // the normal overhang-speed classification on the leaning outer wall.
    CAPTURE(arachne,max_outer_overhang_degree);
    CHECK(max_outer_overhang_degree>0.);
    for (const Layer *layer:{printed->layers().front(),printed->layers().back()}) {
        double solid_area=0.;
        for(const auto *region:layer->regions())
            for(const auto &surface:region->fill_surfaces.surfaces)
                if(surface.is_solid()) solid_area+=surface.area()*SCALING_FACTOR*SCALING_FACTOR;
        CAPTURE(arachne,layer->print_z,solid_area);
        CHECK(solid_area>400.);
    }
    auto snapshot=[](const Print &source) {
        // Loop starts, directions and collection ordering are not geometry.
        // Compare every deposited segment, including role, width and flow.
        std::vector<std::string> result;
        for(const Layer *layer:source.get_object(size_t(0))->layers()) {
            auto record_path=[&](const ExtrusionPath &path) {
                // The existing floating-shell width interpolator can partition
                // the same fill differently when parallel slicing rotates its
                // starting point (also reproducible between fresh Prints).
                // Its input surfaces are compared exactly below instead.
                if(path.role()==erFloatingVerticalShell) return;
                const auto &points=path.polyline.points;
                for(size_t i=1;i<points.size();++i) {
                    Point a=points[i-1],b=points[i];
                    if(a==b) continue;
                    if(a.x()>b.x() || (a.x()==b.x() && a.y()>b.y())) std::swap(a,b);
                    std::ostringstream signature;
                    signature<<std::setprecision(15)<<layer->id()<<':'<<int(path.role())<<':'
                             <<path.width<<':'<<path.height<<':'<<path.mm3_per_mm<<':'<<path.overhang_degree<<':'
                             <<a.x()<<','<<a.y()<<':'<<b.x()<<','<<b.y();
                    result.push_back(signature.str());
                }
            };
            for(const auto *region:layer->regions()) {
                for(const auto &surface:region->fill_surfaces.surfaces) {
                    const Polygons outlines=to_polygons(surface.expolygon);
                    for(const Polygon &outline:outlines)
                        for(size_t i=0;i<outline.points.size();++i) {
                            Point a=outline.points[i],b=outline.points[(i+1)%outline.points.size()];
                            if(a.x()>b.x() || (a.x()==b.x() && a.y()>b.y())) std::swap(a,b);
                            std::ostringstream signature;
                            signature<<layer->id()<<":surface:"<<int(surface.surface_type)<<':'
                                     <<a.x()<<','<<a.y()<<':'<<b.x()<<','<<b.y();
                            result.push_back(signature.str());
                        }
                }
                for(const auto *collection:{&region->perimeters,&region->fills}) {
                    const auto paths=collection->flatten();
                    for(const auto *entity:paths.entities) {
                        if(const auto *path=dynamic_cast<const ExtrusionPath *>(entity)) record_path(*path);
                        else if(const auto *loop=dynamic_cast<const ExtrusionLoop *>(entity))
                            for(const auto &path:loop->paths) record_path(path);
                        else if(const auto *multi=dynamic_cast<const ExtrusionMultiPath *>(entity))
                            for(const auto &path:multi->paths) record_path(path);
                        else FAIL("Unexpected extrusion entity in slope fixture");
                    }
                }
            }
        }
        std::sort(result.begin(),result.end());
        return result;
    };
    const auto reinforced_snapshot=snapshot(print);
    auto reslice=[&](double threshold) {
        modifier->config.set("extra_wall_slope_angle",threshold);
        REQUIRE(print.get_object(size_t(0))->is_step_done(posSupportMaterial));
        print.apply(model,config);
        CHECK_FALSE(print.get_object(size_t(0))->is_step_done(posSupportMaterial));
        CHECK_FALSE(print.get_object(size_t(0))->is_step_done(posDetectOverhangsForLift));
        print.process();
        for(const Layer *layer:print.get_object(size_t(0))->layers()) {
            const bool reinforced=std::any_of(layer->regions().begin(),layer->regions().end(),[](const LayerRegion *region) {
                return region->has_slope_wall_reinforcement;
            });
            CAPTURE(arachne,threshold,layer->print_z);
            CHECK(reinforced==(threshold==30.));
        }
        return snapshot(print);
    };
    // A zero override merges the modifier away, whereas an unmet override
    // retains that region. Compare each reslice with a fresh print of the same
    // regions, rather than requiring identical path partitions across them.
    for(double threshold:{0.,90.,30.}) {
        const auto reused_snapshot=reslice(threshold);
        Print fresh;fresh.set_status_silent();fresh.auto_assign_extruders(object);
        fresh.apply(model,config);fresh.process();
        const auto fresh_snapshot=snapshot(fresh);
        CAPTURE(arachne,threshold);
        CHECK(bool(reused_snapshot==fresh_snapshot));
        if(threshold==30.) {
            std::vector<std::string> removed,added;
            std::set_difference(reinforced_snapshot.begin(),reinforced_snapshot.end(),reused_snapshot.begin(),reused_snapshot.end(),std::back_inserter(removed));
            std::set_difference(reused_snapshot.begin(),reused_snapshot.end(),reinforced_snapshot.begin(),reinforced_snapshot.end(),std::back_inserter(added));
            const std::string first_removed=removed.empty()?"":removed.front();
            const std::string first_added=added.empty()?"":added.front();
            CAPTURE(removed.size(),added.size(),first_removed,first_added);
            CHECK(bool(reused_snapshot==reinforced_snapshot));
        }
    }
}

TEST_CASE("Arachne modifier segmentation never traverses an open wall twice", "[Overhang][Arachne][Segmentation]")
{
    using Algorithm::LineSegmentation::extrusion_segmentation;
    const bool reverse=GENERATE(false,true);
    const bool partly_clipped=GENERATE(false,true);
    CAPTURE(reverse,partly_clipped);
    Arachne::ExtrusionLine line(1,false,false);
    line.emplace_back(Point(0,-1000000),40000,1);
    line.emplace_back(Point(0,1000000),50000,1);
    if(reverse) std::reverse(line.junctions.begin(),line.junctions.end());
    Polygon boundary(Points{Point(-1000000,partly_clipped?0:-2000000),Point(1000000,partly_clipped?0:-2000000),
                            Point(1000000,2000000),Point(-1000000,2000000)});
    const auto segments=extrusion_segmentation(line,{ExPolygons{ExPolygon(boundary)}},1);
    REQUIRE(segments.size()==(partly_clipped?2:1));
    double length=0.;
    Point previous=line.front().p;
    for(const auto &segment:segments) {
        REQUIRE_FALSE(segment.extrusion.empty());
        CHECK(segment.extrusion.front().p==previous);
        for(size_t i=1;i<segment.extrusion.size();++i) {
            const auto &a=segment.extrusion[i-1],&b=segment.extrusion[i];
            CHECK((reverse?b.p.y()<=a.p.y():b.p.y()>=a.p.y()));
            CHECK(b.w>=40000);
            CHECK(b.w<=50000);
        }
        length+=segment.extrusion.polylineLength();
        previous=segment.extrusion.back().p;
    }
    CHECK(previous==line.back().p);
    CHECK(length==Approx(line.polylineLength()));
}

TEST_CASE("Closed Arachne walls retain their coverage across modifier segmentation", "[Overhang][Arachne][Segmentation]")
{
    using Algorithm::LineSegmentation::extrusion_segmentation;
    const bool reverse=GENERATE(false,true);
    const bool partly_clipped=GENERATE(false,true);
    Arachne::ExtrusionLine line(1,false,true);
    for(const Point &p:Points{Point(-1000000,-1000000),Point(1000000,-1000000),
                             Point(1000000,1000000),Point(-1000000,1000000),Point(-1000000,-1000000)})
        line.emplace_back(p,40000,1);
    if(reverse) std::reverse(line.junctions.begin(),line.junctions.end());
    Polygon boundary(Points{Point(partly_clipped?0:-2000000,-2000000),Point(2000000,-2000000),
                            Point(2000000,2000000),Point(partly_clipped?0:-2000000,2000000)});
    const auto segments=extrusion_segmentation(line,{ExPolygons{ExPolygon(boundary)}},1);
    REQUIRE_FALSE(segments.empty());
    double length=0.;
    Point previous=line.front().p;
    for(const auto &segment:segments) {
        REQUIRE_FALSE(segment.extrusion.empty());
        CHECK(segment.extrusion.front().p==previous);
        length+=segment.extrusion.polylineLength();
        previous=segment.extrusion.back().p;
    }
    CHECK(previous==line.back().p);
    CHECK(length==Approx(line.polylineLength()));
}
