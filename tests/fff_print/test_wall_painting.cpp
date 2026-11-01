#include <catch2/catch.hpp>
#include "libslic3r/Model.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Preset.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/TriangleSelector.hpp"
#include "libslic3r/PaintReproject.hpp"

using namespace Slic3r;

static DynamicPrintConfig wall_test_config()
{
    // Create options through their definitions, as profile loading does. The
    // legacy full_print_config clone leaves vector-enum serialization maps unset.
    DynamicPrintConfig config;
    config.apply(static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    return config;
}

TEST_CASE("Painted wall count is a total count and zero is valid", "[WallPainting][Config]")
{
    DynamicPrintConfig config = wall_test_config();
    REQUIRE(config.has("painted_wall_loops"));
    REQUIRE(config.opt_int("painted_wall_loops") == 2);
    config.set_deserialize_strict({{"painted_wall_loops", "0"}});
    REQUIRE(config.opt_int("painted_wall_loops") == 0);
    REQUIRE(config.opt_int("wall_loops") == 2);
    REQUIRE(std::find(Preset::print_options().begin(),Preset::print_options().end(),"painted_wall_loops") != Preset::print_options().end());
}

TEST_CASE("Wall painting copies independently and resets with mesh annotations", "[WallPainting][Model]")
{
    Model model;
    ModelObject *object = model.add_object();
    ModelVolume *volume = object->add_volume(TriangleMesh(its_make_cube(10., 10., 10.)));
    volume->wall_painting_facets.set_triangle_from_string(0, "4");
    REQUIRE_FALSE(volume->wall_painting_facets.empty());
    REQUIRE(volume->supported_facets.empty());
    REQUIRE(volume->fuzzy_skin_facets.empty());
    Model copy(model);
    auto *copied = copy.objects.front()->volumes.front();
    REQUIRE(copied->wall_painting_facets.equals(volume->wall_painting_facets));
    copied->reset_extra_facets();
    REQUIRE(copied->wall_painting_facets.empty());
    REQUIRE_FALSE(volume->wall_painting_facets.empty());
}

#include "libslic3r/WallPainting.hpp"
TEST_CASE("Wall boundary policies retain partial and zero overrides", "[WallPainting][Boundary]")
{
    using namespace WallPainting;
    BoundaryPolicy policy({{Line(Point(0,0), Point(1000,0)), 1, 0},
                           {Line(Point(1000,0), Point(2000,0)), 3, 0},
                           {Line(Point(2000,0), Point(3000,0)), 0, 0}});
    REQUIRE(policy.maximum() == 3);
    REQUIRE_FALSE(policy.uniform_count().has_value());
    REQUIRE(policy.count_at(Point(500,10)) == 1);
    REQUIRE(policy.count_at(Point(1500,10)) == 3);
    REQUIRE(policy.count_at(Point(2500,10)) == 0);
    auto pieces = policy.split(Line(Point(0,10), Point(3000,10)));
    REQUIRE(pieces.size() == 3);
    REQUIRE(pieces[0].second == 1);
    REQUIRE(pieces[1].second == 3);
    REQUIRE(pieces[2].second == 0);
    REQUIRE(policy.adjusted(1).count_at(Point(2500,10)) == 1);
    REQUIRE(policy.adjusted(0,1).count_at(Point(1500,10)) == 2);
    REQUIRE(policy.adjusted(0,0,true).count_at(Point(1500,10)) == 1);
}

#include "libslic3r/Arachne/WallToolPaths.hpp"
TEST_CASE("Arachne accepts independent boundary wall counts", "[WallPainting][Arachne]")
{
    using namespace WallPainting;
    Polygons outline{Polygon({Point(0,0),Point(4000000,0),Point(4000000,4000000),Point(0,4000000)})};
    BoundaryPolicy policy({{Line(outline[0][0],outline[0][1]),1,0},
                           {Line(outline[0][1],outline[0][2]),2,0},
                           {Line(outline[0][2],outline[0][3]),1,0},
                           {Line(outline[0][3],outline[0][0]),1,0}});
    Arachne::WallToolPathsParams params{0.1f,0.1f,0.4f,10.f,0.1f,1};
    Arachne::WallToolPaths paths(outline,40000,40000,2,0,0.2,params,&policy);
    auto walls=paths.getToolPaths();
    size_t left=0,right=0;
    for (const auto &depth : walls) for (const auto &wall : depth)
        for (size_t i=1;i<wall.junctions.size();++i) {
            const auto &a=wall.junctions[i-1], &b=wall.junctions[i];
            if ((a.p.y()<=2000000 && b.p.y()>2000000)||(b.p.y()<=2000000 && a.p.y()>2000000)) {
                if (a.p.x()<2000000) ++left; else ++right;
            }
        }
    REQUIRE(left==1);
    REQUIRE(right==2);
    REQUIRE_FALSE(paths.getInnerContour().empty());
}

#include <boost/filesystem.hpp>
TEST_CASE("Bambu 3MF retains wall painting independently from support painting", "[WallPainting][3mf]")
{
    const auto workspace=boost::filesystem::temp_directory_path()/boost::filesystem::unique_path("wall-project-%%%%-%%%%");
    boost::filesystem::create_directories(workspace);
    const std::string old_data_dir=data_dir();
    const std::string old_temporary_dir=temporary_dir();
    set_data_dir(workspace.string());
    set_temporary_dir(workspace.string());
    struct DataDirCleanup {
        boost::filesystem::path path; std::string original, temporary;
        ~DataDirCleanup() { set_data_dir(original); set_temporary_dir(temporary); boost::system::error_code ec; boost::filesystem::remove_all(path,ec); }
    } data_dir_cleanup{workspace,old_data_dir,old_temporary_dir};
    Model source;
    auto *object=source.add_object();
    auto *volume=object->add_volume(TriangleMesh(its_make_cube(10.,10.,10.)));
    object->add_instance();
    TriangleSelector selector(volume->mesh());
    selector.set_facet(0,EnforcerBlockerType::ENFORCER);
    selector.set_facet(1,EnforcerBlockerType::BLOCKER);
    const auto &face=volume->mesh().its.indices.front();
    const auto &vertices=volume->mesh().its.vertices;
    const Vec3f center=(vertices[face[0]]+vertices[face[1]]+vertices[face[2]])/3.f;
    selector.select_patch(0,TriangleSelector::SinglePointCursor::cursor_factory(
        center,center+Vec3f(0,0,20),1.f,TriangleSelector::CursorType::SPHERE,
        Transform3d::Identity(),TriangleSelector::ClippingPlane()),
        EnforcerBlockerType::NONE,Transform3d::Identity(),true);
    volume->wall_painting_facets.set(selector);
    REQUIRE(volume->wall_painting_facets.get_triangle_as_string(0).size()>1);
    volume->name="Subdivided wall paint";
    volume->supported_facets.set_triangle_from_string(0,"8");
    volume->config.set("painted_wall_loops",5);
    object->config.set("painted_wall_loops",4);
    // Shared geometry must not allow 3MF mesh deduplication to collapse two
    // independent masks. The second part intentionally has different paint.
    auto *shared=object->add_volume(*volume);
    shared->name="Independent wall paint";
    shared->set_offset(Vec3d(20.,0.,0.));
    shared->wall_painting_facets.reset();
    shared->wall_painting_facets.set_triangle_from_string(2,"8");
    shared->config.set("painted_wall_loops",0);
    DynamicPrintConfig config;
    config.set_deserialize_strict({{"painted_wall_loops","3"}});
    const auto file=boost::filesystem::temp_directory_path()/boost::filesystem::unique_path("wall-paint-%%%%-%%%%.3mf");
    const std::string filename=file.string();
    struct Cleanup { boost::filesystem::path p; ~Cleanup(){ boost::system::error_code ec; boost::filesystem::remove(p,ec); } } cleanup{file};
    StoreParams params;
    params.path=filename.c_str(); params.model=&source; params.config=&config;
    REQUIRE(store_bbs_3mf(params));
    Model loaded;
    DynamicPrintConfig loaded_config;
    ConfigSubstitutionContext substitutions(ForwardCompatibilitySubstitutionRule::Disable);
    bool is_bambu=false; Semver version;
    PlateDataPtrs plates; std::vector<Preset*> presets;
    REQUIRE(load_bbs_3mf(filename.c_str(),&loaded_config,&substitutions,&loaded,&plates,&presets,&is_bambu,&version,
                        nullptr,LoadStrategy::LoadModel|LoadStrategy::LoadConfig));
    REQUIRE(loaded.objects.size()==1);
    REQUIRE(loaded.objects[0]->volumes.size()==2);
    REQUIRE(loaded.objects[0]->volumes[0]->wall_painting_facets.equals(volume->wall_painting_facets));
    REQUIRE(loaded.objects[0]->volumes[0]->supported_facets.equals(volume->supported_facets));
    REQUIRE(loaded.objects[0]->volumes[1]->wall_painting_facets.equals(shared->wall_painting_facets));
    REQUIRE(loaded.objects[0]->config.opt_int("painted_wall_loops")==4);
    REQUIRE(loaded.objects[0]->volumes[0]->config.opt_int("painted_wall_loops")==5);
    REQUIRE(loaded.objects[0]->volumes[1]->config.opt_int("painted_wall_loops")==0);
    REQUIRE(loaded_config.opt_int("painted_wall_loops")==3);
    for (auto *plate:plates) delete plate;
}

#include "libslic3r/PerimeterGenerator.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/SurfaceCollection.hpp"

TEST_CASE("Both generators keep one skin wall and reinforce the painted side", "[WallPainting][Perimeters]")
{
    const bool arachne=GENERATE(false,true);
    const int normal=GENERATE(0,1,3);
    const int painted=GENERATE(0,1,2,3);
    CAPTURE(arachne,normal,painted);
    const Polygon square({Point(0,0),Point(4000000,0),Point(4000000,4000000),Point(0,4000000)});
    SurfaceCollection surfaces; surfaces.surfaces.emplace_back(stInternal,ExPolygon(square));
    WallPainting::LayerPaint paint;
    for (size_t i=0;i<4;++i) paint.push_back({Line(square[i],square[(i+1)%4]),0,i==1,-1});
    PrintRegionConfig region; region.wall_loops.value=normal; region.painted_wall_loops.value=painted;
    PrintObjectConfig object; object.top_one_wall_type.value=TopOneWallType::None;
    PrintConfig print;
    Flow flow=Flow::new_from_config_width(frPerimeter,ConfigOptionFloatOrPercent(0.42,false),0.4,0.2);
    ExtrusionEntityCollection loops,gaps; SurfaceCollection fill; ExPolygons no_overlap; std::vector<LoopNode> nodes;
    PerimeterGenerator generator(&surfaces,0.2,flow,&region,&object,&print,false,&loops,&gaps,&fill,&no_overlap,&nodes);
    generator.layer_id=2; generator.slice_z=0.5; generator.wall_painting=&paint;
    const ExPolygons lower{ExPolygon(square)}; generator.lower_slices=&lower;
    PerimeterRegions regions; generator.perimeter_regions=&regions;
    if (arachne) generator.process_arachne(); else generator.process_classic();
    size_t left=0,right=0;
    for (const auto *entity : loops.flatten().entities) {
        const auto polyline=entity->as_polyline();
        for (size_t i=1;i<polyline.points.size();++i) {
            const auto &a=polyline.points[i-1], &b=polyline.points[i];
            if ((a.y()<=2000000 && b.y()>2000000)||(b.y()<=2000000 && a.y()>2000000)) {
                if (a.x()<2000000) ++left; else ++right;
            }
        }
    }
    REQUIRE(left==size_t(normal));
    REQUIRE(right==size_t(painted));
    REQUIRE_FALSE(fill.empty());
}

namespace {
struct GeneratedWalls {
    ExtrusionEntityCollection loops, gaps;
    SurfaceCollection fill;
    ExPolygons no_overlap;
};
void generate_walls(const ExPolygon &shape, const WallPainting::LayerPaint &paint,
                    bool arachne, int normal, int painted, GeneratedWalls &result,
                    int layer_id=2, bool alternate=false, bool single_first=false, bool single_top=false,
                    WallSequence sequence=WallSequence::InnerOuter)
{
    SurfaceCollection surfaces; surfaces.surfaces.emplace_back(stInternal, shape);
    PrintRegionConfig region; region.wall_loops.value=normal; region.painted_wall_loops.value=painted;
    region.alternate_extra_wall.value=alternate;
    region.wall_sequence.value=sequence;
    PrintObjectConfig object; object.top_one_wall_type.value=single_top?TopOneWallType::Alltop:TopOneWallType::None;
    object.only_one_wall_first_layer.value=single_first;
    PrintConfig print;
    Flow flow=Flow::new_from_config_width(frPerimeter,ConfigOptionFloatOrPercent(0.42,false),0.4,0.2);
    std::vector<LoopNode> nodes;
    PerimeterGenerator generator(&surfaces,0.2,flow,&region,&object,&print,false,
                                &result.loops,&result.gaps,&result.fill,&result.no_overlap,&nodes);
    generator.layer_id=layer_id; generator.slice_z=0.5; generator.wall_painting=&paint;
    const ExPolygons lower{shape}; generator.lower_slices=&lower;
    PerimeterRegions regions; generator.perimeter_regions=&regions;
    if (arachne) generator.process_arachne(); else generator.process_classic();
}
std::vector<double> wall_crossings(const ExtrusionEntityCollection &loops, double height)
{
    std::vector<double> positions;
    const coord_t y=scaled<coord_t>(height);
    const auto flattened=loops.flatten();
    for (const auto *entity:flattened.entities) {
        const auto line=entity->as_polyline();
        for (size_t i=1;i<line.points.size();++i) {
            const auto &a=line.points[i-1], &b=line.points[i];
            if ((a.y()<=y && b.y()>y)||(b.y()<=y && a.y()>y))
                positions.push_back(unscaled<double>(a.x()+double(b.x()-a.x())*(y-a.y())/(b.y()-a.y())));
        }
    }
    std::sort(positions.begin(),positions.end());
    return positions;
}
size_t in_interval(const std::vector<double> &points, double left, double right)
{
    return std::count_if(points.begin(),points.end(),[=](double x){return x>left && x<right;});
}
Polygon rectangle(double x0,double y0,double x1,double y1)
{
    return Polygon({Point::new_scale(x0,y0),Point::new_scale(x1,y0),
                    Point::new_scale(x1,y1),Point::new_scale(x0,y1)});
}
void label_boundary(WallPainting::LayerPaint &paint,const Polygon &polygon,bool selected,size_t id=0)
{
    for (size_t i=0;i<polygon.size();++i)
        paint.push_back({Line(polygon[i],polygon[(i+1)%polygon.size()]),id,selected,-1});
}
}

TEST_CASE("Painted holes have independent wall stacks and no extra skin walls", "[WallPainting][Perimeters]")
{
    const bool arachne=GENERATE(false,true);
    CAPTURE(arachne);
    ExPolygon section(rectangle(0,0,60,20));
    for (double x:{10.,30.,50.}) {
        auto hole=rectangle(x-2,7,x+2,13); hole.reverse(); section.holes.push_back(hole);
    }
    WallPainting::LayerPaint paint;
    label_boundary(paint,section.contour,false);
    for (size_t i=0;i<section.holes.size();++i) label_boundary(paint,section.holes[i],i!=1);
    GeneratedWalls result; generate_walls(section,paint,arachne,1,2,result);
    const auto crossings=wall_crossings(result.loops,10.);
    CAPTURE(crossings);
    REQUIRE(in_interval(crossings,0,2)==1);
    REQUIRE(in_interval(crossings,58,60)==1);
    REQUIRE(in_interval(crossings,6,8)==2);
    REQUIRE(in_interval(crossings,12,14)==2);
    REQUIRE(in_interval(crossings,26,28)==1);
    REQUIRE(in_interval(crossings,32,34)==1);
    REQUIRE(in_interval(crossings,46,48)==2);
    REQUIRE(in_interval(crossings,52,54)==2);
}

TEST_CASE("Partial painting terminates added walls inside its extent", "[WallPainting][Perimeters]")
{
    const bool arachne=GENERATE(false,true);
    CAPTURE(arachne);
    ExPolygon section(rectangle(0,0,40,40));
    WallPainting::LayerPaint paint;
    for (size_t i:{0u,2u,3u}) paint.push_back({Line(section.contour[i],section.contour[(i+1)%4]),0,false,-1});
    paint.push_back({Line(Point::new_scale(40,0),Point::new_scale(40,15)),0,false,-1});
    paint.push_back({Line(Point::new_scale(40,15),Point::new_scale(40,25)),0,true,-1});
    paint.push_back({Line(Point::new_scale(40,25),Point::new_scale(40,40)),0,false,-1});
    GeneratedWalls result; generate_walls(section,paint,arachne,1,3,result);
    for (double y:{10.,20.,30.}) {
        auto crossings=wall_crossings(result.loops,y);
        CAPTURE(y,crossings);
        REQUIRE(in_interval(crossings,0,2)==1);
        REQUIRE(in_interval(crossings,38,40)==(y==20.?3:1));
    }
    const auto flattened=result.loops.flatten();
    for (const auto *entity:flattened.entities) {
        const auto path=entity->as_polyline();
        for (size_t i=1;i<path.points.size();++i) {
            const auto &a=path.points[i-1], &b=path.points[i];
            // No radial connector may close a painted patch across the wall stack.
            if (a.x()>scaled<coord_t>(38.) && b.x()>scaled<coord_t>(38.) &&
                a.y()>scaled<coord_t>(14.) && a.y()<scaled<coord_t>(26.))
                REQUIRE_FALSE((std::abs(a.y()-b.y())<2 && std::abs(a.x()-b.x())>scaled<coord_t>(0.2)));
        }
    }
}

TEST_CASE("Narrow painted sections saturate without duplicate central walls", "[WallPainting][Perimeters]")
{
    const bool arachne=GENERATE(false,true);
    const int painted=GENERATE(2,1000);
    const double width=GENERATE(0.45,0.85,1.25);
    CAPTURE(arachne,painted,width);
    ExPolygon section(rectangle(0,0,width,30));
    WallPainting::LayerPaint paint;
    for (size_t i=0;i<4;++i)
        paint.push_back({Line(section.contour[i],section.contour[(i+1)%4]),0,i==1,-1});
    GeneratedWalls result; generate_walls(section,paint,arachne,1,painted,result);
    const auto crossings=wall_crossings(result.loops,15.);
    CAPTURE(crossings);
    REQUIRE_FALSE(crossings.empty());
    REQUIRE(crossings.size()<=4);
    for (size_t i=0;i<crossings.size();++i) {
        REQUIRE(crossings[i]>=0.);
        REQUIRE(crossings[i]<=width);
        if (i>0) REQUIRE(crossings[i]-crossings[i-1]>0.01);
    }
}

TEST_CASE("Uniform paint follows the ordinary generator path", "[WallPainting][Perimeters]")
{
    const bool arachne=GENERATE(false,true);
    const int count=GENERATE(0,1,2,3);
    CAPTURE(arachne,count);
    ExPolygon section(rectangle(0,0,40,40));
    WallPainting::LayerPaint paint; label_boundary(paint,section.contour,true);
    GeneratedWalls ordinary,painted;
    generate_walls(section,{},arachne,count,2,ordinary);
    generate_walls(section,paint,arachne,1,count,painted);
    REQUIRE(wall_crossings(ordinary.loops,20.)==wall_crossings(painted.loops,20.));
    REQUIRE(ordinary.loops.total_volume()==Approx(painted.loops.total_volume()));
    REQUIRE(ordinary.fill.surfaces.size()==painted.fill.surfaces.size());
    for (size_t i=0;i<ordinary.fill.surfaces.size();++i)
        REQUIRE(ordinary.fill.surfaces[i].expolygon.area()==Approx(painted.fill.surfaces[i].expolygon.area()));
}

TEST_CASE("Partial curved holes retain their painted circumference", "[WallPainting][Perimeters][Transition]")
{
    const bool arachne=GENERATE(false,true);
    ExPolygon section(rectangle(0,0,80,40));
    Points circle;
    for (int i=0;i<128;++i) {
        const double angle=-2.*PI*i/128.;
        circle.push_back(Point::new_scale(40.+5.*std::cos(angle),20.+5.*std::sin(angle)));
    }
    section.holes.emplace_back(circle);
    WallPainting::LayerPaint paint; label_boundary(paint,section.contour,false);
    for (size_t i=0;i<circle.size();++i) {
        const Point a=circle[i],b=circle[(i+1)%circle.size()];
        const double x=unscaled<double>((a.x()+b.x())/2)-40.;
        const double y=unscaled<double>((a.y()+b.y())/2)-20.;
        paint.push_back({Line(a,b),0,x>std::abs(y),-1});
    }
    GeneratedWalls result; generate_walls(section,paint,arachne,1,3,result);
    const auto crossings=wall_crossings(result.loops,20.);
    CAPTURE(arachne,crossings);
    REQUIRE(in_interval(crossings,44.9,48.)==3);
    REQUIRE(in_interval(crossings,32.,35.1)==1);
    REQUIRE(in_interval(crossings,0.,3.)==1);
    REQUIRE(in_interval(crossings,77.,80.)==1);
    for (const auto *entity:result.loops.flatten().entities) {
        if (entity->role()!=erPerimeter) continue;
        for (const Point &p:entity->as_polyline().points) {
            const double x=unscaled<double>(p.x())-40.,y=unscaled<double>(p.y())-20.;
            REQUIRE(std::abs(std::atan2(y,x))<=PI/4.+0.02);
            REQUIRE(std::hypot(x,y)>=4.99);
        }
    }
}

TEST_CASE("Short Arachne paint patches keep the common outer wall continuous", "[WallPainting][Arachne][Transition]")
{
    const double length=GENERATE(0.25,1.0);
    ExPolygon section(rectangle(0,0,40,40));
    WallPainting::LayerPaint paint;
    for (size_t i:{0u,2u,3u}) paint.push_back({Line(section.contour[i],section.contour[(i+1)%4]),0,false,-1});
    const double start=20.-length/2.,end=20.+length/2.;
    paint.push_back({Line(Point::new_scale(40,0),Point::new_scale(40,start)),0,false,-1});
    paint.push_back({Line(Point::new_scale(40,start),Point::new_scale(40,end)),0,true,-1});
    paint.push_back({Line(Point::new_scale(40,end),Point::new_scale(40,40)),0,false,-1});
    GeneratedWalls result; generate_walls(section,paint,true,1,3,result);
    for (double y:{start-0.1,20.,end+0.1})
        REQUIRE(in_interval(wall_crossings(result.loops,y),39.5,40.)==1);
    for (const auto *entity:result.loops.flatten().entities) {
        if (entity->role()!=erPerimeter) continue;
        for (const Point &p:entity->as_polyline().points) {
            REQUIRE(unscaled<double>(p.y())>=start-0.01);
            REQUIRE(unscaled<double>(p.y())<=end+0.01);
        }
    }
}

TEST_CASE("Arachne can fill a narrow section from one painted boundary", "[WallPainting][Review][Arachne]")
{
    const double width=GENERATE(0.45,0.85,1.25);
    const int normal=GENERATE(0,1), painted_count=GENERATE(3,4);
    Polygons outline{rectangle(0,0,width,30)};
    WallPainting::LayerPaint paint;
    for (size_t i=0;i<4;++i)
        paint.push_back({Line(outline.front()[i],outline.front()[(i+1)%4]),0,i==1,-1});
    auto policy=WallPainting::BoundaryPolicy::bind(outline,paint,normal,painted_count,10.);
    Arachne::WallToolPathsParams params{0.1f,0.1f,0.4f,10.f,0.1f,1};
    Arachne::WallToolPaths painted(outline,40000,40000,4,0,0.2,params,&policy);
    Arachne::WallToolPaths ordinary(outline,40000,40000,4,0,0.2,params);
    auto cross_section=[](Arachne::WallToolPaths &paths) {
        std::vector<std::pair<double,double>> intervals;
        const coord_t y=scaled<coord_t>(15.);
        for (const auto &depth:paths.getToolPaths()) for (const auto &path:depth)
            for (size_t i=1;i<path.junctions.size();++i) {
                const auto &a=path.junctions[i-1],&b=path.junctions[i];
                if ((a.p.y()<=y && b.p.y()>y)||(b.p.y()<=y && a.p.y()>y)) {
                    const double t=double(y-a.p.y())/(b.p.y()-a.p.y());
                    const double x=a.p.x()+t*(b.p.x()-a.p.x()),w=a.w+t*(b.w-a.w);
                    REQUIRE(w>0.);
                    intervals.emplace_back(unscaled<double>(x-w/2.),unscaled<double>(x+w/2.));
                }
            }
        std::sort(intervals.begin(),intervals.end());
        return intervals;
    };
    const auto expected=cross_section(ordinary),actual=cross_section(painted);
    CAPTURE(width,normal,painted_count,expected,actual);
    REQUIRE(actual.size()==std::min(expected.size(),size_t(normal+painted_count)));
    REQUIRE(actual.front().first==Approx(0.).margin(0.01));
    REQUIRE(actual.back().second==Approx(width).margin(0.01));
    for (size_t i=0;i<actual.size();++i) {
        if (actual.size()==expected.size()) {
            REQUIRE(actual[i].first==Approx(expected[i].first).margin(0.01));
            REQUIRE(actual[i].second==Approx(expected[i].second).margin(0.01));
        }
        // Neither overlap nor an unfilled slit between adjacent native beads.
        if (i) REQUIRE(actual[i].first==Approx(actual[i-1].second).margin(0.01));
    }
}

#include "libslic3r/Print.hpp"
#include "libslic3r/Layer.hpp"
namespace {
TriangleMesh banded_box()
{
    indexed_triangle_set mesh;
    for (float z:{0.f,2.f,4.f,6.f}) {
        mesh.vertices.emplace_back(0.f,0.f,z); mesh.vertices.emplace_back(20.f,0.f,z);
        mesh.vertices.emplace_back(20.f,20.f,z); mesh.vertices.emplace_back(0.f,20.f,z);
    }
    mesh.indices.emplace_back(0,2,1); mesh.indices.emplace_back(0,3,2);
    mesh.indices.emplace_back(12,13,14); mesh.indices.emplace_back(12,14,15);
    for (int band=0;band<3;++band) for (int side=0;side<4;++side) {
        const int a=4*band+side,b=4*band+(side+1)%4;
        mesh.indices.emplace_back(a,b,b+4); mesh.indices.emplace_back(a,b+4,a+4);
    }
    return TriangleMesh(std::move(mesh));
}
}

TEST_CASE("Surface paint is limited to its height band after transforms", "[WallPainting][Slice]")
{
    const bool mirrored=GENERATE(false,true);
    const bool variable_height=GENERATE(false,true);
    CAPTURE(mirrored,variable_height);
    Model model; auto *object=model.add_object(); object->name="wall-paint-band";
    auto *volume=object->add_volume(banded_box());
    TriangleSelector selector(volume->mesh());
    // Middle band's +X face: two triangles, with shared edges to neutral bands.
    selector.set_facet(14,EnforcerBlockerType::ENFORCER);
    selector.set_facet(15,EnforcerBlockerType::ENFORCER);
    volume->wall_painting_facets.set(selector);
    auto *instance=object->add_instance();
    instance->set_mirror(Vec3d(mirrored?-1.:1.,1.,1.));
    instance->set_rotation(Vec3d(0.,0.,0.37));
    instance->set_scaling_factor(Vec3d(1.2,0.8,1.5));
    object->ensure_on_bed();
    if (variable_height)
        object->layer_height_profile.set(std::vector<coordf_t>{0.,0.2,1.,0.2,2.,0.1,7.,0.1,8.,0.25,9.,0.25});
    DynamicPrintConfig config=wall_test_config();
    config.set_deserialize_strict({{"layer_height","0.2"},{"wall_loops","1"},{"painted_wall_loops","2"},
                                  {"top_one_wall_type","not apply"},{"only_one_wall_first_layer","0"}});
    Print print; print.set_status_silent(); print.auto_assign_extruders(object); print.apply(model,config);
    REQUIRE(print.objects().size()==1);
    auto *print_object=print.get_object(size_t(0)); print_object->slice();
    const auto paints=WallPainting::slice_paint(*print_object,[]{});
    REQUIRE(paints.size()==print_object->layers().size());
    if (variable_height)
        REQUIRE(std::any_of(print_object->layers().begin()+1,print_object->layers().end(),
                           [](const Layer *layer){return std::abs(layer->height-0.2)>0.03;}));
    size_t painted_layers=0;
    for (size_t i=0;i<paints.size();++i) {
        const double z=print_object->layers()[i]->slice_z;
        const bool has_paint=std::any_of(paints[i].begin(),paints[i].end(),[](const auto &line){return line.painted;});
        CAPTURE(i,z,has_paint);
        REQUIRE(has_paint==(z>=3. && z<6.));
        painted_layers+=has_paint;
        if (has_paint) {
            auto policy=WallPainting::BoundaryPolicy::bind(to_polygons(print_object->layers()[i]->lslices),paints[i],1,2,scaled<double>(0.03));
            REQUIRE(policy.maximum()==2);
            REQUIRE_FALSE(policy.uniform_count().has_value());
        }
    }
    REQUIRE(painted_layers>0);
}

TEST_CASE("Removed painted surfaces cannot label a nearby compensated cut", "[WallPainting][Slice][Compensation]")
{
    const double compensation=GENERATE(-0.15,0.,0.15);
    CAPTURE(compensation);
    Model model; auto *object=model.add_object();
    auto *volume=object->add_volume(banded_box());
    TriangleSelector selector(volume->mesh());
    selector.set_facet(14,EnforcerBlockerType::ENFORCER);
    selector.set_facet(15,EnforcerBlockerType::ENFORCER);
    volume->wall_painting_facets.set(selector);
    auto cut=TriangleMesh(its_make_cube(5.,22.,8.)); cut.translate(19.99,-1.,-1.);
    auto *negative=object->add_volume(std::move(cut));
    negative->set_type(ModelVolumeType::NEGATIVE_VOLUME);
    object->add_instance(); object->ensure_on_bed();
    auto config=wall_test_config();
    config.set_deserialize_strict({{"layer_height","0.2"},{"wall_loops","1"},{"painted_wall_loops","3"},
                                  {"xy_contour_compensation",std::to_string(compensation)},
                                  {"elefant_foot_compensation","0"}});
    Print print; print.set_status_silent(); print.auto_assign_extruders(object); print.apply(model,config);
    auto *printed=print.get_object(size_t(0)); printed->slice();
    const auto paints=WallPainting::slice_paint(*printed,[]{});
    for (size_t i=0;i<paints.size();++i) {
        auto policy=WallPainting::BoundaryPolicy::bind(to_polygons(printed->layers()[i]->lslices),paints[i],1,3,
                                                      scaled<double>(0.03+std::abs(compensation)));
        REQUIRE(policy.uniform_count()==std::optional<int>(1));
    }
}

TEST_CASE("Arachne tapers added beads within the painted boundary", "[WallPainting][Arachne][Transition]")
{
    Polygons outline{rectangle(0,0,40,40)};
    WallPainting::LayerPaint paint;
    for (size_t i:{0u,2u,3u}) paint.push_back({Line(outline[0][i],outline[0][(i+1)%4]),0,false,-1});
    paint.push_back({Line(Point::new_scale(40,0),Point::new_scale(40,15)),0,false,-1});
    paint.push_back({Line(Point::new_scale(40,15),Point::new_scale(40,25)),0,true,-1});
    paint.push_back({Line(Point::new_scale(40,25),Point::new_scale(40,40)),0,false,-1});
    auto policy=WallPainting::BoundaryPolicy::bind(outline,paint,1,3,10.);
    Arachne::WallToolPathsParams params{0.1f,0.1f,0.4f,10.f,0.1f,1};
    Arachne::WallToolPaths paths(outline,40000,40000,3,0,0.2,params,&policy);
    bool tapered=false, full_width=false;
    for (const auto &depth:paths.getToolPaths()) for (const auto &path:depth) {
        if (path.inset_idx==0) continue;
        for (const auto &junction:path.junctions) {
            const double y=unscaled<double>(junction.p.y());
            if (junction.p.x()<scaled<coord_t>(38.)) continue;
            REQUIRE(y>=15.-EPSILON);
            REQUIRE(y<=25.+EPSILON);
            REQUIRE(junction.w>=SCALED_EPSILON);
            if (y<15.4 || y>24.6) tapered|=junction.w<20000;
            if (y>=15.4-EPSILON && y<=24.6+EPSILON) full_width|=junction.w>35000;
        }
    }
    REQUIRE(tapered);
    REQUIRE(full_width);
}

TEST_CASE("Wall masks survive supported split merge and mesh replacement operations", "[WallPainting][Model]")
{
    Model model; auto *object=model.add_object();
    auto first=TriangleMesh(its_make_cube(10.,10.,10.));
    auto second=TriangleMesh(its_make_cube(10.,10.,10.)); second.translate(20.,0.,0.);
    const size_t first_faces=first.its.indices.size(); first.merge(second);
    auto *volume=object->add_volume(std::move(first));
    TriangleSelector selector(volume->mesh());
    for (size_t i=0;i<first_faces;++i) selector.set_facet(i,EnforcerBlockerType::ENFORCER);
    volume->wall_painting_facets.set(selector);
    REQUIRE(volume->split(1)==2);
    REQUIRE(object->volumes.size()==2);
    size_t painted_parts=0;
    for (const auto *part:object->volumes) painted_parts+=!part->wall_painting_facets.empty();
    REQUIRE(painted_parts==1);
    object->merge();
    REQUIRE(object->volumes.size()==1);
    volume=object->volumes.front();
    REQUIRE(volume->wall_painting_facets.get_facets(*volume,EnforcerBlockerType::ENFORCER).indices.size()==first_faces);
    REQUIRE(volume->set_mesh_keep_paint(TriangleMesh(volume->mesh())));
    REQUIRE(volume->wall_painting_facets.get_facets(*volume,EnforcerBlockerType::ENFORCER).indices.size()==first_faces);
    REQUIRE(volume->supported_facets.empty());
    REQUIRE(volume->seam_facets.empty());
    REQUIRE(volume->fuzzy_skin_facets.empty());
}

TEST_CASE("Infill follows the walls actually emitted around a painted patch", "[WallPainting][Infill]")
{
    const bool arachne=GENERATE(false,true);
    CAPTURE(arachne);
    ExPolygon section(rectangle(0,0,40,40));
    WallPainting::LayerPaint paint;
    for (size_t i:{0u,2u,3u}) paint.push_back({Line(section.contour[i],section.contour[(i+1)%4]),0,false,-1});
    paint.push_back({Line(Point::new_scale(40,0),Point::new_scale(40,15)),0,false,-1});
    paint.push_back({Line(Point::new_scale(40,15),Point::new_scale(40,25)),0,true,-1});
    paint.push_back({Line(Point::new_scale(40,25),Point::new_scale(40,40)),0,false,-1});
    GeneratedWalls result; generate_walls(section,paint,arachne,1,3,result);
    auto infill_at=[&](double x,double y) {
        const Point p=Point::new_scale(x,y);
        return std::any_of(result.no_overlap.begin(),result.no_overlap.end(),[&](const ExPolygon &shape){return shape.contains(p);});
    };
    REQUIRE(infill_at(39.4,10.));
    REQUIRE_FALSE(infill_at(39.4,20.));
    REQUIRE(infill_at(38.4,20.));
    REQUIRE(infill_at(0.6,20.));
}

TEST_CASE("Mixed zero walls keep infill inside the material", "[WallPainting][Review][Infill]")
{
    const bool arachne=GENERATE(false,true);
    const bool paint_zero=GENERATE(false,true);
    CAPTURE(arachne,paint_zero);
    ExPolygon section(rectangle(0,0,40,40));
    auto hole=rectangle(12,12,28,28); hole.reverse(); section.holes.push_back(hole);
    WallPainting::LayerPaint paint;
    label_boundary(paint,section.contour,false);
    label_boundary(paint,section.holes.front(),true);
    GeneratedWalls result;
    generate_walls(section,paint,arachne,paint_zero?3:0,paint_zero?0:3,result);
    const auto outside=diff_ex(to_expolygons(result.fill.surfaces),ExPolygons{section});
    REQUIRE(area(outside)<scaled<double>(0.001)*scaled<double>(0.001));
}

TEST_CASE("Classic orders partial second walls after the outer wall", "[WallPainting][Review][Perimeters]")
{
    const bool paint_zero=GENERATE(false,true);
    CAPTURE(paint_zero);
    ExPolygon section(rectangle(0,0,40,40));
    WallPainting::LayerPaint paint;
    for (size_t i:{0u,2u,3u}) paint.push_back({Line(section.contour[i],section.contour[(i+1)%4]),0,false,-1});
    paint.push_back({Line(Point::new_scale(40,0),Point::new_scale(40,15)),0,false,-1});
    paint.push_back({Line(Point::new_scale(40,15),Point::new_scale(40,25)),0,true,-1});
    paint.push_back({Line(Point::new_scale(40,25),Point::new_scale(40,40)),0,false,-1});
    GeneratedWalls result;
    generate_walls(section,paint,false,paint_zero?3:1,paint_zero?0:3,result,2,false,false,false,WallSequence::InnerOuterInner);
    std::vector<double> ordered;
    const auto paths=result.loops.flatten();
    for (const auto *path:paths.entities) {
        const Polyline line=path->as_polyline();
        for (size_t i=1;i<line.points.size();++i) {
            const Point a=line.points[i-1],b=line.points[i];
            const coord_t y=scaled<coord_t>(20.);
            if ((a.y()<=y && b.y()>y)||(b.y()<=y && a.y()>y)) {
                const double x=unscaled<double>(a.x()+double(b.x()-a.x())*(y-a.y())/(b.y()-a.y()));
                if ((paint_zero && x<3.)||(!paint_zero && x>37.)) ordered.push_back(paint_zero?x:40.-x);
            }
        }
    }
    CAPTURE(ordered);
    REQUIRE(ordered.size()==3);
    // Distances from the boundary: deepest inner, exterior, second wall.
    REQUIRE(ordered[0]>ordered[2]);
    REQUIRE(ordered[2]>ordered[1]);
    REQUIRE(in_interval(wall_crossings(result.loops,20.),37.,40.)==(paint_zero?0:3));
}

TEST_CASE("First layer compensation retains painted wall counts", "[WallPainting][Review][Compensation]")
{
    const bool arachne=GENERATE(false,true);
    Model model; auto *object=model.add_object();
    auto *volume=object->add_volume(banded_box());
    TriangleSelector selector(volume->mesh());
    for (int face:{6,7,14,15,22,23}) selector.set_facet(face,EnforcerBlockerType::ENFORCER);
    volume->wall_painting_facets.set(selector);
    object->add_instance(); object->ensure_on_bed();
    auto config=wall_test_config();
    config.set_deserialize_strict({{"wall_loops","1"},{"painted_wall_loops","3"},
        {"elefant_foot_compensation","0.15"},{"only_one_wall_first_layer","0"},{"top_one_wall_type","not apply"},
        {"wall_generator",arachne?"arachne":"classic"}});
    Print print; print.set_status_silent(); print.auto_assign_extruders(object); print.apply(model,config);
    auto *printed=print.get_object(size_t(0)); print.process();
    const Layer *first=printed->layers().front();
    const double center_y=10.-unscaled<double>(printed->center_offset().y());
    const double right=20.-unscaled<double>(printed->center_offset().x());
    auto crossings=wall_crossings(first->get_region(0)->perimeters,center_y);
    CAPTURE(arachne,crossings,right);
    REQUIRE(in_interval(crossings,right-3.,right)==3);
}

TEST_CASE("Mirrored merges preserve the positions of subdivided wall paint", "[WallPainting][Review][Model]")
{
    const bool selected_merge=GENERATE(false,true);
    Model model; auto *object=model.add_object();
    auto *volume=object->add_volume(TriangleMesh(its_make_cube(10.,10.,10.)));
    TriangleSelector selector(volume->mesh());
    const auto &face=volume->mesh().its.indices.front();
    const auto &vertices=volume->mesh().its.vertices;
    const Vec3f center=0.2f*vertices[face[0]]+0.65f*vertices[face[1]]+0.15f*vertices[face[2]];
    selector.select_patch(0,TriangleSelector::SinglePointCursor::cursor_factory(center,center+Vec3f(0,0,20),0.6f,
        TriangleSelector::CursorType::SPHERE,Transform3d::Identity(),TriangleSelector::ClippingPlane()),
        EnforcerBlockerType::ENFORCER,Transform3d::Identity(),true);
    volume->wall_painting_facets.set(selector);
    REQUIRE(volume->wall_painting_facets.get_triangle_as_string(0).size()>1);
    volume->set_mirror(Vec3d(-1.,1.,1.));
    auto bounds=[](const ModelVolume &v) {
        BoundingBoxf3 box;
        const auto facets=v.wall_painting_facets.get_facets(v,EnforcerBlockerType::ENFORCER);
        for (const Vec3i &f:facets.indices) for (int i=0;i<3;++i)
            box.merge(v.get_matrix()*facets.vertices[f[i]].cast<double>());
        return box;
    };
    const auto before=bounds(*volume);
    auto *other=object->add_volume(TriangleMesh(its_make_cube(10.,10.,10.)));
    other->set_offset(Vec3d(20.,0.,0.));
    ModelObjectPtrs merged;
    if (selected_merge) {
        std::vector<int> selected{0,1}; merged=object->merge_volumes(selected);
        volume=merged.front()->volumes.front();
    } else { object->merge(); volume=object->volumes.front(); }
    const auto after=bounds(*volume);
    for (auto *item:merged) model.objects.push_back(item);
    CAPTURE(selected_merge,before.min,before.max,after.min,after.max);
    // Geometric reprojection uses the existing 0.2 mm subdivision floor.
    REQUIRE((before.min-after.min).norm()<0.2);
    REQUIRE((before.max-after.max).norm()<0.2);
}

TEST_CASE("Reloaded volumes preserve independent paint after face reordering", "[WallPainting][Review][Model]")
{
    const bool reordered=GENERATE(false,true);
    Model model; auto *object=model.add_object();
    auto *source=object->add_volume(TriangleMesh(its_make_cube(10.,10.,10.)));
    source->wall_painting_facets.set_triangle_from_string(0,"4");
    source->supported_facets.set_triangle_from_string(1,"8");
    auto mesh=source->mesh();
    if (reordered) std::reverse(mesh.its.indices.begin(),mesh.its.indices.end());
    auto *replacement=object->add_volume(std::move(mesh));
    replacement->set_transformation(source->get_transformation());
    CAPTURE(reordered,source->get_matrix().matrix(),replacement->get_matrix().matrix(),source->mesh().its.vertices.front(),replacement->mesh().its.vertices.front());
    REQUIRE(reproject_paint_between_volumes(*source,*replacement));
    std::vector<std::string> reloaded_masks;
    for (size_t i=0;i<replacement->mesh().its.indices.size();++i)
        reloaded_masks.push_back(replacement->wall_painting_facets.get_triangle_as_string(int(i)));
    CAPTURE(reloaded_masks);
    const int wall_face=reordered?int(source->mesh().its.indices.size()-1):0;
    const int support_face=reordered?wall_face-1:1;
    REQUIRE(replacement->wall_painting_facets.get_triangle_as_string(wall_face)=="4");
    REQUIRE(replacement->supported_facets.get_triangle_as_string(support_face)=="8");
    REQUIRE(replacement->seam_facets.empty());
    REQUIRE(replacement->fuzzy_skin_facets.empty());
}

TEST_CASE("Other wall rules apply after the painted count override", "[WallPainting][Perimeters]")
{
    const bool arachne=GENERATE(false,true);
    const int rule=GENERATE(0,1,2);
    CAPTURE(arachne,rule);
    ExPolygon section(rectangle(0,0,40,40));
    WallPainting::LayerPaint paint;
    for (size_t i=0;i<4;++i) paint.push_back({Line(section.contour[i],section.contour[(i+1)%4]),0,i==1,-1});
    GeneratedWalls result;
    generate_walls(section,paint,arachne,1,3,result,rule==1?0:3,rule==0,rule==1,rule==2);
    const auto crossings=wall_crossings(result.loops,20.);
    CAPTURE(crossings);
    REQUIRE(in_interval(crossings,0,3)==(rule==0?2:1));
    REQUIRE(in_interval(crossings,37,40)==(rule==0?4:1));
}

TEST_CASE("Part wall-count overrides label merged boundaries without interface walls", "[WallPainting][Slice][Settings]")
{
    const bool arachne=GENERATE(false,true);
    CAPTURE(arachne);
    Model model; auto *object=model.add_object(); object->name="painted-parts";
    for (int part=0;part<2;++part) {
        auto mesh=TriangleMesh(its_make_cube(20.,20.,2.)); mesh.translate(20.*part,0.,0.);
        auto *volume=object->add_volume(std::move(mesh));
        TriangleSelector selector(volume->mesh());
        for (size_t face=0;face<volume->mesh().its.indices.size();++face)
            selector.set_facet(int(face),EnforcerBlockerType::ENFORCER);
        volume->wall_painting_facets.set(selector);
        if (part==1) volume->config.set("painted_wall_loops",3);
    }
    object->add_instance(); object->ensure_on_bed();
    DynamicPrintConfig config=wall_test_config();
    config.set_deserialize_strict({{"layer_height","0.2"},{"wall_loops","1"},{"painted_wall_loops","2"},
                                  {"top_one_wall_type","not apply"},{"only_one_wall_first_layer","0"},
                                  {"wall_generator",arachne?"arachne":"classic"}});
    Print print; print.set_status_silent(); print.auto_assign_extruders(object); print.apply(model,config);
    print.validate(); print.process();
    auto *print_object=print.get_object(size_t(0));
    REQUIRE(print_object->layers().size()>3);
    const Layer *layer=print_object->layers()[3];
    std::vector<double> crossings;
    for (const auto *region:layer->regions()) {
        auto positions=wall_crossings(region->perimeters,0.);
        crossings.insert(crossings.end(),positions.begin(),positions.end());
    }
    CAPTURE(crossings);
    REQUIRE(in_interval(crossings,-20,-17)==2);
    REQUIRE(in_interval(crossings,17,20)==3);
    REQUIRE(in_interval(crossings,-2,2)==0);

    auto resliced_crossings=[&] {
        print.apply(model,config); print.process();
        std::vector<double> positions;
        for (const auto *region:print.get_object(size_t(0))->layers()[3]->regions()) {
            auto region_positions=wall_crossings(region->perimeters,0.);
            positions.insert(positions.end(),region_positions.begin(),region_positions.end());
        }
        return positions;
    };
    object->volumes[0]->wall_painting_facets.reset();
    crossings=resliced_crossings();
    REQUIRE(in_interval(crossings,-20,-17)==1);
    REQUIRE(in_interval(crossings,17,20)==3);
    object->volumes[1]->config.set("painted_wall_loops",4);
    crossings=resliced_crossings();
    REQUIRE(in_interval(crossings,-20,-17)==1);
    REQUIRE(in_interval(crossings,17,20)==4);
}

TEST_CASE("Numeric wall modifiers affect painted boundaries without interface walls", "[WallPainting][Slice][Settings]")
{
    const bool arachne=GENERATE(false,true);
    CAPTURE(arachne);
    Model model; auto *object=model.add_object();
    auto *volume=object->add_volume(TriangleMesh(its_make_cube(40.,20.,2.)));
    TriangleSelector selector(volume->mesh());
    for (size_t face=0;face<volume->mesh().its.indices.size();++face)
        selector.set_facet(int(face),EnforcerBlockerType::ENFORCER);
    volume->wall_painting_facets.set(selector);
    auto modifier_mesh=TriangleMesh(its_make_cube(20.,22.,4.));
    modifier_mesh.translate(20.,-1.,-1.);
    auto *modifier=object->add_volume(std::move(modifier_mesh),ModelVolumeType::PARAMETER_MODIFIER);
    modifier->config.set("painted_wall_loops",3);
    object->add_instance(); object->ensure_on_bed();
    auto config=wall_test_config();
    config.set_deserialize_strict({{"layer_height","0.2"},{"wall_loops","1"},{"painted_wall_loops","2"},
                                  {"top_one_wall_type","not apply"},{"only_one_wall_first_layer","0"},
                                  {"wall_generator",arachne?"arachne":"classic"}});
    Print print; print.set_status_silent(); print.auto_assign_extruders(object); print.apply(model,config);
    print.validate(); print.process();
    std::vector<double> crossings;
    for (const auto *region:print.get_object(size_t(0))->layers()[3]->regions()) {
        auto positions=wall_crossings(region->perimeters,0.);
        crossings.insert(crossings.end(),positions.begin(),positions.end());
    }
    CAPTURE(crossings);
    REQUIRE(in_interval(crossings,-20,-17)==2);
    REQUIRE(in_interval(crossings,17,20)==3);
    REQUIRE(in_interval(crossings,-2,2)==0);
}

#include "libslic3r/Triangulation.hpp"
#include <chrono>
#include <cstdlib>
#include <iostream>
TEST_CASE("Wall painting aircraft slicing benchmark and UI fixture", "[!hide][WallPaintingBenchmark]")
{
    const std::string mode=std::getenv("WALL_PAINT_BENCH_MODE")?std::getenv("WALL_PAINT_BENCH_MODE"):"arachne-painted";
    const bool arachne=mode.find("arachne")!=std::string::npos;
    const bool painted=mode.find("painted")!=std::string::npos;
    ExPolygon section;
    for (int i=0;i<64;++i) {
        const double angle=2.*PI*i/64.;
        section.contour.points.push_back(Point::new_scale(40.*std::cos(angle),10.*std::sin(angle)));
    }
    for (double x:{-25.,0.,25.}) {
        Polygon hole;
        for (int i=0;i<32;++i) {
            const double angle=-2.*PI*i/32.;
            hole.points.push_back(Point::new_scale(x+3.*std::cos(angle),3.*std::sin(angle)));
        }
        section.holes.push_back(std::move(hole));
    }
    const Points points=to_points(section);
    const int vertex_count=int(points.size());
    indexed_triangle_set mesh;
    for (float z:{0.f,30.f}) for (const Point &p:points)
        mesh.vertices.emplace_back(unscaled<float>(p.x()),unscaled<float>(p.y()),z);
    for (Vec3i triangle:Triangulation::triangulate(section)) {
        const auto &a=mesh.vertices[triangle[0]], &b=mesh.vertices[triangle[1]], &c=mesh.vertices[triangle[2]];
        if ((b-a).cross(c-a).z()<0.) std::swap(triangle[0],triangle[1]);
        mesh.indices.emplace_back(triangle[2],triangle[1],triangle[0]);
        mesh.indices.emplace_back(triangle[0]+vertex_count,triangle[1]+vertex_count,triangle[2]+vertex_count);
    }
    std::vector<int> painted_faces;
    int offset=0,contour=0;
    for (const auto &border:to_polygons(section)) {
        for (int i=0;i<int(border.size());++i) {
            const int a=offset+i,b=offset+(i+1)%int(border.size());
            if (contour==1 || contour==3) {
                painted_faces.push_back(int(mesh.indices.size()));
                painted_faces.push_back(int(mesh.indices.size())+1);
            }
            mesh.indices.emplace_back(a,b,b+vertex_count);
            mesh.indices.emplace_back(a,b+vertex_count,a+vertex_count);
        }
        offset+=int(border.size()); ++contour;
    }
    const char *artifact_directory=std::getenv("WALL_PAINT_ARTIFACT_DIR");
    if (artifact_directory) {
        boost::filesystem::create_directories(artifact_directory);
        set_temporary_dir(artifact_directory);
    }
    Model model; auto *object=model.add_object(); object->name="Aircraft wall painting";
    auto *volume=object->add_volume(TriangleMesh(std::move(mesh)));
    if (painted) {
        TriangleSelector selector(volume->mesh());
        for (int face:painted_faces) selector.set_facet(face,EnforcerBlockerType::ENFORCER);
        volume->wall_painting_facets.set(selector);
    }
    object->add_instance()->set_offset(Vec3d(100.,100.,0.)); object->ensure_on_bed();
    DynamicPrintConfig config=wall_test_config();
    config.set_deserialize_strict({{"wall_loops","1"},{"painted_wall_loops","2"},{"layer_height","0.2"},
                                  {"top_one_wall_type","not apply"},{"only_one_wall_first_layer","0"},
                                  {"wall_generator",arachne?"arachne":"classic"}});
    Print print; print.set_status_silent(); print.auto_assign_extruders(object); print.apply(model,config); print.validate();
    const auto start=std::chrono::steady_clock::now();
    print.process();
    const double elapsed=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    std::cout<<"Wall painting benchmark: "<<mode<<", slicing_ms="<<elapsed<<", layers="<<print.get_object(size_t(0))->layers().size()<<'\n';
    REQUIRE(print.get_object(size_t(0))->layers().size()>100);
    if (artifact_directory && painted) {
        DynamicPrintConfig project_config;
        project_config.apply(FullPrintConfig::defaults());
        project_config.set_deserialize_strict({{"wall_loops","1"},{"painted_wall_loops","2"},
                                               {"wall_generator",arachne?"arachne":"classic"}});
        const auto project=(boost::filesystem::path(artifact_directory)/(mode+".3mf")).string();
        PlateData plate; plate.plate_index=0; plate.objects_and_instances.emplace_back(0,0);
        StoreParams params; params.path=project.c_str(); params.model=&model; params.config=&project_config;
        params.plate_data_list.push_back(&plate);
        REQUIRE(store_bbs_3mf(params));
        const auto gcode=(boost::filesystem::path(artifact_directory)/(mode+".gcode")).string();
        GCodeProcessorResult result;
        print.export_gcode(gcode,&result);
        REQUIRE(boost::filesystem::file_size(gcode)>1000);
    }
}
