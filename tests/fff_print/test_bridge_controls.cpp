#include <catch2/catch.hpp>
#include "libslic3r/Print.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/Utils.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Preset.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"
#include "libslic3r/Fill/FillBase.hpp"
#include <boost/filesystem.hpp>
#include <map>
#include <set>
#include <tuple>

using namespace Slic3r;
namespace {
DynamicPrintConfig bridge_controls_config(double density=100., double extend=15., int layers=1)
{
    DynamicPrintConfig config;
    config.apply(static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    config.set_deserialize_strict({
        {"layer_height","0.2"},{"initial_layer_print_height","0.2"},
        {"sparse_infill_density","15%"},{"sparse_infill_pattern","grid"},
        {"top_shell_layers","4"},{"top_shell_thickness","0"},
        {"bottom_shell_layers","3"},{"bottom_shell_thickness","0"},
        {"detect_narrow_internal_solid_infill","0"},{"brim_type","no_brim"},
        {"enable_arc_fitting","0"},{"wall_generator","classic"},
        {"thick_bridges","1"},{"bridge_flow","1"}
    });
    config.set_key_value("bridge_layer_density", new ConfigOptionPercent(density));
    config.set_key_value("bridge_line_travel_extend", new ConfigOptionPercent(extend));
    config.set_key_value("internal_bridge_layers", new ConfigOptionInt(layers));
    return config;
}
Model bridge_controls_model(bool internal=true)
{
    Model model;
    auto *object=model.add_object();
    TriangleMesh mesh(its_make_cube(internal ? 30. : 6.,20.,8.));
    if (!internal) {
        TriangleMesh right(its_make_cube(6.,20.,8.)); right.translate(24.,0.,0.); mesh.merge(right);
        TriangleMesh roof(its_make_cube(30.,20.,1.2)); roof.translate(0.,0.,8.); mesh.merge(roof);
    }
    object->add_volume(mesh); object->add_instance()->set_offset(Vec3d(80.,80.,0.));
    return model;
}
void start_print(Print &print, Model &model, const DynamicPrintConfig &config)
{
    print.set_status_silent(); print.auto_assign_extruders(model.objects.front()); print.apply(model,config); print.process();
}
void collect_paths(const ExtrusionEntity &entity, std::vector<ExtrusionPath> &paths)
{
    if (const auto *path=dynamic_cast<const ExtrusionPath *>(&entity)) paths.push_back(*path);
    else if (const auto *loop=dynamic_cast<const ExtrusionLoop *>(&entity))
        paths.insert(paths.end(),loop->paths.begin(),loop->paths.end());
    else if (const auto *multi=dynamic_cast<const ExtrusionMultiPath *>(&entity))
        paths.insert(paths.end(),multi->paths.begin(),multi->paths.end());
    else if (const auto *collection=dynamic_cast<const ExtrusionEntityCollection *>(&entity))
        for(const auto *child:collection->entities) collect_paths(*child,paths);
}
ExPolygon rectangle(double xmin,double ymin,double xmax,double ymax)
{
    return ExPolygon(Polygon({Point(scale_(xmin),scale_(ymin)),Point(scale_(xmax),scale_(ymin)),
        Point(scale_(xmax),scale_(ymax)),Point(scale_(xmin),scale_(ymax))}));
}
struct Strand {double low,high,across,width,height,mm3;};
std::vector<Strand> rectangular_bridge(double density,double extend, bool thick=true, bool internal=false, bool hole=false, bool split=false, double angle=0., bool tight=false)
{
    auto config=bridge_controls_config(density,extend);
    config.set_deserialize_strict({{"thick_bridges",thick?"1":"0"}});
    auto model=bridge_controls_model(); Print print; start_print(print,model,config);
    auto *layer=print.get_object(size_t(0))->get_layer(5);
    auto *region=layer->get_region(0);
    auto material=tight ? rectangle(-10.05,-8.,10.05,8.) : rectangle(-12.,-8.,12.,8.);
    if(hole) { auto opening=rectangle(10.05,-3.,11.5,3.); opening.contour.make_clockwise(); material.holes.push_back(opening.contour); }
    material.rotate(angle);
    region->slices.surfaces={Surface(stInternal,material)};
    Surface bridge(internal?stInternalBridge:stBottomBridge,rectangle(-10.,-6.13,10.,6.13));
    bridge.bridge_angle=angle;
    bridge.expolygon.rotate(angle);
    region->fill_surfaces.surfaces={bridge};
    if(split) {
        Surface left(bridge,rectangle(-10.,-6.13,-0.15,6.13)), right(bridge,rectangle(0.15,-6.13,10.,6.13));
        left.expolygon.rotate(angle); right.expolygon.rotate(angle);
        region->fill_surfaces.surfaces={left,right};
    }
    region->fill_no_overlap_expolygons={bridge.expolygon};
    region->fill_expolygons={bridge.expolygon};
    layer->make_fills();
    std::vector<ExtrusionPath> paths; collect_paths(region->fills,paths);
    std::vector<Strand> strands;
    for(auto path:paths) {
        REQUIRE(path.role()==erBridgeInfill);
        path.polyline.rotate(-angle);
        for(const auto &line:path.polyline.lines())
            if(std::abs(line.a.y()-line.b.y())<scale_(0.0001) && line.length()>scale_(split?0.1:10.))
                strands.push_back({unscale<double>(std::min(line.a.x(),line.b.x())),unscale<double>(std::max(line.a.x(),line.b.x())),
                    unscale<double>(line.a.y()),path.width,path.height,path.mm3_per_mm});
    }
    std::sort(strands.begin(),strands.end(),[](const Strand &a,const Strand &b){return a.across<b.across;});
    REQUIRE_FALSE(strands.empty());
    return strands;
}
std::map<size_t,ExPolygons> surface_layers(const Print &print,SurfaceType type)
{
    std::map<size_t,ExPolygons> result;
    for(const auto *layer:print.get_object(size_t(0))->layers())
        for(const auto *region:layer->regions())
            for(const auto &surface:region->fill_surfaces.surfaces)
                if(surface.surface_type==type) result[layer->id()].push_back(surface.expolygon);
    return result;
}
void same_surfaces(const std::map<size_t,ExPolygons> &a,const std::map<size_t,ExPolygons> &b)
{
    REQUIRE(a.size()==b.size());
    for(const auto &[layer,polys]:a) {
        REQUIRE(b.count(layer)==1);
        CHECK(diff_ex(polys,b.at(layer)).empty()); CHECK(diff_ex(b.at(layer),polys).empty());
    }
}
}

TEST_CASE("Bridge density changes pitch without changing strands", "[BridgeControls][Density]")
{
    const bool thick=GENERATE(false,true);
    const bool internal=GENERATE(false,true);
    auto full=rectangular_bridge(100.,15.,thick,internal);
    auto half=rectangular_bridge(50.,15.,thick,internal);
    REQUIRE(half.size()<full.size()*0.6);
    REQUIRE(half.size()>full.size()*0.4);
    REQUIRE(half.size()>2);
    CHECK(half[1].across-half[0].across==Approx(2.*(full[1].across-full[0].across)).margin(0.00003));
    for(const auto &line:half) {
        CHECK(line.width==full.front().width); CHECK(line.height==full.front().height); CHECK(line.mm3==full.front().mm3);
    }
}
TEST_CASE("Additional internal bridges preserve configured solid shell planes", "[BridgeControls][Layers]")
{
    const int count=GENERATE(2,3);
    auto baseline_config=bridge_controls_config(); auto model=bridge_controls_model(); Print baseline; start_print(baseline,model,baseline_config);
    auto config=bridge_controls_config(100.,15.,count); Print expanded; start_print(expanded,model,config);
    auto before=surface_layers(baseline,stInternalBridge), after=surface_layers(expanded,stInternalBridge);
    REQUIRE(before.size()==1);
    REQUIRE(after.size()==size_t(count));
    CHECK(after.rbegin()->first==before.begin()->first);
    CHECK(after.begin()->first+count-1==before.begin()->first);
    for(auto type:{stTop,stBottom,stBottomBridge,stSubTop}) same_surfaces(surface_layers(baseline,type),surface_layers(expanded,type));
    // The normal bridge algorithm can add ensuring strips under a relocated
    // base. Every previously configured solid shell polygon must remain.
    const auto old_solids=surface_layers(baseline,stInternalSolid);
    const auto new_solids=surface_layers(expanded,stInternalSolid);
    for(const auto &[index,areas]:old_solids) {
        REQUIRE(new_solids.count(index));
        CHECK(diff_ex(areas,new_solids.at(index)).empty());
    }
}
TEST_CASE("Bridge controls are scalar process options with bounded defaults", "[BridgeControls][Config]")
{
    DynamicPrintConfig config; config.apply(static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    REQUIRE(config.has("bridge_layer_density")); REQUIRE(config.has("bridge_line_travel_extend")); REQUIRE(config.has("internal_bridge_layers"));
    CHECK(config.opt_float("bridge_layer_density")==100.); CHECK(config.opt_float("bridge_line_travel_extend")==15.); CHECK(config.opt_int("internal_bridge_layers")==1);
}

TEST_CASE("Minimum bridge density retains a strand in a small region", "[BridgeControls][Density]")
{
    auto full=rectangular_bridge(100.,15.);
    auto minimum=rectangular_bridge(1.,15.);
    REQUIRE(minimum.size()==1);
    CHECK(minimum.front().width==full.front().width);
    CHECK(minimum.front().mm3==full.front().mm3);
}

TEST_CASE("Bridge stacks use thick flow only on their bottom plane", "[BridgeControls][Layers]")
{
    auto config=bridge_controls_config(100.,15.,3);
    const std::string generator=GENERATE("classic","arachne");
    const double height=GENERATE(0.1,0.2,0.3);
    CAPTURE(generator,height);
    config.set_deserialize_strict({{"wall_generator",generator}});
    config.set_key_value("layer_height",new ConfigOptionFloat(height));
    config.set_key_value("initial_layer_print_height",new ConfigOptionFloat(height));
    auto model=bridge_controls_model(); Print print; start_print(print,model,config);
    auto bridges=surface_layers(print,stInternalBridge);
    REQUIRE(bridges.size()==3);
    for(const auto &[index,areas]:bridges) {
        const auto *layer=print.get_object(size_t(0))->get_layer(int(index));
        std::vector<ExtrusionPath> paths;
        for(const auto *region:layer->regions()) collect_paths(region->fills,paths);
        bool found=false;
        for(const auto &path:paths) if(path.role()==erBridgeInfill) {
            found=true;
            if(index==bridges.begin()->first) CHECK(path.height>layer->height);
            else CHECK(path.height==Approx(layer->height));
        }
        CHECK(found);
    }
}

TEST_CASE("Bridge stacks leave external bottom bridges unchanged", "[BridgeControls][Layers]")
{
    auto config=bridge_controls_config(100.,15.,1); auto model=bridge_controls_model(false);
    Print baseline; start_print(baseline,model,config);
    auto before=surface_layers(baseline,stBottomBridge); REQUIRE_FALSE(before.empty());
    config.set_key_value("internal_bridge_layers",new ConfigOptionInt(3));
    Print expanded; start_print(expanded,model,config);
    same_surfaces(before,surface_layers(expanded,stBottomBridge));
}

TEST_CASE("Changing internal bridge count on a sliced job rebuilds its classification", "[BridgeControls][Layers][Reslice]")
{
    auto config=bridge_controls_config(100.,15.,1); auto model=bridge_controls_model();
    Print print; start_print(print,model,config);
    const auto original=surface_layers(print,stInternalBridge);
    for(int count:{3,2,1}) {
        config.set_key_value("internal_bridge_layers",new ConfigOptionInt(count));
        print.apply(model,config); REQUIRE_FALSE(print.get_object(size_t(0))->is_step_done(posPrepareInfill));
        print.process();
        CHECK(surface_layers(print,stInternalBridge).size()==size_t(count));
        if(count==1) same_surfaces(original,surface_layers(print,stInternalBridge));
    }
}

TEST_CASE("Flow above internal bridges changes only the covered length of a solid fill path",
          "[BridgeControls][FlowAbove]")
{
    ExPolygon internal = rectangle(-8., -5., 8., 5.);
    ExPolygon external = rectangle(-3., -2., 3., 2.);
    internal.holes.push_back(external.contour);
    internal.holes.back().make_clockwise();

    // The solid fill crosses an internal bridge, ordinary material, and
    // the external bridge in its central opening, all at the same height.
    auto paths_at_flow = [&](double percent) {
        auto config = bridge_controls_config();
        config.set_deserialize_strict({{"flow_above_internal_bridges", std::to_string(percent) + "%"},
                                       {"internal_solid_infill_pattern", "monotonic"}});
        Model model = bridge_controls_model();
        Print print;
        start_print(print, model, config);
        Layer *layer = print.get_object(size_t(0))->get_layer(5);
        layer->lower_layer->get_region(0)->fill_surfaces.surfaces = {
            Surface(stInternalBridge, internal), Surface(stBottomBridge, external)};
        LayerRegion *region = layer->get_region(0);
        const ExPolygon solid = rectangle(-12., -8., 12., 8.);
        region->fill_surfaces.surfaces = {Surface(stInternalSolid, solid)};
        region->fill_expolygons = region->fill_no_overlap_expolygons = {solid};
        layer->make_fills();
        std::vector<ExtrusionPath> paths;
        collect_paths(region->fills, paths);
        return paths;
    };

    const auto baseline = paths_at_flow(100.);
    REQUIRE_FALSE(baseline.empty());
    const double nominal = baseline.front().mm3_per_mm;
    REQUIRE(nominal > 0.);
    double baseline_length = 0.;
    for (const ExtrusionPath &path : baseline) {
        REQUIRE(path.mm3_per_mm == Approx(nominal));
        baseline_length += unscale<double>(path.polyline.length());
    }

    const double percent = GENERATE(80., 137.5);
    double inside_length = 0., outside_length = 0., external_length = 0.;
    for (const ExtrusionPath &path : paths_at_flow(percent)) {
        REQUIRE(path.role() == erSolidInfill);
        const double length = unscale<double>(path.polyline.length());
        const double inside = unscale<double>(total_length(intersection_pl(path.polyline, internal)));
        if (inside > 0.001)
            CHECK(path.mm3_per_mm == Approx(nominal * percent / 100.));
        if (length - inside > 0.001)
            CHECK(path.mm3_per_mm == Approx(nominal));
        inside_length += inside;
        outside_length += length - inside;
        external_length += unscale<double>(total_length(intersection_pl(path.polyline, external)));
    }
    CHECK(inside_length > 10.);
    CHECK(outside_length > 10.);
    CHECK(external_length > 10.);
    CHECK(inside_length + outside_length == Approx(baseline_length).margin(0.001));
}

TEST_CASE("Reslicing changes flow directly above the top of an internal bridge stack",
          "[BridgeControls][FlowAbove][Reslice]")
{
    const int count = GENERATE(1, 3);
    auto config = bridge_controls_config(100., 15., count);
    config.set_deserialize_strict({{"use_relative_e_distances", "1"}});
    Model model = bridge_controls_model();
    Print print;
    start_print(print, model, config);
    const auto bridges = surface_layers(print, stInternalBridge);
    REQUIRE(bridges.size() == size_t(count));
    const size_t above_stack = bridges.rbegin()->first + 1;

    // Compare every layer and extrusion role. In particular, a follow-up
    // bridge above a bridge must keep its own bridge flow.
    const auto directory = boost::filesystem::temp_directory_path() / boost::filesystem::unique_path("bridge-flow-%%%%-%%%%");
    boost::filesystem::create_directories(directory);
    ScopeGuard cleanup([&] {
        boost::system::error_code ec;
        boost::filesystem::remove_all(directory, ec);
    });
    auto extruded_filament = [&] {
        const std::string path = (directory / "flow.gcode").string();
        GCodeProcessorResult preview;
        print.export_gcode(path, &preview);
        std::map<std::pair<int, std::string>, double> result;
        int layer = -1;
        std::string role;
        GCodeReader reader;
        reader.apply_config(config);
        REQUIRE(reader.parse_file(path, [&](GCodeReader &state, const GCodeReader::GCodeLine &line) {
            const auto comment = line.comment();
            if (comment == " CHANGE_LAYER")
                ++layer;
            else if (comment.substr(0, 10) == " FEATURE: ")
                role = comment.substr(10);
            if (layer >= 0 && line.extruding(state) && line.dist_XY(state) > 0.)
                result[{layer, role}] += line.dist_E(state);
        }));
        return result;
    };

    const auto baseline = extruded_filament();
    const auto above = std::make_pair(int(above_stack), std::string("Internal solid infill"));
    REQUIRE(baseline.count(above) == 1);
    REQUIRE(baseline.at(above) > 0.);
    for (int percent : {150, 100}) {
        config.set_deserialize_strict({{"flow_above_internal_bridges", std::to_string(percent) + "%"}});
        print.apply(model, config);
        REQUIRE_FALSE(print.get_object(size_t(0))->is_step_done(posInfill));
        print.process();
        const auto actual = extruded_filament();
        REQUIRE(actual.size() == baseline.size());
        for (const auto &[key, filament] : baseline) {
            REQUIRE(actual.count(key) == 1);
            const double factor = key == above ? percent / 100. : 1.;
            CHECK(actual.at(key) == Approx(filament * factor).margin(0.0005));
        }
    }
}

TEST_CASE("Flow above an internal bridge follows each receiving region's override",
          "[BridgeControls][FlowAbove][Modifiers]")
{
    auto config = bridge_controls_config();
    config.set_deserialize_strict({{"internal_solid_infill_pattern", "monotonic"}});
    Model model = bridge_controls_model();
    ModelObject *object = model.objects.front();
    TriangleMesh mesh(its_make_cube(20., 30., 10.));
    mesh.translate(-5., -5., -1.);
    ModelVolume *modifier = object->add_volume(mesh);
    modifier->set_type(ModelVolumeType::PARAMETER_MODIFIER);
    modifier->config.set_key_value("flow_above_internal_bridges", new ConfigOptionPercent(135.));
    Print print;
    start_print(print, model, config);
    Layer *layer = print.get_object(size_t(0))->get_layer(5);
    REQUIRE(layer->regions().size() == 2);

    // A single lower-region bridge spans both upper regions, as at the
    // beginning of a height-range modifier. Both use the same infill pattern.
    const ExPolygon whole = rectangle(-12., -8., 12., 8.);
    for (LayerRegion *region : layer->lower_layer->regions())
        region->fill_surfaces.clear();
    layer->lower_layer->get_region(0)->fill_surfaces.surfaces = {Surface(stInternalBridge, whole)};
    for (LayerRegion *region : layer->regions()) {
        const auto *percent = region->region().config().option<ConfigOptionPercent>("flow_above_internal_bridges");
        REQUIRE(percent != nullptr);
        const ExPolygon side = percent->value == 100. ? rectangle(-12., -8., 0., 8.) : rectangle(0., -8., 12., 8.);
        region->slices.surfaces = {Surface(stInternal, side)};
        region->fill_surfaces.surfaces = {Surface(stInternalSolid, side)};
        region->fill_expolygons = region->fill_no_overlap_expolygons = {side};
    }
    layer->make_fills();
    std::vector<ExtrusionPath> paths;
    for (const LayerRegion *region : layer->regions())
        collect_paths(region->fills, paths);
    std::set<double> left, right;
    for (const ExtrusionPath &path : paths) {
        for (const Line &line : path.polyline.lines()) {
            if (line.midpoint().x() < -scale_(1.))
                left.insert(path.mm3_per_mm);
            if (line.midpoint().x() > scale_(1.))
                right.insert(path.mm3_per_mm);
        }
    }
    REQUIRE(left.size() == 1);
    REQUIRE(right.size() == 1);
    CHECK(*right.begin() == Approx(*left.begin() * 1.35));
}
