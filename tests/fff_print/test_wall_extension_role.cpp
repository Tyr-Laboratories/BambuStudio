#include <catch2/catch.hpp>
#include "libslic3r/Print.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"

#include <boost/filesystem.hpp>
#include <algorithm>
#include <cmath>
#include <set>

using namespace Slic3r;

namespace {
struct RoleWorkspace {
    boost::filesystem::path path = boost::filesystem::temp_directory_path() /
        boost::filesystem::unique_path("wall-extension-role-%%%%-%%%%");
    std::string old_data = data_dir(), old_temp = temporary_dir();
    RoleWorkspace() {
        boost::filesystem::create_directories(path);
        set_data_dir(path.string());
        set_temporary_dir(path.string());
    }
    ~RoleWorkspace() {
        set_data_dir(old_data);
        set_temporary_dir(old_temp);
        boost::system::error_code ec;
        boost::filesystem::remove_all(path, ec);
    }
};

// A closed square loop of role erWallExtension with the given flow, centred on `centre`.
ExtrusionLoop square_loop(const Point &centre, double size_mm, const Flow &flow)
{
    const coord_t h = scaled<coord_t>(size_mm / 2.);
    ExtrusionPath path(erWallExtension, flow.mm3_per_mm(), flow.width(), flow.height());
    path.polyline.points = { centre + Point(-h, -h), centre + Point(h, -h), centre + Point(h, h),
                             centre + Point(-h, h), centre + Point(-h, -h) };
    ExtrusionLoop loop;
    loop.paths.emplace_back(std::move(path));
    return loop;
}
}

TEST_CASE("Wall extension is its own line type", "[WallExtension][Role]")
{
    CHECK(ExtrusionEntity::role_to_string(erWallExtension) == "Wall extension");
    CHECK(ExtrusionEntity::string_to_role("Wall extension") == erWallExtension);
    CHECK_FALSE(is_perimeter(erWallExtension));
    CHECK_FALSE(is_infill(erWallExtension));
    CHECK_FALSE(is_bridge(erWallExtension));
    // Appended after the last printed role, so roles stored by number elsewhere keep their values.
    CHECK(int(erWallExtension) == int(erFlush) + 1);
    CHECK(int(erMixed) == int(erWallExtension) + 1);
}

TEST_CASE("Wall extension lines print at inner-wall settings", "[WallExtension][Role][GCode]")
{
    RoleWorkspace workspace;
    DynamicPrintConfig config;
    config.apply(static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    config.set_deserialize_strict({
        {"layer_height", "0.2"}, {"initial_layer_print_height", "0.2"},
        {"wall_internal_extension", "0"}, {"wall_generator", "classic"},
        {"inner_wall_speed", "77"}, {"outer_wall_speed", "33"}, {"bridge_speed", "23"},
        {"inner_wall_acceleration", "900"}, {"bridge_acceleration", "450"}, {"default_acceleration", "2000"},
        {"machine_max_acceleration_extruding", "10000"},
        {"filament_max_volumetric_speed", "100"}, {"slow_down_for_layer_cooling", "0"},
        {"enable_arc_fitting", "0"}, {"brim_type", "no_brim"}
    });
    Model model;
    auto *object = model.add_object();
    object->add_volume(TriangleMesh(its_make_cube(20., 20., 6.)));
    object->add_instance()->set_offset(Vec3d(80., 80., 0.));
    Print print;
    print.set_status_silent();
    print.auto_assign_extruders(model.objects.front());
    print.apply(model, config);
    print.process();

    PrintObject *print_object = print.get_object(size_t(0));
    Layer *layer = nullptr;
    for (Layer *candidate : print_object->layers())
        if (std::abs(candidate->print_z - 3.2) < EPSILON)
            layer = candidate;
    REQUIRE(layer != nullptr);
    LayerRegion *region = layer->get_region(0);
    const Point centre = get_extents(layer->lslices).center();
    const Flow flow = region->flow(frExternalPerimeter);
    ExtrusionEntityCollection extension;
    extension.append(square_loop(centre, 8., flow));
    region->perimeters.append(extension);

    const auto filename = (workspace.path / "wall-extension.gcode").string();
    GCodeProcessorResult result;
    print.export_gcode(filename, &result);

    std::set<int> speeds, accelerations;
    GCodeReader reader;
    reader.apply_config(config);
    std::string feature;
    int acceleration = 0;
    REQUIRE(reader.parse_file(filename, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        const auto tag = line.comment().find(" FEATURE: ");
        if (tag != std::string_view::npos)
            feature = std::string(line.comment().substr(tag + 10));
        if (line.cmd_is("M204")) {
            float value;
            if (line.has_value('S', value) || line.has_value('P', value))
                acceleration = int(value);
        }
        if (feature != "Wall extension" || !line.extruding(self) || line.dist_XY(self) <= 0.)
            return;
        speeds.insert(int(std::lround(line.new_F(self))));
        accelerations.insert(acceleration);
    }));
    CHECK(speeds == std::set<int>{ 77 * 60 });
    CHECK(accelerations == std::set<int>{ 900 });

    const auto &roles_times = result.print_statistics.modes[0].roles_times;
    const auto  it = std::find_if(roles_times.begin(), roles_times.end(),
                                  [](const std::pair<ExtrusionRole, float> &item) { return item.first == erWallExtension; });
    REQUIRE(it != roles_times.end());
    CHECK(it->second > 0.f);
    REQUIRE(result.print_statistics.used_filaments_per_role.count(erWallExtension) == 1);
    CHECK(result.print_statistics.used_filaments_per_role.at(erWallExtension).first > 0.);
}
