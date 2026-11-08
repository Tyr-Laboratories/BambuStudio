#include <catch2/catch.hpp>

#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Support/SupportCommon.hpp"
#include "libslic3r/Support/SupportLayer.hpp"

using namespace Slic3r;

namespace {
std::vector<size_t> interface_phase_ids(const char *style)
{
    Model model;
    ModelObject *model_object = model.add_object();
    model_object->add_volume(TriangleMesh(its_make_cube(1., 1., 1.)));
    model_object->add_instance();
    Print print;
    print.set_status_silent();
    print.auto_assign_extruders(model_object);
    DynamicPrintConfig config;
    config.apply(static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    config.set_deserialize_strict({
        {"layer_height", "0.2"}, {"initial_layer_print_height", "0.2"},
        {"enable_support", "1"}, {"support_type", "tree(auto)"},
        {"support_style", style}, {"support_top_z_distance", "0.2"}
    });
    print.apply(model, config);
    PrintObject &object = *print.get_object(size_t(0));
    object.slice();
    SupportGeneratorLayerStorage storage;
    SupportGeneratorLayersPtr bottoms, tops, interfaces;
    auto add = [&](SupportGeneratorLayersPtr &layers, SupporLayerType type, double z) {
        SupportGeneratorLayer &layer = storage.allocate(type);
        layer.print_z = z;
        layer.bottom_z = z - 0.2;
        layer.height = 0.2;
        const coord_t side = scaled<coord_t>(1.);
        layer.polygons = {Polygon({Point(coord_t(0), coord_t(0)), Point(side, coord_t(0)),
                                   Point(side, side), Point(coord_t(0), side)})};
        layers.push_back(&layer);
    };
    add(bottoms, sltBottomContact, 0.2);
    add(interfaces, sltBottomInterface, 0.4);
    // This top contact follows a bottom-only row and shares its own Z with
    // another bottom interface. Neither bottom row may determine its phase.
    add(tops, sltTopContact, 0.6);
    add(interfaces, sltBottomInterface, 0.6);
    add(interfaces, sltTopInterface, 0.8);
    add(tops, sltTopContact, 1.0);
    generate_support_layers(object, {}, bottoms, tops, {}, interfaces, {});
    std::vector<size_t> ids;
    for (const SupportLayer *layer : object.support_layers())
        ids.push_back(layer->interface_id());
    return ids;
}
}

TEST_CASE("Organic top contact phase is independent of adjacent and coincident bottom interfaces", "[OrganicFoundation][InterfacePhaseIds]")
{
    CHECK(interface_phase_ids("tree_organic") == std::vector<size_t>{0, 1, 1, 0, 1});
    // Default tree style resolves to organic and follows the same path.
    CHECK(interface_phase_ids("default") == std::vector<size_t>{0, 1, 1, 0, 1});
}

TEST_CASE("Classic tree interfaces retain their existing shared phase sequence", "[OrganicFoundation][InterfacePhaseIds]")
{
    CHECK(interface_phase_ids("tree_hybrid") == std::vector<size_t>{0, 1, 2, 3, 4});
}
