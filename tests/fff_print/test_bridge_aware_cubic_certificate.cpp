#include <catch2/catch.hpp>

#include "libslic3r/Fill/BridgeAwareCubicCertificate.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Print.hpp"

#include <stdexcept>

using namespace Slic3r;
namespace BAC = Slic3r::BridgeAwareCubic;

namespace {
struct SnapshotFixture {
    Model model;
    Print print;
    PrintObject *object = nullptr;
    LayerRegion *region = nullptr;
    std::vector<BAC::SerializedXY> phases{BAC::SerializedXY{}};

    SnapshotFixture()
    {
        auto *part = model.add_object();
        part->add_volume(TriangleMesh(its_make_cube(10., 10., 1.)));
        part->add_instance()->set_offset(Vec3d(40., 40., 0.));
        DynamicPrintConfig config;
        config.apply(FullPrintConfig::defaults());
        print.set_status_silent();
        print.auto_assign_extruders(part);
        print.apply(model, config);
        object = print.get_object(size_t(0));
        REQUIRE(object != nullptr);
        REQUIRE(object->num_printing_regions() == 1);
        Layer *layer = object->add_layer(0, .2, .2, .1);
        region = layer->add_region(&object->printing_region(0));
        const ExPolygon polygon(Polygon(Points{Point(0, 0), Point(1000, 0), Point(1000, 1000), Point(0, 1000)}));
        layer->lslices = {polygon};
        region->slices.append(ExPolygons{polygon}, stInternal);
        region->fill_surfaces.append(ExPolygons{polygon}, stInternal);
        auto *path = new ExtrusionPath(erInternalInfill, .08, .4f, .2f);
        path->polyline = Polyline(Points{Point(0, 0), Point(1000, 0)});
        region->fills.entities.push_back(path);
    }

    ExtrusionPath &path() { return *static_cast<ExtrusionPath *>(region->fills.entities.front()); }
    auto capture() const { return BAC::DepositionStateSnapshot::capture(*object, *object, phases); }
};
}

TEST_CASE("Bridge aware cubic snapshot detects exact path and omitted surface-copy fields", "[BridgeAwareCubic][BACCertificate]")
{
    SnapshotFixture fixture;
    const auto snapshot = fixture.capture();
    REQUIRE(snapshot);
    CHECK(snapshot->matches(*fixture.object, *fixture.object, fixture.phases));
    SECTION("width") { fixture.path().width = .41f; }
    SECTION("suppressed flow") { fixture.path().set_force_no_extrusion(true); }
    SECTION("point") { fixture.path().polyline.points.back().x() += 1; }
    SECTION("cooling metadata") { fixture.path().set_cooling_node(2); }
    SECTION("collection metadata") { fixture.region->fills.loop_node_range = {2, 3}; }
    SECTION("collection child boundary") {
        auto *nested = new ExtrusionEntityCollection;
        nested->entities.swap(fixture.region->fills.entities);
        fixture.region->fills.entities.push_back(nested);
    }
    SECTION("surface compensation") { fixture.region->slices.surfaces.front().counter_circle_compensation = true; }
    SECTION("hole compensation") { fixture.region->fill_surfaces.surfaces.front().holes_circle_compensation = {0}; }
    SECTION("bridge followup") { fixture.region->fill_surfaces.surfaces.front().bridge_followup = true; }
    SECTION("layer Z") { fixture.object->get_layer(0)->print_z += .001; }
    SECTION("resolved gap") {
        const_cast<PrintRegionConfig &>(fixture.object->printing_region(0).config()).bridge_aware_cubic_max_gap.value += .001;
    }
    SECTION("full nozzle vector") {
        const_cast<PrintConfig &>(fixture.print.config()).extruder_offset.values.push_back(Vec2d::Zero());
    }
    SECTION("nullable byte vector") {
        auto &config = const_cast<PrintConfig &>(fixture.print.config());
        bool changed = false;
        for (const auto &key : config.keys_ref()) {
            auto *option = config.optptr(key);
            if (option->type() == coBools && option->nullable()) {
                auto *vector = dynamic_cast<ConfigOptionVector<unsigned char> *>(option);
                REQUIRE(vector != nullptr);
                vector->values = {255, 1};
                changed = true;
                break;
            }
        }
        REQUIRE(changed);
    }
    CHECK_FALSE(snapshot->matches(*fixture.object, *fixture.object, fixture.phases));
}

TEST_CASE("Bridge aware cubic snapshot phases are component exact and require coverage", "[BridgeAwareCubic][BACCertificate]")
{
    SnapshotFixture fixture;
    fixture.phases.push_back({Vec2d(.0005, 0.), Vec2d::Zero(), Vec2d::Zero()});
    const auto snapshot = fixture.capture();
    REQUIRE(snapshot);
    fixture.phases.erase(fixture.phases.begin());
    CHECK(snapshot->matches(*fixture.object, *fixture.object, fixture.phases));
    fixture.phases.front().shift.x() += 1.;
    fixture.phases.front().nozzle_offset.x() += 1.;
    CHECK_FALSE(snapshot->matches(*fixture.object, *fixture.object, fixture.phases));
    fixture.phases.clear();
    CHECK_FALSE(snapshot->matches(*fixture.object, *fixture.object, fixture.phases));
}

TEST_CASE("Bridge aware cubic snapshot declines unsupported trees and bounded admission", "[BridgeAwareCubic][BACCertificate]")
{
    SnapshotFixture fixture;
    SECTION("byte cap") {
        BAC::DepositionStateSnapshot::Limits limits;
        limits.bytes = 1;
        CHECK_FALSE(BAC::DepositionStateSnapshot::capture(*fixture.object, *fixture.object, fixture.phases, {}, limits));
    }
    SECTION("item cap") {
        BAC::DepositionStateSnapshot::Limits limits;
        limits.items = 1;
        CHECK_FALSE(BAC::DepositionStateSnapshot::capture(*fixture.object, *fixture.object, fixture.phases, {}, limits));
    }
    SECTION("sloped path, even if suppressed") {
        const auto snapshot = fixture.capture();
        REQUIRE(snapshot);
        auto *sloped = new ExtrusionPathSloped(fixture.path(), {}, {});
        sloped->set_force_no_extrusion(true);
        fixture.region->fills.entities.push_back(sloped);
        CHECK_FALSE(fixture.capture());
        CHECK_FALSE(snapshot->matches(*fixture.object, *fixture.object, fixture.phases));
    }
    SECTION("fitting metadata on a suppressed path") {
        fixture.path().set_force_no_extrusion(true);
        const auto snapshot = fixture.capture();
        REQUIRE(snapshot);
        fixture.path().polyline.fitting_result.emplace_back();
        CHECK_FALSE(fixture.capture());
        CHECK_FALSE(snapshot->matches(*fixture.object, *fixture.object, fixture.phases));
    }
    SECTION("foreign region binding") {
        PrintRegion foreign(PrintRegionConfig(fixture.object->printing_region(0).config()));
        fixture.object->get_layer(0)->add_region(&foreign);
        CHECK_FALSE(fixture.capture());
    }
}

TEST_CASE("Bridge aware cubic snapshot honors cancellation on a warm match", "[BridgeAwareCubic][BACCertificate]")
{
    SnapshotFixture fixture;
    const auto snapshot = fixture.capture();
    REQUIRE(snapshot);
    const auto cancel = [] { throw std::runtime_error("certificate canceled"); };
    CHECK_THROWS_WITH(snapshot->matches(*fixture.object, *fixture.object, fixture.phases, cancel), "certificate canceled");
    CHECK_THROWS_WITH(BAC::DepositionStateSnapshot::capture(*fixture.object, *fixture.object, fixture.phases, cancel), "certificate canceled");
}
