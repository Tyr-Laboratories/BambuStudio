#include <catch2/catch.hpp>

#include "libslic3r/Fill/BridgeAwareCubicModelCollection.hpp"
#include "libslic3r/Fill/BridgeAwareCubicValidation.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Print.hpp"

#include <new>
#include <stdexcept>

using namespace Slic3r;
namespace BAC = Slic3r::BridgeAwareCubic;
namespace Validation = Slic3r::BridgeAwareCubicValidation;

namespace {
using Inputs = BAC::ModelCollectionInputs;
using Match = Inputs::Match;

ExPolygon collection_box(double x = 0.)
{
    return ExPolygon(Polygon(Points{Point::new_scale(x, 0.), Point::new_scale(x + 10., 0.),
        Point::new_scale(x + 10., 10.), Point::new_scale(x, 10.)}));
}

ExtrusionPath *collection_path(ExtrusionRole role, double x = 1., double y = 2.)
{
    auto *path = new ExtrusionPath(role, .08, .4f, .2f);
    path->polyline = Polyline::new_scale({Vec2d(x, y), Vec2d(x + 2., y)});
    return path;
}

// Real PrintObject/Layer/region/entity inputs, constructed without slicing.
// Layer4 has a model strand above a two-plane bridge ancestry; earlier layers
// also provide empty model streams and mutable ordinary sparse deposition.
struct CollectionFixture {
    Model model;
    Print print;
    PrintObject *object = nullptr;
    std::vector<LayerRegion *> regions;
    const std::set<size_t> checked{4};

    CollectionFixture()
    {
        ModelObject *part = model.add_object();
        part->add_volume(TriangleMesh(its_make_cube(10., 10., 1.)));
        part->add_instance()->set_offset(Vec3d(40., 40., 0.));
        DynamicPrintConfig config;
        config.apply(FullPrintConfig::defaults());
        config.set_key_value("sparse_infill_pattern", new ConfigOptionEnum<InfillPattern>(ipBridgeAwareCubic));
        config.set_key_value("sparse_infill_density", new ConfigOptionPercent(15.));
        print.set_status_silent();
        print.auto_assign_extruders(part);
        print.apply(model, config);
        object = print.get_object(size_t(0));
        REQUIRE(object != nullptr);
        REQUIRE(object->num_printing_regions() == 1);
        for (size_t i = 0; i < 5; ++i) {
            Layer *layer = object->add_layer(int(i), .2, .2 * (i + 1), .2 * i + .1);
            LayerRegion *region = layer->add_region(&object->printing_region(0));
            regions.push_back(region);
            layer->lslices = {collection_box()};
            region->slices.append(ExPolygons{collection_box()}, stInternal);
            region->fill_surfaces.append(ExPolygons{collection_box()},
                i == 2 || i == 3 ? stInternalBridge : stInternal);
            if (i == 3) region->fill_surfaces.surfaces.front().bridge_followup = true;
            region->fills.entities.push_back(collection_path(i == 4 ? erSolidInfill : erInternalInfill));
        }
    }

    ExtrusionPath &path(size_t layer = 4)
    { return *static_cast<ExtrusionPath *>(regions.at(layer)->fills.entities.front()); }
    PrintRegionConfig &config()
    { return const_cast<PrintRegionConfig &>(object->printing_region(0).config()); }
    auto capture() const { return Inputs::capture(*object); }
};
}

TEST_CASE("Model collection identity owns values and ignores changed lower deposition", "[BridgeAwareCubic][ModelCollectionInputs]")
{
    CollectionFixture fixture;
    const auto inputs = fixture.capture();
    REQUIRE(inputs);
    REQUIRE(inputs->matches(*fixture.object, fixture.checked) == Match::Equal);
    // A fresh equal entity must hit; an address/generation-only key would miss.
    ExtrusionEntity *original = fixture.regions[4]->fills.entities.front();
    fixture.regions[4]->fills.entities.front() = original->clone();
    delete original;
    CHECK(inputs->matches(*fixture.object, fixture.checked) == Match::Equal);
    // Lower deposited geometry is not a full model-collection input. The caller
    // still must run its current physical proof after this identity hit.
    fixture.path(3).polyline.points.back().x() += scale_(2.);
    fixture.path(3).width = .8f;
    CHECK(inputs->matches(*fixture.object, fixture.checked) == Match::Equal);
    CHECK(inputs->owned_usage().bytes > 0);
    CHECK(inputs->owned_usage().vertices > 0);
    CHECK(inputs->owned_usage().items > 0);
}

TEST_CASE("Model collection identity rechecks remote ancestry and source material", "[BridgeAwareCubic][ModelCollectionInputs]")
{
    CollectionFixture fixture;
    const auto inputs = fixture.capture();
    REQUIRE(inputs);
    SECTION("base becomes another followup") { fixture.regions[2]->fill_surfaces.surfaces.front().bridge_followup = true; }
    SECTION("remote sparse source becomes solid") { fixture.regions[1]->fill_surfaces.surfaces.front().surface_type = stInternalSolid; }
    SECTION("remote base polygon") { fixture.regions[2]->fill_surfaces.surfaces.front().expolygon.contour.points.back().x() += 1; }
    SECTION("source material polygon") { fixture.regions[1]->slices.surfaces.front().expolygon.contour.points.back().y() += 1; }
    SECTION("previously irrelevant old layer becomes a bridge") { fixture.regions[0]->fill_surfaces.surfaces.front().surface_type = stInternalBridge; }
    SECTION("region sparse policy") { fixture.config().sparse_infill_pattern.value = ipCubic; }
    SECTION("region participation density") { fixture.config().sparse_infill_density.value = 100.; }
    SECTION("gap") { fixture.config().bridge_aware_cubic_max_gap.value += 1.; }
    SECTION("ordered surface insertion") { fixture.regions[1]->fill_surfaces.append(ExPolygons{collection_box(20.)}, stInternal); }
    CHECK(inputs->matches(*fixture.object, fixture.checked) == Match::Different);
}

TEST_CASE("Model collection identity preserves every copied model path value", "[BridgeAwareCubic][ModelCollectionInputs]")
{
    CollectionFixture fixture;
    const auto inputs = fixture.capture();
    REQUIRE(inputs);
    SECTION("width") { fixture.path().width = .5f; }
    SECTION("height") { fixture.path().height = .3f; }
    SECTION("role") { fixture.path().set_extrusion_role(erTopSolidInfill); }
    SECTION("flow") { fixture.path().mm3_per_mm += .001; }
    SECTION("suppression") { fixture.path().set_force_no_extrusion(true); }
    SECTION("point") { fixture.path().polyline.points.back().x() += 1; }
    SECTION("upper print Z") { fixture.object->get_layer(4)->print_z += .001; }
    SECTION("reversal") { fixture.path().set_reverse(); }
    SECTION("overhang") { fixture.path().overhang_degree = 1.; }
    SECTION("curve metadata") { fixture.path().curve_degree = 1; }
    SECTION("smooth speed") { fixture.path().smooth_speed = 1.; }
    SECTION("cooling") { fixture.path().set_cooling_node(1); }
    SECTION("customization") { fixture.path().set_customize_flag(cfCircleCompensation); }
    CHECK(inputs->matches(*fixture.object, fixture.checked) == Match::Different);
}

TEST_CASE("Model collection identity includes formerly unselected and newly eligible paths", "[BridgeAwareCubic][ModelCollectionInputs]")
{
    CollectionFixture fixture;
    // Outside all current material/bridge selections. Frozen Targets alone
    // cannot record this path, but moving it inside changes collection.
    auto *unselected = collection_path(erSolidInfill, 30.);
    fixture.regions[4]->fills.entities.push_back(unselected);
    auto *suppressed = collection_path(erSolidInfill, 3.);
    suppressed->set_force_no_extrusion(true);
    fixture.regions[4]->fills.entities.push_back(suppressed);
    auto *ordinary = collection_path(erInternalInfill, 4.);
    fixture.regions[4]->fills.entities.push_back(ordinary);
    auto *zero_flow = collection_path(erSolidInfill, 5.);
    zero_flow->mm3_per_mm = 0.;
    fixture.regions[4]->fills.entities.push_back(zero_flow);
    auto *short_path = collection_path(erSolidInfill, 6.);
    short_path->polyline.points.resize(1);
    fixture.regions[4]->fills.entities.push_back(short_path);
    const auto inputs = fixture.capture();
    REQUIRE(inputs);
    SECTION("outside path moves inside") { unselected->polyline.translate(Point::new_scale(-29., 0.)); }
    SECTION("new active model leaf") { fixture.regions[4]->fills.entities.push_back(collection_path(erSolidInfill, 5.)); }
    SECTION("suppressed leaf activates") { suppressed->set_force_no_extrusion(false); }
    SECTION("ordinary leaf becomes solid") { ordinary->set_extrusion_role(erSolidInfill); }
    SECTION("zero-flow leaf activates") { zero_flow->mm3_per_mm = .08; }
    SECTION("one-point leaf gains a segment") { short_path->polyline.points.push_back(Point::new_scale(8., 2.)); }
    CHECK(inputs->matches(*fixture.object, fixture.checked) == Match::Different);
}

TEST_CASE("Model collection identity compares traversal order and actual free-end membership", "[BridgeAwareCubic][ModelCollectionInputs]")
{
    CollectionFixture fixture;
    fixture.regions[4]->fills.entities.push_back(collection_path(erSolidInfill, 5.));
    const auto inputs = fixture.capture();
    REQUIRE(inputs);
    SECTION("reordered leaves") { std::swap(fixture.regions[4]->fills.entities[0], fixture.regions[4]->fills.entities[1]); }
    SECTION("same leaf becomes a multipath join") {
        auto &slot = fixture.regions[4]->fills.entities[0];
        auto *replacement = new ExtrusionMultiPath(*static_cast<ExtrusionPath *>(slot));
        delete slot;
        slot = replacement;
    }
    SECTION("same open leaf moves to perimeters") {
        auto &leaves = fixture.regions[4]->fills.entities;
        fixture.regions[4]->perimeters.entities.push_back(leaves.front());
        leaves.erase(leaves.begin());
    }
    CHECK(inputs->matches(*fixture.object, fixture.checked) == Match::Different);
}

TEST_CASE("Model collection identity permits harmless collection nesting", "[BridgeAwareCubic][ModelCollectionInputs]")
{
    CollectionFixture fixture;
    const auto inputs = fixture.capture();
    REQUIRE(inputs);
    auto *nested = new ExtrusionEntityCollection;
    nested->entities.swap(fixture.regions[4]->fills.entities);
    fixture.regions[4]->fills.entities.push_back(nested);
    CHECK(inputs->matches(*fixture.object, fixture.checked) == Match::Equal);
}

TEST_CASE("Model collection identity guards requested layers with zero frozen targets", "[BridgeAwareCubic][ModelCollectionInputs]")
{
    CollectionFixture fixture;
    const auto inputs = fixture.capture();
    REQUIRE(inputs);
    const std::set<size_t> empty_layer{2};
    REQUIRE(inputs->matches(*fixture.object, empty_layer) == Match::Equal);
    fixture.regions[2]->fills.entities.push_back(collection_path(erSolidInfill));
    CHECK(inputs->matches(*fixture.object, empty_layer) == Match::Different);
    // Unrequested upper paths do not affect collection at layer4. Surface and
    // policy ancestry is still compared globally on every lookup.
    CHECK(inputs->matches(*fixture.object, fixture.checked) == Match::Equal);
}

TEST_CASE("Model collection identity declines fitted and derived eligible paths", "[BridgeAwareCubic][ModelCollectionInputs]")
{
    CollectionFixture fixture;
    const auto inputs = fixture.capture();
    REQUIRE(inputs);
    SECTION("fitting metadata") { fixture.path().polyline.fitting_result.emplace_back(); }
    SECTION("oriented leaf hides its copied base reversal flag") {
        auto &slot = fixture.regions[4]->fills.entities.front();
        const auto &source = fixture.path();
        auto *replacement = new ExtrusionPathOriented(source.role(), source.mm3_per_mm, source.width, source.height);
        replacement->polyline = source.polyline;
        delete slot;
        slot = replacement;
        CHECK_FALSE(fixture.capture());
        replacement->set_reverse();
    }
    SECTION("sloped leaf") {
        auto &slot = fixture.regions[4]->fills.entities.front();
        auto *replacement = new ExtrusionPathSloped(fixture.path(), {}, {});
        delete slot;
        slot = replacement;
    }
    CHECK(inputs->matches(*fixture.object, fixture.checked) == Match::Unavailable);
    CHECK_FALSE(fixture.capture());
}

TEST_CASE("Model collection identity accounts existing prepared storage before capture", "[BridgeAwareCubic][ModelCollectionInputs]")
{
    CollectionFixture fixture;
    const auto baseline = fixture.capture();
    REQUIRE(baseline);
    const auto own = baseline->owned_usage();
    REQUIRE(own.bytes > 0);
    REQUIRE(own.vertices > 0);
    REQUIRE(own.items > 0);
    Inputs::Limits limits;
    SECTION("combined bytes exceed allowance") { limits.bytes = own.bytes + 64; limits.other_retained.bytes = 65; }
    SECTION("combined vertices exceed allowance") { limits.vertices = own.vertices + 1; limits.other_retained.vertices = 2; }
    SECTION("combined items exceed allowance") { limits.items = own.items + 1; limits.other_retained.items = 2; }
    CHECK_FALSE(Inputs::capture(*fixture.object, limits));
    // A nonzero existing prepared snapshot and the identity must fit together;
    // exact usage reports must not omit paths, surfaces or container capacities.
    limits = {};
    limits.other_retained = {own.bytes + 64, own.vertices, own.items};
    const auto combined = Inputs::capture(*fixture.object, limits);
    REQUIRE(combined);
    CHECK(combined->owned_usage().bytes + limits.other_retained.bytes <= limits.bytes);
    CHECK(combined->owned_usage().vertices + limits.other_retained.vertices <= limits.vertices);
    CHECK(combined->owned_usage().items + limits.other_retained.items <= limits.items);
}

TEST_CASE("Model collection optional work exhaustion preserves the consumed allowance", "[BridgeAwareCubic][ModelCollectionInputs]")
{
    CollectionFixture fixture;
    const auto inputs = fixture.capture();
    REQUIRE(inputs);
    Validation::WorkBudget tiny;
    tiny.remaining_work = 1;
    CHECK(inputs->matches(*fixture.object, fixture.checked, &tiny) == Match::Unavailable);
    CHECK(tiny.remaining_work == 0);
    CHECK(tiny.charged_work[size_t(Validation::WorkCategory::Validation)] == 1);
    // The original collector receives THIS budget on fallback; a miss does not
    // manufacture a fresh pass allowance or publish partial identity success.
    CHECK_FALSE(Inputs::capture(*fixture.object, &tiny));
    CHECK(tiny.remaining_work == 0);
    Validation::WorkBudget normal;
    CHECK(inputs->matches(*fixture.object, fixture.checked, &normal) == Match::Equal);
    CHECK(normal.remaining_work < 50000000);
}

TEST_CASE("Model collection cancellation escapes capture and warm comparison", "[BridgeAwareCubic][ModelCollectionInputs]")
{
    CollectionFixture fixture;
    const auto cancel = [] { throw std::runtime_error("model collection canceled"); };
    CHECK_THROWS_WITH(Inputs::capture(*fixture.object, nullptr, cancel), "model collection canceled");
    const auto inputs = fixture.capture();
    REQUIRE(inputs);
    CHECK_THROWS_WITH(inputs->matches(*fixture.object, fixture.checked, nullptr, cancel), "model collection canceled");
    size_t polls = 0;
    CHECK_THROWS_AS(inputs->matches(*fixture.object, fixture.checked, nullptr, [&] {
        if (++polls == 2) throw std::bad_alloc();
    }), std::bad_alloc);
    CHECK(inputs->matches(*fixture.object, fixture.checked) == Match::Equal);
}

TEST_CASE("Model collection identity permits same-layer sparse and ironing regeneration", "[BridgeAwareCubic][ModelCollectionInputs]")
{
    CollectionFixture fixture;
    auto *sparse = collection_path(erInternalInfill, 4.);
    auto *ironing = collection_path(erIroning, 6.);
    fixture.regions[4]->fills.entities.push_back(sparse);
    fixture.regions[4]->fills.entities.push_back(ironing);
    const auto inputs = fixture.capture();
    REQUIRE(inputs);
    sparse->polyline.points.back().x() += scale_(2.);
    sparse->width = .6f;
    ironing->polyline.points.front().y() += scale_(1.);
    fixture.regions[4]->fills.entities.push_back(collection_path(erInternalInfill, 7.));
    CHECK(inputs->matches(*fixture.object, fixture.checked) == Match::Equal);
}

TEST_CASE("Model collection warm lookup admits temporary membership alongside retained inputs", "[BridgeAwareCubic][ModelCollectionInputs]")
{
    CollectionFixture fixture;
    const auto baseline = fixture.capture();
    REQUIRE(baseline);
    Inputs::Limits limits;
    limits.items = baseline->owned_usage().items + 4;
    const auto inputs = Inputs::capture(*fixture.object, limits);
    REQUIRE(inputs);
    REQUIRE(inputs->matches(*fixture.object, fixture.checked) == Match::Equal);
    for (size_t i = 0; i < 8; ++i)
        fixture.regions[4]->fills.entities.push_back(collection_path(erInternalInfill, double(i)));
    CHECK(inputs->matches(*fixture.object, fixture.checked) == Match::Unavailable);
    CHECK(inputs->capture_peak_usage().items <= limits.items);
}

TEST_CASE("Model collection identity refuses invalid coverage and foreign region bindings", "[BridgeAwareCubic][ModelCollectionInputs]")
{
    CollectionFixture fixture;
    const auto inputs = fixture.capture();
    REQUIRE(inputs);
    CHECK(inputs->matches(*fixture.object, {999}) == Match::Unavailable);
    PrintRegion foreign(PrintRegionConfig(fixture.object->printing_region(0).config()));
    fixture.object->get_layer(0)->add_region(&foreign);
    CHECK(inputs->matches(*fixture.object, fixture.checked) != Match::Equal);
    CHECK_FALSE(fixture.capture());
}
