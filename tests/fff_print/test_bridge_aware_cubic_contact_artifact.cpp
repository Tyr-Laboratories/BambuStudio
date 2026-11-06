#include <catch2/catch.hpp>

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Fill/BridgeAwareCubicContactArtifactCache.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <new>
#include <stdexcept>

using namespace Slic3r;
namespace BAC = Slic3r::BridgeAwareCubic;
namespace Contact = Slic3r::BridgeAwareCubicContact;
namespace Validation = Slic3r::BridgeAwareCubicValidation;
using ArtifactCache = BAC::ContactArtifactCache;

namespace {
ExPolygon rectangle(double x0, double y0, double x1, double y1)
{
    return ExPolygon(Polygon(Points{Point::new_scale(x0, y0), Point::new_scale(x1, y0),
        Point::new_scale(x1, y1), Point::new_scale(x0, y1)}));
}

ExtrusionPath bead(std::initializer_list<Vec2d> points, float width = .45f, float height = .2f)
{
    ExtrusionPath path(erInternalInfill, .08, width, height);
    path.polyline = Polyline::new_scale(std::vector<Vec2d>(points));
    return path;
}

struct Computed {
    Contact::PathContact contact;
    ExPolygons selected;
};

struct Fixture {
    ExtrusionPath upper = bead({{0., 0.}, {10., 0.}});
    std::vector<ExtrusionPath> deposited;
    BAC::SerializedXY phase;
    ExPolygons demand{ExPolygon(Polygon(Points{Point::new_scale(-1., -1.), Point::new_scale(11., -1.),
        Point::new_scale(11., 1.), Point::new_scale(-1., 1.)}))};
    Contact::PreparedLowerPtr lower;
    double gap = 8., selection_width = double(upper.width), upper_z = .4;

    explicit Fixture(double right_begin = 8.)
        : deposited{bead({{-1., 0.}, {2., 0.}}), bead({{right_begin, 0.}, {11., 0.}}),
                    bead({{0., 100.}, {10., 100.}})}
    { prepare(); }

    void prepare()
    {
        std::vector<Contact::Bead> inputs;
        for (const auto &path : deposited) inputs.push_back({&path, .2});
        lower = Contact::prepare_lower(inputs, upper_z, upper.height);
    }

    ArtifactCache::Query query(ArtifactCache::Purpose purpose = ArtifactCache::Purpose::DirectRoof,
                              Contact::QueryCache *masks = nullptr) const
    { return {purpose, phase, gap, selection_width, demand, *lower, upper, upper_z, masks}; }

    Computed compute(Contact::QueryCache *masks = nullptr) const
    {
        Computed result;
        result.contact = Contact::footprint(*lower, upper, upper_z, 0., Contact::FootprintMode::ContactsOnly,
                                            nullptr, masks);
        Polylines unsupported;
        for (const auto &segment : result.contact.segments) {
            const size_t i = segment.segment_index;
            append(unsupported, diff_pl(Polylines{Polyline(upper.polyline.points[i], upper.polyline.points[i + 1])},
                                       segment.centerline_support));
        }
        if (!result.contact.input_valid || !result.contact.completed || result.contact.segments.empty())
            unsupported = {upper.polyline};
        result.selected = intersection_ex(demand, union_ex(offset(unsupported, scale_(.5 * selection_width))));
        return result;
    }
};

void same_polygons(const ExPolygons &actual, const ExPolygons &expected)
{
    REQUIRE(actual.size() == expected.size());
    for (size_t i = 0; i < actual.size(); ++i) {
        CHECK(actual[i].contour.points == expected[i].contour.points);
        REQUIRE(actual[i].holes.size() == expected[i].holes.size());
        for (size_t hole = 0; hole < actual[i].holes.size(); ++hole)
            CHECK(actual[i].holes[hole].points == expected[i].holes[hole].points);
    }
}

void same_contact(const Contact::PathContact &actual, const Contact::PathContact &expected)
{
    CHECK(actual.input_valid == expected.input_valid);
    CHECK(actual.completed == expected.completed);
    CHECK(actual.resource_limited == expected.resource_limited);
    CHECK(actual.failure_reason == expected.failure_reason);
    CHECK(actual.contact_z == expected.contact_z);
    CHECK(actual.coordinate_uncertainty_mm == expected.coordinate_uncertainty_mm);
    same_polygons(actual.physical_footprint, expected.physical_footprint);
    REQUIRE(actual.physical_lines.size() == expected.physical_lines.size());
    for (size_t i = 0; i < actual.physical_lines.size(); ++i)
        CHECK(actual.physical_lines[i].points == expected.physical_lines[i].points);
    REQUIRE(actual.segments.size() == expected.segments.size());
    for (size_t i = 0; i < actual.segments.size(); ++i) {
        const auto &a = actual.segments[i], &e = expected.segments[i];
        CHECK(a.segment_index == e.segment_index);
        same_polygons(a.centerline_support, e.centerline_support);
        REQUIRE(a.line_contacts.size() == e.line_contacts.size());
        for (size_t j = 0; j < a.line_contacts.size(); ++j) {
            CHECK(a.line_contacts[j].begin_numerator == e.line_contacts[j].begin_numerator);
            CHECK(a.line_contacts[j].begin_denominator == e.line_contacts[j].begin_denominator);
            CHECK(a.line_contacts[j].end_numerator == e.line_contacts[j].end_numerator);
            CHECK(a.line_contacts[j].end_denominator == e.line_contacts[j].end_denominator);
            CHECK(a.line_contacts[j].position_uncertainty_mm == e.line_contacts[j].position_uncertainty_mm);
        }
    }
}

size_t polygon_capacity_bytes(const ExPolygons &polygons)
{
    size_t bytes = polygons.capacity() * sizeof(ExPolygon);
    for (const auto &polygon : polygons) {
        bytes += polygon.contour.points.capacity() * sizeof(Point) + polygon.holes.capacity() * sizeof(Polygon);
        for (const auto &hole : polygon.holes) bytes += hole.points.capacity() * sizeof(Point);
    }
    return bytes;
}

size_t visible_payload_bytes(const ArtifactCache::Artifact &artifact)
{
    const auto &contact = artifact.contact();
    size_t bytes = sizeof(contact) + sizeof(ExPolygons) + contact.failure_reason.capacity() + 1 +
        contact.segments.capacity() * sizeof(Contact::SegmentContact) +
        contact.physical_lines.capacity() * sizeof(Polyline) + polygon_capacity_bytes(contact.physical_footprint) +
        polygon_capacity_bytes(artifact.selected());
    for (const auto &line : contact.physical_lines) bytes += line.points.capacity() * sizeof(Point);
    for (const auto &segment : contact.segments)
        bytes += polygon_capacity_bytes(segment.centerline_support) +
                 segment.line_contacts.capacity() * sizeof(Contact::ExactInterval);
    return bytes;
}

bool publish(ArtifactCache &cache, const Fixture &fixture)
{
    const auto result = fixture.compute();
    REQUIRE(result.contact.input_valid);
    REQUIRE(result.contact.completed);
    REQUIRE_FALSE(result.selected.empty());
    return cache.publish(fixture.query(), result.contact, result.selected);
}
}

TEST_CASE("Direct roof artifacts retain exact completed contact and selected geometry",
          "[BridgeAwareCubic][ContactArtifact]")
{
    Fixture fixture;
    fixture.upper.polyline = Polyline::new_scale({{0., 0.}, {5., 0.}, {5., 0.}, {10., 0.}});
    fixture.prepare();
    auto computed = fixture.compute();
    const auto expected = computed;
    REQUIRE(computed.contact.input_valid);
    REQUIRE(computed.contact.completed);
    REQUIRE(computed.contact.segments.size() == 2);
    REQUIRE_FALSE(computed.selected.empty());
    ArtifactCache cache;
    CHECK_FALSE(cache.find(fixture.query()));
    REQUIRE(cache.publish(fixture.query(), computed.contact, computed.selected));
    const auto artifact = cache.find(fixture.query());
    REQUIRE(artifact);
    computed.contact.segments.clear();
    computed.selected.clear();
    same_contact(artifact->contact(), expected.contact);
    same_polygons(artifact->selected(), expected.selected);
    CHECK(artifact->owned_bytes() > 0);
    CHECK(artifact->owned_bytes() >= visible_payload_bytes(*artifact));
    CHECK(cache.statistics().entries == 1);
    CHECK(cache.statistics().publications == 1);
    CHECK(cache.statistics().hits == 1);
    CHECK(cache.statistics().resident_bytes >= artifact->owned_bytes());
}

TEST_CASE("Direct roof artifact hits cannot certify a remotely opened physical cell",
          "[BridgeAwareCubic][ContactArtifact]")
{
    Fixture fixture;
    fixture.upper = bead({{0., 0.}, {6., 0.}});
    fixture.gap = 9.;
    fixture.demand = {rectangle(-1., -1., 7., 1.)};
    fixture.deposited = {bead({{0., -3.}, {0., 3.}}, .6f), bead({{6., -3.}, {6., 3.}}, .6f),
        bead({{-1., -3.}, {7., -3.}}, .6f), bead({{-1., 3.}, {7., 3.}}, .6f)};
    fixture.prepare();
    const auto original = fixture.compute();
    const ExPolygons domain{rectangle(-2., -5., 8., 5.)};
    const auto physical = Contact::physical_sections(*fixture.lower);
    const auto before = Validation::validate_contacts(domain, physical.physical_footprint,
        {fixture.upper.polyline}, original.selected, fixture.gap, 0., {original.contact});
    REQUIRE(before.input_valid);
    REQUIRE(before.completed);
    REQUIRE(before.passed());
    REQUIRE_FALSE(before.void_cells.empty());
    ArtifactCache cache;
    REQUIRE(cache.publish(fixture.query(), original.contact, original.selected));

    // The top wall is remote from this entire upper query. Its removal still
    // opens the selected physical cell to the complete-domain boundary.
    fixture.deposited.pop_back();
    fixture.prepare();
    const auto artifact = cache.find(fixture.query());
    REQUIRE(artifact);
    const auto fresh = fixture.compute();
    same_contact(artifact->contact(), fresh.contact);
    same_polygons(artifact->selected(), fresh.selected);
    const auto current_physical = Contact::physical_sections(*fixture.lower);
    const auto after = Validation::validate_contacts(domain, current_physical.physical_footprint,
        {fixture.upper.polyline}, artifact->selected(), fixture.gap, 0., {artifact->contact()});
    REQUIRE(after.input_valid);
    REQUIRE(after.completed);
    CHECK_FALSE(after.passed());
    CHECK(std::any_of(after.void_cells.begin(), after.void_cells.end(),
        [](const Validation::VoidCell &cell) { return !cell.closed; }));
}

TEST_CASE("Direct roof artifacts reuse only freshly equal local contact inputs",
          "[BridgeAwareCubic][ContactArtifact]")
{
    Fixture fixture;
    for (int i = 0; i < 80; ++i)
        fixture.deposited.push_back(bead({{double(i), 20.}, {double(i), 21.}}));
    fixture.prepare();
    const auto expected = fixture.compute();
    ArtifactCache cache;
    REQUIRE(cache.publish(fixture.query(), expected.contact, expected.selected));
    fixture.deposited.back() = bead({{-30., 30.}, {-30., 31.}});
    fixture.deposited.push_back(bead({{-50., -20.}, {-50., -19.}}));
    std::reverse(fixture.deposited.begin(), fixture.deposited.end());
    fixture.prepare();
    const auto artifact = cache.find(fixture.query());
    REQUIRE(artifact);
    const auto fresh = fixture.compute();
    same_contact(artifact->contact(), fresh.contact);
    same_polygons(artifact->selected(), fresh.selected);
    // This hit supplies computation only. No physical-cell verdict is stored
    // or inferred from the unchanged local contact/selection.
    CHECK(artifact->contact().physical_footprint.empty());
    CHECK(artifact->contact().physical_lines.empty());
}

TEST_CASE("Direct roof artifacts invalidate removed added and whole merged contributors",
          "[BridgeAwareCubic][ContactArtifact]")
{
    const int mutation = GENERATE(0, 1, 2, 3);
    CAPTURE(mutation);
    Fixture fixture;
    ArtifactCache cache;
    REQUIRE(publish(cache, fixture));
    if (mutation == 0) fixture.deposited.erase(fixture.deposited.begin());
    if (mutation == 1) fixture.deposited.push_back(bead({{5., -1.}, {5., 1.}}));
    if (mutation == 2) fixture.deposited.push_back(bead({{-20., 0.}, {-1., 0.}}));
    if (mutation == 3) fixture.deposited.front().width = std::nextafter(fixture.deposited.front().width, 1.f);
    fixture.prepare();
    CHECK_FALSE(cache.find(fixture.query()));
    Fixture original;
    CHECK(cache.find(original.query()));
}

TEST_CASE("Direct roof artifacts compare complete policy and exact upper inputs",
          "[BridgeAwareCubic][ContactArtifact]")
{
    Fixture original;
    ArtifactCache cache;
    REQUIRE(publish(cache, original));
    for (int mutation = 0; mutation < 13; ++mutation) {
        CAPTURE(mutation);
        Fixture changed;
        if (mutation == 0) changed.gap = std::nextafter(changed.gap, 20.);
        if (mutation == 1) changed.selection_width = std::nextafter(changed.selection_width, 1.);
        if (mutation == 2) changed.demand.front().contour.points.front().x() += 1;
        if (mutation == 3) {
            auto &points = changed.demand.front().contour.points;
            std::rotate(points.begin(), points.begin() + 1, points.end());
        }
        if (mutation == 4) { changed.phase.shift.x() += 1.; changed.phase.plate_origin.x() += 1.; }
        if (mutation == 5) { changed.phase.shift.y() += 1.; changed.phase.nozzle_offset.y() += 1.; }
        if (mutation == 6) changed.upper.polyline.points.back().x() -= 1;
        if (mutation == 7) changed.upper.width = std::nextafter(changed.upper.width, 1.f);
        if (mutation == 8) changed.upper.height = .19f;
        if (mutation == 9) changed.upper.mm3_per_mm = std::nextafter(changed.upper.mm3_per_mm, 1.);
        if (mutation == 10) {
            ExtrusionPath properties(erSolidInfill, changed.upper.mm3_per_mm, changed.upper.width, changed.upper.height);
            changed.upper = ExtrusionPath(changed.upper.polyline, properties);
        }
        if (mutation == 11) changed.upper_z = .40001;
        if (mutation == 12) changed.upper.polyline.points.insert(changed.upper.polyline.points.begin() + 1,
            changed.upper.polyline.points.front());
        changed.prepare();
        CHECK_FALSE(cache.find(changed.query()));
        CHECK(cache.find(original.query()));
    }
    CHECK_FALSE(cache.find(original.query(ArtifactCache::Purpose::Ineligible)));
}

TEST_CASE("Direct roof artifacts refuse invalid current preparation and foreign caches",
          "[BridgeAwareCubic][ContactArtifact]")
{
    Fixture fixture;
    ArtifactCache cache;
    REQUIRE(publish(cache, fixture));
    fixture.deposited.back().mm3_per_mm = std::numeric_limits<double>::quiet_NaN();
    fixture.prepare();
    CHECK_FALSE(cache.find(fixture.query()));
    Fixture valid;
    Contact::QueryCache foreign(fixture.lower);
    CHECK_FALSE(cache.find(valid.query(ArtifactCache::Purpose::DirectRoof, &foreign)));
    const auto computed = valid.compute();
    CHECK_FALSE(cache.publish(valid.query(ArtifactCache::Purpose::DirectRoof, &foreign),
                              computed.contact, computed.selected));
    CHECK(cache.find(valid.query()));
    valid.upper.mm3_per_mm = 0.;
    CHECK_FALSE(cache.find(valid.query()));
}

TEST_CASE("Direct roof artifacts refuse malformed policy without replacing valid state",
          "[BridgeAwareCubic][ContactArtifact]")
{
    Fixture original;
    const auto computed = original.compute();
    ArtifactCache cache;
    REQUIRE(cache.publish(original.query(), computed.contact, computed.selected));
    const auto before = cache.statistics();
    for (int invalid = 0; invalid < 14; ++invalid) {
        CAPTURE(invalid);
        Fixture changed;
        if (invalid == 0) changed.gap = 0.;
        if (invalid == 1) changed.gap = -1.;
        if (invalid == 2) changed.gap = std::numeric_limits<double>::infinity();
        if (invalid == 3) changed.selection_width = 0.;
        if (invalid == 4) changed.selection_width = -1.;
        if (invalid == 5) changed.selection_width = std::numeric_limits<double>::quiet_NaN();
        if (invalid == 6) changed.phase.shift.x() = std::numeric_limits<double>::infinity();
        if (invalid == 7) changed.phase.nozzle_offset.y() = std::numeric_limits<double>::quiet_NaN();
        if (invalid == 8) changed.phase.plate_origin.x() = std::numeric_limits<double>::infinity();
        if (invalid == 9) changed.demand.front().contour.points.resize(2);
        if (invalid == 10) changed.demand.front().contour.reverse();
        if (invalid == 11) changed.demand.front().contour.points.front().x() =
            std::numeric_limits<coord_t>::max() / 4 + 1;
        if (invalid == 12) changed.demand.front().contour.points.resize(200001, Point(0, 0));
        if (invalid == 13) changed.demand.clear();
        CHECK_FALSE(cache.find(changed.query()));
        CHECK_FALSE(cache.publish(changed.query(), computed.contact, computed.selected));
        CHECK(cache.find(original.query()));
        CHECK(cache.statistics().resident_bytes == before.resident_bytes);
        CHECK(cache.statistics().entries == before.entries);
    }
    CHECK_FALSE(cache.publish(original.query(ArtifactCache::Purpose::Ineligible), computed.contact, computed.selected));
}

TEST_CASE("Direct roof artifacts admit only complete ContactsOnly output and nonempty selection",
          "[BridgeAwareCubic][ContactArtifact]")
{
    Fixture fixture;
    const auto computed = fixture.compute();
    ArtifactCache cache;
    REQUIRE(cache.publish(fixture.query(), computed.contact, computed.selected));
    const auto before = cache.statistics();
    for (int invalid = 0; invalid < 11; ++invalid) {
        CAPTURE(invalid);
        auto contact = computed.contact;
        auto selected = computed.selected;
        if (invalid == 0) contact.input_valid = false;
        if (invalid == 1) contact.completed = false;
        if (invalid == 2) contact.resource_limited = true;
        if (invalid == 3) contact.failure_reason = "partial contact computation";
        if (invalid == 4) contact = Contact::footprint(*fixture.lower, fixture.upper, fixture.upper_z, 0.,
                                                    Contact::FootprintMode::Full);
        if (invalid == 5) contact.physical_lines.push_back(fixture.upper.polyline);
        if (invalid == 6) contact.segments.front().segment_index = 1;
        if (invalid == 7) contact.contact_z = std::nextafter(contact.contact_z, 1.);
        if (invalid == 8) selected.clear();
        if (invalid == 9) selected.front().contour.points.resize(2);
        if (invalid == 10) selected.front().contour.points.front().y() =
            std::numeric_limits<coord_t>::max() / 4 + 1;
        CHECK_FALSE(cache.publish(fixture.query(), contact, selected));
        CHECK(cache.find(fixture.query()));
        CHECK(cache.statistics().entries == before.entries);
        CHECK(cache.statistics().resident_bytes == before.resident_bytes);
    }
}

TEST_CASE("Direct roof artifacts retain two exact lower versions for rollback",
          "[BridgeAwareCubic][ContactArtifact]")
{
    Fixture first(8.), second(7.), third(6.);
    ArtifactCache cache;
    REQUIRE(publish(cache, first));
    REQUIRE(publish(cache, second));
    REQUIRE(cache.find(first.query()));
    REQUIRE(cache.find(second.query()));
    CHECK(cache.statistics().entries == 2);
    REQUIRE(publish(cache, third));
    CHECK_FALSE(cache.find(first.query()));
    CHECK(cache.find(second.query()));
    CHECK(cache.find(third.query()));
    CHECK(cache.statistics().entries == 2);
    CHECK(cache.statistics().evictions >= 1);
}

TEST_CASE("Direct roof artifact staging cannot evict existing state before admission",
          "[BridgeAwareCubic][ContactArtifact]")
{
    Fixture fixture;
    ArtifactCache measured;
    REQUIRE(publish(measured, fixture));
    const size_t one_entry = measured.statistics().peak_bytes;
    REQUIRE(one_entry > 0);
    ArtifactCache bounded(one_entry, 2);
    REQUIRE(publish(bounded, fixture));
    Fixture alternate;
    alternate.phase.shift.x() = 1.;
    CHECK_FALSE(publish(bounded, alternate));
    CHECK(bounded.find(fixture.query()));
    CHECK_FALSE(bounded.find(alternate.query()));
    CHECK(bounded.statistics().evictions == 0);
    CHECK(bounded.statistics().resident_bytes <= one_entry);
    CHECK(bounded.statistics().peak_bytes <= one_entry);
    ArtifactCache absent(0, 0);
    CHECK_FALSE(publish(absent, fixture));
    CHECK(absent.statistics().resident_bytes == 0);

    Fixture second(7.), third(6.);
    ArtifactCache measured_versions;
    REQUIRE(publish(measured_versions, fixture));
    REQUIRE(publish(measured_versions, second));
    REQUIRE(publish(measured_versions, third));
    const size_t third_peak = measured_versions.statistics().peak_bytes;
    REQUIRE(third_peak > 1);
    ArtifactCache two_versions(third_peak - 1, 3);
    REQUIRE(publish(two_versions, fixture));
    REQUIRE(publish(two_versions, second));
    CHECK_FALSE(publish(two_versions, third));
    CHECK(two_versions.find(fixture.query()));
    CHECK(two_versions.find(second.query()));
    CHECK_FALSE(two_versions.find(third.query()));
    CHECK(two_versions.statistics().entries == 2);
    CHECK(two_versions.statistics().evictions == 0);
}

TEST_CASE("Direct roof artifacts own payload without retaining old preparations or query caches",
          "[BridgeAwareCubic][ContactArtifact]")
{
    ArtifactCache cache;
    ArtifactCache::Handle artifact;
    std::weak_ptr<const Contact::PreparedLower> old_preparation;
    Computed expected;
    {
        Fixture temporary;
        old_preparation = temporary.lower;
        Contact::QueryCache masks(temporary.lower, 5);
        expected = temporary.compute(&masks);
        REQUIRE(masks.statistics().evictions > 0);
        REQUIRE(cache.publish(temporary.query(ArtifactCache::Purpose::DirectRoof, &masks),
                              expected.contact, expected.selected));
        artifact = cache.find(temporary.query(ArtifactCache::Purpose::DirectRoof, &masks));
        REQUIRE(artifact);
    }
    CHECK(old_preparation.expired());
    same_contact(artifact->contact(), expected.contact);
    same_polygons(artifact->selected(), expected.selected);
    Fixture restored;
    REQUIRE(cache.find(restored.query())); // Identical live geometry at new addresses.
}

TEST_CASE("Direct roof artifact quota includes immutable handles retained after eviction",
          "[BridgeAwareCubic][ContactArtifact]")
{
    Fixture fixture;
    ArtifactCache measured;
    REQUIRE(publish(measured, fixture));
    // Keep the measured temporary capture peak in addition to space for
    // retained handles; resident-only bytes are not an allocation-peak oracle.
    const size_t capacity = measured.statistics().peak_bytes + 8 * measured.statistics().resident_bytes;
    ArtifactCache cache(capacity, 1);
    std::vector<ArtifactCache::Handle> retained;
    bool declined = false;
    for (int phase = 0; phase < 32; ++phase) {
        fixture.phase.shift.x() = phase;
        if (!publish(cache, fixture)) { declined = true; break; }
        auto artifact = cache.find(fixture.query());
        REQUIRE(artifact);
        retained.push_back(std::move(artifact));
        size_t retained_bytes = 0;
        for (const auto &handle : retained) retained_bytes += handle->owned_bytes();
        CHECK(cache.statistics().resident_bytes >= retained_bytes);
        CHECK(cache.statistics().resident_bytes <= capacity);
        CHECK(cache.statistics().peak_bytes <= capacity);
        CHECK(cache.statistics().entries == 1);
    }
    REQUIRE(declined);
    REQUIRE(retained.size() > 1);
    const size_t held = cache.statistics().resident_bytes;
    retained.clear();
    CHECK(cache.statistics().resident_bytes < held);
    REQUIRE(publish(cache, fixture));
    CHECK(cache.find(fixture.query()));
}

TEST_CASE("Direct roof artifact failure and cancellation preserve both prior versions",
          "[BridgeAwareCubic][ContactArtifact]")
{
    Fixture first(8.), second(7.), third(6.);
    const auto computed = third.compute();
    ArtifactCache cache;
    Validation::WorkBudget none{0};
    CHECK_FALSE(cache.publish(third.query(), computed.contact, computed.selected, &none));
    REQUIRE(publish(cache, first));
    REQUIRE(publish(cache, second));
    const auto before = cache.statistics();
    CHECK_FALSE(cache.find(first.query(), &none));
    Validation::WorkBudget partial{3};
    CHECK_FALSE(cache.publish(third.query(), computed.contact, computed.selected, &partial));
    CHECK(partial.remaining_work < 3);
    CHECK(cache.find(first.query()));
    CHECK(cache.find(second.query()));
    CHECK(cache.statistics().resident_bytes == before.resident_bytes);
    CHECK(cache.statistics().evictions == before.evictions);
    const auto canceled = [] { throw std::runtime_error("artifact cache canceled"); };
    CHECK_THROWS_WITH(cache.find(first.query(), nullptr, canceled), "artifact cache canceled");
    CHECK_THROWS_WITH(cache.publish(third.query(), computed.contact, computed.selected, nullptr, canceled),
                      "artifact cache canceled");

    ArtifactCache counted;
    REQUIRE(publish(counted, first));
    REQUIRE(publish(counted, second));
    size_t polls = 0;
    REQUIRE(counted.publish(third.query(), computed.contact, computed.selected, nullptr, [&] { ++polls; }));
    REQUIRE(polls > 1);
    size_t current = 0;
    CHECK_THROWS_AS(cache.publish(third.query(), computed.contact, computed.selected, nullptr, [&] {
        if (++current == polls) throw std::bad_alloc();
    }), std::bad_alloc);
    CHECK(cache.find(first.query()));
    CHECK(cache.find(second.query()));
    CHECK_FALSE(cache.find(third.query()));
    CHECK(cache.statistics().entries == before.entries);
    CHECK(cache.statistics().resident_bytes == before.resident_bytes);
    CHECK(cache.statistics().evictions == before.evictions);
}

TEST_CASE("Direct roof artifacts count spare capacities and keep publication headroom",
          "[BridgeAwareCubic][ContactArtifact]")
{
    Fixture fixture;
    Polygon hole = rectangle(4., -.1, 5., .1).contour;
    hole.make_clockwise();
    fixture.demand.front().holes.push_back(std::move(hole));
    fixture.demand.front().holes.front().points.reserve(20);
    auto computed = fixture.compute();
    REQUIRE(std::any_of(computed.selected.begin(), computed.selected.end(),
        [](const ExPolygon &polygon) { return !polygon.holes.empty(); }));
    computed.contact.segments.reserve(computed.contact.segments.size() + 8);
    computed.contact.physical_footprint.reserve(3);
    computed.contact.physical_lines.reserve(4);
    for (auto &segment : computed.contact.segments) {
        segment.centerline_support.reserve(segment.centerline_support.size() + 5);
        segment.line_contacts.reserve(segment.line_contacts.size() + 7);
        for (auto &polygon : segment.centerline_support) {
            polygon.contour.points.reserve(polygon.contour.points.size() + 11);
            polygon.holes.reserve(polygon.holes.size() + 3);
            for (auto &hole : polygon.holes) hole.points.reserve(hole.points.size() + 9);
        }
    }
    computed.selected.reserve(computed.selected.size() + 5);
    for (auto &polygon : computed.selected) {
        polygon.contour.points.reserve(polygon.contour.points.size() + 13);
        polygon.holes.reserve(polygon.holes.size() + 4);
        for (auto &hole : polygon.holes) hole.points.reserve(hole.points.size() + 9);
    }
    ArtifactCache measured;
    REQUIRE(measured.publish(fixture.query(), computed.contact, computed.selected));
    auto artifact = measured.find(fixture.query());
    REQUIRE(artifact);
    same_contact(artifact->contact(), computed.contact);
    same_polygons(artifact->selected(), computed.selected);
    CHECK(artifact->owned_bytes() >= visible_payload_bytes(*artifact));
    CHECK(artifact->contact().segments.capacity() == computed.contact.segments.capacity());
    CHECK(artifact->selected().capacity() == computed.selected.capacity());
    const size_t cap = measured.statistics().peak_bytes;
    ArtifactCache too_small(cap - 1);
    CHECK_FALSE(too_small.publish(fixture.query(), computed.contact, computed.selected));
    CHECK(too_small.statistics().entries == 0);
    CHECK(too_small.statistics().peak_bytes <= cap - 1);

    // Exercise quota-driven turnover, independently of the two-version and
    // maximum-entry limits. Unborrowed older artifacts must leave staging room.
    ArtifactCache turnover(8 * cap, 20000);
    for (int version = 0; version < 160; ++version) {
        fixture.phase.shift.x() = version;
        REQUIRE(turnover.publish(fixture.query(), computed.contact, computed.selected));
        REQUIRE(turnover.find(fixture.query()));
        CHECK(turnover.statistics().peak_bytes <= 8 * cap);
    }
    CHECK(turnover.statistics().evictions > 0);
    CHECK(turnover.statistics().entries < 160);
}
