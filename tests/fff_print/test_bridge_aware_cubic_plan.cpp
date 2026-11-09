#include <catch2/catch.hpp>

#include "bridge_aware_cubic_fixtures.hpp"

#include "libslic3r/BridgeAwareCubicMaterial.hpp"

#include <algorithm>
#include <random>

using namespace Slic3r;
using namespace Slic3r::BridgeAwareCubic;
using namespace BACFixtures;

namespace {

// A 40 x 30 x 12 mm block with a 0.3 mm wide strip through its middle set to 100% rectilinear infill: a solid surface narrower
// than 1.1 solid spacings, which may print but is never counted on.
Model narrow_strip_block()
{
    Model model = single_part("Bridge aware cubic narrow strip", TriangleMesh(its_make_cube(40., 30., 12.)));
    TriangleMesh strip(its_make_cube(0.3, 30., 12.));
    strip.translate(20., 0., 0.);
    ModelVolume *modifier = model.objects.front()->add_volume(std::move(strip));
    modifier->set_type(ModelVolumeType::PARAMETER_MODIFIER);
    modifier->config.set_key_value("sparse_infill_density", new ConfigOptionPercent(100.));
    modifier->config.set_key_value("sparse_infill_pattern", new ConfigOptionEnum<InfillPattern>(ipRectilinear));
    return model;
}

// The regions that print the pattern, one group each (enough for the material of one object).
std::vector<std::vector<size_t>> pattern_groups(const PrintObject &object)
{
    std::vector<std::vector<size_t>> groups;
    for (size_t i = 0; i < object.num_printing_regions(); ++i) {
        const PrintRegionConfig &config = object.printing_region(i).config();
        if (config.sparse_infill_pattern.value == ipBridgeAwareCubic && config.sparse_infill_density.value > 0. && config.sparse_infill_density.value < 100.)
            groups.push_back({ i });
    }
    return groups;
}

ExPolygons surfaces_of(const Layer &layer, const std::function<bool(const LayerRegion &, const Surface &)> &take)
{
    ExPolygons out;
    for (const LayerRegion *region : layer.regions())
        for (const Surface &surface : region->fill_surfaces.surfaces)
            if (take(*region, surface))
                out.push_back(surface.expolygon);
    return out;
}

bool is_sparse(const LayerRegion &region, const Surface &surface)
{
    const double density = region.region().config().sparse_infill_density.value;
    return surface.surface_type == stInternal && density > 0. && density < 100.;
}

} // namespace

// M1: printed material, the rule applied to a sliced part's surfaces. A solid strip narrower than 1.1 spacings may print (roof
// side) but surely does not (support side); the wall band is material on both sides; the void meets the sparse surfaces; the fill
// boundary is FillCubic's inset, less the courses' beads kept at w/2 − 0.05 from their centre lines; the area index agrees with
// the polygons. (W1 reads the seam wall that separates two groups' voids on the export.)
TEST_CASE("Bridge aware cubic material has one definition with two sides", "[BridgeAwareCubic][M1]")
{
    SECTION("a narrow solid strip is a roof above and void below") {
        Print print;
        Model model = narrow_strip_block();
        slice(print, model, fixture_config(5., "bridgeawarecubic"));
        const PrintObject &object = *print.get_object(size_t(0));
        const Layer &layer = *object.layers()[object.layers().size() / 2];
        const LayerMaterial material = printed_material(layer, pattern_groups(object));
        const ExPolygons strip = surfaces_of(layer, [](const LayerRegion &region, const Surface &surface) {
            return region.region().config().sparse_infill_density.value >= 100. || surface.surface_type == stInternalSolid; });
        REQUIRE_FALSE(strip.empty());
        CHECK(area(strip) * SCALING_FACTOR * SCALING_FACTOR < 0.4 * 30.);
        CHECK(diff_ex(offset_ex(strip, -scale_(0.001)), material.may_print).empty());
        // Outside the wall band (the strip's fill surface reaches into the walls by the infill overlap, where it is material).
        CHECK(intersection_ex(offset_ex(diff_ex(strip, material.walls), -scale_(0.001)), material.surely_prints).empty());
        CHECK(diff_ex(offset_ex(material.walls, -scale_(0.001)), material.surely_prints).empty());
        CHECK(diff_ex(offset_ex(material.walls, -scale_(0.001)), material.may_print).empty());
        const ExPolygons sparse = diff_ex(surfaces_of(layer, is_sparse), material.walls);
        CHECK_FALSE(intersection_ex(material.void_area, sparse).empty());
        CHECK(diff_ex(offset_ex(sparse, -scale_(0.001)), material.void_area).empty());
        REQUIRE(material.void_of_group.size() == 1);
        CHECK_FALSE(material.void_of_group.front().empty());
    }

    SECTION("the fill boundary is the island inset by half a spacing, less the courses' beads") {
        const double s_f = 0.357, w = 0.4;
        const ExPolygon island(Polygon({ Point::new_scale(0., 0.), Point::new_scale(30., 0.), Point::new_scale(30., 20.), Point::new_scale(0., 20.) }));
        const ExPolygons plain = fill_boundary(island, {}, s_f, w);
        const ExPolygons expected = offset_ex(island, float(coord_t(float(scale_(-0.5 * s_f)))), ClipperLib::jtMiter, 3.);
        CHECK(diff_ex(plain, expected).empty());
        CHECK(diff_ex(expected, plain).empty());

        const Polylines courses{ Polyline(Point::new_scale(-1., 10.), Point::new_scale(31., 10.)) };
        const ExPolygons cut = fill_boundary(island, courses, s_f, w);
        const double keep = 0.5 * w - 0.05;    // a line ends inside the course's bead (3.8 as amended): the filler's nub fuses it
        const AreaIndex index(cut);
        CHECK_FALSE(index.contains(Point::new_scale(15., 10. + keep - 0.01)));
        CHECK(index.contains(Point::new_scale(15., 10. + keep + 0.01)));
        CHECK_FALSE(index.contains(Point::new_scale(15., 10. - keep + 0.01)));
        CHECK(index.contains(Point::new_scale(15., 10. - keep - 0.01)));

        std::mt19937 rng(7);
        std::uniform_real_distribution<double> x(-1., 31.), y(-1., 21.);
        for (int i = 0; i < 1000; ++i) {
            const Point p = Point::new_scale(x(rng), y(rng));
            const bool expected_inside = std::any_of(cut.begin(), cut.end(), [&p](const ExPolygon &part) { return part.contains(p); });
            CHECK(index.contains(p) == expected_inside);
        }
        const auto intervals = index.segment_inside(Point::new_scale(15., -2.), Point::new_scale(15., 22.));
        REQUIRE(intervals.size() == 2);
        const double inset = unscale<double>(coord_t(float(scale_(0.5 * s_f))));
        // Along y from −2 to 22 (24 mm): the boundary runs from the inset edge to the course band and from the band to the edge.
        CHECK(-2. + intervals[0].first * 24. == Approx(inset).margin(1e-5));
        CHECK(-2. + intervals[0].second * 24. == Approx(10. - keep).margin(1e-5));
        CHECK(-2. + intervals[1].first * 24. == Approx(10. + keep).margin(1e-5));
        CHECK(-2. + intervals[1].second * 24. == Approx(20. - inset).margin(1e-5));
        CHECK(index.segment_inside(Point::new_scale(-5., 10.), Point::new_scale(-3., 10.)).empty());
    }
}
