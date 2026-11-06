#ifndef slic3r_BridgeAwareCubicSerialization_hpp_
#define slic3r_BridgeAwareCubicSerialization_hpp_

#include "../ExPolygon.hpp"
#include "../Polyline.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace Slic3r { namespace BridgeAwareCubic {

// Exact XY command coordinates from GCode::point_to_gcode followed by the
// writer's plate-origin subtraction and three-decimal std::round formatting.
// All offsets are millimetres; shift must be unscale(instance.shift), and the
// nozzle offset must be the effective offset of this actual emitted tool.
// Preserve the operation order: precombining offsets or translating scaled
// integers first can change which side of a rounding tie a point occupies.
//
// Results share a command frame ONLY for a common effective nozzle offset.
// Comparing different tools physically requires their compensation as well;
// an inverse transform rounded onto Point's grid is not exact in general.
// operator() is pointwise; emitted_polyline also validates and models BAC's
// retained source segments. Neither models other export edits or repairs topology.
struct SerializedXY {
    Vec2d shift = Vec2d::Zero();
    Vec2d nozzle_offset = Vec2d::Zero();
    Vec2d plate_origin = Vec2d::Zero();

    Point operator()(const Point &point) const
    {
        return Point(axis(point.x(), 0), axis(point.y(), 1));
    }

    Polyline operator()(const Polyline &path) const
    {
        if (!path.fitting_result.empty())
            throw std::invalid_argument("Bridge-aware cubic XY serialization requires unfitted straight paths.");
        return Polyline(transform_points(path.points));
    }

    // Call this for every final model path and output phase, after path edits
    // and before emitting any part of that path. BAC retains even microscopic
    // nonzero source edges, so every original vertex needs a valid coordinate.
    void validate_path(const Polyline &path) const
    {
        if (!path.fitting_result.empty())
            throw std::invalid_argument("Bridge-aware cubic XY serialization requires unfitted straight paths.");
        for (const Point &point : path.points) (void)(*this)(point);
    }

    // The BAC branch of GCode::_extrude emits every nonzero source segment,
    // including segments shorter than the ordinary exporter's EPSILON cutoff.
    // Skip only exact source duplicates, never equal rounded XY positions:
    // nonzero source segments can emit E while their output points coincide.
    // The caller retains the source extrusion's width, height, flow and role.
    Polyline emitted_polyline(const Polyline &path) const
    {
        validate_path(path);
        Points result;
        result.reserve(path.size());
        if (!path.empty()) result.push_back((*this)(path.first_point()));
        for (const Line &line : path.lines())
            if (line.a != line.b)
                result.push_back((*this)(line.b));
        return Polyline(std::move(result));
    }

    Polylines operator()(const Polylines &paths) const
    {
        Polylines result;
        result.reserve(paths.size());
        for (const Polyline &path : paths) result.push_back((*this)(path));
        return result;
    }

    Polygon operator()(const Polygon &polygon) const
    {
        return Polygon(transform_points(polygon.points));
    }

    ExPolygon operator()(const ExPolygon &polygon) const
    {
        ExPolygon result;
        result.contour = (*this)(polygon.contour);
        result.holes.reserve(polygon.holes.size());
        for (const Polygon &hole : polygon.holes) result.holes.push_back((*this)(hole));
        return result;
    }

    ExPolygons operator()(const ExPolygons &polygons) const
    {
        ExPolygons result;
        result.reserve(polygons.size());
        for (const ExPolygon &polygon : polygons) result.push_back((*this)(polygon));
        return result;
    }

private:
    coord_t axis(coord_t coordinate, unsigned index) const
    {
        double value = unscale<double>(coordinate);
        value += shift[index];
        value -= nozzle_offset[index];
        value -= plate_origin[index];
        const double rounded = std::round(value * 1000.);
        if (!std::isfinite(rounded) || rounded < double(std::numeric_limits<coord_t>::min()) ||
            rounded >= double(std::numeric_limits<coord_t>::max()))
            throw std::out_of_range("Bridge-aware cubic XY serialization has a non-finite or out-of-range coordinate.");
        const coord_t thousandths = coord_t(rounded);
        // Convert the emitted decimal integer directly. Passing its double
        // millimetre value through Point::new_scale would truncate again.
        static_assert(SCALING_FACTOR == .00001, "Update serialized XY grid conversion when Point precision changes.");
        constexpr coord_t point_units_per_thousandth = 100;
        if (thousandths < std::numeric_limits<coord_t>::min() / point_units_per_thousandth ||
            thousandths > std::numeric_limits<coord_t>::max() / point_units_per_thousandth)
            throw std::out_of_range("Bridge-aware cubic serialized XY coordinate exceeds the geometry grid.");
        return thousandths * point_units_per_thousandth;
    }

    Points transform_points(const Points &points) const
    {
        Points result;
        result.reserve(points.size());
        for (const Point &point : points) result.push_back((*this)(point));
        return result;
    }
};

}} // namespace Slic3r::BridgeAwareCubic

#endif
