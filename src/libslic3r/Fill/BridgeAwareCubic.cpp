#include "BridgeAwareCubic.hpp"

#include "../ClipperUtils.hpp"
#include "../Geometry/ConvexHull.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <tuple>

namespace Slic3r { namespace BridgeAwareCubic {

namespace {

constexpr double sqrt_three = 1.7320508075688772935;
// Covers both endpoint rounding and the section polygon's rounding. Never add
// this allowance to the user's limit: construction aims below that limit.
constexpr double geometry_tolerance = 4. * SCALING_FACTOR;

struct Cell {
    // Integer address at this depth. Keeping dyadic addresses avoids floating
    // point aliases when adjacent selected parents share a partition plane.
    std::array<int64_t, 3> address;
    unsigned depth = 0;
    bool split = false;
    size_t reason_demand = std::numeric_limits<size_t>::max();
    double reason_diameter = 0.;
};

struct PlaneKey {
    unsigned axis;
    unsigned level;
    int64_t numerator;

    bool operator<(const PlaneKey &rhs) const
    {
        return std::tie(axis, level, numerator) < std::tie(rhs.axis, rhs.level, rhs.numerator);
    }
};

using Interval = std::pair<double, double>;
using PlaneIntervals = std::map<PlaneKey, std::vector<Interval>>;

Point scaled_point(const Vec3d &point)
{
    constexpr double limit = double(std::numeric_limits<coord_t>::max()) - 1.;
    const double x = point.x() / SCALING_FACTOR;
    const double y = point.y() / SCALING_FACTOR;
    if (!std::isfinite(x) || !std::isfinite(y) || std::abs(x) > limit || std::abs(y) > limit)
        throw std::runtime_error("Bridge aware cubic cell exceeds the representable coordinate range");
    return Point(coord_t(std::llround(x)), coord_t(std::llround(y)));
}

double section_diameter(const Polygon &polygon)
{
    double squared = 0.;
    for (size_t i = 0; i < polygon.points.size(); ++i)
        for (size_t j = i + 1; j < polygon.points.size(); ++j)
            squared = std::max(squared, (polygon.points[i].cast<double>() -
                                        polygon.points[j].cast<double>()).squaredNorm());
    return std::sqrt(squared) * SCALING_FACTOR;
}

bool bounds_overlap(const BoundingBox &a, const BoundingBox &b)
{
    // Inclusive comparison preserves finite-width footprints along a face.
    return a.defined && b.defined && a.min.x() <= b.max.x() && a.max.x() >= b.min.x() &&
           a.min.y() <= b.max.y() && a.max.y() >= b.min.y();
}

} // namespace

struct Plan::Impl {
    Parameters parameters;
    std::vector<Demand> demands;
    Statistics statistics;
    double bulk_edge = 0.;
    Matrix3d to_world;
    Vec3d origin;
    std::vector<Cell> cells;
    std::vector<BoundingBox> demand_bounds;

    Impl(Parameters input_parameters, std::vector<Demand> input_demands)
        : parameters(std::move(input_parameters)), demands(std::move(input_demands))
    {
        validate();
        cancel();
        bulk_edge = std::sqrt(6.) * parameters.flow_spacing / parameters.density;
        if (!std::isfinite(bulk_edge) || bulk_edge >
                double(std::numeric_limits<coord_t>::max()) * SCALING_FACTOR)
            throw std::invalid_argument("Bridge aware cubic bulk spacing is not representable; increase sparse infill density");

        origin = Vec3d(.5 * (parameters.bounds.min.x() + parameters.bounds.max.x()),
                       .5 * (parameters.bounds.min.y() + parameters.bounds.max.y()), 0.);
        const double angle = std::remainder(parameters.angle, 2. * PI) + .5 * PI;
        for (unsigned axis = 0; axis < 3; ++axis) {
            const double azimuth = angle + axis * 2. * PI / 3.;
            to_world.col(axis) = Vec3d(std::sqrt(2. / 3.) * std::cos(azimuth),
                                      std::sqrt(2. / 3.) * std::sin(azimuth), 1. / sqrt_three);
        }

        Vec3d low = Vec3d::Constant(std::numeric_limits<double>::infinity());
        Vec3d high = -low;
        for (unsigned corner = 0; corner < 8; ++corner) {
            Vec3d world;
            for (unsigned axis = 0; axis < 3; ++axis)
                world[axis] = corner & (1u << axis) ? parameters.bounds.max[axis] : parameters.bounds.min[axis];
            const Vec3d local = to_world.transpose() * (world - origin);
            low = low.cwiseMin(local);
            high = high.cwiseMax(local);
        }
        std::array<int64_t, 3> first, last;
        size_t root_count = 1;
        for (unsigned axis = 0; axis < 3; ++axis) {
            const double lo = std::floor(low[axis] / bulk_edge);
            const double hi = std::floor(high[axis] / bulk_edge);
            // Child addresses and midpoint numerators must remain exact signed
            // integers throughout the configured depth range.
            const double address_limit = double(std::numeric_limits<int64_t>::max() >> (parameters.max_depth + 2));
            if (!std::isfinite(lo) || !std::isfinite(hi) || std::abs(lo) > address_limit ||
                    std::abs(hi) > address_limit)
                throw std::runtime_error("Bridge aware cubic lattice address exceeds the supported range");
            first[axis] = int64_t(lo);
            last[axis] = int64_t(hi);
            const uint64_t count = uint64_t(last[axis] - first[axis]) + 1;
            if (count > parameters.max_nodes || root_count > parameters.max_nodes / count)
                throw std::runtime_error("Bridge aware cubic exceeds the cell work limit while constructing its bulk lattice");
            root_count *= size_t(count);
        }

        demand_bounds.reserve(demands.size());
        std::vector<size_t> candidates;
        for (size_t i = 0; i < demands.size(); ++i) {
            demand_bounds.push_back(get_extents(demands[i].footprint));
            if (!demands[i].footprint.empty())
                candidates.push_back(i);
        }
        cells.reserve(std::min(parameters.max_nodes, root_count * 2));
        statistics.root_cells = root_count;
        size_t work = 0;
        for (int64_t u = first[0]; u <= last[0]; ++u)
            for (int64_t v = first[1]; v <= last[1]; ++v)
                for (int64_t w = first[2]; w <= last[2]; ++w) {
                    cancel();
                    append_and_refine(Cell { { u, v, w } }, candidates, work);
                }
        statistics.nodes = cells.size();
        statistics.leaves = statistics.nodes - statistics.splits;
    }

    void cancel() const
    {
        if (parameters.throw_if_canceled)
            parameters.throw_if_canceled();
    }

    void validate() const
    {
        if (!parameters.bounds.defined || !parameters.bounds.min.allFinite() ||
                !parameters.bounds.max.allFinite() ||
                (parameters.bounds.max.array() <= parameters.bounds.min.array()).any())
            throw std::invalid_argument("Bridge aware cubic requires finite, nonempty object bounds");
        if (!std::isfinite(parameters.flow_spacing) || parameters.flow_spacing <= geometry_tolerance ||
                !std::isfinite(parameters.density) || parameters.density <= 0. || parameters.density >= 1. ||
                !std::isfinite(parameters.angle))
            throw std::invalid_argument("Bridge aware cubic requires positive representable flow spacing, fractional sparse density, and finite infill direction");
        if (parameters.max_nodes == 0 || parameters.max_nodes > std::numeric_limits<size_t>::max() / 64 ||
                parameters.max_depth > 30)
            throw std::invalid_argument("Bridge aware cubic has an invalid cell or subdivision work limit");
        if (demands.size() > parameters.max_nodes)
            throw std::runtime_error("Bridge aware cubic exceeds the support-demand work limit");
        size_t point_count = 0;
        for (const Demand &demand : demands) {
            if (!std::isfinite(demand.z) || demand.z < parameters.bounds.min.z() ||
                    demand.z > parameters.bounds.max.z() ||
                    !std::isfinite(demand.gap) || demand.gap <= geometry_tolerance ||
                    demand.gap > double(std::numeric_limits<coord_t>::max()) * SCALING_FACTOR)
                throw std::invalid_argument("Bridge aware cubic requires a positive representable gap and a support elevation within the object bounds");
            for (const ExPolygon &footprint : demand.footprint) {
                point_count += footprint.contour.points.size();
                for (const Polygon &hole : footprint.holes)
                    point_count += hole.points.size();
                if (point_count > parameters.max_nodes * 16)
                    throw std::runtime_error("Bridge aware cubic exceeds the support-footprint work limit");
                if (!footprint.is_valid() || footprint.contour.area() <= 0. ||
                        std::any_of(footprint.holes.begin(), footprint.holes.end(),
                                    [](const Polygon &hole) { return hole.area() >= 0.; }))
                    throw std::invalid_argument("Bridge aware cubic requires valid finite-width support footprints");
            }
        }
    }

    double edge(const Cell &cell) const { return std::ldexp(bulk_edge, -int(cell.depth)); }

    Vec3d lower(const Cell &cell) const
    {
        const double size = edge(cell);
        return Vec3d(double(cell.address[0]) * size, double(cell.address[1]) * size,
                     double(cell.address[2]) * size);
    }

    Polygon section(const Cell &cell, double z) const
    {
        const double size = edge(cell);
        const Vec3d lo = lower(cell);
        const double t = z * sqrt_three - lo.sum();
        if (t <= 0. || t >= 3. * size)
            return {};
        Points vertices;
        vertices.reserve(12);
        const double numeric_tolerance = std::numeric_limits<double>::epsilon() *
            64. * std::max({ 1., size, lo.cwiseAbs().maxCoeff() });
        // A section vertex lies on one of the twelve physical cube edges.
        for (unsigned free_axis = 0; free_axis < 3; ++free_axis) {
            const unsigned a = (free_axis + 1) % 3;
            const unsigned b = (free_axis + 2) % 3;
            for (unsigned bits = 0; bits < 4; ++bits) {
                Vec3d p = Vec3d::Zero();
                p[a] = bits & 1 ? size : 0.;
                p[b] = bits & 2 ? size : 0.;
                p[free_axis] = t - p[a] - p[b];
                if (p[free_axis] >= -numeric_tolerance && p[free_axis] <= size + numeric_tolerance) {
                    p[free_axis] = std::clamp(p[free_axis], 0., size);
                    vertices.push_back(scaled_point(origin + to_world * (lo + p)));
                }
            }
        }
        return Geometry::convex_hull(std::move(vertices));
    }

    void append_and_refine(Cell cell, const std::vector<size_t> &candidates, size_t &work)
    {
        if (cells.size() >= parameters.max_nodes)
            throw std::runtime_error("Bridge aware cubic exceeds the cell work limit during local support refinement");
        const size_t id = cells.size();
        cells.push_back(cell);
        statistics.max_depth = std::max(statistics.max_depth, cell.depth);
        std::vector<size_t> local;
        for (const size_t demand_id : candidates) {
            if (++work > parameters.max_nodes * 64)
                throw std::runtime_error("Bridge aware cubic exceeds the footprint-intersection work limit");
            if ((work & 255) == 0)
                cancel();
            const Demand &demand = demands[demand_id];
            const Polygon polygon = section(cell, demand.z);
            if (polygon.empty() || !bounds_overlap(get_extents(polygon), demand_bounds[demand_id]) ||
                    intersection_ex(ExPolygon(polygon), demand.footprint).empty())
                continue;
            local.push_back(demand_id);
            const double diameter = section_diameter(polygon);
            if (diameter > demand.gap - geometry_tolerance && !cell.split) {
                cell.split = true;
                cell.reason_demand = demand_id;
                cell.reason_diameter = diameter;
            }
        }
        if (!cell.split)
            return;
        // For this physical-cell convention density is sqrt(6)*spacing/edge.
        // A child doubles local density. Once that reaches solid density the
        // line-wall hierarchy needs a separately validated local solid fill;
        // silently adding overlapping walls is not a saturation strategy.
        const bool saturated = std::ldexp(parameters.density, int(cell.depth + 1)) >= 1.;
        if (cell.depth >= parameters.max_depth || saturated || edge(cell) * .5 < parameters.flow_spacing) {
            std::ostringstream message;
            message << "Bridge aware cubic cannot meet gap " << demands[cell.reason_demand].gap
                    << " mm at support Z " << demands[cell.reason_demand].z
                    << " mm: remaining cell diameter " << cell.reason_diameter
                    << " mm reaches the " << (cell.depth >= parameters.max_depth ? "subdivision work" :
                                               saturated ? "dense saturation" : "printable spacing")
                    << " limit";
            throw std::runtime_error(message.str());
        }
        cells[id] = cell;
        ++statistics.splits;
        // Selection is repeated against the original finite-width fragments in
        // every child. Neither a target depth nor a clipped-mask boundary is
        // inherited as a reason to subdivide an untouched sibling.
        for (unsigned octant = 0; octant < 8; ++octant) {
            Cell child;
            child.depth = cell.depth + 1;
            for (unsigned axis = 0; axis < 3; ++axis)
                child.address[axis] = cell.address[axis] * 2 + ((octant >> axis) & 1);
            append_and_refine(child, local, work);
        }
    }

    bool has_elevation(double z) const
    {
        if (!std::isfinite(z))
            throw std::invalid_argument("Bridge aware cubic requires a finite output elevation");
        return z >= parameters.bounds.min.z() && z <= parameters.bounds.max.z();
    }

    // Complete midplane intersections, from their inherited-edge birth to
    // their inherited-edge death. The footprint never clips these intervals.
    PlaneIntervals refinement_intervals(double z) const
    {
        PlaneIntervals result;
        size_t visited = 0;
        for (const Cell &cell : cells) {
            if ((++visited & 255) == 0)
                cancel();
            if (!cell.split)
                continue;
            const double size = edge(cell);
            const Vec3d lo = lower(cell);
            const double t = z * sqrt_three - lo.sum() - .5 * size;
            if (t <= 0. || t >= 2. * size)
                continue;
            for (unsigned axis = 0; axis < 3; ++axis) {
                const unsigned a = (axis + 1) % 3;
                const double first = lo[a] + std::max(0., t - size);
                const double last = lo[a] + std::min(size, t);
                if (last > first)
                    result[{ axis, cell.depth + 1, cell.address[axis] * 2 + 1 }].emplace_back(first, last);
            }
        }
        return result;
    }

    Polylines refinement(double z, const ExPolygon &domain) const
    {
        Polylines raw;
        PlaneIntervals planes = refinement_intervals(z);
        for (auto &entry : planes) {
            cancel();
            const PlaneKey &plane = entry.first;
            auto &intervals = entry.second;
            std::sort(intervals.begin(), intervals.end());
            const double fixed = std::ldexp(bulk_edge * double(plane.numerator), -int(plane.level));
            const unsigned a = (plane.axis + 1) % 3;
            const unsigned b = (plane.axis + 2) % 3;
            auto emit = [&](double first, double last) {
                Vec3d p = Vec3d::Zero();
                p[plane.axis] = fixed;
                p[a] = first;
                p[b] = z * sqrt_three - fixed - first;
                const Point p0 = scaled_point(origin + to_world * p);
                p[a] = last;
                p[b] = z * sqrt_three - fixed - last;
                const Point p1 = scaled_point(origin + to_world * p);
                if (p0 != p1)
                    raw.emplace_back(p0, p1);
            };
            double first = intervals.front().first;
            double last = intervals.front().second;
            for (size_t i = 1; i < intervals.size(); ++i) {
                // Only bridge floating arithmetic roundoff, never an actual
                // gap or a material hole. Domain clipping is done afterward.
                const double eps = 64. * std::numeric_limits<double>::epsilon() *
                    std::max({ 1., std::abs(last), std::abs(intervals[i].first) });
                if (intervals[i].first <= last + eps)
                    last = std::max(last, intervals[i].second);
                else {
                    emit(first, last);
                    first = intervals[i].first;
                    last = intervals[i].second;
                }
            }
            emit(first, last);
        }
        return intersection_pl(raw, domain);
    }

    Polylines bulk(double z, const ExPolygon &domain) const
    {
        Polylines raw;
        if (domain.contour.empty())
            return raw;
        const BoundingBox bounds = get_extents(domain);
        const Vec2d lo = unscaled(bounds.min);
        const Vec2d hi = unscaled(bounds.max);
        const Vec2d center = .5 * (lo + hi);
        size_t count = 0;
        for (unsigned axis = 0; axis < 3; ++axis) {
            const Vec2d normal = to_world.col(axis).head<2>();
            const Vec2d direction(-normal.y(), normal.x());
            double low = std::numeric_limits<double>::infinity();
            double high = -low;
            for (unsigned corner = 0; corner < 4; ++corner) {
                const Vec2d xy(corner & 1 ? hi.x() : lo.x(), corner & 2 ? hi.y() : lo.y());
                const double local = normal.dot(xy - origin.head<2>()) + z / sqrt_three;
                low = std::min(low, local);
                high = std::max(high, local);
            }
            const int64_t first = int64_t(std::ceil(low / bulk_edge));
            const int64_t last = int64_t(std::floor(high / bulk_edge));
            if (last >= first && uint64_t(last - first) >= parameters.max_nodes)
                throw std::runtime_error("Bridge aware cubic exceeds the output line work limit");
            for (int64_t plane = first; plane <= last; ++plane) {
                if (++count > parameters.max_nodes)
                    throw std::runtime_error("Bridge aware cubic exceeds the output line work limit");
                if ((count & 255) == 0)
                    cancel();
                const double offset = plane * bulk_edge - z / sqrt_three;
                const Vec2d midpoint = center + normal *
                    ((offset - normal.dot(center - origin.head<2>())) / normal.squaredNorm());
                // Clip the infinite line in doubles before scaling. Arbitrary
                // long endpoints can exceed coord_t even when every point in
                // the actual printable domain is representable.
                double begin = -std::numeric_limits<double>::infinity();
                double end = std::numeric_limits<double>::infinity();
                for (unsigned xy_axis = 0; xy_axis < 2; ++xy_axis) {
                    if (std::abs(direction[xy_axis]) < 1.e-15) {
                        if (midpoint[xy_axis] < lo[xy_axis] - geometry_tolerance ||
                                midpoint[xy_axis] > hi[xy_axis] + geometry_tolerance)
                            end = begin;
                    } else {
                        double a = (lo[xy_axis] - midpoint[xy_axis]) / direction[xy_axis];
                        double b = (hi[xy_axis] - midpoint[xy_axis]) / direction[xy_axis];
                        if (a > b)
                            std::swap(a, b);
                        begin = std::max(begin, a);
                        end = std::min(end, b);
                    }
                }
                if (end > begin && std::isfinite(begin) && std::isfinite(end)) {
                    const Vec2d p0 = (midpoint + direction * begin).cwiseMax(lo).cwiseMin(hi);
                    const Vec2d p1 = (midpoint + direction * end).cwiseMax(lo).cwiseMin(hi);
                    const Point first_point = scaled_point(Vec3d(p0.x(), p0.y(), z));
                    const Point last_point = scaled_point(Vec3d(p1.x(), p1.y(), z));
                    if (first_point != last_point)
                        raw.emplace_back(first_point, last_point);
                }
            }
        }
        return intersection_pl(raw, domain);
    }
};

Plan::Plan(Parameters parameters, std::vector<Demand> demands)
    : m_impl(std::make_unique<Impl>(std::move(parameters), std::move(demands))) {}
Plan::~Plan() = default;
Plan::Plan(Plan&&) noexcept = default;
Plan& Plan::operator=(Plan&&) noexcept = default;

Polylines Plan::lines(double z, const ExPolygon &domain) const
{
    m_impl->cancel();
    if (!m_impl->has_elevation(z))
        return {};
    Polylines result = m_impl->bulk(z, domain);
    Polylines added = m_impl->refinement(z, domain);
    result.insert(result.end(), std::make_move_iterator(added.begin()), std::make_move_iterator(added.end()));
    return result;
}

Polylines Plan::refinement_lines(double z, const ExPolygon &domain) const
{
    m_impl->cancel();
    return m_impl->has_elevation(z) ? m_impl->refinement(z, domain) : Polylines {};
}

ExPolygons Plan::refinement_area(double z) const
{
    m_impl->cancel();
    if (!m_impl->has_elevation(z))
        return {};
    Polygons selected;
    size_t visited = 0;
    for (const Cell &cell : m_impl->cells) {
        if ((++visited & 255) == 0)
            m_impl->cancel();
        if (!cell.split)
            continue;
        const double size = m_impl->edge(cell);
        const double t = z * sqrt_three - m_impl->lower(cell).sum() - .5 * size;
        if (t > 0. && t < 2. * size) {
            Polygon polygon = m_impl->section(cell, z);
            if (!polygon.empty())
                selected.push_back(std::move(polygon));
        }
    }
    return union_ex(selected);
}

std::vector<LeafSection> Plan::leaf_sections(double z) const
{
    m_impl->cancel();
    if (!m_impl->has_elevation(z))
        return {};
    std::vector<LeafSection> sections;
    for (size_t id = 0; id < m_impl->cells.size(); ++id) {
        if ((id & 255) == 0)
            m_impl->cancel();
        const Cell &cell = m_impl->cells[id];
        if (!cell.split) {
            Polygon polygon = m_impl->section(cell, z);
            if (!polygon.empty())
                sections.push_back({ id, cell.depth, m_impl->edge(cell), std::move(polygon) });
        }
    }
    return sections;
}

const Parameters& Plan::parameters() const { return m_impl->parameters; }
const std::vector<Demand>& Plan::demands() const { return m_impl->demands; }
const Statistics& Plan::statistics() const { return m_impl->statistics; }
double Plan::bulk_cell_edge() const { return m_impl->bulk_edge; }

}} // namespace Slic3r::BridgeAwareCubic
