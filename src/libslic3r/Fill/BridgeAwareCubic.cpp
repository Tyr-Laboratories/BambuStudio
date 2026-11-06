#include "BridgeAwareCubic.hpp"
#include "BridgeAwareCubicClipMask.hpp"
#include "BridgeAwareCubicValidation.hpp"

#include "../ClipperUtils.hpp"
#include "../Geometry/ConvexHull.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <limits>
#include <map>
#include <numeric>
#include <sstream>
#include <set>
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
    bool solid = false;
    size_t reason_demand = std::numeric_limits<size_t>::max();
    double reason_diameter = 0.;
    std::vector<size_t> reason_demands;
    // Exact successful footprint predicates, including demands whose gap did
    // not split this node. Retained only on split nodes, for their children.
    std::vector<size_t> intersecting_demands;

    CellId id() const { return { address, depth }; }
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

struct TaggingUnavailable {};
struct TaggingCanceled { std::exception_ptr exception; };

// Optional metadata has its own bounded admission in addition to the Plan's
// ordinary output bounds. Count whole vector capacities and conservatively
// keep retired temporary allocations charged for this one request. Thus the
// admitted total bounds peak metadata storage, including reserve relocation.
class TaggingWork {
public:
    TaggingWork(size_t capacity, BridgeAwareCubicValidation::WorkBudget *budget,
                const std::function<void()> &cancel, bool owned_geometry = false)
        : m_capacity(std::min(capacity, size_t(200000))),
          m_budget(budget == nullptr ? &m_local : budget), m_cancel(cancel), m_owned_geometry(owned_geometry) {}
    bool owns_geometry() const noexcept { return m_owned_geometry; }
    void poll() const
    {
        if (m_cancel) {
            try { m_cancel(); }
            catch (...) { throw TaggingCanceled{std::current_exception()}; }
        }
    }
    void charge(size_t count = 1)
    {
        if (count > m_remaining || count > m_budget->remaining_work) throw TaggingUnavailable{};
        m_remaining -= count;
        m_budget->remaining_work -= count;
        m_budget->charged_work[size_t(BridgeAwareCubicValidation::WorkCategory::Validation)] += count;
        m_since_poll += count;
        if (m_since_poll >= 256) { m_since_poll = 0; poll(); }
    }
    void admit(size_t units, size_t records)
    {
        charge();
        if (units > m_capacity - m_units || records > 20000 - m_records) throw TaggingUnavailable{};
        m_units += units;
        m_records += records;
    }
    template<class T> static constexpr size_t units_for()
    {
        return (sizeof(T) + sizeof(Point) - 1) / sizeof(Point);
    }
    template<class T> void reserve(std::vector<T> &items, size_t capacity)
    {
        if (capacity <= items.capacity()) return;
        if (capacity > 20000) throw TaggingUnavailable{};
        // The old buffer may coexist with the new allocation while moving.
        admit(capacity * units_for<T>(), capacity);
        charge(capacity);
        items.reserve(capacity);
        if (items.capacity() > capacity)
            admit((items.capacity() - capacity) * units_for<T>(), items.capacity() - capacity);
    }
    template<class T> void grow(std::vector<T> &items)
    {
        if (items.size() == items.capacity())
            reserve(items, std::min(size_t(20000), std::max(size_t(1), items.capacity() * 2)));
        if (items.size() == items.capacity()) throw TaggingUnavailable{};
    }
    void lookup(size_t items)
    {
        charge();
        for (; items != 0; items >>= 1) charge(2);
    }
private:
    size_t m_capacity, m_units = 0, m_records = 0, m_remaining = 50000000, m_since_poll = 0;
    BridgeAwareCubicValidation::WorkBudget m_local;
    BridgeAwareCubicValidation::WorkBudget *m_budget;
    const std::function<void()> &m_cancel;
    bool m_owned_geometry;
};

} // namespace

struct Plan::Impl {
    // Identity consumers retain only this nonempty nonce, never the Plan's
    // geometry or callback captures. Each constructor below initializes a
    // fresh token; exact clones share the immutable Impl and its token.
    struct Identity { unsigned char nonce = 0; };
    const std::shared_ptr<const Identity> identity = std::make_shared<const Identity>();
    Parameters parameters;
    std::vector<Demand> demands;
    Statistics statistics;
    double bulk_edge = 0.;
    Matrix3d to_world;
    Vec3d origin;
    std::vector<Cell> cells;
    std::vector<BoundingBox> demand_bounds;
    std::vector<BoundedClipMask> demand_masks;
    std::vector<LayerClosure> closures;
    std::vector<LayerAttachment> layer_attachments;
    std::set<CellId> forced_leaves;
    std::set<CellId> active_owners;
    std::vector<Refinement> refinements;
    std::vector<size_t> active_demands;

    Impl(const Impl &) = delete;

    Impl(Parameters input_parameters, std::vector<Demand> input_demands,
         std::vector<LayerClosure> input_closures = {}, std::vector<LayerAttachment> input_attachments = {},
         std::set<CellId> input_forced_leaves = {})
        : parameters(std::move(input_parameters)), demands(std::move(input_demands)),
          closures(std::move(input_closures)), layer_attachments(std::move(input_attachments)),
          forced_leaves(std::move(input_forced_leaves))
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
        demand_masks.reserve(demands.size());
        for (size_t i = 0; i < demands.size(); ++i) {
            if ((i & 255) == 0) cancel();
            demand_bounds.push_back(get_extents(demands[i].footprint));
            demand_masks.emplace_back(to_polygons(demands[i].footprint));
        }
        cells.reserve(std::min(parameters.max_nodes, root_count * 2));
        std::vector<bool> enabled(demands.size(), false);
        for (size_t i = 0; i < demands.size(); ++i)
            enabled[i] = !demands[i].dependency;
        size_t work = 0;
        // Begin every rebuild from DIRECT demands. Carrying an old activation
        // set into a collapsed trial would preserve unrooted dependency cycles.
        for (;;) {
            cells.clear();
            statistics = {};
            statistics.root_cells = root_count;
            active_demands.clear();
            for (size_t i = 0; i < enabled.size(); ++i)
                if (enabled[i] && !demands[i].footprint.empty()) active_demands.push_back(i);
            for (int64_t u = first[0]; u <= last[0]; ++u)
                for (int64_t v = first[1]; v <= last[1]; ++v)
                    for (int64_t w = first[2]; w <= last[2]; ++w) {
                        cancel();
                        append_and_refine(Cell { { u, v, w } }, active_demands, work);
                    }
            active_owners.clear();
            refinements.clear();
            for (const Cell &cell : cells) {
                if (!cell.split && !cell.solid) continue;
                active_owners.insert(cell.id());
                const double low_z = lower(cell).sum() / sqrt_three;
                const double size = edge(cell);
                refinements.push_back({ cell.id(), cell.solid,
                    low_z + (cell.solid ? 0. : .5 * size / sqrt_three),
                    low_z + (cell.solid ? 3. : 2.5) * size / sqrt_three, cell.reason_demands });
            }
            bool activated = false;
            for (size_t i = 0; i < enabled.size(); ++i)
                if (!enabled[i] && has_owner(demands[i].owners, &work)) {
                    enabled[i] = true;
                    activated = true;
                }
            if (!activated) break;
        }
        statistics.nodes = cells.size();
        statistics.leaves = statistics.nodes - statistics.splits;
    }

    Impl(const Impl &source, const CellId &forced_leaf)
        : parameters(source.parameters), demands(source.demands), bulk_edge(source.bulk_edge),
          to_world(source.to_world), origin(source.origin), demand_bounds(source.demand_bounds),
          demand_masks(source.demand_masks), closures(source.closures), layer_attachments(source.layer_attachments),
          forced_leaves(source.forced_leaves)
    {
        // All immutable inputs were already validated. Adding a forced leaf
        // can only remove nodes and reachable demands; it cannot introduce a
        // new geometric predicate or exceed the source's allocation bounds.
        cancel();
        forced_leaves.insert(forced_leaf);
        cells.reserve(source.cells.size());
        std::vector<size_t> subtree_end(source.cells.size(), source.cells.size());
        std::vector<size_t> ancestors;
        ancestors.reserve(parameters.max_depth + 1);
        for (size_t i = 0; i < source.cells.size(); ++i) {
            if ((i & 255) == 0) cancel();
            while (!ancestors.empty() && source.cells[ancestors.back()].depth >= source.cells[i].depth) {
                subtree_end[ancestors.back()] = i;
                ancestors.pop_back();
            }
            ancestors.push_back(i);
        }
        std::vector<bool> enabled(demands.size(), false);
        for (size_t i = 0; i < demands.size(); ++i)
            enabled[i] = !demands[i].dependency;
        size_t work = 0;
        for (;;) {
            cells.clear();
            statistics = {};
            statistics.root_cells = source.statistics.root_cells;
            active_demands.clear();
            for (size_t i = 0; i < enabled.size(); ++i)
                if (enabled[i] && !demands[i].footprint.empty()) active_demands.push_back(i);
            for (size_t index = 0; index < source.cells.size();) {
                cancel();
                append_cached(source.cells, subtree_end, index, active_demands, work);
            }
            active_owners.clear();
            refinements.clear();
            for (const Cell &cell : cells) {
                if (!cell.split && !cell.solid) continue;
                active_owners.insert(cell.id());
                const double low_z = lower(cell).sum() / sqrt_three;
                const double size = edge(cell);
                refinements.push_back({ cell.id(), cell.solid,
                    low_z + (cell.solid ? 0. : .5 * size / sqrt_three),
                    low_z + (cell.solid ? 3. : 2.5) * size / sqrt_three, cell.reason_demands });
            }
            bool activated = false;
            for (size_t i = 0; i < enabled.size(); ++i)
                if (!enabled[i] && has_owner(demands[i].owners, &work)) {
                    enabled[i] = true;
                    activated = true;
                }
            if (!activated) break;
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
        size_t owner_count = 0;
        const auto validate_owners = [&](const std::vector<CellId> &owners) {
            const size_t limit = parameters.max_nodes * 16;
            if (owners.size() > limit - owner_count)
                throw std::runtime_error("Bridge aware cubic exceeds the ownership reference work limit");
            owner_count += owners.size();
            for (size_t i = 0; i < owners.size(); ++i) {
                if ((i & 255) == 0) cancel();
                if (owners[i].depth > parameters.max_depth)
                    throw std::invalid_argument("Bridge aware cubic owner exceeds the configured lattice depth");
            }
        };
        size_t validated = 0;
        for (const Demand &demand : demands) {
            if ((++validated & 255) == 0) cancel();
            validate_owners(demand.owners);
            if (demand.dependency && demand.owners.empty())
                throw std::invalid_argument("Bridge aware cubic dependencies require an actual refinement owner");
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
        if (closures.size() + layer_attachments.size() > parameters.max_nodes)
            throw std::runtime_error("Bridge aware cubic exceeds the immutable path payload work limit");
        for (const LayerClosure &closure : closures) {
            if ((++validated & 255) == 0) cancel();
            validate_owners(closure.owners);
            if (!std::isfinite(closure.z) || closure.z < parameters.bounds.min.z() || closure.z > parameters.bounds.max.z())
                throw std::invalid_argument("Bridge aware cubic closure elevation is outside the object bounds");
            for (const Polyline &path : closure.paths) {
                point_count += path.points.size();
                if (path.points.size() < 2)
                    throw std::invalid_argument("Bridge aware cubic closure paths require two endpoints");
            }
            for (const ExPolygon &reservation : closure.reservation) {
                point_count += reservation.contour.points.size();
                for (const Polygon &hole : reservation.holes) point_count += hole.points.size();
                if (!reservation.is_valid())
                    throw std::invalid_argument("Bridge aware cubic closure reservations require valid polygons");
            }
        }
        for (const LayerAttachment &attachment : layer_attachments) {
            if ((++validated & 255) == 0) cancel();
            validate_owners(attachment.owners);
            point_count += attachment.original.points.size() + attachment.replacement.points.size();
            if (!std::isfinite(attachment.z) || !std::isfinite(attachment.width) ||
                    !std::isfinite(attachment.height) || attachment.width <= 0. || attachment.height <= 0. ||
                    attachment.original.size() < 2 || attachment.replacement.size() < 2)
                throw std::invalid_argument("Bridge aware cubic attachments require finite deposition geometry and paths");
            const auto count_selection_ring = [&](const Polygon &ring) {
                const size_t limit = parameters.max_nodes * 16;
                if (point_count > limit || ring.points.size() > limit - point_count)
                    throw std::runtime_error("Bridge aware cubic exceeds the immutable path payload vertex work limit");
                point_count += ring.points.size();
                // Match the final verifier's safe arithmetic range before
                // externally supplied selection reaches polygon operations.
                constexpr coord_t coordinate_limit = std::numeric_limits<coord_t>::max() / 4;
                for (size_t i = 0; i < ring.points.size(); ++i) {
                    if ((i & 255) == 0) cancel();
                    const Point &point = ring.points[i];
                    if (point.x() < -coordinate_limit || point.x() > coordinate_limit ||
                            point.y() < -coordinate_limit || point.y() > coordinate_limit)
                        throw std::invalid_argument("Bridge aware cubic attachment demand selection exceeds finite coordinate bounds");
                }
            };
            for (const ExPolygon &selection : attachment.demand_selection) {
                if ((++validated & 255) == 0) cancel();
                count_selection_ring(selection.contour);
                for (const Polygon &hole : selection.holes) count_selection_ring(hole);
                if (!selection.is_valid() || selection.contour.area() <= 0. ||
                        std::any_of(selection.holes.begin(), selection.holes.end(),
                                    [](const Polygon &hole) { return hole.area() >= 0.; }))
                    throw std::invalid_argument("Bridge aware cubic attachment demand selections require valid finite-width polygons");
            }
        }
        if (point_count > parameters.max_nodes * 16)
            throw std::runtime_error("Bridge aware cubic exceeds the immutable path payload vertex work limit");
    }

    bool has_owner(const std::vector<CellId> &owners, size_t *work = nullptr) const
    {
        for (size_t i = 0; i < owners.size(); ++i) {
            if ((i & 255) == 0) cancel();
            if (work != nullptr && ++*work > parameters.max_nodes * 64)
                throw std::runtime_error("Bridge aware cubic exceeds the dependency ownership work limit");
            if (active_owners.find(owners[i]) != active_owners.end()) return true;
        }
        return false;
    }

    bool closure_active(const LayerClosure &closure, double z) const
    {
        return std::abs(closure.z - z) <= 1.e-9 && (closure.owners.empty() || has_owner(closure.owners));
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
        if (forced_leaves.find(cell.id()) != forced_leaves.end())
            return;
        std::vector<size_t> local;
        for (const size_t demand_id : candidates) {
            if (++work > parameters.max_nodes * 64)
                throw std::runtime_error("Bridge aware cubic exceeds the footprint-intersection work limit");
            if ((work & 255) == 0)
                cancel();
            const Demand &demand = demands[demand_id];
            const Polygon polygon = section(cell, demand.z);
            const BoundingBox bounds = get_extents<true>(polygon.points);
            if (polygon.empty() || !bounds_overlap(bounds, demand_bounds[demand_id]) ||
                    intersection_ex(ExPolygon(polygon), demand_masks[demand_id].candidates(bounds)).empty())
                continue;
            local.push_back(demand_id);
            const double diameter = section_diameter(polygon);
            if (diameter > demand.gap - geometry_tolerance) {
                cell.reason_demands.push_back(demand_id);
                if (!cell.split) {
                    cell.split = true;
                    cell.reason_demand = demand_id;
                    cell.reason_diameter = diameter;
                }
            }
        }
        if (!cell.split)
            return;
        // For this physical-cell convention density is sqrt(6)*spacing/edge.
        // A child doubles local density. Once that reaches solid density the
        // line-wall hierarchy needs a separately validated local solid fill;
        // silently adding overlapping walls is not a saturation strategy.
        const bool saturated = std::ldexp(parameters.density, int(cell.depth + 1)) >= 1.;
        if (saturated && parameters.allow_solid_fallback) {
            cell.split = false;
            cell.solid = true;
            cells[id] = std::move(cell);
            ++statistics.solid_cells;
            return;
        }
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
        cell.intersecting_demands = std::move(local);
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
            append_and_refine(child, cell.intersecting_demands, work);
        }
    }

    void append_cached(const std::vector<Cell> &source, const std::vector<size_t> &subtree_end,
                       size_t &index, const std::vector<size_t> &candidates, size_t &work)
    {
        const size_t source_id = index++;
        const Cell &previous = source[source_id];
        Cell cell { previous.address, previous.depth };
        if (cells.size() >= parameters.max_nodes)
            throw std::runtime_error("Bridge aware cubic exceeds the cell work limit during local support refinement");
        const size_t id = cells.size();
        cells.push_back(cell);
        statistics.max_depth = std::max(statistics.max_depth, cell.depth);
        if (forced_leaves.find(cell.id()) != forced_leaves.end()) {
            index = subtree_end[source_id];
            return;
        }
        // Preserve the cold constructor's work charges, including candidates
        // that do not intersect or whose gap is already satisfied. The cached
        // lists are sorted by original demand index. Looking up only currently
        // inherited candidates avoids repeatedly scanning disabled demands.
        for (size_t demand : candidates) {
            if (++work > parameters.max_nodes * 64)
                throw std::runtime_error("Bridge aware cubic exceeds the footprint-intersection work limit");
            if ((work & 255) == 0) cancel();
            if (std::binary_search(previous.reason_demands.begin(), previous.reason_demands.end(), demand))
                cell.reason_demands.push_back(demand);
            if (previous.split && std::binary_search(previous.intersecting_demands.begin(),
                                                     previous.intersecting_demands.end(), demand))
                cell.intersecting_demands.push_back(demand);
        }
        if (cell.reason_demands.empty()) {
            index = subtree_end[source_id];
            return;
        }
        cell.reason_demand = cell.reason_demands.front();
        cell.reason_diameter = cell.reason_demand == previous.reason_demand ? previous.reason_diameter :
            section_diameter(section(cell, demands[cell.reason_demand].z));
        cell.split = previous.split;
        cell.solid = previous.solid;
        cells[id] = cell;
        if (cell.solid) {
            ++statistics.solid_cells;
            index = subtree_end[source_id];
            return;
        }
        ++statistics.splits;
        for (unsigned octant = 0; octant < 8; ++octant)
            append_cached(source, subtree_end, index, cell.intersecting_demands, work);
    }

    bool has_elevation(double z) const
    {
        if (!std::isfinite(z))
            throw std::invalid_argument("Bridge aware cubic requires a finite output elevation");
        return z >= parameters.bounds.min.z() && z <= parameters.bounds.max.z();
    }

    // Complete midplane intersections, from their inherited-edge birth to
    // their inherited-edge death. The footprint never clips these intervals.
    PlaneIntervals refinement_intervals(double z, const CellId *owner = nullptr,
                                        const PlaneIntervals *selected_planes = nullptr,
                                        TaggingWork *tag_work = nullptr) const
    {
        PlaneIntervals result;
        size_t visited = 0;
        for (const Cell &cell : cells) {
            if (tag_work != nullptr) tag_work->charge();
            if ((++visited & 255) == 0)
                tag_work != nullptr ? tag_work->poll() : cancel();
            if (!cell.split || (owner != nullptr && !(cell.id() == *owner)))
                continue;
            const double size = edge(cell);
            const Vec3d lo = lower(cell);
            const double t = z * sqrt_three - lo.sum() - .5 * size;
            if (t <= 0. || t >= 2. * size)
                continue;
            for (unsigned axis = 0; axis < 3; ++axis) {
                const PlaneKey key{axis, cell.depth + 1, cell.address[axis] * 2 + 1};
                if (selected_planes != nullptr && selected_planes->find(key) == selected_planes->end())
                    continue;
                const unsigned a = (axis + 1) % 3;
                const double first = lo[a] + std::max(0., t - size);
                const double last = lo[a] + std::min(size, t);
                if (last > first) {
                    if (tag_work != nullptr && tag_work->owns_geometry()) {
                        tag_work->lookup(result.size());
                        auto found = result.find(key);
                        if (found == result.end()) {
                            tag_work->admit(TaggingWork::units_for<PlaneIntervals::value_type>() +
                                            4 * TaggingWork::units_for<void *>(), 1);
                            found = result.emplace(key, std::vector<Interval>{}).first;
                        }
                        tag_work->grow(found->second);
                        found->second.emplace_back(first, last);
                    } else result[key].emplace_back(first, last);
                }
            }
        }
        return result;
    }

    using Partitions = std::vector<std::pair<CellId, Polylines>>;

    Polylines refinement(double z, const ExPolygon &domain, const ExPolygons *solid = nullptr,
                         Partitions *partitions = nullptr, TaggedLineBatch *tagged = nullptr,
                         TaggingWork *tag_work = nullptr) const
    {
        struct RawProducer { CellId id; double first, last; };
        std::map<PlaneKey, std::vector<RawProducer>> producers;
        const bool retain_sources = partitions != nullptr || tagged != nullptr;
        const auto check_cancel = [&] { tag_work != nullptr ? tag_work->poll() : cancel(); };
        if (retain_sources) {
            for (const Cell &cell : cells) {
                if (tag_work != nullptr) tag_work->charge();
                if (!cell.split) continue;
                check_cancel();
                const double size = edge(cell);
                const Vec3d lo = lower(cell);
                const double t = z * sqrt_three - lo.sum() - .5 * size;
                if (t <= 0. || t >= 2. * size) continue;
                for (unsigned axis = 0; axis < 3; ++axis) {
                    const unsigned a = (axis + 1) % 3;
                    const double first = lo[a] + std::max(0., t - size);
                    const double last = lo[a] + std::min(size, t);
                    if (last > first) {
                        const PlaneKey key{axis, cell.depth + 1, cell.address[axis] * 2 + 1};
                        if (tag_work != nullptr) tag_work->lookup(producers.size());
                        auto found = producers.find(key);
                        if (found == producers.end()) {
                            if (tag_work != nullptr) {
                                // Include the map value and its tree links.
                                tag_work->admit(TaggingWork::units_for<decltype(producers)::value_type>() +
                                                4 * TaggingWork::units_for<void *>(), 1);
                                tag_work->lookup(producers.size());
                            }
                            found = producers.emplace(key, std::vector<RawProducer>{}).first;
                        }
                        if (tag_work != nullptr) tag_work->grow(found->second);
                        found->second.push_back({cell.id(), first, last});
                    }
                }
            }
        }
        Polylines raw;
        std::vector<LineSource> original;
        size_t work = 0;
        const auto visit = [&] {
            if (tag_work != nullptr) tag_work->charge();
            if (++work > parameters.max_nodes * size_t(64)) {
                if (tag_work != nullptr) throw TaggingUnavailable{};
                throw std::runtime_error("Bridge aware cubic exceeds the partition ownership work limit");
            }
            if ((work & 255) == 0) check_cancel();
        };
        PlaneIntervals planes = refinement_intervals(z, nullptr, nullptr, tag_work);
        for (auto &entry : planes) {
            check_cancel();
            const PlaneKey &plane = entry.first;
            auto &intervals = entry.second;
            std::sort(intervals.begin(), intervals.end(), [&](const Interval &a, const Interval &b) {
                if (tag_work != nullptr && tag_work->owns_geometry()) tag_work->charge();
                return a < b;
            });
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
                if (p0 == p1) return;
                if (tag_work != nullptr && tag_work->owns_geometry()) {
                    tag_work->grow(raw);
                    Points points;
                    tag_work->reserve(points, 2);
                    points.push_back(p0); points.push_back(p1);
                    raw.emplace_back(std::move(points));
                } else raw.emplace_back(p0, p1);
                if (!retain_sources) return;
                LineSource source{LineSource::Kind::Partition, p0, p1, {}};
                if (tag_work != nullptr) tag_work->lookup(producers.size());
                for (const RawProducer &owner : producers.at(plane)) {
                    visit();
                    const double begin = (std::max(first, owner.first) - first) / (last - first);
                    const double end = (std::min(last, owner.last) - first) / (last - first);
                    if (end > begin) {
                        if (tag_work != nullptr) tag_work->grow(source.producers);
                        source.producers.push_back({owner.id, begin, end});
                    }
                }
                if (tag_work != nullptr) tag_work->grow(original);
                original.push_back(std::move(source));
            };
            double first = intervals.front().first;
            double last = intervals.front().second;
            for (size_t i = 1; i < intervals.size(); ++i) {
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
        // Preserve one identical boolean batch for emission and ownership.
        // Clipper's handling of touching open subjects can depend on the other
        // subjects in its scan; separately clipping a face loses real output.
        if (!retain_sources) {
            if (solid != nullptr && !solid->empty()) raw = diff_pl(raw, *solid);
            return intersection_pl(raw, domain);
        }
        std::vector<size_t> source_ids;
        if (tag_work != nullptr) tag_work->reserve(source_ids, raw.size());
        source_ids.resize(raw.size());
        if (tag_work != nullptr) tag_work->charge(source_ids.size());
        std::iota(source_ids.begin(), source_ids.end(), size_t(0));
        const auto clip = [&](ClipperLib::ClipType operation, const auto &mask) {
            size_t admitted_output_points = 0;
            if (tag_work != nullptr && tag_work->owns_geometry()) {
                // reference_lines supplies only two-point open inputs and one
                // four-edge rectangle. Each input can add at most four clip
                // crossings plus its two endpoints. Open/open crossings add
                // no output points. Admit the transferred contour buffers
                // before Execute allocates them; Clipper's internal scan and
                // edge allocator remains under the existing geometry limits.
                for (const Polyline &line : raw) {
                    tag_work->charge();
                    if (line.size() != 2) throw TaggingUnavailable{};
                }
                if (operation != ClipperLib::ctIntersection) throw TaggingUnavailable{};
                admitted_output_points = raw.size() * 6;
                tag_work->admit(admitted_output_points, admitted_output_points);
                tag_work->charge(admitted_output_points + 4);
                tag_work->poll();
            }
            ClipperLib::Clipper clipper;
            clipper.AddPaths(ClipperUtils::PolylinesProvider(raw), ClipperLib::ptSubject, false);
            clipper.AddPaths(mask, ClipperLib::ptClip, true);
            ClipperLib::PolyTree result;
            clipper.Execute(operation, result, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
            Polylines clipped;
            std::vector<size_t> clipped_sources;
            if (tag_work != nullptr && tag_work->owns_geometry()) tag_work->reserve(clipped, result.Total());
            else clipped.reserve(result.Total());
            if (tag_work != nullptr) tag_work->reserve(clipped_sources, result.Total());
            else clipped_sources.reserve(result.Total());
            // Match PolyTreeToPolylines' traversal exactly. The metadata is
            // carried by open output records, not inferred from rounded axes.
            for (auto *node = result.GetFirst(); node != nullptr; node = node->GetNext()) {
                if (node->Contour.empty()) continue;
                visit();
                if (tag_work != nullptr && tag_work->owns_geometry()) {
                    if (node->Contour.capacity() > admitted_output_points) throw TaggingUnavailable{};
                    admitted_output_points -= node->Contour.capacity();
                }
                if (!node->IsOpen() || node->OpenPathIndex() >= source_ids.size()) {
                    if (tag_work != nullptr) throw TaggingUnavailable{};
                    throw std::runtime_error("Bridge aware cubic lost a clipped partition's source identity");
                }
                clipped_sources.push_back(source_ids[node->OpenPathIndex()]);
                clipped.emplace_back(std::move(node->Contour));
            }
            raw = std::move(clipped);
            source_ids = std::move(clipped_sources);
        };
        if (solid != nullptr && !solid->empty())
            clip(ClipperLib::ctDifference, ClipperUtils::ExPolygonsProvider(*solid));
        clip(ClipperLib::ctIntersection, ClipperUtils::ExPolygonProvider(domain));

        if (tagged != nullptr && partitions == nullptr) {
            // Retain every original source, not just output-referenced rows.
            // Near-coincident axes outside this domain can matter to the
            // consumer's separate legacy-classification ambiguity guard.
            tagged->sources = std::move(original);
            tagged->source_for_path = std::move(source_ids);
            return raw;
        }

        std::map<CellId, Polylines> owned;
        for (size_t line_index = 0; line_index < raw.size(); ++line_index) {
            const Polyline &line = raw[line_index];
            const LineSource &source = original.at(source_ids[line_index]);
            const Vec2d direction = (source.canonical_b - source.canonical_a).cast<double>();
            const double squared = direction.squaredNorm();
            for (size_t i = 1; i < line.size(); ++i) {
                check_cancel();
                const Point &start = line.points[i - 1], &finish = line.points[i];
                const double from = direction.dot((start - source.canonical_a).cast<double>()) / squared;
                const double to = direction.dot((finish - source.canonical_a).cast<double>()) / squared;
                if (from == to) continue;
                for (const ProducerInterval &owner : source.producers) {
                    visit();
                    const double low = std::max(owner.first, std::min(from, to));
                    const double high = std::min(owner.last, std::max(from, to));
                    if (high <= low) continue;
                    const auto point_at = [&](double position) {
                        if (position == from) return start;
                        if (position == to) return finish;
                        const Vec2d point = start.cast<double>() +
                            (finish - start).cast<double>() * ((position - from) / (to - from));
                        return Point(coord_t(std::llround(point.x())), coord_t(std::llround(point.y())));
                    };
                    const Point a = point_at(low), b = point_at(high);
                    if (a != b) {
                        if (tag_work != nullptr) {
                            tag_work->lookup(owned.size());
                            auto found = owned.find(owner.owner);
                            if (found == owned.end()) {
                                tag_work->admit(TaggingWork::units_for<decltype(owned)::value_type>() +
                                                4 * TaggingWork::units_for<void *>(), 1);
                                found = owned.emplace(owner.owner, Polylines{}).first;
                            }
                            tag_work->grow(found->second);
                            Points points;
                            tag_work->reserve(points, 2);
                            points.push_back(a); points.push_back(b);
                            found->second.emplace_back(std::move(points));
                        } else owned[owner.owner].emplace_back(a, b);
                    }
                }
            }
        }
        if (tag_work != nullptr) tag_work->reserve(*partitions, owned.size());
        else partitions->reserve(owned.size());
        for (auto &entry : owned) {
            if (tag_work != nullptr) tag_work->charge();
            partitions->emplace_back(entry.first, std::move(entry.second));
        }
        if (tagged != nullptr) {
            tagged->sources = std::move(original);
            tagged->source_for_path = std::move(source_ids);
        }
        return raw;
    }

    Polylines bulk(double z, const ExPolygon &domain, TaggingWork *tag_work = nullptr) const
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
                if (tag_work != nullptr) tag_work->charge();
                if (++count > parameters.max_nodes)
                    throw std::runtime_error("Bridge aware cubic exceeds the output line work limit");
                if ((count & 255) == 0)
                    tag_work != nullptr ? tag_work->poll() : cancel();
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

Plan::Plan(Parameters parameters, std::vector<Demand> demands,
           std::vector<LayerClosure> closures, std::vector<LayerAttachment> attachments)
    : m_impl(std::make_shared<Impl>(std::move(parameters), std::move(demands),
                                   std::move(closures), std::move(attachments))) {}
Plan::Plan(std::shared_ptr<const Impl> implementation) : m_impl(std::move(implementation)) {}
Plan::~Plan() = default;
Plan::Plan(Plan&&) noexcept = default;
Plan& Plan::operator=(Plan&&) noexcept = default;

Polylines Plan::lines(double z, const ExPolygon &domain) const
{
    m_impl->cancel();
    if (!m_impl->has_elevation(z))
        return {};
    const ExPolygons solid = solid_area(z);
    Polylines emitted = m_impl->bulk(z, domain);
    if (!solid.empty()) emitted = diff_pl(emitted, solid);
    Polylines added = m_impl->refinement(z, domain, &solid);
    emitted.insert(emitted.end(), std::make_move_iterator(added.begin()), std::make_move_iterator(added.end()));
    return emitted;
}

TaggedLineBatch Plan::tagged_lines(double z, const ExPolygon &domain, size_t capacity_units,
                                 BridgeAwareCubicValidation::WorkBudget *budget) const
{
    m_impl->cancel();
    if (!m_impl->has_elevation(z)) {
        TaggedLineBatch empty;
        empty.complete = true;
        return empty;
    }
    const auto ordinary = [&] {
        TaggedLineBatch unavailable;
        unavailable.paths = lines(z, domain);
        return unavailable;
    };
    // Initial provenance consumers cannot yet prove the solid/closure output
    // and attachment policy as one emitter. Keep their ordinary path intact.
    if (capacity_units == 0 || !solid_area(z).empty()) return ordinary();
    for (const LayerClosure &closure : m_impl->closures)
        if (m_impl->closure_active(closure, z)) return ordinary();
    try {
        TaggingWork work(capacity_units, budget, m_impl->parameters.throw_if_canceled);
        work.poll();
        work.admit(8, 1); // Batch containers and scalar state.
        TaggedLineBatch result;
        Polylines bulk = m_impl->bulk(z, domain, &work);
        Polylines added = m_impl->refinement(z, domain, nullptr, nullptr, &result, &work);

        // Refinement records remain in their original deterministic order,
        // including unreferenced ones. Bulk has no producer ranges: a record
        // per actual bulk output is enough to carry its origin through the
        // filler's later mixed clipping batch without changing bulk endpoints.
        std::vector<size_t> source_ids;
        work.reserve(source_ids, bulk.size() + result.source_for_path.size());
        for (const Polyline &path : bulk) {
            work.charge();
            if (path.size() < 2 || path.front() == path.back()) throw TaggingUnavailable{};
            work.grow(result.sources);
            source_ids.push_back(result.sources.size());
            result.sources.push_back({LineSource::Kind::Bulk, path.front(), path.back(), {}});
        }
        for (size_t source : result.source_for_path) {
            work.charge();
            source_ids.push_back(source);
        }
        result.source_for_path = std::move(source_ids);
        result.paths = std::move(bulk);
        result.paths.insert(result.paths.end(), std::make_move_iterator(added.begin()),
                                               std::make_move_iterator(added.end()));
        work.poll();
        result.complete = true;
        return result;
    } catch (const TaggingCanceled &canceled) {
        std::rethrow_exception(canceled.exception);
    } catch (const TaggingUnavailable &) {
    } catch (const std::bad_alloc &) {
    }
    // All partial metadata dies before the fallback; spent shared work stays
    // charged. Re-run the unchanged ordinary path, never return partial output.
    return ordinary();
}

ReferenceLineBatch Plan::reference_lines(double z, const BoundingBox &bounds, size_t capacity_units,
                                       BridgeAwareCubicValidation::WorkBudget *budget) const
{
    m_impl->cancel();
    const bool in_range = m_impl->has_elevation(z);
    if (!bounds.defined || bounds.min.x() >= bounds.max.x() || bounds.min.y() >= bounds.max.y()) return {};
    // Match the consumer's safe integer domain before any native clipping;
    // optional malformed bounds must decline, not enter Clipper range errors.
    constexpr int64_t coordinate_limit = std::min<int64_t>(std::numeric_limits<coord_t>::max() / 4,
                                                           std::numeric_limits<int32_t>::max() / 4);
    for (const Point &point : {bounds.min, bounds.max})
        if (point.x() < -coordinate_limit || point.x() > coordinate_limit ||
            point.y() < -coordinate_limit || point.y() > coordinate_limit) return {};
    try {
        TaggingWork work(capacity_units, budget, m_impl->parameters.throw_if_canceled, true);
        work.poll();
        work.admit(TaggingWork::units_for<ReferenceLineBatch>() + TaggingWork::units_for<TaggedLineBatch>() +
                   TaggingWork::units_for<ExPolygon>() + 16, 3);
        ReferenceLineBatch result;
        if (in_range) {
            // A solid section's zero-area birth/death may also decline. This
            // optional scope never substitutes sparse provenance for solid.
            for (const Refinement &refinement : m_impl->refinements) {
                work.charge();
                if (refinement.solid && z >= refinement.bottom_z && z <= refinement.top_z) return {};
            }
            for (const LayerClosure &closure : m_impl->closures) {
                work.charge();
                if (std::abs(closure.z - z) > 1.e-9) continue;
                if (closure.owners.empty()) return {};
                for (const CellId &owner : closure.owners) {
                    work.lookup(m_impl->active_owners.size());
                    if (m_impl->active_owners.find(owner) != m_impl->active_owners.end()) return {};
                }
            }
            Points corners;
            work.reserve(corners, 4);
            corners.push_back(bounds.min);
            corners.emplace_back(bounds.max.x(), bounds.min.y());
            corners.push_back(bounds.max);
            corners.emplace_back(bounds.min.x(), bounds.max.y());
            const ExPolygon domain(Polygon(std::move(corners)));
            TaggedLineBatch tagged;
            result.paths = m_impl->refinement(z, domain, nullptr, &result.partitions, &tagged, &work);
            result.sources = std::move(tagged.sources);
            result.source_for_path = std::move(tagged.source_for_path);
        }
        result.plan_identity = immutable_identity();
        result.z = z;
        result.bounds = bounds;
        work.poll();
        result.complete = true;
        return result;
    } catch (const TaggingCanceled &canceled) {
        std::rethrow_exception(canceled.exception);
    } catch (const TaggingUnavailable &) {
    } catch (const std::bad_alloc &) {
    }
    // Reference-only failure never needs to allocate replacement geometry.
    return {};
}

Polylines Plan::refinement_lines(double z, const ExPolygon &domain) const
{
    m_impl->cancel();
    if (!m_impl->has_elevation(z)) return {};
    const ExPolygons solid = solid_area(z);
    Polylines result = m_impl->refinement(z, domain, &solid);
    Polylines closures = intersection_pl(closure_lines(z), domain);
    result.insert(result.end(), std::make_move_iterator(closures.begin()), std::make_move_iterator(closures.end()));
    return result;
}

Polylines Plan::partition_lines(const CellId &owner, double z, const ExPolygon &domain) const
{
    auto partitions = partition_lines(z, domain);
    const auto found = std::lower_bound(partitions.begin(), partitions.end(), owner,
        [](const auto &entry, const CellId &id) { return entry.first < id; });
    return found != partitions.end() && found->first == owner ? std::move(found->second) : Polylines{};
}

std::vector<std::pair<CellId, Polylines>> Plan::partition_lines(double z, const ExPolygon &domain) const
{
    m_impl->cancel();
    if (!m_impl->has_elevation(z)) return {};
    const ExPolygons solid = solid_area(z);
    Impl::Partitions partitions;
    m_impl->refinement(z, domain, &solid, &partitions);
    return partitions;
}

Polylines Plan::closure_lines(double z) const
{
    m_impl->cancel();
    if (!m_impl->has_elevation(z)) return {};
    Polylines paths;
    for (const LayerClosure &closure : m_impl->closures)
        if (m_impl->closure_active(closure, z))
            paths.insert(paths.end(), closure.paths.begin(), closure.paths.end());
    return paths;
}

std::vector<LayerClosure> Plan::closures(double z) const
{
    m_impl->cancel();
    if (!m_impl->has_elevation(z)) return {};
    std::vector<LayerClosure> result;
    for (const LayerClosure &closure : m_impl->closures)
        if (m_impl->closure_active(closure, z)) result.push_back(closure);
    return result;
}

bool Plan::has_closures(double z) const
{
    m_impl->cancel();
    if (!m_impl->has_elevation(z)) return false;
    size_t visited = 0;
    for (const LayerClosure &closure : m_impl->closures) {
        if ((++visited & 255) == 0) m_impl->cancel();
        if (m_impl->closure_active(closure, z)) return true;
    }
    return false;
}

std::vector<LayerAttachment> Plan::attachments(double z) const
{
    std::vector<LayerAttachment> result;
    visit_attachments(z, [&](const LayerAttachment &attachment) { result.push_back(attachment); return true; });
    return result;
}

bool Plan::visit_attachments(double z, const std::function<bool(const LayerAttachment &)> &visitor) const
{
    m_impl->cancel();
    if (!std::isfinite(z)) throw std::invalid_argument("Bridge aware cubic requires a finite attachment elevation");
    if (!visitor) throw std::invalid_argument("Bridge aware cubic requires an attachment visitor");
    size_t visited = 0;
    for (const LayerAttachment &attachment : m_impl->layer_attachments)
    {
        if ((++visited & 255) == 0) m_impl->cancel();
        if (std::abs(attachment.z - z) <= 1.e-9 &&
                (attachment.owners.empty() || m_impl->has_owner(attachment.owners)) && !visitor(attachment)) return false;
    }
    return true;
}

ExPolygons Plan::refinement_area(double z) const
{
    m_impl->cancel();
    if (!m_impl->has_elevation(z))
        return {};
    ExPolygons selected;
    size_t visited = 0;
    for (const Cell &cell : m_impl->cells) {
        if ((++visited & 255) == 0)
            m_impl->cancel();
        if (!cell.split && !cell.solid)
            continue;
        const double size = m_impl->edge(cell);
        const double t = z * sqrt_three - m_impl->lower(cell).sum() - .5 * size;
        if (cell.solid || (t > 0. && t < 2. * size)) {
            Polygon polygon = m_impl->section(cell, z);
            if (!polygon.empty())
                selected.emplace_back(std::move(polygon));
        }
    }
    for (const LayerClosure &closure : m_impl->closures)
        if (m_impl->closure_active(closure, z))
            selected.insert(selected.end(), closure.reservation.begin(), closure.reservation.end());
    return union_ex(selected);
}

Polygon Plan::shadow_plain_cell_section(const CellId &id,double z,BridgeAwareCubicValidation::WorkBudget &work) const
{
    m_impl->cancel();
    if (!std::isfinite(z) || id.depth>m_impl->parameters.max_depth || m_impl->statistics.solid_cells ||
        m_impl->closures.size()>2048 || m_impl->layer_attachments.size()>2048) return {};
    const size_t count=32+m_impl->closures.size()+m_impl->layer_attachments.size();
    if(count>work.remaining_work) return {};
    work.remaining_work-=count;
    work.charged_work[size_t(BridgeAwareCubicValidation::WorkCategory::Validation)]+=count;
    // Raw matching-elevation payloads are conservatively ineligible even if their owners are inactive.
    size_t i=0;
    for (const auto &v:m_impl->closures) {
        if ((++i & 255)==0) m_impl->cancel();
        if (std::abs(v.z-z)<=EPSILON) return {};
    }
    for (const auto &v:m_impl->layer_attachments) {
        if ((++i & 255)==0) m_impl->cancel();
        if (std::abs(v.z-z)<=EPSILON) return {};
    }
    return m_impl->section(Cell{id.address,id.depth},z);
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
                sections.push_back({ id, cell.depth, m_impl->edge(cell), std::move(polygon), cell.id(), cell.solid });
        }
    }
    return sections;
}

std::vector<LeafSection> Plan::solid_sections(double z) const
{
    m_impl->cancel();
    if (!m_impl->has_elevation(z)) return {};
    if (m_impl->statistics.solid_cells == 0) return {};
    std::vector<LeafSection> sections;
    for (size_t id = 0; id < m_impl->cells.size(); ++id) {
        if ((id & 255) == 0) m_impl->cancel();
        const Cell &cell = m_impl->cells[id];
        if (cell.solid) {
            Polygon polygon = m_impl->section(cell, z);
            if (!polygon.empty())
                sections.push_back({ id, cell.depth, m_impl->edge(cell), std::move(polygon), cell.id(), true });
        }
    }
    return sections;
}

ExPolygons Plan::solid_area(double z) const
{
    ExPolygons polygons;
    for (LeafSection &section : solid_sections(z)) polygons.emplace_back(std::move(section.polygon));
    return union_ex(polygons);
}

const std::vector<Refinement>& Plan::refinements() const { return m_impl->refinements; }
const std::vector<size_t>& Plan::active_demands() const { return m_impl->active_demands; }

Plan Plan::clone() const
{
    m_impl->cancel();
    return Plan(m_impl);
}

std::shared_ptr<const void> Plan::immutable_identity() const noexcept
{
    return m_impl ? std::shared_ptr<const void>(m_impl->identity) : std::shared_ptr<const void>{};
}

Plan Plan::with_demands(std::vector<Demand> demands) const
{
    m_impl->cancel();
    return Plan(std::make_shared<Impl>(m_impl->parameters, std::move(demands),
                                       m_impl->closures, m_impl->layer_attachments, m_impl->forced_leaves));
}

Plan Plan::with_payloads(std::vector<LayerClosure> closures, std::vector<LayerAttachment> attachments) const
{
    m_impl->cancel();
    return Plan(std::make_shared<Impl>(m_impl->parameters, m_impl->demands,
                                       std::move(closures), std::move(attachments), m_impl->forced_leaves));
}

Plan Plan::collapsed(const CellId &id) const
{
    m_impl->cancel();
    if (m_impl->active_owners.find(id) == m_impl->active_owners.end())
        throw std::invalid_argument("Bridge aware cubic can only collapse an active split or solid cell");
    return Plan(std::make_shared<Impl>(*m_impl, id));
}

Plan Plan::coarsened(const std::function<bool(const Plan&)> &validate, size_t max_trials) const
{
    m_impl->cancel();
    if (!validate) throw std::invalid_argument("Bridge aware cubic coarsening requires a final-deposition validator");
    Plan current(m_impl);
    if (!validate(current))
        throw std::runtime_error("Bridge aware cubic cannot coarsen an uncertified starting plan");
    size_t attempts = 0;
    for (;;) {
        std::vector<Refinement> candidates = current.refinements();
        std::sort(candidates.begin(), candidates.end(), [](const Refinement &a, const Refinement &b) {
            return a.id.depth == b.id.depth ? a.id.address < b.id.address : a.id.depth > b.id.depth;
        });
        bool accepted = false;
        for (const Refinement &candidate : candidates) {
            current.m_impl->cancel();
            if (attempts++ >= max_trials)
                throw std::runtime_error("Bridge aware cubic exceeds the coarsening work limit before reaching an irreducible plan");
            Plan trial = current.collapsed(candidate.id);
            if (validate(trial)) {
                current = std::move(trial);
                accepted = true;
                break;
            }
        }
        if (!accepted) return current;
    }
}

const Parameters& Plan::parameters() const { return m_impl->parameters; }
const std::vector<Demand>& Plan::demands() const { return m_impl->demands; }
const Statistics& Plan::statistics() const { return m_impl->statistics; }
double Plan::bulk_cell_edge() const { return m_impl->bulk_edge; }

}} // namespace Slic3r::BridgeAwareCubic
