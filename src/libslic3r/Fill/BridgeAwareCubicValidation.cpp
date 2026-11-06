#include "BridgeAwareCubicValidation.hpp"
#include "BridgeAwareCubicContact.hpp"

#include "../ClipperUtils.hpp"
#include "../Geometry/ConvexHull.hpp"

#include <boost/multiprecision/cpp_int.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <numeric>
#include <set>

namespace Slic3r { namespace BridgeAwareCubicValidation {

namespace {
// Limit both allocations and the potentially quadratic intersection/diameter
// work. Reaching either cap is an incomplete check, never a certificate.
constexpr size_t max_vertices = 200000;
constexpr size_t max_work = 50000000;
constexpr coord_t coordinate_limit = std::numeric_limits<coord_t>::max() / 4;
using Scalar = long double;

struct WorkLimit { bool aggregate = false; const WorkBudget *shared = nullptr; };
struct Budget {
    size_t remaining = max_work;
    WorkBudget *aggregate = nullptr;
    explicit Budget(WorkBudget *shared = nullptr) : aggregate(shared) {}
    void consume(size_t count = 1)
    {
        if (aggregate != nullptr && count > aggregate->remaining_work) {
            aggregate->unspent_at_limit = aggregate->remaining_work;
            aggregate->rejected_request = count;
            aggregate->remaining_work = 0;
            throw WorkLimit{true, aggregate};
        }
        if (count > remaining)
            throw WorkLimit{};
        remaining -= count;
        if (aggregate != nullptr) {
            aggregate->remaining_work -= count;
            aggregate->charged_work[size_t(WorkCategory::Validation)] += count;
        }
    }
};

struct Position {
    Scalar x, y;
    explicit Position(const Point &p) : x(p.x()), y(p.y()) {}
    Position(Scalar x_, Scalar y_) : x(x_), y(y_) {}
};

Position operator-(const Position &a, const Position &b) { return {a.x - b.x, a.y - b.y}; }
Scalar cross(const Position &a, const Position &b) { return a.x * b.y - a.y * b.x; }
Scalar dot(const Position &a, const Position &b) { return a.x * b.x + a.y * b.y; }
// Segment endpoints are integer points within coordinate_limit. Differences
// and both products therefore fit int64_t, including their sum/difference.
int64_t integer_cross(const Position &a, const Position &b)
{
    return int64_t(a.x) * int64_t(b.y) - int64_t(a.y) * int64_t(b.x);
}
int64_t integer_dot(const Position &a, const Position &b)
{
    return int64_t(a.x) * int64_t(b.x) + int64_t(a.y) * int64_t(b.y);
}
Scalar cross_error(const Position &a, const Position &b)
{
    return 64 * std::numeric_limits<Scalar>::epsilon() *
           (std::abs(a.x * b.y) + std::abs(a.y * b.x) + 1);
}
Position at(const Position &a, const Position &delta, Scalar t)
{
    return {a.x + delta.x * t, a.y + delta.y * t};
}

struct Bounds {
    Scalar x0, y0, x1, y1;
    explicit Bounds(const Points &points) : x0(points.front().x()), y0(points.front().y()), x1(x0), y1(y0)
    {
        for (const Point &p : points) {
            x0 = std::min(x0, Scalar(p.x())); y0 = std::min(y0, Scalar(p.y()));
            x1 = std::max(x1, Scalar(p.x())); y1 = std::max(y1, Scalar(p.y()));
        }
    }
    Bounds(const Position &a, const Position &b)
        : x0(std::min(a.x, b.x)), y0(std::min(a.y, b.y)), x1(std::max(a.x, b.x)), y1(std::max(a.y, b.y)) {}
    bool contains(const Position &p) const { return p.x >= x0 && p.x <= x1 && p.y >= y0 && p.y <= y1; }
    bool overlaps(const Bounds &other) const
    {
        return x0 <= other.x1 && other.x0 <= x1 && y0 <= other.y1 && other.y0 <= y1;
    }
};

enum class Location { Outside, Boundary, Inside };

class RingLocator {
public:
    RingLocator(const Polygon &ring, Budget &budget) : m_ring(ring)
    {
        // Small contact masks are cheaper to scan directly. Large demand rings
        // are queried repeatedly along every original upper-path segment.
        if (ring.points.size() <= 64)
            return;
        budget.consume(ring.points.size());
        m_order.resize(ring.points.size());
        std::iota(m_order.begin(), m_order.end(), size_t(0));
        m_nodes.reserve(ring.points.size() / 2);
        build(0, m_order.size(), budget);
    }

    Location locate(const Position &p, Budget &budget) const
    {
        bool inside = false;
        if (!m_nodes.empty()) {
            if (query(0, p, inside, budget))
                return Location::Boundary;
        } else {
            for (size_t i = 0; i < m_ring.points.size(); ++i) {
                budget.consume();
                if (visit_edge(i, p, inside))
                    return Location::Boundary;
            }
        }
        return inside ? Location::Inside : Location::Outside;
    }

private:
    struct Node {
        coord_t min_y, max_y;
        size_t begin, end, left = 0, right = 0;
    };

    size_t previous(size_t i) const { return i == 0 ? m_ring.points.size() - 1 : i - 1; }

    bool visit_edge(size_t i, const Position &p, bool &inside) const
    {
        const Position a(m_ring.points[previous(i)]), b(m_ring.points[i]);
        const Position delta = b - a, relative = p - a;
        // These are the original boundary and ray-crossing predicates. Only
        // edges whose closed Y interval excludes p are omitted by the index.
        if (std::abs(cross(delta, relative)) <= cross_error(delta, relative) && Bounds(a, b).contains(p))
            return true;
        if ((a.y > p.y) != (b.y > p.y) && p.x < a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y))
            inside = !inside;
        return false;
    }

    size_t build(size_t begin, size_t end, Budget &budget)
    {
        budget.consume(end - begin);
        coord_t min_y = std::numeric_limits<coord_t>::max();
        coord_t max_y = std::numeric_limits<coord_t>::lowest();
        for (size_t k = begin; k < end; ++k) {
            const size_t i = m_order[k];
            const coord_t a = m_ring.points[previous(i)].y(), b = m_ring.points[i].y();
            min_y = std::min(min_y, std::min(a, b));
            max_y = std::max(max_y, std::max(a, b));
        }
        const size_t node = m_nodes.size();
        m_nodes.push_back({min_y, max_y, begin, end});
        if (end - begin > 8) {
            const size_t middle = begin + (end - begin) / 2;
            std::nth_element(m_order.begin() + begin, m_order.begin() + middle, m_order.begin() + end,
                [&](size_t a, size_t b) {
                    budget.consume();
                    return int64_t(m_ring.points[previous(a)].y()) + m_ring.points[a].y() <
                           int64_t(m_ring.points[previous(b)].y()) + m_ring.points[b].y();
                });
            const size_t left = build(begin, middle, budget);
            const size_t right = build(middle, end, budget);
            m_nodes[node].left = left;
            m_nodes[node].right = right;
        }
        return node;
    }

    bool query(size_t index, const Position &p, bool &inside, Budget &budget) const
    {
        budget.consume();
        const Node &node = m_nodes[index];
        // Inclusive bounds retain horizontal edges and shared vertices.
        if (p.y < node.min_y || p.y > node.max_y)
            return false;
        if (node.end - node.begin > 8)
            return query(node.left, p, inside, budget) || query(node.right, p, inside, budget);
        for (size_t k = node.begin; k < node.end; ++k) {
            budget.consume();
            if (visit_edge(m_order[k], p, inside))
                return true;
        }
        return false;
    }

    const Polygon &m_ring;
    std::vector<size_t> m_order;
    std::vector<Node> m_nodes;
};

struct Edge {
    Position a, b;
    Bounds bounds;
    Edge(const Point &a_, const Point &b_) : a(a_), b(b_), bounds(a, b) {}
};

struct Parameter {
    int64_t numerator, denominator;
    Scalar value;
    bool boundary_contact;
    Parameter(int64_t n, int64_t d, bool contact = false) : boundary_contact(contact)
    {
        const int64_t divisor = std::gcd(n, d);
        numerator = n / divisor;
        denominator = d / divisor;
        value = Scalar(numerator) / Scalar(denominator);
    }
};

using WideInteger = boost::multiprecision::int128_t;

bool operator<(const Parameter &a, const Parameter &b)
{
    return WideInteger(a.numerator) * b.denominator < WideInteger(b.numerator) * a.denominator;
}

bool operator==(const Parameter &a, const Parameter &b)
{
    return a.numerator == b.numerator && a.denominator == b.denominator;
}

Scalar parameter_distance(const Parameter &begin, const Parameter &end)
{
    // Each determinant is bounded by 2^61, so these products fit int128_t.
    // Subtract before converting: distinct intersections may have equal Scalar
    // values, but their positive local distance must never disappear.
    const WideInteger numerator = WideInteger(end.numerator) * begin.denominator -
                                  WideInteger(begin.numerator) * end.denominator;
    const WideInteger denominator = WideInteger(begin.denominator) * end.denominator;
    return numerator.convert_to<Scalar>() / denominator.convert_to<Scalar>();
}

class PolygonSet {
public:
    explicit PolygonSet(const ExPolygons &polygons, Budget &budget) : m_polygons(polygons)
    {
        for (const ExPolygon &polygon : polygons) {
            m_bounds.emplace_back(polygon.contour.points);
            m_outer_bounds.push_back(m_bounds.back());
            m_outer_locators.emplace_back(polygon.contour, budget);
            m_hole_bounds.emplace_back();
            m_hole_bounds.back().reserve(polygon.holes.size());
            m_hole_locators.emplace_back();
            m_hole_locators.back().reserve(polygon.holes.size());
            const size_t first_edge = m_edges.size();
            append_edges(polygon.contour);
            for (const Polygon &hole : polygon.holes) {
                m_hole_bounds.back().emplace_back(hole.points);
                // Input validity does not prove that every hole lies within
                // its contour. Boundary queries must retain all ring edges.
                const Bounds &hole_bounds = m_hole_bounds.back().back();
                Bounds &bounds = m_bounds.back();
                bounds.x0 = std::min(bounds.x0, hole_bounds.x0);
                bounds.y0 = std::min(bounds.y0, hole_bounds.y0);
                bounds.x1 = std::max(bounds.x1, hole_bounds.x1);
                bounds.y1 = std::max(bounds.y1, hole_bounds.y1);
                m_hole_locators.back().emplace_back(hole, budget);
                append_edges(hole);
            }
            m_edge_ranges.emplace_back(first_edge, m_edges.size());
        }
        // Demand from actual unsupported strands can contain thousands of
        // disconnected polygons. Index their closed boxes once, retaining
        // original polygon/edge order for the unchanged exact predicates.
        if (m_bounds.size() > 64) {
            budget.consume(m_bounds.size());
            m_order.resize(m_bounds.size());
            std::iota(m_order.begin(), m_order.end(), size_t(0));
            m_nodes.reserve(m_bounds.size() / 2);
            build(0, m_order.size(), budget);
        }
    }

    bool contains(const Position &p, bool boundary, Budget &budget) const
    {
        bool inside = false;
        visit_candidates(Bounds(p, p), budget, [&](size_t i) {
            if (!m_outer_bounds[i].contains(p))
                return false;
            const ExPolygon &polygon = m_polygons[i];
            const Location outer = m_outer_locators[i].locate(p, budget);
            if (outer == Location::Outside)
                return false;
            if (outer == Location::Boundary) {
                inside = boundary;
                return true;
            }
            bool in_hole = false;
            for (size_t j = 0; j < polygon.holes.size(); ++j) {
                budget.consume();
                if (!m_hole_bounds[i][j].contains(p))
                    continue;
                const Location location = m_hole_locators[i][j].locate(p, budget);
                if (location == Location::Boundary) {
                    inside = boundary;
                    return true;
                }
                if (location == Location::Inside) {
                    in_hole = true;
                    break;
                }
            }
            if (!in_hole) {
                inside = true;
                return true;
            }
            return false;
        });
        return inside;
    }

    bool contour_in(const PolygonSet &other, const Bounds &bounds, Budget &budget) const
    {
        return visit_candidates(bounds, budget, [&](size_t i) {
            return other.contains(Position(m_polygons[i].contour.points.front()), true, budget);
        });
    }

    // The event positions belong to the original segment, including tangent
    // and collinear boundary contacts. They are never extrusion endpoints.
    bool boundary_events(const Position &a, const Position &b, std::vector<Parameter> &events, Budget &budget) const
    {
        const Position delta = b - a;
        const int64_t squared_length = integer_dot(delta, delta);
        const Bounds bounds(a, b);
        bool touches = false;
        visit_candidates(bounds, budget, [&](size_t polygon) {
            const auto range = m_edge_ranges[polygon];
            for (size_t index = range.first; index < range.second; ++index) {
                const Edge &edge = m_edges[index];
                budget.consume();
                if (!bounds.overlaps(edge.bounds))
                    continue;
                const Position direction = edge.b - edge.a, relative = edge.a - a;
                int64_t denominator = integer_cross(delta, direction);
                if (denominator == 0) {
                    if (integer_cross(relative, delta) != 0)
                        continue;
                    int64_t t0 = integer_dot(relative, delta);
                    int64_t t1 = integer_dot(edge.b - a, delta);
                    if (t0 > t1)
                        std::swap(t0, t1);
                    if (t1 >= 0 && t0 <= squared_length) {
                        events.emplace_back(std::max(int64_t(0), t0), squared_length, true);
                        events.emplace_back(std::min(squared_length, t1), squared_length, true);
                        touches = true;
                    }
                } else {
                    int64_t t = integer_cross(relative, direction);
                    int64_t u = integer_cross(relative, delta);
                    if (denominator < 0) {
                        denominator = -denominator;
                        t = -t;
                        u = -u;
                    }
                    if (t >= 0 && t <= denominator && u >= 0 && u <= denominator) {
                        events.emplace_back(t, denominator, true);
                        touches = true;
                    }
                }
            }
            return false;
        });
        return touches;
    }

private:
    struct Node {
        Bounds bounds;
        size_t begin, end, left = 0, right = 0;
    };

    size_t build(size_t begin, size_t end, Budget &budget)
    {
        budget.consume(end - begin);
        Bounds bounds = m_bounds[m_order[begin]];
        for (size_t k = begin + 1; k < end; ++k) {
            const Bounds &box = m_bounds[m_order[k]];
            bounds.x0 = std::min(bounds.x0, box.x0);
            bounds.y0 = std::min(bounds.y0, box.y0);
            bounds.x1 = std::max(bounds.x1, box.x1);
            bounds.y1 = std::max(bounds.y1, box.y1);
        }
        const size_t node = m_nodes.size();
        m_nodes.push_back({bounds, begin, end});
        if (end - begin > 8) {
            const bool split_x = bounds.x1 - bounds.x0 >= bounds.y1 - bounds.y0;
            const size_t middle = begin + (end - begin) / 2;
            std::nth_element(m_order.begin() + begin, m_order.begin() + middle, m_order.begin() + end,
                [&](size_t a, size_t b) {
                    budget.consume();
                    return split_x ? m_bounds[a].x0 + m_bounds[a].x1 < m_bounds[b].x0 + m_bounds[b].x1 :
                                     m_bounds[a].y0 + m_bounds[a].y1 < m_bounds[b].y0 + m_bounds[b].y1;
                });
            const size_t left = build(begin, middle, budget);
            const size_t right = build(middle, end, budget);
            m_nodes[node].left = left;
            m_nodes[node].right = right;
        }
        return node;
    }

    void query(size_t index, const Bounds &bounds, std::vector<size_t> &candidates, Budget &budget) const
    {
        budget.consume();
        const Node &node = m_nodes[index];
        if (!node.bounds.overlaps(bounds))
            return;
        if (node.end - node.begin > 8) {
            query(node.left, bounds, candidates, budget);
            query(node.right, bounds, candidates, budget);
        } else {
            for (size_t k = node.begin; k < node.end; ++k) {
                budget.consume();
                if (m_bounds[m_order[k]].overlaps(bounds))
                    candidates.push_back(m_order[k]);
            }
        }
    }

    template<class Visitor> bool visit_candidates(const Bounds &bounds, Budget &budget, Visitor visitor) const
    {
        if (m_nodes.empty()) {
            for (size_t i = 0; i < m_bounds.size(); ++i) {
                budget.consume();
                if (m_bounds[i].overlaps(bounds) && visitor(i))
                    return true;
            }
        } else {
            std::vector<size_t> candidates;
            query(0, bounds, candidates, budget);
            // Containment historically stops at the first containing or
            // boundary polygon, including overlapping per-segment masks.
            // Preserve that order rather than let tree shape alter semantics.
            std::sort(candidates.begin(), candidates.end(), [&](size_t a, size_t b) {
                budget.consume();
                return a < b;
            });
            for (size_t i : candidates) {
                budget.consume();
                if (visitor(i))
                    return true;
            }
        }
        return false;
    }

    void append_edges(const Polygon &ring)
    {
        for (size_t i = 0, previous = ring.points.size() - 1; i < ring.points.size(); previous = i++)
            if (ring.points[previous] != ring.points[i])
                m_edges.emplace_back(ring.points[previous], ring.points[i]);
    }
    const ExPolygons &m_polygons;
    std::vector<Bounds> m_bounds, m_outer_bounds;
    std::vector<std::vector<Bounds>> m_hole_bounds;
    std::vector<RingLocator> m_outer_locators;
    std::vector<std::vector<RingLocator>> m_hole_locators;
    std::vector<Edge> m_edges;
    std::vector<std::pair<size_t, size_t>> m_edge_ranges;
    std::vector<size_t> m_order;
    std::vector<Node> m_nodes;
};

struct PathPosition {
    size_t segment;
    Parameter parameter;
    PathPosition(size_t index, const Parameter &fraction) : segment(index), parameter(fraction)
    {
        if (parameter.numerator == parameter.denominator) {
            ++segment;
            parameter = Parameter(0, 1);
        }
    }
};

bool operator<(const PathPosition &a, const PathPosition &b)
{
    return a.segment < b.segment || (a.segment == b.segment && a.parameter < b.parameter);
}

struct Interval {
    PathPosition begin, end;
    double begin_uncertainty = 0., end_uncertainty = 0.;
};

struct PathSegment {
    Position a, b;
    Scalar length_mm;
    Scalar prefix_mm;
    size_t original_index;
};

void append_interval(std::vector<Interval> &intervals, const PathPosition &begin, const PathPosition &end,
                     double begin_uncertainty = 0., double end_uncertainty = 0.)
{
    if (!intervals.empty() && !(intervals.back().end < begin)) {
        if (!(intervals.back().begin < begin))
            intervals.back().begin_uncertainty = std::min(intervals.back().begin_uncertainty, begin_uncertainty);
        if (intervals.back().end < end) {
            intervals.back().end = end;
            intervals.back().end_uncertainty = end_uncertainty;
        } else if (!(end < intervals.back().end))
            intervals.back().end_uncertainty = std::min(intervals.back().end_uncertainty, end_uncertainty);
    }
    else
        intervals.push_back({begin, end, begin_uncertainty, end_uncertainty});
}

void segment_intervals(const Position &a, const Position &b, size_t segment_index,
                       const PolygonSet &polygons, bool boundary,
                       std::vector<Interval> &intervals, Budget &budget)
{
    std::vector<Parameter> events{{0, 1}, {1, 1}};
    polygons.boundary_events(a, b, events, budget);
    std::sort(events.begin(), events.end());
    size_t distinct = 0;
    for (const Parameter &event : events) {
        if (distinct > 0 && events[distinct - 1] == event)
            events[distinct - 1].boundary_contact |= event.boundary_contact;
        else
            events[distinct++] = event;
    }
    events.erase(events.begin() + distinct, events.end());
    const Position delta = b - a;
    for (size_t i = 0; i < events.size(); ++i) {
        const Scalar t = events[i].value;
        const PathPosition begin(segment_index, events[i]);
        // Boundary events are exact proven contacts, including tangent points;
        // retesting their rounded XY position could incorrectly lose demand.
        if (boundary && (events[i].boundary_contact || polygons.contains(at(a, delta, t), true, budget)))
            append_interval(intervals, begin, begin);
        if (i + 1 < events.size()) {
            const Scalar next = events[i + 1].value;
            const Scalar midpoint = (t + next) / 2;
            // If the midpoint cannot be represented strictly between the exact
            // events, conservatively include demand and exclude support. This
            // retains microscopic air spans without inventing an anchor.
            const bool uncertain = midpoint <= t || midpoint >= next;
            if (uncertain ? boundary : polygons.contains(at(a, delta, midpoint), boundary, budget))
                append_interval(intervals, begin, PathPosition(segment_index, events[i + 1]));
        }
    }
}

bool selected(const PathPosition &begin, const PathPosition &end, const std::vector<Interval> &demand)
{
    for (const Interval &interval : demand) {
        if (end < interval.begin)
            return false;
        if (!(interval.end < begin))
            return true;
    }
    return false;
}

Scalar path_position_mm(const PathPosition &position, const std::vector<PathSegment> &segments)
{
    if (position.segment == segments.size())
        return segments.back().prefix_mm + segments.back().length_mm;
    return segments[position.segment].prefix_mm + position.parameter.value * segments[position.segment].length_mm;
}

Scalar local_arclength(const PathPosition &begin, const PathPosition &end,
                      const std::vector<PathSegment> &segments, Budget &budget)
{
    Scalar length = 0, correction = 0;
    const Parameter zero(0, 1), one(1, 1);
    for (size_t i = begin.segment; i <= end.segment && i < segments.size(); ++i) {
        budget.consume();
        const Parameter &from = i == begin.segment ? begin.parameter : zero;
        const Parameter &to = i == end.segment ? end.parameter : one;
        const Scalar piece = parameter_distance(from, to) * segments[i].length_mm;
        const Scalar adjusted = piece - correction;
        const Scalar next = length + adjusted;
        correction = (next - length) - adjusted;
        length = next;
    }
    return length;
}

template<class Visitor> bool visit_rings(const ExPolygon &polygon, Visitor visitor)
{
    if (visitor(polygon.contour))
        return true;
    for (const Polygon &hole : polygon.holes)
        if (visitor(hole))
            return true;
    return false;
}

bool boundary_touches(const ExPolygon &cell, const PolygonSet &other, Budget &budget)
{
    return visit_rings(cell, [&](const Polygon &ring) {
        for (size_t i = 0, previous = ring.points.size() - 1; i < ring.points.size(); previous = i++) {
            if (ring.points[previous] == ring.points[i])
                continue;
            std::vector<Parameter> events;
            if (other.boundary_events(Position(ring.points[previous]), Position(ring.points[i]), events, budget))
                return true;
        }
        return false;
    });
}

// Clipper does not expose a cancellable allocation budget. Bound its possible
// edge interactions before calling it, so a small raw input (for example a grid
// of crossed strips) cannot allocate millions of cells first. The sweep itself
// is charged to the same work budget and keeps only bounded edge boxes.
void guard_boolean(const ExPolygons &first, const ExPolygons &second, Budget &budget)
{
    std::vector<Bounds> edges;
    const auto append = [&](const ExPolygons &polygons) {
        for (const ExPolygon &polygon : polygons)
            visit_rings(polygon, [&](const Polygon &ring) {
                for (size_t i = 0, previous = ring.points.size() - 1; i < ring.points.size(); previous = i++)
                    if (ring.points[previous] != ring.points[i])
                        edges.emplace_back(Position(ring.points[previous]), Position(ring.points[i]));
                return false;
            });
    };
    append(first);
    append(second);
    if (edges.size() > max_vertices)
        throw WorkLimit{};
    budget.consume(edges.size());
    std::sort(edges.begin(), edges.end(), [](const Bounds &a, const Bounds &b) { return a.x0 < b.x0; });
    size_t interactions = 0;
    for (size_t i = 0; i < edges.size(); ++i)
        for (size_t j = i + 1; j < edges.size() && edges[j].x0 <= edges[i].x1; ++j) {
            budget.consume();
            if (edges[i].overlaps(edges[j]) && ++interactions > max_vertices)
                throw WorkLimit{};
        }
}

bool cell_selected(const ExPolygon &cell, const PolygonSet &demand_set, Budget &budget)
{
    if (boundary_touches(cell, demand_set, budget) ||
        demand_set.contains(Position(cell.contour.points.front()), true, budget))
        return true;
    // With no boundary intersections, overlap is possible only by containment.
    // Test a point of each selection contour against the cell, including holes.
    const ExPolygons singleton{cell};
    const PolygonSet cell_set(singleton, budget);
    return demand_set.contour_in(cell_set, Bounds(cell.contour.points), budget);
}

bool checked_points(const Points &points, size_t &count)
{
    if (points.size() > max_vertices - count)
        return false;
    count += points.size();
    return std::all_of(points.begin(), points.end(), [](const Point &p) {
        return p.x() >= -coordinate_limit && p.x() <= coordinate_limit &&
               p.y() >= -coordinate_limit && p.y() <= coordinate_limit;
    });
}

bool checked_polygons(const ExPolygons &polygons, size_t &count)
{
    if (polygons.size() > max_vertices)
        return false;
    for (const ExPolygon &polygon : polygons) {
        if (polygon.contour.points.size() < 3 || !checked_points(polygon.contour.points, count))
            return false;
        for (const Polygon &hole : polygon.holes)
            if (hole.points.size() < 3 || !checked_points(hole.points, count))
                return false;
        if (!polygon.is_valid())
            return false;
    }
    return true;
}

void record_reason(Result &result, const char *reason)
{
    if (result.failure_reason.empty())
        result.failure_reason = reason;
}

void record_limit(Result &result, const WorkLimit &limit)
{
    result.completed = false;
    result.resource_limited = true;
    result.failure_reason = limit.aggregate ?
        "Bridge aware cubic verification exceeded its aggregate geometry work limit; verification is incomplete." :
        "Bridge aware cubic verification exceeded its finite geometry work or allocation limit; verification is incomplete.";
    if (limit.aggregate && limit.shared != nullptr)
        result.failure_reason += " " + limit.shared->describe_work(WorkCategory::Validation);
}

VoidCell measure_cell(ExPolygon cell, const PolygonSet &domain, Budget &budget)
{
    VoidCell measured;
    measured.closed = !boundary_touches(cell, domain, budget);
    const Polygon hull = Geometry::convex_hull(cell.contour.points);
    Scalar squared_diameter = 0;
    for (size_t i = 0; i < hull.points.size(); ++i)
        for (size_t j = i + 1; j < hull.points.size(); ++j) {
            budget.consume();
            const Position delta = Position(hull.points[i]) - Position(hull.points[j]);
            squared_diameter = std::max(squared_diameter, dot(delta, delta));
        }
    measured.diameter_mm = double(std::sqrt(squared_diameter) * SCALING_FACTOR);
    measured.geometry = std::move(cell);
    return measured;
}

// Only candidate selection is indexed. Complete cells remain in their original
// order, and every candidate still runs the unchanged exact selection checks.
class PreparedCellIndex {
public:
    PreparedCellIndex(const std::vector<VoidCell> &cells, size_t owned_vertices, Budget &budget)
    {
        // Bound the index together with all retained physical/cell geometry,
        // including reserved node storage rather than only populated nodes.
        size_t allocated = owned_vertices;
        const auto admit = [&](size_t count) {
            if (count > max_vertices - allocated)
                throw WorkLimit{};
            allocated += count;
        };
        admit(cells.size()); // Bounds.
        admit(cells.size()); // Original cell IDs.
        admit(cells.size() / 2); // Balanced tree with leaves of at most eight.
        admit(cells.size()); // At most one live query candidate per cell.
        m_bounds.reserve(cells.size());
        m_order.resize(cells.size());
        m_nodes.reserve(cells.size() / 2);
        budget.consume(cells.size());
        std::iota(m_order.begin(), m_order.end(), size_t(0));
        for (const VoidCell &cell : cells) {
            budget.consume();
            m_bounds.push_back(polygon_bounds(cell.geometry, budget));
        }
        build(0, cells.size(), budget);
    }

    template<class Visitor> void visit_candidates(const ExPolygons &selection, Budget &budget, Visitor visitor) const
    {
        // Ordered insertion deduplicates overlapping demand boxes without a
        // per-query scan or zero-initialization of every cached cell. Charge
        // actual ordering comparisons, and preserve original cell order.
        Candidates candidates(CandidateLess{&budget});
        for (const ExPolygon &polygon : selection) {
            budget.consume();
            query(0, polygon_bounds(polygon, budget), candidates, budget);
            if (candidates.size() == m_bounds.size())
                break;
        }
        for (size_t index : candidates) {
            budget.consume();
            visitor(index);
        }
    }

private:
    struct Node {
        Bounds bounds;
        size_t begin, end, left = 0, right = 0;
    };
    struct CandidateLess {
        Budget *budget;
        bool operator()(size_t a, size_t b) const
        {
            budget->consume();
            return a < b;
        }
    };
    using Candidates = std::set<size_t, CandidateLess>;

    static void extend(Bounds &bounds, const Bounds &other)
    {
        bounds.x0 = std::min(bounds.x0, other.x0);
        bounds.y0 = std::min(bounds.y0, other.y0);
        bounds.x1 = std::max(bounds.x1, other.x1);
        bounds.y1 = std::max(bounds.y1, other.y1);
    }

    static Bounds polygon_bounds(const ExPolygon &polygon, Budget &budget)
    {
        budget.consume(polygon.contour.points.size());
        Bounds bounds(polygon.contour.points);
        for (const Polygon &hole : polygon.holes) {
            budget.consume(hole.points.size() + 1);
            extend(bounds, Bounds(hole.points));
        }
        return bounds;
    }

    size_t build(size_t begin, size_t end, Budget &budget)
    {
        budget.consume(end - begin);
        Bounds bounds = m_bounds[m_order[begin]];
        for (size_t i = begin + 1; i < end; ++i)
            extend(bounds, m_bounds[m_order[i]]);
        if (m_nodes.size() == m_nodes.capacity())
            throw WorkLimit{};
        budget.consume();
        const size_t node = m_nodes.size();
        m_nodes.push_back({bounds, begin, end});
        if (end - begin > 8) {
            const bool split_x = bounds.x1 - bounds.x0 >= bounds.y1 - bounds.y0;
            const size_t middle = begin + (end - begin) / 2;
            std::nth_element(m_order.begin() + begin, m_order.begin() + middle, m_order.begin() + end,
                [&](size_t a, size_t b) {
                    budget.consume();
                    const Scalar center_a = split_x ? m_bounds[a].x0 + m_bounds[a].x1 : m_bounds[a].y0 + m_bounds[a].y1;
                    const Scalar center_b = split_x ? m_bounds[b].x0 + m_bounds[b].x1 : m_bounds[b].y0 + m_bounds[b].y1;
                    return center_a < center_b || (center_a == center_b && a < b);
                });
            const size_t left = build(begin, middle, budget);
            const size_t right = build(middle, end, budget);
            m_nodes[node].left = left;
            m_nodes[node].right = right;
        }
        return node;
    }

    void query(size_t index, const Bounds &bounds, Candidates &candidates, Budget &budget) const
    {
        budget.consume();
        const Node &node = m_nodes[index];
        // Inclusive overlap preserves edge/vertex touches and either direction
        // of containment; holes are filtered only by the exact predicate.
        if (!node.bounds.overlaps(bounds) || candidates.size() == m_bounds.size())
            return;
        if (node.end - node.begin > 8) {
            query(node.left, bounds, candidates, budget);
            query(node.right, bounds, candidates, budget);
        } else {
            for (size_t i = node.begin; i < node.end; ++i) {
                budget.consume();
                if (m_bounds[m_order[i]].overlaps(bounds)) {
                    budget.consume();
                    // At most one entry per admitted cell, never one per
                    // cell/demand pair. The retained cell count is bounded.
                    candidates.insert(m_order[i]);
                }
            }
        }
    }

    std::vector<Bounds> m_bounds;
    std::vector<size_t> m_order;
    std::vector<Node> m_nodes;
};

} // namespace

std::string WorkBudget::describe_work(WorkCategory active) const
{
    static constexpr const char *names[] = {
        "contact_input", "contact_sections", "contact_candidates", "contact_masks", "validation"
    };
    std::string description = "Charged work:";
    for (size_t i = 0; i < charged_work.size(); ++i)
        description += std::string(" ") + names[i] + "=" + std::to_string(charged_work[i]);
    if (size_t(active) < charged_work.size())
        description += std::string("; active=") + names[size_t(active)];
    return description + "; unspent=" + std::to_string(unspent_at_limit) +
           ", rejected_request=" + std::to_string(rejected_request) + ".";
}

struct PreparedGeometry {
    Result status;
    ExPolygons support, domain;
    std::vector<VoidCell> cells;
    std::unique_ptr<const PreparedCellIndex> cell_index;
};

PreparedGeometryPtr prepare_geometry(const ExPolygons &material_domain,
                                     const ExPolygons &deposited_support, WorkBudget *shared)
{
    auto prepared = std::make_shared<PreparedGeometry>();
    size_t vertices = 0;
    if (!checked_polygons(material_domain, vertices) || !checked_polygons(deposited_support, vertices)) {
        prepared->status.input_valid = false;
        prepared->status.failure_reason = "Prepared physical geometry is invalid or exceeds coordinate or vertex bounds.";
        return prepared;
    }
    Budget budget(shared);
    try {
        budget.consume(vertices);
        // Contact-mode path-only checks have no use for physical cell unions.
        if (material_domain.empty())
            return prepared;
        guard_boolean(deposited_support, {}, budget);
        prepared->support = union_ex(deposited_support);
        guard_boolean(material_domain, {}, budget);
        prepared->domain = union_ex(material_domain);
        vertices = 0;
        if (!checked_polygons(prepared->support, vertices) || !checked_polygons(prepared->domain, vertices))
            throw WorkLimit{};
        const size_t physical_vertices = vertices;
        guard_boolean(prepared->domain, prepared->support, budget);
        ExPolygons cells = diff_ex(prepared->domain, prepared->support);
        vertices = 0;
        if (!checked_polygons(cells, vertices))
            throw WorkLimit{};
        if (vertices > max_vertices - physical_vertices)
            throw WorkLimit{};
        const size_t owned_vertices = physical_vertices + vertices;
        const PolygonSet domain(prepared->domain, budget);
        for (ExPolygon &cell : cells)
            prepared->cells.push_back(measure_cell(std::move(cell), domain, budget));
        if (prepared->cells.size() > 64)
            prepared->cell_index = std::make_unique<PreparedCellIndex>(prepared->cells, owned_vertices, budget);
    } catch (const WorkLimit &limit) {
        record_limit(prepared->status, limit);
    } catch (const std::exception &error) {
        prepared->status.completed = false;
        prepared->status.failure_reason = std::string("Bridge aware cubic physical geometry preparation failed: ") + error.what();
    }
    return prepared;
}

bool Result::passed() const
{
    if (!input_valid || !completed)
        return false;
    for (const auto &run : unsupported_runs)
        if (run.kind != UnsupportedRunKind::TwoEndedBridge || run.exceeds_gap)
            return false;
    for (const auto &cell : void_cells)
        if (!cell.closed || cell.exceeds_gap)
            return false;
    return true;
}

static Result validate_impl(const ExPolygons &material_domain, const ExPolygons &deposited_support,
                const Polylines &required_paths, const ExPolygons &demand,
                double max_gap_mm, double geometry_tolerance_mm,
                const std::vector<BridgeAwareCubicContact::PathContact> *contacts,
                WorkBudget *shared = nullptr, const PreparedGeometry *prepared = nullptr,
                bool report_passing_cells = true, bool preserve_cell_maximum = false)
{
    Result result;
    if (prepared != nullptr && (!prepared->status.input_valid || !prepared->status.completed))
        return prepared->status;
    const double tolerance = std::max(geometry_tolerance_mm, SCALING_FACTOR);
    if (!std::isfinite(max_gap_mm) || max_gap_mm <= 0 ||
        max_gap_mm > 2. * double(coordinate_limit) * SCALING_FACTOR ||
        !std::isfinite(geometry_tolerance_mm) || geometry_tolerance_mm < 0 || tolerance >= max_gap_mm) {
        result.input_valid = false;
        result.failure_reason = "Gap and tolerance must be finite, positive and representable, with tolerance smaller than gap.";
        return result;
    }
    size_t vertices = 0;
    if (!checked_polygons(material_domain, vertices) || !checked_polygons(deposited_support, vertices) ||
        !checked_polygons(demand, vertices) || required_paths.size() > max_vertices) {
        result.input_valid = false;
        result.failure_reason = "Polygon input is invalid or exceeds the verifier coordinate or vertex bounds.";
        return result;
    }
    for (const Polyline &path : required_paths) {
        if (path.points.size() < 2 || !checked_points(path.points, vertices)) {
            result.input_valid = false;
            result.failure_reason = "Required path is invalid or exceeds the verifier coordinate or vertex bounds.";
            return result;
        }
        for (const PathFittingData &fit : path.fitting_result)
            if (fit.path_type == EMovePathType::Arc_move_cw || fit.path_type == EMovePathType::Arc_move_ccw) {
                result.input_valid = false;
                result.failure_reason = "Fitted arcs require an arc-aware check; endpoint chords cannot certify their unsupported travel.";
                return result;
            }
    }
    if (contacts != nullptr) {
        if (contacts->size() != required_paths.size()) {
            result.input_valid = false;
            result.failure_reason = "Contact certificates must correspond one-to-one with full required paths.";
            return result;
        }
        for (size_t i = 0; i < contacts->size(); ++i) {
            const auto &contact = (*contacts)[i];
            const bool finite_contact = std::isfinite(contact.contact_z) &&
                std::isfinite(contact.coordinate_uncertainty_mm) && contact.coordinate_uncertainty_mm >= 0. &&
                contact.coordinate_uncertainty_mm <= 2. * double(coordinate_limit) * SCALING_FACTOR;
            if (!contact.input_valid || !contact.completed || !finite_contact) {
                result.input_valid = contact.input_valid && finite_contact;
                result.completed = contact.completed;
                result.resource_limited = contact.resource_limited;
                result.failure_reason = contact.failure_reason.empty() ? "Invalid bead contact certificate." : contact.failure_reason;
                return result;
            }
            const Points &points = required_paths[i].points;
            size_t next = 0;
            for (size_t j = 1; j < points.size(); ++j) {
                if (points[j] == points[j - 1]) continue;
                if (next >= contact.segments.size() || contact.segments[next].segment_index != j - 1 ||
                    !checked_polygons(contact.segments[next].centerline_support, vertices)) {
                    result.input_valid = false;
                    result.failure_reason = "Contact segment correspondence or contact mask geometry is invalid.";
                    return result;
                }
                for (const auto &interval : contact.segments[next].line_contacts) {
                    if (interval.begin_denominator <= 0 || interval.end_denominator <= 0 ||
                        interval.begin_numerator < 0 || interval.end_numerator < 0 ||
                        interval.begin_numerator > interval.begin_denominator ||
                        interval.end_numerator > interval.end_denominator ||
                        !std::isfinite(interval.position_uncertainty_mm) || interval.position_uncertainty_mm < 0. ||
                        interval.position_uncertainty_mm > 2. * double(coordinate_limit) * SCALING_FACTOR ||
                        Parameter(interval.end_numerator, interval.end_denominator) <
                            Parameter(interval.begin_numerator, interval.begin_denominator) || vertices == max_vertices) {
                        result.input_valid = false;
                        result.failure_reason = "Qualified line-contact interval is invalid or exceeds verifier bounds.";
                        return result;
                    }
                    ++vertices;
                }
                ++next;
            }
            if (next != contact.segments.size()) {
                result.input_valid = false;
                result.failure_reason = "Contact certificate contains extra segments.";
                return result;
            }
        }
    }
    if (demand.empty())
        return result;

    const Scalar permitted = max_gap_mm - tolerance;
    Budget budget(shared);
    try {
        budget.consume(vertices);
        ExPolygons support, domain;
        // Contact masks already certify path support. In path-only mode the
        // global raw footprint is unused, so neither union nor difference may
        // be performed merely because it was supplied for another check mode.
        if (contacts == nullptr || (prepared == nullptr && !material_domain.empty())) {
            guard_boolean(deposited_support, {}, budget);
            support = union_ex(deposited_support);
        }
        guard_boolean(demand, {}, budget);
        const ExPolygons selection = union_ex(demand);
        if (prepared == nullptr && !material_domain.empty()) {
            guard_boolean(material_domain, {}, budget);
            domain = union_ex(material_domain);
        }
        size_t generated_vertices = 0;
        if (!checked_polygons(support, generated_vertices) || !checked_polygons(selection, generated_vertices) ||
            !checked_polygons(domain, generated_vertices))
            throw WorkLimit{};
        const PolygonSet support_set(support, budget), demand_set(selection, budget), domain_set(domain, budget);
        for (size_t path_index = 0; path_index < required_paths.size(); ++path_index) {
            const Points &points = required_paths[path_index].points;
            std::vector<Interval> supported, demanded;
            std::vector<PathSegment> segments;
            segments.reserve(points.size() - 1);
            Scalar total = 0;
            for (size_t i = 1; i < points.size(); ++i) {
                const Position a(points[i - 1]), b(points[i]);
                const Position delta = b - a;
                const Scalar length = std::sqrt(dot(delta, delta)) * SCALING_FACTOR;
                if (length == 0)
                    continue;
                segment_intervals(a, b, segments.size(), demand_set, true, demanded, budget);
                segments.push_back({a, b, length, total, i - 1});
                total += length;
            }
            if (total == 0) {
                result.input_valid = false;
                result.failure_reason = "Required path has no positive-length extrusion.";
                return result;
            }
            if (demanded.empty())
                continue;
            for (size_t i = 0; i < segments.size(); ++i) {
                if (contacts == nullptr)
                    segment_intervals(segments[i].a, segments[i].b, i, support_set, false, supported, budget);
                else {
                    const auto &contact = (*contacts)[path_index].segments[i];
                    const PolygonSet segment_support(contact.centerline_support, budget);
                    // Keep the local lists separate until sorting: exact line
                    // events need not be supplied in geometric order.
                    std::vector<Interval> local;
                    segment_intervals(segments[i].a, segments[i].b, i, segment_support, false, local, budget);
                    supported.insert(supported.end(), local.begin(), local.end());
                    for (const auto &line : contact.line_contacts) {
                        budget.consume();
                        supported.push_back({PathPosition(i, Parameter(line.begin_numerator, line.begin_denominator)),
                                             PathPosition(i, Parameter(line.end_numerator, line.end_denominator)),
                                             line.position_uncertainty_mm, line.position_uncertainty_mm});
                    }
                }
            }
            if (contacts != nullptr) {
                std::sort(supported.begin(), supported.end(), [](const Interval &a, const Interval &b) {
                    return a.begin < b.begin || (!(b.begin < a.begin) && a.end < b.end);
                });
                std::vector<Interval> merged;
                for (const Interval &interval : supported)
                    append_interval(merged, interval.begin, interval.end,
                                    interval.begin_uncertainty, interval.end_uncertainty);
                supported = std::move(merged);
            }
            const auto add_run = [&](const PathPosition &begin, const PathPosition &end, bool left_anchor, bool right_anchor,
                                     double left_uncertainty, double right_uncertainty) {
                if (!(begin < end) || !selected(begin, end, demanded))
                    return;
                const Scalar nominal_length = local_arclength(begin, end, segments, budget);
                double uncertainty = left_uncertainty + right_uncertainty;
                if (contacts != nullptr) {
                    const size_t after_last = end.segment + (end.parameter.numerator != 0 ? 1 : 0);
                    const size_t traversed = after_last - begin.segment;
                    uncertainty += 2. * (*contacts)[path_index].coordinate_uncertainty_mm * double(traversed);
                }
                const Scalar length = nominal_length + uncertainty;
                UnsupportedRun run;
                run.path_index = path_index;
                run.begin_mm = double(path_position_mm(begin, segments));
                run.end_mm = double(path_position_mm(end, segments));
                run.length_mm = double(length);
                run.uncertainty_mm = uncertainty;
                run.kind = left_anchor && right_anchor ? UnsupportedRunKind::TwoEndedBridge :
                           left_anchor || right_anchor ? UnsupportedRunKind::OneSidedOverhang : UnsupportedRunKind::Floating;
                run.exceeds_gap = length > permitted;
                result.max_unsupported_mm = std::max(result.max_unsupported_mm, run.length_mm);
                result.unsupported_runs.push_back(run);
                if (run.kind == UnsupportedRunKind::Floating)
                    record_reason(result, "A demanded extrusion run has no physical end anchors.");
                else if (run.kind == UnsupportedRunKind::OneSidedOverhang)
                    record_reason(result, "A demanded extrusion run is one-sided and requires an overhang-support check.");
                else if (run.exceeds_gap)
                    record_reason(result, "A demanded extrusion run exceeds the unsupported gap.");
            };
            const PathPosition start(0, Parameter(0, 1)), finish(segments.size(), Parameter(0, 1));
            PathPosition cursor = start;
            bool left_anchor = false;
            double cursor_uncertainty = 0.;
            for (const Interval &interval : supported) {
                add_run(cursor, interval.begin, left_anchor, true, cursor_uncertainty, interval.begin_uncertainty);
                if (cursor < interval.end) {
                    cursor = interval.end;
                    cursor_uncertainty = interval.end_uncertainty;
                } else if (!(interval.end < cursor))
                    cursor_uncertainty = left_anchor ? std::min(cursor_uncertainty, interval.end_uncertainty) : interval.end_uncertainty;
                left_anchor = true;
            }
            add_run(cursor, finish, left_anchor, false, cursor_uncertainty, 0.);
        }

        const auto record_cell = [&](VoidCell measured) {
            measured.exceeds_gap = measured.diameter_mm > permitted;
            result.max_void_diameter_mm = std::max(result.max_void_diameter_mm, measured.diameter_mm);
            if (!measured.closed)
                record_reason(result, "A demanded void reaches an unclosed physical model or cavity boundary.");
            else if (measured.exceeds_gap)
                record_reason(result, "A demanded void cell exceeds the unsupported gap.");
            result.void_cells.push_back(std::move(measured));
        };
        if (prepared != nullptr) {
            const auto visit_cell = [&](size_t index) {
                const VoidCell &cell = prepared->cells[index];
                // Selection cannot change the acceptance of a safe cell. If
                // complete metrics are needed, only a diameter larger than
                // the selected maximum can matter, and its selection remains
                // exact. Potential failures always retain the whole-cell test.
                if (!report_passing_cells && cell.closed && cell.diameter_mm <= permitted) {
                    if (preserve_cell_maximum && cell.diameter_mm > result.max_void_diameter_mm &&
                        cell_selected(cell.geometry, demand_set, budget))
                        result.max_void_diameter_mm = std::max(result.max_void_diameter_mm, cell.diameter_mm);
                    return;
                }
                if (cell_selected(cell.geometry, demand_set, budget)) {
                    budget.consume(cell.geometry.contour.points.size());
                    for (const Polygon &hole : cell.geometry.holes)
                        budget.consume(hole.points.size());
                    record_cell(cell);
                }
            };
            if (prepared->cell_index != nullptr)
                prepared->cell_index->visit_candidates(selection, budget, visit_cell);
            else
                for (size_t i = 0; i < prepared->cells.size(); ++i) {
                    budget.consume();
                    visit_cell(i);
                }
        } else if (!domain.empty()) {
            guard_boolean(domain, support, budget);
            ExPolygons cells = diff_ex(domain, support);
            generated_vertices = 0;
            if (!checked_polygons(cells, generated_vertices))
                throw WorkLimit{};
            for (ExPolygon &cell : cells)
                if (cell_selected(cell, demand_set, budget))
                    record_cell(measure_cell(std::move(cell), domain_set, budget));
        }
    } catch (const WorkLimit &limit) {
        record_limit(result, limit);
    } catch (const std::exception &error) {
        result.completed = false;
        result.failure_reason = std::string("Bridge aware cubic geometry verification failed: ") + error.what();
    }
    return result;
}

Result validate(const ExPolygons &material_domain, const ExPolygons &deposited_support,
                const Polylines &required_paths, const ExPolygons &demand,
                double max_gap_mm, double geometry_tolerance_mm)
{
    return validate_impl(material_domain, deposited_support, required_paths, demand,
                         max_gap_mm, geometry_tolerance_mm, nullptr);
}

Result validate_contacts(const ExPolygons &material_domain, const ExPolygons &deposited_support,
                         const Polylines &required_paths, const ExPolygons &demand,
                         double max_gap_mm, double geometry_tolerance_mm,
                         const std::vector<BridgeAwareCubicContact::PathContact> &contacts,
                         WorkBudget *budget)
{
    return validate_impl(material_domain, deposited_support, required_paths, demand,
                         max_gap_mm, geometry_tolerance_mm, &contacts, budget);
}

Result validate_prepared_contacts(const PreparedGeometry &prepared,
                                  const Polylines &required_paths, const ExPolygons &demand,
                                  double max_gap_mm, double geometry_tolerance_mm,
                                  const std::vector<BridgeAwareCubicContact::PathContact> &contacts,
                                  WorkBudget *budget)
{
    return validate_impl({}, {}, required_paths, demand,
                         max_gap_mm, geometry_tolerance_mm, &contacts, budget, &prepared);
}

Result validate_prepared_contacts_for_trial(const PreparedGeometry &prepared,
                                            const Polylines &required_paths, const ExPolygons &demand,
                                            double max_gap_mm, double geometry_tolerance_mm,
                                            const std::vector<BridgeAwareCubicContact::PathContact> &contacts,
                                            WorkBudget *budget)
{
    return validate_impl({}, {}, required_paths, demand,
                         max_gap_mm, geometry_tolerance_mm, &contacts, budget, &prepared, false);
}

Result validate_prepared_contact_failures(const PreparedGeometry &prepared,
                                          const Polylines &required_paths, const ExPolygons &demand,
                                          double max_gap_mm, double geometry_tolerance_mm,
                                          const std::vector<BridgeAwareCubicContact::PathContact> &contacts,
                                          WorkBudget *budget)
{
    return validate_impl({}, {}, required_paths, demand,
                         max_gap_mm, geometry_tolerance_mm, &contacts, budget, &prepared, false, true);
}

}} // namespace Slic3r::BridgeAwareCubicValidation
