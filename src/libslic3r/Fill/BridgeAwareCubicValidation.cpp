#include "BridgeAwareCubicValidation.hpp"

#include "../ClipperUtils.hpp"
#include "../Geometry/ConvexHull.hpp"

#include <boost/multiprecision/cpp_int.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <numeric>

namespace Slic3r { namespace BridgeAwareCubicValidation {

namespace {
// Limit both allocations and the potentially quadratic intersection/diameter
// work. Reaching either cap is an incomplete check, never a certificate.
constexpr size_t max_vertices = 200000;
constexpr size_t max_work = 50000000;
constexpr coord_t coordinate_limit = std::numeric_limits<coord_t>::max() / 4;
using Scalar = long double;

struct WorkLimit {};
struct Budget {
    size_t remaining = max_work;
    void consume(size_t count = 1)
    {
        if (count > remaining)
            throw WorkLimit{};
        remaining -= count;
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

Location locate(const Polygon &ring, const Position &p, Budget &budget)
{
    budget.consume(ring.points.size());
    bool inside = false;
    for (size_t i = 0, previous = ring.points.size() - 1; i < ring.points.size(); previous = i++) {
        const Position a(ring.points[previous]), b(ring.points[i]);
        const Position delta = b - a, relative = p - a;
        if (std::abs(cross(delta, relative)) <= cross_error(delta, relative) && Bounds(a, b).contains(p))
            return Location::Boundary;
        if ((a.y > p.y) != (b.y > p.y) && p.x < a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y))
            inside = !inside;
    }
    return inside ? Location::Inside : Location::Outside;
}

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
    explicit PolygonSet(const ExPolygons &polygons) : m_polygons(polygons)
    {
        for (const ExPolygon &polygon : polygons) {
            m_bounds.emplace_back(polygon.contour.points);
            append_edges(polygon.contour);
            for (const Polygon &hole : polygon.holes)
                append_edges(hole);
        }
    }

    bool contains(const Position &p, bool boundary, Budget &budget) const
    {
        for (size_t i = 0; i < m_polygons.size(); ++i) {
            budget.consume();
            if (!m_bounds[i].contains(p))
                continue;
            const ExPolygon &polygon = m_polygons[i];
            const Location outer = locate(polygon.contour, p, budget);
            if (outer == Location::Outside)
                continue;
            if (outer == Location::Boundary)
                return boundary;
            bool in_hole = false;
            for (const Polygon &hole : polygon.holes) {
                const Location location = locate(hole, p, budget);
                if (location == Location::Boundary)
                    return boundary;
                if (location == Location::Inside) {
                    in_hole = true;
                    break;
                }
            }
            if (!in_hole)
                return true;
        }
        return false;
    }

    // The event positions belong to the original segment, including tangent
    // and collinear boundary contacts. They are never extrusion endpoints.
    bool boundary_events(const Position &a, const Position &b, std::vector<Parameter> &events, Budget &budget) const
    {
        const Position delta = b - a;
        const int64_t squared_length = integer_dot(delta, delta);
        const Bounds bounds(a, b);
        bool touches = false;
        for (const Edge &edge : m_edges) {
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
        return touches;
    }

private:
    void append_edges(const Polygon &ring)
    {
        for (size_t i = 0, previous = ring.points.size() - 1; i < ring.points.size(); previous = i++)
            if (ring.points[previous] != ring.points[i])
                m_edges.emplace_back(ring.points[previous], ring.points[i]);
    }
    const ExPolygons &m_polygons;
    std::vector<Bounds> m_bounds;
    std::vector<Edge> m_edges;
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

struct Interval { PathPosition begin, end; };

struct PathSegment {
    Position a, b;
    Scalar length_mm;
    Scalar prefix_mm;
};

void append_interval(std::vector<Interval> &intervals, const PathPosition &begin, const PathPosition &end)
{
    if (!intervals.empty() && !(intervals.back().end < begin)) {
        if (intervals.back().end < end)
            intervals.back().end = end;
    }
    else
        intervals.push_back({begin, end});
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

bool cell_selected(const ExPolygon &cell, const PolygonSet &demand_set,
                   const ExPolygons &selection, Budget &budget)
{
    if (boundary_touches(cell, demand_set, budget) ||
        demand_set.contains(Position(cell.contour.points.front()), true, budget))
        return true;
    // With no boundary intersections, overlap is possible only by containment.
    // Test a point of each selection contour against the cell, including holes.
    const ExPolygons singleton{cell};
    const PolygonSet cell_set(singleton);
    for (const ExPolygon &polygon : selection)
        if (cell_set.contains(Position(polygon.contour.points.front()), true, budget))
            return true;
    return false;
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

} // namespace

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

Result validate(const ExPolygons &material_domain, const ExPolygons &deposited_support,
                const Polylines &required_paths, const ExPolygons &demand,
                double max_gap_mm, double geometry_tolerance_mm)
{
    Result result;
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
    if (demand.empty())
        return result;

    const Scalar permitted = max_gap_mm - tolerance;
    Budget budget;
    try {
        guard_boolean(deposited_support, {}, budget);
        const ExPolygons support = union_ex(deposited_support);
        guard_boolean(demand, {}, budget);
        const ExPolygons selection = union_ex(demand);
        guard_boolean(material_domain, {}, budget);
        const ExPolygons domain = union_ex(material_domain);
        size_t generated_vertices = 0;
        if (!checked_polygons(support, generated_vertices) || !checked_polygons(selection, generated_vertices) ||
            !checked_polygons(domain, generated_vertices))
            throw WorkLimit{};
        const PolygonSet support_set(support), demand_set(selection), domain_set(domain);
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
                segments.push_back({a, b, length, total});
                total += length;
            }
            if (total == 0) {
                result.input_valid = false;
                result.failure_reason = "Required path has no positive-length extrusion.";
                return result;
            }
            if (demanded.empty())
                continue;
            for (size_t i = 0; i < segments.size(); ++i)
                segment_intervals(segments[i].a, segments[i].b, i, support_set, false, supported, budget);
            const auto add_run = [&](const PathPosition &begin, const PathPosition &end, bool left_anchor, bool right_anchor) {
                if (!(begin < end) || !selected(begin, end, demanded))
                    return;
                const Scalar length = local_arclength(begin, end, segments, budget);
                UnsupportedRun run;
                run.path_index = path_index;
                run.begin_mm = double(path_position_mm(begin, segments));
                run.end_mm = double(path_position_mm(end, segments));
                run.length_mm = double(length);
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
            for (const Interval &interval : supported) {
                add_run(cursor, interval.begin, start < cursor, true);
                if (cursor < interval.end)
                    cursor = interval.end;
            }
            add_run(cursor, finish, start < cursor, false);
        }

        guard_boolean(domain, support, budget);
        ExPolygons cells = diff_ex(domain, support);
        generated_vertices = 0;
        if (!checked_polygons(cells, generated_vertices))
            throw WorkLimit{};
        for (ExPolygon &cell : cells) {
            if (!cell_selected(cell, demand_set, selection, budget))
                continue;
            VoidCell measured;
            measured.closed = !boundary_touches(cell, domain_set, budget);
            const Polygon hull = Geometry::convex_hull(cell.contour.points);
            Scalar squared_diameter = 0;
            for (size_t i = 0; i < hull.points.size(); ++i)
                for (size_t j = i + 1; j < hull.points.size(); ++j) {
                    budget.consume();
                    const Position delta = Position(hull.points[i]) - Position(hull.points[j]);
                    squared_diameter = std::max(squared_diameter, dot(delta, delta));
                }
            measured.diameter_mm = double(std::sqrt(squared_diameter) * SCALING_FACTOR);
            measured.exceeds_gap = measured.diameter_mm > permitted;
            result.max_void_diameter_mm = std::max(result.max_void_diameter_mm, measured.diameter_mm);
            if (!measured.closed)
                record_reason(result, "A demanded void reaches an unclosed physical model or cavity boundary.");
            else if (measured.exceeds_gap)
                record_reason(result, "A demanded void cell exceeds the unsupported gap.");
            measured.geometry = std::move(cell);
            result.void_cells.push_back(std::move(measured));
        }
    } catch (const WorkLimit &) {
        result.completed = false;
        result.failure_reason = "Bridge aware cubic verification exceeded its finite geometry work or allocation limit.";
    } catch (const std::exception &error) {
        result.completed = false;
        result.failure_reason = std::string("Bridge aware cubic geometry verification failed: ") + error.what();
    }
    return result;
}

}} // namespace Slic3r::BridgeAwareCubicValidation
