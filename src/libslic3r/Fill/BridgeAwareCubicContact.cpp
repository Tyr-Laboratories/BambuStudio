#include "BridgeAwareCubicContact.hpp"
#include "BridgeAwareCubicValidation.hpp"

#include "../ClipperUtils.hpp"
#include "../Geometry/ConvexHull.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <limits>
#include <map>
#include <numeric>
#include <stdexcept>
#include <tuple>

namespace Slic3r { namespace BridgeAwareCubicContact {

namespace {
using Scalar = long double;
constexpr coord_t coordinate_limit = std::numeric_limits<coord_t>::max() / 4;
constexpr size_t max_vertices = 200000;
constexpr size_t max_work = 50000000;
constexpr int64_t interval_denominator = 1000000000000000LL;

enum class LimitKind { LocalWork, AggregateWork, Vertices, Coordinates };
struct Limit {
    LimitKind kind = LimitKind::LocalWork;
    size_t used = 0, requested = 0;
};
struct Budget;
void ensure_query_capacity(QueryCacheData *, size_t, Budget &);
struct Budget {
    size_t work = 0, vertices = 0;
    BridgeAwareCubicValidation::WorkBudget *aggregate = nullptr;
    QueryCacheData *cache = nullptr;
    BridgeAwareCubicValidation::WorkCategory category = BridgeAwareCubicValidation::WorkCategory::ContactInput;
    explicit Budget(BridgeAwareCubicValidation::WorkBudget *shared = nullptr) : aggregate(shared) {}
    void consume(size_t n = 1)
    {
        if (aggregate != nullptr && n > aggregate->remaining_work) {
            aggregate->unspent_at_limit = aggregate->remaining_work;
            aggregate->rejected_request = n;
            aggregate->remaining_work = 0;
            throw Limit{LimitKind::AggregateWork};
        }
        if (n > max_work - work)
            throw Limit{LimitKind::LocalWork, work, n};
        work += n;
        if (aggregate != nullptr) {
            aggregate->remaining_work -= n;
            aggregate->charged_work[size_t(category)] += n;
        }
    }
    void allocate(size_t n)
    {
        if (cache != nullptr)
            ensure_query_capacity(cache, n, *this);
        if (n > max_vertices - vertices)
            throw Limit{LimitKind::Vertices, vertices, n};
        consume(n);
        vertices += n;
    }
};
struct V {
    Scalar x, y;
    explicit V(const Point &p) : x(p.x()), y(p.y()) {}
    V(Scalar x_, Scalar y_) : x(x_), y(y_) {}
};
V operator+(V a, V b) { return {a.x + b.x, a.y + b.y}; }
V operator-(V a, V b) { return {a.x - b.x, a.y - b.y}; }
V operator*(V a, Scalar s) { return {a.x * s, a.y * s}; }
Scalar dot(V a, V b) { return a.x * b.x + a.y * b.y; }
int64_t idot(V a, V b) { return int64_t(a.x) * int64_t(b.x) + int64_t(a.y) * int64_t(b.y); }
int64_t icross(V a, V b) { return int64_t(a.x) * int64_t(b.y) - int64_t(a.y) * int64_t(b.x); }

bool dimensions(double width, double height, double top)
{
    return std::isfinite(width) && std::isfinite(height) && std::isfinite(top) &&
           width >= height && height > 0. && width < double(coordinate_limit) * SCALING_FACTOR &&
           std::abs(top) < double(coordinate_limit) * SCALING_FACTOR && top - height < top;
}

bool valid_path(const ExtrusionPath &path, double top, Budget &budget,
                bool permit_unmodeled_lower = false, bool *positive_xy = nullptr,
                const char **invalid_reason = nullptr)
{
    const auto reject = [&](const char *reason) {
        if (invalid_reason != nullptr)
            *invalid_reason = reason;
        return false;
    };
    const double model_width = permit_unmodeled_lower ? std::max(path.width, path.height) : path.width;
    if (!dimensions(model_width, path.height, top) || !std::isfinite(path.width) || path.width <= 0.)
        return reject("dimensions or elevation do not define a finite supported bead section.");
    if (!std::isfinite(path.mm3_per_mm) || path.mm3_per_mm <= 0.)
        return reject("extrusion amount must be finite and positive.");
    if (path.is_force_no_extrusion())
        return reject("path explicitly suppresses extrusion.");
    if (path.polyline.points.size() < 2)
        return reject("path has fewer than two XY vertices.");
    budget.allocate(path.polyline.points.size());
    bool positive = false;
    const Points &points = path.polyline.points;
    for (size_t i = 0; i < points.size(); ++i) {
        const Point &p = points[i];
        if (p.x() < -coordinate_limit || p.x() > coordinate_limit ||
            p.y() < -coordinate_limit || p.y() > coordinate_limit)
            return reject("XY coordinate exceeds the supported geometry range.");
        positive |= i > 0 && p != points[i - 1];
    }
    for (const PathFittingData &fit : path.polyline.fitting_result)
        if (fit.path_type == EMovePathType::Arc_move_cw || fit.path_type == EMovePathType::Arc_move_ccw)
            return reject("fitted arc requires segmented geometry.");
    if (positive_xy != nullptr)
        *positive_xy = positive;
    return positive || permit_unmodeled_lower || reject("path has no positive-length XY segment.");
}

// Only representation uncertainty of a float height and double Z arithmetic.
// This is independent of the user-facing XY geometry tolerance.
double z_roundoff(float height, double top)
{
    return .5 * double(std::nextafter(height, std::numeric_limits<float>::infinity()) - height) +
           8. * std::numeric_limits<double>::epsilon() * std::max(1., std::abs(top));
}

double snapped_section(const ExtrusionPath &path, double top, double z)
{
    const double bottom = top - path.height;
    const double error = z_roundoff(path.height, top);
    if (std::abs(z - top) <= error || std::abs(z - bottom) <= error)
        return double(path.width) - double(path.height);
    return section_width_mm(path.width, path.height, top, z);
}

Point rounded(V p)
{
    if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
        std::abs(p.x) > coordinate_limit - 4 || std::abs(p.y) > coordinate_limit - 4)
        throw Limit{LimitKind::Coordinates};
    return Point(coord_t(std::llround(p.x)), coord_t(std::llround(p.y)));
}

// Rounding a vertex moves it at most sqrt(1/2) coordinate units. Eroding the
// convex rounded hull by three units therefore leaves an inner approximation.
// Negative offset of this one convex polygon has bounded complexity.
ExPolygons inner_hull(const std::vector<V> &corners, double reserve_mm, Budget &budget)
{
    // These convex inputs contain at most eight vertices. Charge their bounded
    // intersection work before entering the hull/offset operations.
    budget.consume(corners.size() * corners.size());
    Points points;
    points.reserve(corners.size());
    for (V corner : corners)
        points.push_back(rounded(corner));
    Polygon hull = Geometry::convex_hull(std::move(points));
    if (hull.points.size() < 3 || hull.area() <= 0.)
        return {};
    const double erosion = reserve_mm / SCALING_FACTOR + 3.;
    float rounded_erosion = float(erosion);
    if (double(rounded_erosion) < erosion)
        rounded_erosion = std::nextafter(rounded_erosion, std::numeric_limits<float>::infinity());
    ExPolygons result = offset_ex(hull, -rounded_erosion, ClipperLib::jtMiter);
    for (const ExPolygon &polygon : result)
        budget.allocate(polygon.contour.points.size());
    return result;
}

struct LowerSection {
    Point a, b;
    Scalar radius;
    std::vector<V> corners;
    std::tuple<int64_t, int64_t, int64_t, Scalar> line_key() const
    {
        int64_t dx = int64_t(b.x()) - a.x(), dy = int64_t(b.y()) - a.y();
        const int64_t divisor = std::gcd(dx, dy);
        dx /= divisor;
        dy /= divisor;
        return {dx, dy, dx * int64_t(a.y()) - dy * int64_t(a.x()), radius};
    }
};

// Immutable broad phase over the already validated, canonically ordered
// sections. Extra diagonal projections are used only when they prove the
// existing upper-normal rejection; otherwise queries use the original XY
// reach test. This preserves even masks which do not meet the upper axis.
class SectionIndex {
    struct Node {
        std::array<int64_t, 4> low, high;
        Scalar radius = 0;
        size_t begin = 0, end = 0, left = 0, right = 0;
    };
    std::vector<size_t> m_order;
    std::vector<Node> m_nodes;

    static std::array<int64_t, 4> projection(const Point &p)
    {
        return {{p.x(), p.y(), int64_t(p.x()) + p.y(), int64_t(p.x()) - p.y()}};
    }

    size_t build(const std::vector<LowerSection> &sections, size_t begin, size_t end, Budget &budget)
    {
        budget.consume(end - begin);
        Node node;
        node.begin = begin;
        node.end = end;
        node.low.fill(std::numeric_limits<int64_t>::max());
        node.high.fill(std::numeric_limits<int64_t>::min());
        auto center_low = node.low, center_high = node.high;
        for (size_t i = begin; i < end; ++i) {
            const LowerSection &section = sections[m_order[i]];
            const auto a = projection(section.a), b = projection(section.b);
            node.radius = std::max(node.radius, section.radius);
            for (size_t axis = 0; axis < 4; ++axis) {
                node.low[axis] = std::min(node.low[axis], std::min(a[axis], b[axis]));
                node.high[axis] = std::max(node.high[axis], std::max(a[axis], b[axis]));
                center_low[axis] = std::min(center_low[axis], a[axis] + b[axis]);
                center_high[axis] = std::max(center_high[axis], a[axis] + b[axis]);
            }
        }
        budget.allocate(1);
        const size_t index = m_nodes.size();
        m_nodes.push_back(node);
        if (end - begin > 8) {
            size_t axis = 0;
            Scalar greatest = -1;
            for (size_t i = 0; i < 4; ++i) {
                const Scalar span = center_high[i] - center_low[i];
                const Scalar extent = span * span / (i < 2 ? 1 : 2);
                if (extent > greatest) { axis = i; greatest = extent; }
            }
            const size_t middle = begin + (end - begin) / 2;
            std::nth_element(m_order.begin() + begin, m_order.begin() + middle, m_order.begin() + end,
                [&](size_t a, size_t b) {
                    budget.consume();
                    const int64_t ca = projection(sections[a].a)[axis] + projection(sections[a].b)[axis];
                    const int64_t cb = projection(sections[b].a)[axis] + projection(sections[b].b)[axis];
                    return ca < cb || (ca == cb && a < b);
                });
            const size_t left = build(sections, begin, middle, budget);
            const size_t right = build(sections, middle, end, budget);
            m_nodes[index].left = left;
            m_nodes[index].right = right;
        }
        return index;
    }

    static bool disjoint(const Node &node, const Point &a, const Point &b,
                         V direction, Scalar length, Scalar upper_radius)
    {
        const Scalar reach = node.radius + upper_radius + 4;
        if (std::max(a.x(), b.x()) + reach < node.low[0] ||
            std::min(a.x(), b.x()) - reach > node.high[0] ||
            std::max(a.y(), b.y()) + reach < node.low[1] ||
            std::min(a.y(), b.y()) - reach > node.high[1])
            return true;
        size_t axis;
        int64_t coefficient;
        if (direction.x == direction.y) {
            axis = 3;
            coefficient = -int64_t(direction.x);
        } else if (direction.x == -direction.y) {
            axis = 2;
            coefficient = int64_t(direction.x);
        } else return false;
        const int64_t origin = projection(a)[axis];
        // These products have the same coordinate-derived bound as icross.
        Scalar first = Scalar(coefficient * (node.low[axis] - origin));
        Scalar last = Scalar(coefficient * (node.high[axis] - origin));
        if (first > last) std::swap(first, last);
        const Scalar limit = reach * length;
        // At least the rounding allowance of every original per-pair normal
        // test, including a pair with both endpoints at one extreme.
        const Scalar error = 64 * std::numeric_limits<Scalar>::epsilon() *
            (2 * std::max(std::abs(first), std::abs(last)) + std::abs(limit) + 1);
        return first > limit + error || last < -limit - error;
    }

    void query(size_t index, const Point &a, const Point &b, V direction, Scalar length,
               Scalar upper_radius, std::vector<size_t> &result, Budget &budget) const
    {
        budget.consume();
        const Node &node = m_nodes[index];
        if (disjoint(node, a, b, direction, length, upper_radius)) return;
        if (node.end - node.begin > 8) {
            query(node.left, a, b, direction, length, upper_radius, result, budget);
            query(node.right, a, b, direction, length, upper_radius, result, budget);
        } else for (size_t i = node.begin; i < node.end; ++i) {
            if (result.size() == result.capacity()) {
                // One query-wide scratch buffer is reused by every upper
                // segment. Admit retained storage only when capacity grows;
                // clearing its entries does not release that admission.
                const size_t old_capacity = result.capacity();
                const size_t requested = std::min(m_order.size(), std::max(size_t(8), 2 * old_capacity));
                budget.allocate(requested - old_capacity);
                result.reserve(requested);
                if (result.capacity() > requested)
                    budget.allocate(result.capacity() - requested);
            }
            budget.consume(); // Every candidate push remains charged on reuse.
            result.push_back(m_order[i]);
        }
    }

public:
    void prepare(const std::vector<LowerSection> &sections, Budget &budget)
    {
        if (sections.size() <= 64) return;
        budget.allocate(sections.size());
        m_order.resize(sections.size());
        std::iota(m_order.begin(), m_order.end(), size_t(0));
        build(sections, 0, sections.size(), budget);
    }

    bool empty() const { return m_nodes.empty(); }

    void candidates(const Point &a, const Point &b, V direction, Scalar length,
                    Scalar upper_radius, std::vector<size_t> &result, Budget &budget) const
    {
        result.clear();
        query(0, a, b, direction, length, upper_radius, result, budget);
        std::sort(result.begin(), result.end(), [&](size_t a, size_t b) {
            budget.consume();
            return a < b;
        });
    }
};

bool point_less(const Point &a, const Point &b)
{
    return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
}

// A real shared butt seam has no air gap. Merge exact collinear intervals
// before rounding so the numerical reserve cannot manufacture a gap there.
// Nonzero gaps, even below the reserve, are never merged.
void merge_collinear_sections(std::vector<LowerSection> &sections, Budget &budget)
{
    for (auto &section : sections)
        if (point_less(section.b, section.a))
            std::swap(section.a, section.b);
    budget.consume(sections.size() * size_t(2 + std::ceil(std::log2(std::max(size_t(1), sections.size())))));
    std::sort(sections.begin(), sections.end(), [](const LowerSection &a, const LowerSection &b) {
        const auto first = a.line_key(), second = b.line_key();
        return first < second || (first == second && point_less(a.a, b.a));
    });
    size_t count = 0;
    for (auto &section : sections) {
        if (count > 0 && sections[count - 1].line_key() == section.line_key() &&
            !point_less(sections[count - 1].b, section.a)) {
            if (point_less(sections[count - 1].b, section.b))
                sections[count - 1].b = section.b;
        } else {
            if (&sections[count] != &section)
                sections[count] = std::move(section);
            ++count;
        }
    }
    sections.resize(count);
}

std::vector<V> rectangle_corners(const Point &a, const Point &b, Scalar radius)
{
    const V delta = V(b) - V(a);
    const Scalar length = std::sqrt(dot(delta, delta));
    const V transverse(-delta.y / length * radius, delta.x / length * radius);
    return {V(a) + transverse, V(a) - transverse, V(b) - transverse, V(b) + transverse};
}

// A necessary contact condition: both finite axes must reach each other's
// normal strip. Their actual cross-sections fit inside radius-sum strips, so
// separation proves that neither a mask nor a finite contact line can meet the
// upper segment. Keep the existing four-unit reach reserve, and retain every
// numerically ambiguous pair. This never changes the physical cell material.
bool separated_normal(const Point &origin, V direction, Scalar length,
                      const Point &a, const Point &b, Scalar reach, Budget &budget)
{
    budget.consume();
    const Scalar first = Scalar(icross(direction, V(a) - V(origin)));
    const Scalar second = Scalar(icross(direction, V(b) - V(origin)));
    const Scalar limit = reach * length;
    const Scalar error = 64 * std::numeric_limits<Scalar>::epsilon() *
                         (std::abs(first) + std::abs(second) + std::abs(limit) + 1);
    return std::min(first, second) > limit + error || std::max(first, second) < -limit - error;
}

// Clip a line parameter s to lo <= value + slope*s <= hi.
bool slab(Scalar value, Scalar slope, Scalar lo, Scalar hi, Scalar &begin, Scalar &end)
{
    if (slope == 0)
        return value >= lo && value <= hi;
    Scalar first = (lo - value) / slope, last = (hi - value) / slope;
    if (first > last) std::swap(first, last);
    begin = std::max(begin, first);
    end = std::min(end, last);
    return begin < end;
}

// Keep the finite witness calculation independent of output storage so an
// optional whole-path probe can inspect exactly these intervals without
// allocating speculative certificates or contact masks.
template<class Emit>
void visit_line_contacts(const Point &upper_a, const Point &upper_b, const LowerSection &lower,
                         Scalar upper_radius, Scalar reserve, bool require_rounding_reserve,
                         Emit &&emit)
{
    const V u = V(upper_b) - V(upper_a), v = V(lower.b) - V(lower.a);
    const V relative = V(lower.a) - V(upper_a);
    const int64_t uu = idot(u, u), uv = idot(u, v), ur = idot(u, relative);
    const Scalar length_u = std::sqrt(Scalar(uu)), length_v = std::sqrt(dot(v, v));
    const int64_t cross_r = icross(u, relative), cross_v = icross(u, v);
    if (upper_radius == 0) {
        // Two tangent circular axes have a finite common line only when
        // collinear. Their ordinary crossing is one physical point.
        if (require_rounding_reserve || cross_r != 0 || cross_v != 0)
            return;
        int64_t a = ur, b = ur + uv;
        if (a > b) std::swap(a, b);
        a = std::max(int64_t(0), a);
        b = std::min(uu, b);
        if (a < b && Scalar(b - a) / length_u > 2 * reserve) {
            emit(ExactInterval{a, uu, b, uu});
        }
        return;
    }
    const Scalar radius = upper_radius - reserve;
    if (radius <= 0)
        return;
    const auto add_transverse_anchor = [&] {
        if (!require_rounding_reserve || cross_v == 0 || upper_radius <= 3 * reserve)
            return;
        int64_t denominator = cross_v;
        int64_t numerator = icross(relative, v), lower_numerator = -cross_r;
        if (denominator < 0) {
            denominator = -denominator;
            numerator = -numerator;
            lower_numerator = -lower_numerator;
        }
        if (numerator <= 0 || numerator >= denominator || lower_numerator <= 0 || lower_numerator >= denominator)
            return;
        // Each emitted endpoint may move by reserve. The change in each unit
        // direction is bounded by 4*reserve/length. This lower bound on the
        // crossing sine then bounds intersection movement by 2*reserve/sine.
        // Reserving that movement at BOTH original segments, plus three more
        // reserves, retains a finite lower-line witness inside the actual
        // upper flat after endpoint and angle perturbations.
        const Scalar sine = std::abs(Scalar(cross_v)) / (length_u * length_v);
        const Scalar sine_lower = sine - 4 * reserve / length_u - 4 * reserve / length_v;
        if (sine_lower <= 0)
            return;
        const Scalar margin = 2 * reserve / sine_lower + 3 * reserve;
        const Scalar t = Scalar(numerator) / denominator;
        const Scalar s = Scalar(lower_numerator) / denominator;
        if (std::min(t, 1 - t) * length_u <= margin || std::min(s, 1 - s) * length_v <= margin)
            return;
        emit(ExactInterval{numerator, denominator, numerator, denominator,
                           double(2 * reserve / sine_lower * SCALING_FACTOR)});
    };
    Scalar begin = 0, end = 1;
    if (!slab(Scalar(cross_r) / length_u, Scalar(cross_v) / length_u,
              -radius, radius, begin, end))
        return;
    if (uv == 0) {
        // The lower line lies along one upper transverse section. Its finite
        // physical contact length certifies this exact arclength event.
        if (require_rounding_reserve) {
            if (upper_radius <= 3 * reserve)
                return;
            begin = 0;
            end = 1;
            if (!slab(Scalar(cross_r) / length_u, Scalar(cross_v) / length_u,
                      -upper_radius + 3 * reserve, upper_radius - 3 * reserve, begin, end) ||
                (end - begin) * length_v <= 6 * reserve)
                return;
        }
        // The midpoint of the reserved lower witness stays inside both real
        // segments after endpoint perturbation. The upper direction can move
        // by 4*r/L, so include its effect on the transverse witness position.
        const Scalar longitudinal_margin = require_rounding_reserve ?
            (5 * reserve + 4 * reserve * upper_radius / length_u) * length_u : 0.;
        const bool within = require_rounding_reserve ?
            Scalar(ur) > longitudinal_margin && Scalar(uu - ur) > longitudinal_margin :
            ur >= 0 && ur <= uu;
        if (within && (end - begin) * length_v > 2 * reserve) {
            const double uncertainty = require_rounding_reserve ?
                double((2 * reserve + 4 * reserve * upper_radius / length_u) * SCALING_FACTOR) : 0.;
            emit(ExactInterval{ur, uu, ur, uu, uncertainty});
        }
        return;
    }
    if (require_rounding_reserve) {
        // Positive projected support is represented by an inward contact mask
        // in the caller. Keep uncertain witnesses as isolated anchors so their
        // location bounds are never mistaken for a guaranteed support span.
        add_transverse_anchor();
        return;
    }
    Scalar a = (Scalar(ur) + Scalar(uv) * begin) / Scalar(uu);
    Scalar b = (Scalar(ur) + Scalar(uv) * end) / Scalar(uu);
    if (a > b) std::swap(a, b);
    if (require_rounding_reserve) {
        // A nonzero projected contact interval must also survive independent
        // endpoint-coordinate rounding along the upper extrusion direction.
        a += reserve / length_u;
        b -= reserve / length_u;
    }
    // Clamp in upper parameter space. Reconstructing an exact endpoint by
    // subtracting large projected positions could create a spurious tiny tail.
    a = std::max(Scalar(0), a);
    b = std::min(Scalar(1), b);
    if (a >= b || (b - a) * Scalar(uu) / std::abs(Scalar(uv)) * length_v <= 2 * reserve) {
        // Near-perpendicular contact is still a finite physical line, even
        // when its longitudinal projection is smaller than the XY reserve.
        // Certify its exact centreline crossing with a perturbation bound.
        add_transverse_anchor();
        return;
    }
    // These bounds have a geometric reserve above. Round inward again when
    // representing irrational slab intersections as exact rational events.
    const int64_t first = std::max(int64_t(0), int64_t(std::ceil(a * interval_denominator)));
    const int64_t last = std::min(interval_denominator, int64_t(std::floor(b * interval_denominator)));
    if (first < last) {
        emit(ExactInterval{first, interval_denominator, last, interval_denominator});
    } else
        add_transverse_anchor();
}

void add_line_contact(const Point &upper_a, const Point &upper_b, const LowerSection &lower,
                      Scalar upper_radius, Scalar reserve, bool require_rounding_reserve,
                      SegmentContact &result, Budget &budget)
{
    visit_line_contacts(upper_a, upper_b, lower, upper_radius, reserve, require_rounding_reserve,
        [&](const ExactInterval &interval) {
            budget.allocate(1);
            result.line_contacts.push_back(interval);
        });
}
} // namespace

double section_width_mm(double width, double height, double print_z, double z)
{
    if (!dimensions(width, height, print_z) || !std::isfinite(z))
        return std::numeric_limits<double>::quiet_NaN();
    const double bottom = print_z - height;
    if (z < bottom || z > print_z)
        return 0.;
    return width - height + 2. * std::sqrt(std::max(0., (z - bottom) * (print_z - z)));
}

// No input path pointers survive preparation. The status object has no output
// geometry; unsuccessful factories discard every partially prepared section.
struct PreparedLower {
    double upper_print_z = 0.;
    float upper_height = 0.f;
    size_t deposited_paths = 0;
    size_t admission_vertices = 0;
    bool sections_prepared = false;
    PathContact status;
    std::vector<LowerSection> sections;
    SectionIndex index;
};

// Store values, never section indices or snapshot addresses. Canonical section
// indices may shift when unrelated lower material changes. The Scalar radius
// is compared as a number: long double may contain indeterminate padding.
struct QuerySignature {
    struct Section { Point a, b; Scalar radius; };
    struct Segment { size_t original_index; std::vector<Section> sections; };
    Points upper_points;
    float width = 0.f, height = 0.f;
    double mm3_per_mm = 0., upper_print_z = 0., tolerance = 0., contact_z = 0.;
    ExtrusionRole role = erNone;
    std::vector<Segment> segments;
    size_t units = 0;
};

size_t query_signature_units(const QuerySignature &signature) noexcept { return signature.units; }

struct QueryCacheData {
    using Key = std::tuple<size_t, Scalar, Scalar, double>;
    struct Entry { ExPolygons polygons; size_t units; };
    struct Less {
        Budget **active;
        bool operator()(const Key &a, const Key &b) const
        {
            (*active)->consume();
            return a < b;
        }
    };
    PreparedLowerPtr prepared;
    size_t capacity = 0;
    QueryCacheStatistics stats;
    Budget *active_budget = nullptr;
    std::map<Key, Entry, Less> entries;
    QueryCacheData(PreparedLowerPtr source, size_t limit)
        : prepared(std::move(source)), entries(Less{&active_budget})
    {
        if (!prepared) throw std::invalid_argument("A contact query cache requires a prepared lower snapshot.");
        capacity = std::min(limit, max_vertices - prepared->admission_vertices);
    }
};

QueryCache::QueryCache(PreparedLowerPtr prepared, size_t capacity_units)
    : m_data(std::make_unique<QueryCacheData>(std::move(prepared), capacity_units)) {}
QueryCache::~QueryCache() = default;
QueryCacheStatistics QueryCache::statistics() const { return m_data->stats; }

namespace {
class CacheScope {
    Budget &m_budget;
    QueryCacheData *m_cache;
    Budget *m_previous = nullptr;
public:
    CacheScope(Budget &budget, QueryCacheData *cache) : m_budget(budget), m_cache(cache)
    {
        if (cache != nullptr) {
            m_previous = cache->active_budget;
            cache->active_budget = &budget;
            budget.cache = cache;
        }
    }
    ~CacheScope()
    {
        if (m_cache != nullptr) {
            m_cache->active_budget = m_previous;
            m_budget.cache = nullptr;
        }
    }
};

void evict_first(QueryCacheData &cache, Budget &budget)
{
    budget.consume();
    const auto first = cache.entries.begin();
    cache.stats.resident_units -= first->second.units;
    cache.entries.erase(first);
    ++cache.stats.evictions;
}

void ensure_query_capacity(QueryCacheData *cache, size_t requested, Budget &budget)
{
    // Cache occupancy is optional. Never reject an otherwise admissible query
    // merely because masks retained by earlier queries occupy the allowance.
    while (!cache->entries.empty() &&
           (requested > max_vertices - budget.vertices ||
            cache->stats.resident_units > max_vertices - budget.vertices - requested))
        evict_first(*cache, budget);
}

bool cache_matches(PathContact &result, const PreparedLower &prepared, QueryCacheData *cache)
{
    if (cache == nullptr || cache->prepared.get() == &prepared) return true;
    result.input_valid = false;
    result.failure_reason = "Contact query cache belongs to a different prepared lower snapshot.";
    return false;
}

size_t mask_vertices(const ExPolygons &mask)
{
    size_t count = 0;
    for (const auto &polygon : mask) {
        count += polygon.contour.points.size();
        for (const auto &hole : polygon.holes) count += hole.points.size();
    }
    return count;
}

ExPolygons contact_mask(size_t lower_index, const LowerSection &lower, V transverse,
                        double mask_reserve, Budget &budget)
{
    if (!std::isfinite(transverse.x) || !std::isfinite(transverse.y) || !std::isfinite(mask_reserve))
        throw Limit{LimitKind::Coordinates};
    QueryCacheData *cache = budget.cache;
    const QueryCacheData::Key key{lower_index, transverse.x, transverse.y, mask_reserve};
    if (cache != nullptr) {
        budget.consume();
        const auto found = cache->entries.find(key);
        if (found != cache->entries.end()) {
            ++cache->stats.hits;
            const size_t count = found->second.units - 1;
            if (count <= max_vertices - budget.vertices - cache->stats.resident_units) {
                // This preflight proves the allocation hook cannot evict the
                // borrowed entry. Charge exactly one output copy before it.
                budget.allocate(count);
                return found->second.polygons;
            }
            // Preserve ownership before an allocation hook can erase entries.
            budget.consume();
            ExPolygons mask = std::move(found->second.polygons);
            cache->stats.resident_units -= found->second.units;
            cache->entries.erase(found);
            ++cache->stats.evictions;
            budget.allocate(count);
            return mask;
        }
        ++cache->stats.misses;
    }
    std::vector<V> corners;
    corners.reserve(8);
    if (lower.radius == 0) {
        corners = {V(lower.a) + transverse, V(lower.a) - transverse,
                   V(lower.b) + transverse, V(lower.b) - transverse};
    } else for (V corner : lower.corners) {
        corners.push_back(corner + transverse);
        corners.push_back(corner - transverse);
    }
    ExPolygons mask = inner_hull(corners, mask_reserve, budget);
    if (cache != nullptr) {
        const size_t units = mask_vertices(mask) + 1;
        if (units <= cache->capacity && units <= max_vertices - budget.vertices) {
            while (!cache->entries.empty() &&
                   (cache->stats.resident_units > cache->capacity - units ||
                    cache->stats.resident_units > max_vertices - budget.vertices - units))
                evict_first(*cache, budget);
            // Admission covers the temporary copy and its eventual map entry.
            // A throwing copy/comparison publishes no partial cached mask.
            budget.consume(units);
            QueryCacheData::Entry entry{mask, units};
            const auto inserted = cache->entries.emplace(key, std::move(entry));
            if (inserted.second) cache->stats.resident_units += units;
        }
    }
    return mask;
}
void describe_limit(PathContact &result, const Limit &limit, const Budget &budget,
                    const PreparedLower &prepared)
{
    result.completed = false;
    result.resource_limited = true;
    switch (limit.kind) {
    case LimitKind::AggregateWork:
        result.failure_reason = "Bridge aware cubic contact generation exceeded its aggregate geometry work limit; verification is incomplete. " +
            budget.aggregate->describe_work(budget.category) + " Current contact call charged=" + std::to_string(budget.work) +
            ", upper_segments=" + std::to_string(result.segments.size()) +
            ", deposited_paths=" + std::to_string(prepared.deposited_paths) + ", prepared_lower_sections=" +
            (prepared.sections_prepared ? std::to_string(prepared.sections.size()) : "not_prepared") + ".";
        break;
    case LimitKind::LocalWork:
        result.failure_reason = "Bead contact geometry exceeded its local work limit (used " + std::to_string(limit.used) +
            ", requested " + std::to_string(limit.requested) + ", limit " + std::to_string(max_work) + "); verification is incomplete.";
        break;
    case LimitKind::Vertices:
        result.failure_reason = "Bead contact geometry exceeded its vertex allocation limit (used " + std::to_string(limit.used) +
            ", requested " + std::to_string(limit.requested) + ", limit " + std::to_string(max_vertices) + "); verification is incomplete.";
        break;
    case LimitKind::Coordinates:
        result.failure_reason = "Bead contact geometry exceeds its finite coordinate bound; verification is incomplete.";
        break;
    }

}

bool query_arguments(PathContact &result, double tolerance, FootprintMode mode)
{
    if (!std::isfinite(tolerance) || tolerance < 0. ||
        tolerance >= double(coordinate_limit) * SCALING_FACTOR ||
        (mode != FootprintMode::Full && mode != FootprintMode::ContactsOnly)) {
        result.input_valid = false;
        result.failure_reason = "Contact tolerance, footprint mode or deposited path count is invalid.";
        return false;
    }
    return true;
}

bool upper_valid(PathContact &result, const ExtrusionPath &upper, double top, Budget &budget)
{
    const char *reason = nullptr;
    if (valid_path(upper, top, budget, false, nullptr, &reason))
        return true;
    result.input_valid = false;
    result.failure_reason = std::string("Upper bead ") + reason;
    return false;
}

void prepare_impl(PreparedLower &prepared, const std::vector<Bead> &deposited, Budget &budget)
{
    PathContact &result = prepared.status;
    const double upper_print_z = prepared.upper_print_z;
    const float upper_height = prepared.upper_height;
    if (deposited.size() > max_vertices || !dimensions(upper_height, upper_height, upper_print_z)) {
        result.input_valid = false;
        result.failure_reason = "Contact preparation height, elevation or deposited path count is invalid.";
        return;
    }
    const size_t initial_vertices = budget.vertices;
    double lower_top = -std::numeric_limits<double>::infinity();
    std::vector<bool> usable(deposited.size(), false);
    for (size_t i = 0; i < deposited.size(); ++i) {
        const Bead &bead = deposited[i];
        bool positive_xy = false;
        const char *reason = "path is absent.";
        if (bead.path == nullptr || !valid_path(*bead.path, bead.print_z, budget, true, &positive_xy, &reason)) {
            result.input_valid = false;
            result.failure_reason = std::string("A deposited bead ") + reason;
            return;
        }
        // Validate even future, out-of-plane and unmodeled deposition first.
        usable[i] = positive_xy && bead.path->width >= bead.path->height;
        if (usable[i] && bead.print_z < upper_print_z)
            lower_top = std::max(lower_top, bead.print_z);
    }
    // Keep the validated input admission baseline as well as owned geometry.
    // It is not charged again as work, but cached queries must obey the same
    // combined lower-input/upper-input/output limit as the uncached wrapper.
    prepared.admission_vertices = budget.vertices - initial_vertices;
    const double upper_bottom = upper_print_z - upper_height;
    result.contact_z = upper_bottom;
    if (!std::isfinite(lower_top) || upper_bottom > lower_top + z_roundoff(upper_height, upper_print_z))
        return;
    if (std::abs(upper_bottom - lower_top) <= z_roundoff(upper_height, upper_print_z))
        result.contact_z = lower_top;
    else {
        double shallowest = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < deposited.size(); ++i)
            if (usable[i] && deposited[i].print_z == lower_top)
                shallowest = std::min(shallowest, double(deposited[i].path->height));
        result.contact_z = lower_top - .5 * std::min(lower_top - upper_bottom, shallowest);
    }
    budget.category = BridgeAwareCubicValidation::WorkCategory::ContactSections;
    for (size_t i = 0; i < deposited.size(); ++i) {
        const Bead &bead = deposited[i];
        if (!usable[i] || bead.print_z >= upper_print_z ||
            result.contact_z < bead.print_z - bead.path->height - z_roundoff(bead.path->height, bead.print_z) ||
            result.contact_z > bead.print_z + z_roundoff(bead.path->height, bead.print_z))
            continue;
        const Scalar radius = .5L * snapped_section(*bead.path, bead.print_z, result.contact_z) / SCALING_FACTOR;
        const Points &points = bead.path->polyline.points;
        for (size_t j = 1; j < points.size(); ++j) {
            budget.consume();
            if (points[j] == points[j - 1]) continue;
            budget.allocate(2); // Both owned axis vertices.
            prepared.sections.push_back({points[j - 1], points[j], radius > 3. ? radius : 0., {}});
        }
    }
    merge_collinear_sections(prepared.sections, budget);
    prepared.sections_prepared = true;
    // Retain the existing cumulative admission, including merged axis records,
    // and explicitly account for every owned corner in cached queries.
    for (LowerSection &section : prepared.sections)
        if (section.radius > 0) {
            budget.allocate(4);
            section.corners = rectangle_corners(section.a, section.b, section.radius);
        }
    prepared.index.prepare(prepared.sections, budget);
    prepared.admission_vertices = budget.vertices - initial_vertices;
}

void physical_impl(PathContact &result, const PreparedLower &prepared, Budget &budget)
{
    budget.category = BridgeAwareCubicValidation::WorkCategory::ContactSections;
    for (const LowerSection &section : prepared.sections) {
        if (section.radius > 0) {
            ExPolygons material = inner_hull(section.corners, 0., budget);
            for (ExPolygon &polygon : material) {
                // Both original axis endpoints belong to the real butt
                // rectangle. Their hull with an inner convex section is
                // still inside that rectangle, and preserves the actual
                // common vertex at near-collinear path joins. This only
                // repairs physical wall topology; contact masks below
                // retain their complete anchoring/rounding reserve.
                Points points = polygon.contour.points;
                points.push_back(section.a);
                points.push_back(section.b);
                budget.allocate(2);
                polygon.contour = Geometry::convex_hull(std::move(points));
            }
            result.physical_footprint.insert(result.physical_footprint.end(),
                std::make_move_iterator(material.begin()), std::make_move_iterator(material.end()));
        } else {
            // Its axis is still genuine material. Keeping an axis loses
            // tiny area conservatively instead of inflating it.
            budget.allocate(2);
            result.physical_lines.emplace_back(Points{section.a, section.b});
        }
    }
}

bool prove_entire_path(PathContact &result, const PreparedLower &prepared, const ExtrusionPath &upper,
                       double upper_print_z, double xy_tolerance_mm, Budget &budget)
{
    if (xy_tolerance_mm != 0. || prepared.sections.empty()) return false;
    const Scalar upper_radius = .5L * snapped_section(upper, upper_print_z, prepared.status.contact_z) / SCALING_FACTOR;
    const auto axis_key = [](const LowerSection &section) {
        const auto key = section.line_key();
        return std::make_tuple(std::get<0>(key), std::get<1>(key), std::get<2>(key));
    };
    const Points &points = upper.polyline.points;
    size_t positive_segments = 0;
    budget.category = BridgeAwareCubicValidation::WorkCategory::ContactCandidates;
    for (size_t i = 1; i < points.size(); ++i) {
        budget.consume();
        if (points[i - 1] == points[i]) continue;
        ++positive_segments;
        LowerSection axis{points[i - 1], points[i], 0., {}};
        if (point_less(axis.b, axis.a)) std::swap(axis.a, axis.b);
        const auto key = axis_key(axis);
        auto candidate = std::lower_bound(prepared.sections.begin(), prepared.sections.end(), key,
            [&](const LowerSection &section, const auto &wanted) {
                budget.consume();
                return axis_key(section) < wanted;
            });
        bool complete = false;
        // This optional search has bounded overhead even with many radii or
        // disjoint intervals on one axis. A miss preserves exhaustive checking.
        for (size_t examined = 0; candidate != prepared.sections.end() && examined < 16; ++candidate, ++examined) {
            budget.consume();
            if (axis_key(*candidate) != key) break;
            if (point_less(axis.a, candidate->a) || point_less(candidate->b, axis.b)) continue;
            visit_line_contacts(points[i - 1], points[i], *candidate, upper_radius, 3., false,
                [&](const ExactInterval &interval) {
                    budget.consume();
                    complete |= interval.begin_numerator == 0 &&
                        interval.end_numerator == interval.end_denominator &&
                        interval.position_uncertainty_mm == 0.;
                });
            if (complete) break;
        }
        if (!complete) return false;
    }
    if (positive_segments == 0) return false;
    // Nothing was allocated by the probe. Publish certificates only after all
    // original positive segments passed; failed output admission stays fatal.
    result.contact_z = prepared.status.contact_z;
    result.coordinate_uncertainty_mm = 0.;
    for (size_t i = 1; i < points.size(); ++i)
        if (points[i - 1] != points[i]) {
            budget.allocate(2); // One segment record and one exact interval.
            result.segments.push_back({i - 1, {}, {{0, 1, 1, 1, 0.}}});
        }
    return true;
}

// The only candidate enumeration used by ordinary contact generation and its
// optional query identity. Both retain canonical merged-section order and the
// exact original broad/narrow rejection predicates. A visitor may stop after
// an identity mismatch; ordinary queries always visit every surviving pair.
template<class Visit, class Poll>
bool visit_query_sections(const PreparedLower &prepared, const Point &a, const Point &b,
                          Scalar upper_radius, std::vector<size_t> &candidates,
                          Budget &budget, Visit &&visit, Poll &&poll)
{
    const V u = V(b) - V(a);
    const Scalar length = std::sqrt(dot(u, u));
    const bool indexed = !prepared.index.empty();
    budget.category = BridgeAwareCubicValidation::WorkCategory::ContactCandidates;
    poll();
    if (indexed)
        prepared.index.candidates(a, b, u, length, upper_radius, candidates, budget);
    poll();
    const size_t lower_count = indexed ? candidates.size() : prepared.sections.size();
    for (size_t i = 0; i < lower_count; ++i) {
        if ((i & 255) == 0) poll();
        const size_t lower_index = indexed ? candidates[i] : i;
        const LowerSection &lower = prepared.sections[lower_index];
        budget.category = BridgeAwareCubicValidation::WorkCategory::ContactCandidates;
        budget.consume();
        const Scalar reach = lower.radius + upper_radius + 4;
        if (std::max(a.x(), b.x()) + reach < std::min(lower.a.x(), lower.b.x()) ||
            std::min(a.x(), b.x()) - reach > std::max(lower.a.x(), lower.b.x()) ||
            std::max(a.y(), b.y()) + reach < std::min(lower.a.y(), lower.b.y()) ||
            std::min(a.y(), b.y()) - reach > std::max(lower.a.y(), lower.b.y()))
            continue;
        const V lower_delta = V(lower.b) - V(lower.a);
        const Scalar lower_length = std::sqrt(dot(lower_delta, lower_delta));
        if (separated_normal(a, u, length, lower.a, lower.b, reach, budget) ||
            separated_normal(lower.a, lower_delta, lower_length, a, b, reach, budget))
            continue;
        if (!visit(lower_index, lower, length, lower_length)) return false;
    }
    return true;
}

void query_impl(PathContact &result, const PreparedLower &prepared, const ExtrusionPath &upper,
                double upper_print_z, double xy_tolerance_mm, FootprintMode mode, Budget &budget)
{
    if (!prepared.status.input_valid || !prepared.status.completed) {
        result = prepared.status;
        return;
    }
    if (upper_print_z != prepared.upper_print_z || upper.height != prepared.upper_height) {
        result.input_valid = false;
        result.failure_reason = "Upper bead elevation or height does not match the prepared lower sections.";
        return;
    }
    result.contact_z = prepared.status.contact_z;
    result.coordinate_uncertainty_mm = xy_tolerance_mm;
    const Points &upper_points = upper.polyline.points;
    for (size_t i = 1; i < upper_points.size(); ++i)
        if (upper_points[i] != upper_points[i - 1]) {
            budget.allocate(1);
            result.segments.push_back({i - 1, {}, {}});
        }
    if (prepared.sections.empty())
        return;
    const Scalar upper_radius = .5L * snapped_section(upper, upper_print_z, result.contact_z) / SCALING_FACTOR;
    const Scalar reserve = xy_tolerance_mm / SCALING_FACTOR + 3.;
    if (mode == FootprintMode::Full) physical_impl(result, prepared, budget);
    std::vector<size_t> candidates;
    for (SegmentContact &segment : result.segments) {
        const Point &a = upper_points[segment.segment_index], &b = upper_points[segment.segment_index + 1];
        const V u = V(b) - V(a);
        const Scalar length = std::sqrt(dot(u, u));
        const V transverse(-u.y / length * upper_radius, u.x / length * upper_radius);
        visit_query_sections(prepared, a, b, upper_radius, candidates, budget,
          [&](size_t lower_index, const LowerSection &lower, Scalar, Scalar lower_length) {
            budget.category = BridgeAwareCubicValidation::WorkCategory::ContactMasks;
            // Both centreline endpoints can move, and both section
            // directions can rotate. Erode the convex contact region by
            // the corresponding support-function displacement bound.
            const double mask_reserve = xy_tolerance_mm *
                double(2 + 4 * upper_radius / length + 4 * lower.radius / lower_length);
            ExPolygons mask = contact_mask(lower_index, lower, transverse, mask_reserve, budget);
            segment.centerline_support.insert(segment.centerline_support.end(),
                std::make_move_iterator(mask.begin()), std::make_move_iterator(mask.end()));
            // An axis remains genuine material when a very thin positive
            // section loses its polygon mask to the rounding reserve.
            add_line_contact(a, b, lower, upper_radius, reserve, xy_tolerance_mm > 0., segment, budget);
            return true;
          }, [] {});
    }

}

struct SignatureUnavailable {};
struct SignatureCanceled { std::exception_ptr exception; };

// An optional acceleration failure must not swallow the caller's cancellation
// exception, even if its type is also used for allocation/geometry failures.
void signature_poll(const QuerySignatureCancel &cancel)
{
    if (cancel) {
        try { cancel(); }
        catch (...) { throw SignatureCanceled{std::current_exception()}; }
    }
}

bool signature_arguments(const PreparedLower &prepared, const ExtrusionPath &upper,
                         double upper_print_z, double tolerance, Budget &budget,
                         QueryCacheData *cache)
{
    PathContact status;
    if (!query_arguments(status, tolerance, FootprintMode::ContactsOnly) ||
        !cache_matches(status, prepared, cache) ||
        !prepared.status.input_valid || !prepared.status.completed ||
        prepared.status.resource_limited || !prepared.status.failure_reason.empty())
        return false;
    // Run the ordinary guards freshly on every lookup. Nonempty fitting data
    // is conservatively cache-ineligible even if its entries describe lines.
    return upper_valid(status, upper, upper_print_z, budget) &&
           upper.polyline.fitting_result.empty() &&
           upper_print_z == prepared.upper_print_z && upper.height == prepared.upper_height;
}

void admit_signature(size_t count, size_t capacity, size_t &units, Budget &budget)
{
    if (count > capacity - units) throw SignatureUnavailable{};
    budget.allocate(count);
    units += count;
}

template<class T>
void grow_signature(std::vector<T> &items, size_t item_units, size_t capacity,
                    size_t &units, Budget &budget)
{
    if (items.size() != items.capacity()) return;
    const size_t available = (capacity - units) / item_units;
    if (available == 0) throw SignatureUnavailable{};
    const size_t old_capacity = items.capacity();
    const size_t growth = std::min(available, std::max(size_t(1), old_capacity));
    admit_signature(growth * item_units, capacity, units, budget);
    items.reserve(old_capacity + growth);
    if (items.capacity() > old_capacity + growth)
        admit_signature((items.capacity() - old_capacity - growth) * item_units, capacity, units, budget);
}

bool signature_upper_matches(const QuerySignature &signature, const PreparedLower &prepared,
                             const ExtrusionPath &upper, double upper_print_z, double tolerance,
                             Budget &budget, const QuerySignatureCancel &cancel)
{
    budget.consume(16); // Exact scalar/key and container metadata comparison.
    if (signature.width != upper.width || signature.height != upper.height ||
        signature.mm3_per_mm != upper.mm3_per_mm || signature.role != upper.role() ||
        signature.upper_print_z != upper_print_z || signature.tolerance != tolerance ||
        signature.contact_z != prepared.status.contact_z ||
        signature.upper_points.size() != upper.polyline.points.size())
        return false;
    for (size_t i = 0; i < signature.upper_points.size(); ++i) {
        if ((i & 255) == 0) signature_poll(cancel);
        budget.consume();
        if (signature.upper_points[i] != upper.polyline.points[i]) return false;
    }
    return true;
}
} // namespace

QuerySignaturePtr capture_query_signature(const PreparedLower &prepared, const ExtrusionPath &upper,
                                           double upper_print_z, double tolerance,
                                           BridgeAwareCubicValidation::WorkBudget *shared,
                                           size_t capacity_units, const QuerySignatureCancel &cancel,
                                           QueryCache *cache)
{
    Budget budget(shared);
    budget.vertices = prepared.admission_vertices;
    QueryCacheData *cache_data = cache == nullptr ? nullptr : cache->m_data.get();
    CacheScope cache_scope(budget, cache_data);
    try {
        signature_poll(cancel);
        if (!signature_arguments(prepared, upper, upper_print_z, tolerance, budget, cache_data)) return {};
        signature_poll(cancel);
        const size_t capacity = std::min(capacity_units, max_vertices);
        size_t units = 0;
        admit_signature(16, capacity, units, budget); // All owned metadata/containers.
        auto signature = std::make_shared<QuerySignature>();
        const Points &points = upper.polyline.points;
        admit_signature(points.size(), capacity, units, budget);
        signature->upper_points = points;
        if (signature->upper_points.capacity() > points.size())
            admit_signature(signature->upper_points.capacity() - points.size(), capacity, units, budget);
        signature->width = upper.width;
        signature->height = upper.height;
        signature->mm3_per_mm = upper.mm3_per_mm;
        signature->role = upper.role();
        signature->upper_print_z = upper_print_z;
        signature->tolerance = tolerance;
        signature->contact_z = prepared.status.contact_z;
        const Scalar radius = .5L * snapped_section(upper, upper_print_z, signature->contact_z) / SCALING_FACTOR;
        std::vector<size_t> candidates;
        const auto poll = [&] { signature_poll(cancel); };
        for (size_t i = 1; i < points.size(); ++i) {
            poll();
            budget.category = BridgeAwareCubicValidation::WorkCategory::ContactCandidates;
            budget.consume();
            if (points[i - 1] == points[i]) continue;
            // Admission includes spare vector capacity, not just used records.
            grow_signature(signature->segments, 2, capacity, units, budget);
            signature->segments.push_back({i - 1, {}});
            auto &segment = signature->segments.back();
            visit_query_sections(prepared, points[i - 1], points[i], radius, candidates, budget,
                [&](size_t, const LowerSection &lower, Scalar, Scalar) {
                    grow_signature(segment.sections, 3, capacity, units, budget);
                    segment.sections.push_back({lower.a, lower.b, lower.radius});
                    return true;
                }, poll);
        }
        signature_poll(cancel);
        signature->units = units;
        return signature;
    } catch (const SignatureCanceled &canceled) {
        std::rethrow_exception(canceled.exception);
    } catch (const SignatureUnavailable &) {
        return {};
    } catch (const Limit &) {
        return {};
    } catch (const std::exception &) {
        return {};
    }
}

bool matches_query_signature(const QuerySignature &signature, const PreparedLower &prepared,
                             const ExtrusionPath &upper, double upper_print_z, double tolerance,
                             BridgeAwareCubicValidation::WorkBudget *shared,
                             const QuerySignatureCancel &cancel, QueryCache *cache)
{
    Budget budget(shared);
    budget.vertices = prepared.admission_vertices;
    QueryCacheData *cache_data = cache == nullptr ? nullptr : cache->m_data.get();
    CacheScope cache_scope(budget, cache_data);
    try {
        signature_poll(cancel);
        if (!signature_arguments(prepared, upper, upper_print_z, tolerance, budget, cache_data) ||
            !signature_upper_matches(signature, prepared, upper, upper_print_z, tolerance, budget, cancel))
            return false;
        const Points &points = upper.polyline.points;
        const Scalar radius = .5L * snapped_section(upper, upper_print_z, prepared.status.contact_z) / SCALING_FACTOR;
        std::vector<size_t> candidates;
        size_t segment_index = 0;
        const auto poll = [&] { signature_poll(cancel); };
        for (size_t i = 1; i < points.size(); ++i) {
            poll();
            budget.category = BridgeAwareCubicValidation::WorkCategory::ContactCandidates;
            budget.consume();
            if (points[i - 1] == points[i]) continue;
            if (segment_index == signature.segments.size()) return false;
            const auto &segment = signature.segments[segment_index++];
            budget.consume();
            if (segment.original_index != i - 1) return false;
            size_t section_index = 0;
            if (!visit_query_sections(prepared, points[i - 1], points[i], radius, candidates, budget,
                [&](size_t, const LowerSection &lower, Scalar, Scalar) {
                    budget.consume(3);
                    if (section_index == segment.sections.size()) return false;
                    const auto &expected = segment.sections[section_index++];
                    return expected.a == lower.a && expected.b == lower.b && expected.radius == lower.radius;
                }, poll) || section_index != segment.sections.size())
                return false;
        }
        signature_poll(cancel);
        return segment_index == signature.segments.size();
    } catch (const SignatureCanceled &canceled) {
        std::rethrow_exception(canceled.exception);
    } catch (const Limit &) {
        return false;
    } catch (const std::exception &) {
        return false;
    }
}

PreparedLowerPtr prepare_lower(const std::vector<Bead> &deposited, double upper_print_z,
                               float upper_height, BridgeAwareCubicValidation::WorkBudget *shared)
{
    auto prepared = std::make_shared<PreparedLower>();
    prepared->upper_print_z = upper_print_z;
    prepared->upper_height = upper_height;
    prepared->deposited_paths = deposited.size();
    Budget budget(shared);
    try {
        prepare_impl(*prepared, deposited, budget);
    } catch (const Limit &limit) {
        describe_limit(prepared->status, limit, budget, *prepared);
    } catch (const std::exception &error) {
        prepared->status.completed = false;
        prepared->status.failure_reason = std::string("Bead contact geometry failed: ") + error.what();
    }
    if (!prepared->status.input_valid || !prepared->status.completed) {
        std::vector<LowerSection>().swap(prepared->sections);
        prepared->index = SectionIndex{};
        prepared->admission_vertices = 0;
    }
    return prepared;
}

namespace {
SupportQuery query_prepared(const PreparedLower &prepared, const ExtrusionPath &upper,
                            double upper_print_z, double xy_tolerance_mm, FootprintMode mode,
                            BridgeAwareCubicValidation::WorkBudget *shared, QueryCacheData *cache_data,
                            bool try_entire_path)
{
    SupportQuery query;
    PathContact &result = query.contact;
    Budget budget(shared);
    budget.vertices = prepared.admission_vertices;
    CacheScope cache_scope(budget, cache_data);
    try {
        if (!query_arguments(result, xy_tolerance_mm, mode))
            return query;
        if (!cache_matches(result, prepared, cache_data)) return query;
        // A failed snapshot supplies no usable evidence. Preserve its precise
        // cause even when preparation exhausted the shared work allowance.
        if (!prepared.status.input_valid || !prepared.status.completed) {
            result = prepared.status;
            return query;
        }
        if (!upper_valid(result, upper, upper_print_z, budget))
            return query;
        // A mismatched key must reach the ordinary error below, before any
        // witness can use this snapshot's common physical plane.
        if (try_entire_path && upper_print_z == prepared.upper_print_z && upper.height == prepared.upper_height &&
            prove_entire_path(result, prepared, upper, upper_print_z, xy_tolerance_mm, budget)) {
            query.entire_path_supported = true;
            return query;
        }
        budget.category = BridgeAwareCubicValidation::WorkCategory::ContactInput;
        query_impl(result, prepared, upper, upper_print_z, xy_tolerance_mm, mode, budget);
    } catch (const Limit &limit) {
        describe_limit(result, limit, budget, prepared);
    } catch (const std::exception &error) {
        result.completed = false;
        result.failure_reason = std::string("Bead contact geometry failed: ") + error.what();
    }
    return query;
}
} // namespace

PathContact footprint(const PreparedLower &prepared, const ExtrusionPath &upper,
                      double upper_print_z, double xy_tolerance_mm, FootprintMode mode,
                      BridgeAwareCubicValidation::WorkBudget *shared, QueryCache *cache)
{
    return query_prepared(prepared, upper, upper_print_z, xy_tolerance_mm, mode, shared,
                          cache == nullptr ? nullptr : cache->m_data.get(), false).contact;
}

SupportQuery footprint_for_support(const PreparedLower &prepared, const ExtrusionPath &upper,
                                   double upper_print_z, double xy_tolerance_mm,
                                   BridgeAwareCubicValidation::WorkBudget *shared, QueryCache *cache)
{
    return query_prepared(prepared, upper, upper_print_z, xy_tolerance_mm, FootprintMode::ContactsOnly,
                          shared, cache == nullptr ? nullptr : cache->m_data.get(), true);
}

PathContact physical_sections(const PreparedLower &prepared,
                              BridgeAwareCubicValidation::WorkBudget *shared, QueryCache *cache)
{
    PathContact result;
    Budget budget(shared);
    budget.vertices = prepared.admission_vertices;
    QueryCacheData *cache_data = cache == nullptr ? nullptr : cache->m_data.get();
    CacheScope cache_scope(budget, cache_data);
    try {
        if (!cache_matches(result, prepared, cache_data)) return result;
        if (!prepared.status.input_valid || !prepared.status.completed) return prepared.status;
        result.contact_z = prepared.status.contact_z;
        physical_impl(result, prepared, budget);
    } catch (const Limit &limit) {
        describe_limit(result, limit, budget, prepared);
    } catch (const std::exception &error) {
        result.completed = false;
        result.failure_reason = std::string("Bead contact geometry failed: ") + error.what();
    }
    if (!result.input_valid || !result.completed) {
        result.physical_footprint.clear();
        result.physical_lines.clear();
    }
    return result;
}

PathContact footprint(const std::vector<Bead> &deposited, const ExtrusionPath &upper,
                      double upper_print_z, double xy_tolerance_mm, FootprintMode mode,
                      BridgeAwareCubicValidation::WorkBudget *shared)
{
    // Keep the uncached entry point's upper-first validation and use exactly
    // the same preparation/query operations and shared work charges.
    PathContact result;
    PreparedLower prepared;
    prepared.upper_print_z = upper_print_z;
    prepared.upper_height = upper.height;
    prepared.deposited_paths = deposited.size();
    Budget budget(shared);
    try {
        if (!query_arguments(result, xy_tolerance_mm, mode))
            return result;
        if (deposited.size() > max_vertices) {
            result.input_valid = false;
            result.failure_reason = "Contact tolerance, footprint mode or deposited path count is invalid.";
            return result;
        }
        if (!upper_valid(result, upper, upper_print_z, budget))
            return result;
        prepare_impl(prepared, deposited, budget);
        query_impl(result, prepared, upper, upper_print_z, xy_tolerance_mm, mode, budget);
    } catch (const Limit &limit) {
        describe_limit(result, limit, budget, prepared);
    } catch (const std::exception &error) {
        result.completed = false;
        result.failure_reason = std::string("Bead contact geometry failed: ") + error.what();
    }
    return result;
}

}} // namespace Slic3r::BridgeAwareCubicContact
