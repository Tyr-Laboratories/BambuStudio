#ifndef slic3r_BridgeAwareCubicContainment_hpp_
#define slic3r_BridgeAwareCubicContainment_hpp_

#include "../Polyline.hpp"
#include "BridgeAwareCubicValidation.hpp"

#include <boost/multiprecision/cpp_int.hpp>

#include <algorithm>
#include <cstdint>
#include <functional>
#include <limits>
#include <new>
#include <vector>

namespace Slic3r { namespace BridgeAwareCubic {

// Optional success-only classification of one complete path against the
// original, nonzero-winding mask rings. False means use the original clipping
// predicate. No result here is a certificate of deposited-material support.
class StrictMaskContainment {
public:
    explicit StrictMaskContainment(const Polygons &rings,
        BridgeAwareCubicValidation::WorkBudget *budget = nullptr,
        const std::function<void()> &cancel = {})
    {
        if (cancel) cancel();
        try {
            Work work(budget, cancel);
            work.charge();
            if (rings.empty() || rings.size() > max_items) return;
            size_t count = 0;
            for (const Polygon &ring : rings) {
                work.charge();
                if (ring.size() < 3 || ring.size() > max_vertices / 2 - count) return;
                count += ring.size();
                for (const Point &point : ring.points) {
                    work.charge();
                    if (!safe(point)) return;
                }
            }
            // Each edge owns two points. No polygon-mask copy is retained.
            m_edges.reserve(count);
            for (const Polygon &ring : rings)
                for (size_t i = 0; i < ring.size(); ++i) {
                    work.charge();
                    m_edges.emplace_back(ring.points[i], ring.points[(i + 1) % ring.size()]);
                }
            m_order.resize(count);
            for (size_t i = 0; i < count; ++i) {
                work.charge();
                m_order[i] = i;
            }
            m_nodes.reserve(std::min(max_items, 2 * count));
            build(0, count, work);
            m_available = true;
        } catch (const Unavailable &) {
            release();
        } catch (const std::bad_alloc &) {
            release();
        }
    }

    bool available() const noexcept { return m_available; }

    bool proves(const Polyline &path, BridgeAwareCubicValidation::WorkBudget *budget = nullptr,
                const std::function<void()> &cancel = {}) const
    {
        if (cancel) cancel();
        if (!m_available || path.size() < 2 || path.size() > max_path_vertices || !path.fitting_result.empty()) return false;
        try {
            Work work(budget, cancel);
            bool positive = false;
            for (size_t i = 0; i < path.size(); ++i) {
                work.charge();
                if (!safe(path.points[i])) return false;
                positive |= i > 0 && path.points[i] != path.points[i - 1];
            }
            if (!positive || winding(0, path.points.front(), work) == 0) return false;
            for (size_t i = 1; i < path.size(); ++i)
                if (path.points[i] != path.points[i - 1] &&
                    near_boundary(0, Edge(path.points[i - 1], path.points[i]), work)) return false;
            return true;
        } catch (const Unavailable &) {
            return false;
        } catch (const std::bad_alloc &) {
            return false;
        }
    }

private:
    static constexpr size_t max_vertices = 200000, max_items = 20000, max_path_vertices = 4096;
    static constexpr int64_t coordinate_limit = std::numeric_limits<int32_t>::max() / 4;
    // At these coordinates Clipper's TopX arithmetic has less than 1e-6
    // floating error plus at most .5 unit rounding per edge. Strictly more
    // than four units keeps independently clipped open/closed edges apart.
    // This is an optional-classification guard, never a support tolerance.
    static constexpr int64_t guard = 4;
    // The path cap also bounds the two double length-sum errors in the caller's
    // complete_construction_match to less than sixteen grid units. The production
    // classifier uses 216 units; this shortcut must not replace a stricter
    // length-based predicate without its own rounding audit.
    using Wide = boost::multiprecision::int128_t;
    struct Unavailable {};

    class Work {
    public:
        Work(BridgeAwareCubicValidation::WorkBudget *budget, const std::function<void()> &cancel)
            : m_budget(budget), m_cancel(cancel) {}
        void charge()
        {
            if (m_remaining == 0 || (m_budget != nullptr && m_budget->remaining_work == 0)) throw Unavailable{};
            --m_remaining;
            if (m_budget != nullptr) {
                --m_budget->remaining_work;
                ++m_budget->charged_work[size_t(BridgeAwareCubicValidation::WorkCategory::Validation)];
            }
            if ((++m_spent & 255) == 0 && m_cancel) m_cancel();
        }
    private:
        size_t m_remaining = 50000000, m_spent = 0;
        BridgeAwareCubicValidation::WorkBudget *m_budget;
        const std::function<void()> &m_cancel;
    };

    struct Bounds {
        int64_t low_x, low_y, high_x, high_y;
        Bounds(const Point &a, const Point &b)
            : low_x(std::min(a.x(), b.x())), low_y(std::min(a.y(), b.y())),
              high_x(std::max(a.x(), b.x())), high_y(std::max(a.y(), b.y())) {}
        void merge(const Bounds &b)
        {
            low_x = std::min(low_x, b.low_x); low_y = std::min(low_y, b.low_y);
            high_x = std::max(high_x, b.high_x); high_y = std::max(high_y, b.high_y);
        }
        bool separated(const Bounds &b) const
        {
            return high_x + guard < b.low_x || b.high_x + guard < low_x ||
                high_y + guard < b.low_y || b.high_y + guard < low_y;
        }
    };
    struct Edge {
        Point a, b;
        Bounds bounds;
        Edge(const Point &first, const Point &last) : a(first), b(last), bounds(first, last) {}
    };
    struct Node {
        Bounds bounds;
        size_t begin, end, left = 0, right = 0;
        Node(Bounds bounds, size_t begin, size_t end) : bounds(bounds), begin(begin), end(end) {}
    };
    bool m_available = false;
    std::vector<Edge> m_edges;
    std::vector<size_t> m_order;
    std::vector<Node> m_nodes;

    static bool safe(const Point &p)
    {
        return p.x() >= -coordinate_limit && p.x() <= coordinate_limit &&
            p.y() >= -coordinate_limit && p.y() <= coordinate_limit;
    }
    void release() noexcept
    {
        m_available = false;
        std::vector<Edge>().swap(m_edges);
        std::vector<size_t>().swap(m_order);
        std::vector<Node>().swap(m_nodes);
    }
    size_t build(size_t begin, size_t end, Work &work)
    {
        Bounds bounds = m_edges[m_order[begin]].bounds;
        for (size_t i = begin; i < end; ++i) {
            work.charge();
            bounds.merge(m_edges[m_order[i]].bounds);
        }
        if (m_nodes.size() >= max_items) throw Unavailable{};
        const size_t index = m_nodes.size();
        m_nodes.emplace_back(bounds, begin, end);
        if (end - begin > 16) {
            const bool x_axis = bounds.high_x - bounds.low_x >= bounds.high_y - bounds.low_y;
            const size_t middle = begin + (end - begin) / 2;
            std::nth_element(m_order.begin() + begin, m_order.begin() + middle, m_order.begin() + end,
                [&](size_t a, size_t b) {
                    work.charge();
                    const Bounds &left = m_edges[a].bounds, &right = m_edges[b].bounds;
                    const int64_t ca = x_axis ? left.low_x + left.high_x : left.low_y + left.high_y;
                    const int64_t cb = x_axis ? right.low_x + right.high_x : right.low_y + right.high_y;
                    return ca < cb || (ca == cb && a < b);
                });
            const size_t left = build(begin, middle, work), right = build(middle, end, work);
            m_nodes[index].left = left;
            m_nodes[index].right = right;
        }
        return index;
    }
    static int64_t cross(const Point &a, const Point &b, const Point &p)
    {
        return (int64_t(b.x()) - a.x()) * (int64_t(p.y()) - a.y()) -
            (int64_t(b.y()) - a.y()) * (int64_t(p.x()) - a.x());
    }
    static bool on_segment(const Point &p, const Edge &edge)
    {
        return p.x() >= edge.bounds.low_x && p.x() <= edge.bounds.high_x &&
            p.y() >= edge.bounds.low_y && p.y() <= edge.bounds.high_y;
    }
    static bool opposite(int64_t a, int64_t b) { return (a < 0 && b > 0) || (a > 0 && b < 0); }
    static bool near_point(const Point &p, const Edge &edge)
    {
        const int64_t dx = int64_t(edge.b.x()) - edge.a.x(), dy = int64_t(edge.b.y()) - edge.a.y();
        const int64_t px = int64_t(p.x()) - edge.a.x(), py = int64_t(p.y()) - edge.a.y();
        const int64_t squared = dx * dx + dy * dy, projection = px * dx + py * dy;
        if (projection <= 0) return px * px + py * py <= guard * guard;
        if (projection >= squared) {
            const int64_t qx = int64_t(p.x()) - edge.b.x(), qy = int64_t(p.y()) - edge.b.y();
            return qx * qx + qy * qy <= guard * guard;
        }
        const int64_t determinant = dx * py - dy * px;
        // Differences fit 31 bits, determinants fit 61; their squared values
        // fit signed 128 bits. No floating projection can certify clearance.
        return Wide(determinant) * determinant <= Wide(guard * guard) * squared;
    }
    static bool near(const Edge &a, const Edge &b)
    {
        const int64_t ab0 = cross(a.a, a.b, b.a), ab1 = cross(a.a, a.b, b.b);
        const int64_t ba0 = cross(b.a, b.b, a.a), ba1 = cross(b.a, b.b, a.b);
        if ((opposite(ab0, ab1) && opposite(ba0, ba1)) ||
            (ab0 == 0 && on_segment(b.a, a)) || (ab1 == 0 && on_segment(b.b, a)) ||
            (ba0 == 0 && on_segment(a.a, b)) || (ba1 == 0 && on_segment(a.b, b))) return true;
        return near_point(a.a, b) || near_point(a.b, b) || near_point(b.a, a) || near_point(b.b, a);
    }
    int64_t winding(size_t index, const Point &point, Work &work) const
    {
        work.charge();
        const Node &node = m_nodes[index];
        if (point.y() < node.bounds.low_y || point.y() >= node.bounds.high_y || point.x() > node.bounds.high_x) return 0;
        if (node.left != 0) return winding(node.left, point, work) + winding(node.right, point, work);
        int64_t result = 0;
        for (size_t i = node.begin; i < node.end; ++i) {
            work.charge();
            const Edge &edge = m_edges[m_order[i]];
            if (edge.a.y() <= point.y()) {
                if (edge.b.y() > point.y() && cross(edge.a, edge.b, point) > 0) ++result;
            } else if (edge.b.y() <= point.y() && cross(edge.a, edge.b, point) < 0) --result;
        }
        return result;
    }
    bool near_boundary(size_t index, const Edge &candidate, Work &work) const
    {
        work.charge();
        const Node &node = m_nodes[index];
        if (candidate.bounds.separated(node.bounds)) return false;
        if (node.left != 0) return near_boundary(node.left, candidate, work) || near_boundary(node.right, candidate, work);
        for (size_t i = node.begin; i < node.end; ++i) {
            work.charge();
            const Edge &edge = m_edges[m_order[i]];
            if (!candidate.bounds.separated(edge.bounds) && near(candidate, edge)) return true;
        }
        return false;
    }
};

}} // namespace Slic3r::BridgeAwareCubic

#endif
