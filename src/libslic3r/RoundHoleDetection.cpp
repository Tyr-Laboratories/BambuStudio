#include "RoundHoleDetection.hpp"
#include "TriangleMesh.hpp"
#include "Geometry/Circle.hpp"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <optional>
#include <unordered_set>

namespace Slic3r::WallPainting {
namespace {
void append_round_hole_caps(const indexed_triangle_set &mesh, const Transform3d &transform,
    std::vector<RoundHole> &holes, double normal_z_limit, const std::function<void(float)> &progress);
}

std::vector<RoundHole> detect_round_internal_hole_overhangs(
    const indexed_triangle_set &mesh, const Transform3d &transform, double angle_deg,
    const std::function<void(float)> &progress)
{
    auto report = [&](float value) { if (progress) progress(value); };
    report(0.f);
    if (!std::isfinite(angle_deg) || angle_deg <= 0.) {
        report(1.f);
        return {};
    }
    auto holes = detect_round_internal_holes(mesh, transform, [&](float p) { report(0.8f * p); });
    if (holes.empty()) {
        report(1.f);
        return holes;
    }

    const Matrix3d normal_matrix = transform.linear().inverse().transpose();
    const double limit = -std::cos(std::min(angle_deg, 90.) * M_PI / 180.);
    append_round_hole_caps(mesh, transform, holes, limit, [&](float p) { report(0.8f + 0.15f * p); });
    size_t total = 0, completed = 0;
    for (const auto &hole : holes)
        total += hole.facets.size();
    for (auto &hole : holes) {
        auto end = std::remove_if(hole.facets.begin(), hole.facets.end(), [&](int facet) {
            const auto &face = mesh.indices[facet];
            const Vec3d a = mesh.vertices[face[0]].cast<double>();
            const Vec3d normal = normal_matrix *
                (mesh.vertices[face[1]].cast<double>() - a).cross(mesh.vertices[face[2]].cast<double>() - a);
            if ((++completed & 1023) == 0)
                report(float(0.95 + 0.05 * completed / total));
            // Reject vertical walls even when round-off leaves a tiny negative
            // Z component at the 90-degree limit.
            return !normal.allFinite() || normal.norm() == 0. || normal.normalized().z() >= limit - 1e-8;
        });
        hole.facets.erase(end, hole.facets.end());
    }
    holes.erase(std::remove_if(holes.begin(), holes.end(), [](const RoundHole &hole) {
        return hole.facets.empty();
    }), holes.end());
    report(1.f);
    return holes;
}

namespace {
constexpr double max_turn_cos = 0.8660254037844386; // 30 degrees
constexpr double axis_growth_tolerance = 0.03;
constexpr double axis_fit_tolerance = 0.015;

struct Edge {
    uint64_t key;
    int halfedge;
    bool operator<(const Edge &other) const { return key < other.key; }
};

// A cap is accepted only when a connected inward-facing planar patch is fully
// contained by the bore and its complete boundary is exactly one bore rim.
// This is support-only: ordinary wall painting still targets cylindrical walls.
void append_round_hole_caps(const indexed_triangle_set &mesh, const Transform3d &transform,
    std::vector<RoundHole> &holes, double normal_z_limit, const std::function<void(float)> &progress)
{
    if (std::none_of(holes.begin(), holes.end(), [&](const RoundHole &hole) {
        return -std::abs(hole.axis.z()) < normal_z_limit - 1e-8;
    })) {
        progress(1.f);
        return;
    }
    auto edge_key = [](int a, int b) {
        return (uint64_t(uint32_t(std::min(a, b))) << 32) | uint32_t(std::max(a, b));
    };
    std::vector<Edge> edges;
    edges.reserve(3 * mesh.indices.size());
    for (size_t i = 0; i < mesh.indices.size(); ++i) {
        const auto &face = mesh.indices[i];
        if ((face.array() >= 0).all() && (face.array() < int(mesh.vertices.size())).all())
            for (int j = 0; j < 3; ++j)
                edges.push_back({edge_key(face[j], face[(j + 1) % 3]), int(3 * i + j)});
        if ((i & 4095) == 0)
            progress(float(0.4 * i / mesh.indices.size()));
    }
    std::sort(edges.begin(), edges.end());
    progress(0.4f);
    auto neighbor = [&](uint64_t key, int face) {
        const auto first = std::lower_bound(edges.begin(), edges.end(), Edge{key, 0});
        if (first == edges.end() || first->key != key || first + 1 == edges.end() || (first + 1)->key != key ||
            (first + 2 != edges.end() && (first + 2)->key == key))
            return -1;
        const int a = first->halfedge, b = (first + 1)->halfedge;
        if (mesh.indices[a / 3][a % 3] == mesh.indices[b / 3][b % 3])
            return -1; // Nonmanifold or inconsistent winding cannot enclose a cap.
        return a / 3 == face ? b / 3 : (b / 3 == face ? a / 3 : -1);
    };
    const Matrix3d normal_matrix = transform.linear().inverse().transpose();
    for (size_t i = 0; i < holes.size(); ++i) {
        RoundHole &hole = holes[i];
        auto check_cancel = [&] { progress(float(0.4 + 0.6 * i / holes.size())); };
        check_cancel();
        if (-std::abs(hole.axis.z()) >= normal_z_limit - 1e-8)
            continue;
        const auto bore_facets = hole.facets;
        std::vector<Edge> bore_edges;
        double lower = std::numeric_limits<double>::max(), upper = -lower;
        for (int facet : bore_facets) {
            const auto &face = mesh.indices[facet];
            for (int j = 0; j < 3; ++j) {
                bore_edges.push_back({edge_key(face[j], face[(j + 1) % 3]), 3 * facet + j});
                const double z = (transform * mesh.vertices[face[j]].cast<double>() - hole.center).dot(hole.axis);
                lower = std::min(lower, z); upper = std::max(upper, z);
            }
            if ((bore_edges.size() & 4095) < 3) check_cancel();
        }
        std::sort(bore_edges.begin(), bore_edges.end());
        const double tolerance = std::max(0.0025, hole.radius * 0.0015);
        for (int side : {0, 1}) {
            const double end_position = side == 0 ? lower : upper;
            const Vec3d inward = side == 0 ? hole.axis : -hole.axis;
            if (inward.z() >= normal_z_limit - 1e-8) continue;
            std::vector<uint64_t> rim;
            int seed = -1;
            for (size_t j = 0; j < bore_edges.size();) {
                size_t end = j + 1;
                while (end < bore_edges.size() && bore_edges[end].key == bore_edges[j].key) ++end;
                if (end == j + 1) {
                    const int halfedge = bore_edges[j].halfedge;
                    const auto &face = mesh.indices[halfedge / 3];
                    const Vec3d a = transform * mesh.vertices[face[halfedge % 3]].cast<double>() - hole.center;
                    const Vec3d b = transform * mesh.vertices[face[(halfedge + 1) % 3]].cast<double>() - hole.center;
                    if (std::abs(a.dot(hole.axis) - end_position) <= tolerance &&
                        std::abs(b.dot(hole.axis) - end_position) <= tolerance) {
                        rim.push_back(bore_edges[j].key);
                        seed = neighbor(bore_edges[j].key, halfedge / 3);
                    }
                }
                j = end;
                if ((j & 4095) < 3) check_cancel();
            }
            if (rim.empty() || seed < 0) continue;
            auto contained = [&](int facet) {
                const auto &face = mesh.indices[facet];
                const Vec3d a = mesh.vertices[face[0]].cast<double>();
                const Vec3d normal = normal_matrix *
                    (mesh.vertices[face[1]].cast<double>() - a).cross(mesh.vertices[face[2]].cast<double>() - a);
                if (!normal.allFinite() || normal.norm() == 0. || normal.normalized().dot(inward) < 1. - 1e-6)
                    return false;
                for (int vertex : {face[0], face[1], face[2]}) {
                    const Vec3d point = transform * mesh.vertices[vertex].cast<double>() - hole.center;
                    const double z = point.dot(hole.axis);
                    if (std::abs(z - end_position) > tolerance || (point - z * hole.axis).norm() > hole.radius + tolerance)
                        return false;
                }
                return true;
            };
            std::vector<int> cap {seed};
            std::unordered_set<int> visited {seed};
            std::vector<uint64_t> boundary;
            bool enclosed = true;
            for (size_t head = 0; head < cap.size() && enclosed; ++head) {
                if ((head & 1023) == 0) check_cancel();
                const int facet = cap[head];
                if (!contained(facet)) { enclosed = false; break; }
                const auto &face = mesh.indices[facet];
                for (int j = 0; j < 3; ++j) {
                    const uint64_t key = edge_key(face[j], face[(j + 1) % 3]);
                    const int other = neighbor(key, facet);
                    if (other < 0) { enclosed = false; break; }
                    if (std::binary_search(rim.begin(), rim.end(), key)) {
                        if (!std::binary_search(bore_facets.begin(), bore_facets.end(), other)) {
                            enclosed = false; break;
                        }
                        boundary.push_back(key);
                    } else if (visited.insert(other).second) {
                        cap.push_back(other);
                    }
                }
            }
            std::sort(boundary.begin(), boundary.end());
            if (enclosed && boundary == rim)
                hole.facets.insert(hole.facets.end(), cap.begin(), cap.end());
        }
        std::sort(hole.facets.begin(), hole.facets.end());
    }
    progress(1.f);
}

// Angular coordinates can be lifted continuously on an open cylindrical strip,
// but not on a patch which wraps all the way around its axis. A contradictory
// lift on a graph cycle therefore proves full circumference, including bores
// whose oblique rims leave no complete horizontal cross-section.
bool wraps_axis(const indexed_triangle_set &mesh, const std::vector<int> &facets,
                const std::vector<int> &vertex_ids, const Vec2ds &points,
                const Vec2d &center, const std::function<void()> &check_cancel)
{
    const int count = int(vertex_ids.size());
    std::vector<int> parent(count), rank(count, 0);
    std::vector<double> lift(count, 0.), angle(count);
    std::iota(parent.begin(), parent.end(), 0);
    for (int i = 0; i < count; ++i)
        angle[i] = std::atan2(points[i].y() - center.y(), points[i].x() - center.x());
    auto root = [&](int v) {
        int r = v;
        double sum = 0.;
        while (parent[r] != r) { sum += lift[r]; r = parent[r]; }
        while (parent[v] != v) {
            const int next = parent[v];
            const double step = lift[v];
            parent[v] = r; lift[v] = sum; sum -= step; v = next;
        }
        return r;
    };
    size_t steps = 0;
    for (int face : facets) {
        if ((steps++ & 4095) == 0) check_cancel();
        const auto &triangle = mesh.indices[face];
        for (int edge = 0; edge < 3; ++edge) {
            int a = int(std::lower_bound(vertex_ids.begin(), vertex_ids.end(), triangle[edge]) - vertex_ids.begin());
            int b = int(std::lower_bound(vertex_ids.begin(), vertex_ids.end(), triangle[(edge + 1) % 3]) - vertex_ids.begin());
            const double delta = std::remainder(angle[b] - angle[a], 2. * M_PI);
            const int ra = root(a), rb = root(b);
            if (ra == rb) {
                if (std::abs(lift[b] - lift[a] - delta) > M_PI)
                    return true;
            } else if (rank[ra] < rank[rb]) {
                parent[ra] = rb;
                lift[ra] = lift[b] - lift[a] - delta;
            } else {
                parent[rb] = ra;
                lift[rb] = delta + lift[a] - lift[b];
                if (rank[ra] == rank[rb]) ++rank[ra];
            }
        }
    }
    return false;
}

std::optional<RoundHole> fit_hole(const indexed_triangle_set &mesh,
    const std::vector<Vec3d> &vertices, const std::vector<Vec3f> &normals,
    const std::vector<Vec3i> &neighbors, std::vector<int> facets,
    double precision, const Vec3d &origin, const std::function<void()> &check_cancel)
{
    if (facets.size() < 12)
        return {};
    std::sort(facets.begin(), facets.end());
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    std::vector<int> vertex_ids;
    vertex_ids.reserve(3 * facets.size());
    size_t steps = 0;
    for (int face : facets) {
        if ((steps++ & 4095) == 0) check_cancel();
        // Ambiguous, open, or reversed mesh edges are not a reliable bore.
        if ((neighbors[face].array() < 0).any()) return {};
        const auto &tri = mesh.indices[face];
        const double area = (vertices[tri[1]] - vertices[tri[0]]).cross(vertices[tri[2]] - vertices[tri[0]]).norm();
        const Vec3d normal = normals[face].cast<double>();
        covariance += area * normal * normal.transpose();
        for (int v : {tri[0], tri[1], tri[2]}) vertex_ids.push_back(v);
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(covariance);
    if (eigen.info() != Eigen::Success || eigen.eigenvalues()[1] <= 1e-14)
        return {};
    Vec3d axis = eigen.eigenvectors().col(0).normalized();
    // Deterministic orientation for callers and tests; inwardness does not
    // depend on this otherwise arbitrary sign.
    Eigen::Index largest;
    axis.cwiseAbs().maxCoeff(&largest);
    if (axis[largest] < 0.) axis = -axis;
    const Vec3d u = axis.unitOrthogonal(), v = axis.cross(u);
    std::sort(vertex_ids.begin(), vertex_ids.end());
    check_cancel();
    vertex_ids.erase(std::unique(vertex_ids.begin(), vertex_ids.end()), vertex_ids.end());
    Vec2ds points;
    points.reserve(vertex_ids.size());
    const Vec3d reference = vertices[vertex_ids.front()];
    double min_z = std::numeric_limits<double>::max(), max_z = -min_z;
    for (int id : vertex_ids) {
        if ((steps++ & 4095) == 0) check_cancel();
        const Vec3d p = vertices[id] - reference;
        points.emplace_back(p.dot(u), p.dot(v));
        const double z = p.dot(axis);
        min_z = std::min(min_z, z); max_z = std::max(max_z, z);
    }
    const auto circle = Geometry::circle_taubin_newton(points);
    if (!circle.center.allFinite() || !std::isfinite(circle.radius) || circle.radius <= 0.)
        return {};
    // Source coordinates are floats. Keep their quantization allowance even
    // after a large translation or nonuniform transform, but reject geometry
    // too imprecise to distinguish a circle reliably.
    if (precision > circle.radius * 0.02)
        return {};
    const double radial_tolerance = std::max({0.0025, circle.radius * 0.0015, precision * 4.});
    if (max_z - min_z <= std::max(0.0001, precision * 2.))
        return {};
    for (const Vec2d &p : points)
        if (std::abs((p - circle.center).norm() - circle.radius) > radial_tolerance)
            return {};

    const Vec3d center = reference + circle.center.x() * u + circle.center.y() * v + 0.5 * (min_z + max_z) * axis;
    std::vector<double> normal_angles;
    normal_angles.reserve(facets.size());
    for (int face : facets) {
        if ((steps++ & 4095) == 0) check_cancel();
        const Vec3d normal = normals[face].cast<double>();
        if (std::abs(normal.dot(axis)) > axis_fit_tolerance)
            return {};
        const auto &tri = mesh.indices[face];
        Vec3d radial = (vertices[tri[0]] + vertices[tri[1]] + vertices[tri[2]]) / 3. - center;
        radial -= axis * radial.dot(axis);
        if (radial.squaredNorm() <= 0. || normal.dot(radial.normalized()) > -0.96)
            return {};
        normal_angles.push_back(std::atan2(normal.dot(v), normal.dot(u)));
    }
    std::sort(normal_angles.begin(), normal_angles.end());
    double max_gap = normal_angles.front() + 2. * M_PI - normal_angles.back();
    for (size_t i = 1; i < normal_angles.size(); ++i)
        max_gap = std::max(max_gap, normal_angles[i] - normal_angles[i - 1]);

    // A latitude mesh of a sphere can contain a perfectly cylindrical narrow
    // equatorial belt. Reject that belt when *both* of its ends continue only
    // into gentle concave axial curvature. A flat mouth, sharp step, or an
    // outward-opening blend keeps even a very shallow real bore eligible.
    if (max_z - min_z < 0.5 * circle.radius) {
        bool lower_end = false, upper_end = false, curved_belt = true;
        for (int face : facets) {
            if ((steps++ & 4095) == 0) check_cancel();
            const Vec3d normal = normals[face].cast<double>();
            const auto &tri = mesh.indices[face];
            for (int edge = 0; edge < 3; ++edge) {
                const int other = neighbors[face][edge];
                if (std::binary_search(facets.begin(), facets.end(), other)) continue;
                const Vec3d other_normal = normals[other].cast<double>();
                const auto &other_tri = mesh.indices[other];
                const Vec3d other_center = (vertices[other_tri[0]] + vertices[other_tri[1]] + vertices[other_tri[2]]) / 3.;
                const Vec3d edge_center = 0.5 * (vertices[tri[edge]] + vertices[tri[(edge + 1) % 3]]);
                if (normal.dot(other_normal) < max_turn_cos ||
                    std::abs(other_normal.dot(axis)) <= axis_growth_tolerance ||
                    normal.dot(other_center - edge_center) <= std::max(1e-10, precision * 0.25)) {
                    curved_belt = false; break;
                }
                const double end_position = (edge_center - center).dot(axis);
                lower_end |= end_position < -0.25 * (max_z - min_z);
                upper_end |= end_position >  0.25 * (max_z - min_z);
            }
            if (!curved_belt) break;
        }
        if (curved_belt && lower_end && upper_end) return {};
    }
    // Excludes polygonal bores and patches whose normal coverage has a missing
    // sector. The topology check below also excludes narrow open slots.
    if (max_gap > M_PI / 6. + 0.002 || !wraps_axis(mesh, facets, vertex_ids, points, circle.center, check_cancel))
        return {};
    return RoundHole{std::move(facets), axis, center + origin, circle.radius};
}
} // namespace

std::vector<RoundHole> detect_round_internal_holes(const indexed_triangle_set &mesh,
    const Transform3d &transform, const std::function<void(float)> &progress)
{
    auto report = [&](float value) { if (progress) progress(value); };
    report(0.f);
    std::vector<RoundHole> holes;
    const size_t count = mesh.indices.size();
    if (count == 0 || mesh.vertices.empty() || !transform.matrix().allFinite() ||
        std::abs(transform.linear().determinant()) < 1e-15) {
        report(1.f); return holes;
    }
    const Vec3d local_origin = mesh.vertices.front().cast<double>();
    const Vec3d origin = transform * local_origin;
    std::vector<Vec3d> vertices;
    vertices.reserve(mesh.vertices.size());
    double max_coordinate = 0.;
    for (size_t i = 0; i < mesh.vertices.size(); ++i) {
        const Vec3d p = mesh.vertices[i].cast<double>();
        vertices.push_back(transform.linear() * (p - local_origin));
        max_coordinate = std::max(max_coordinate, p.cwiseAbs().maxCoeff());
        if ((i & 4095) == 0) report(float(0.1 * i / mesh.vertices.size()));
    }
    const double precision = max_coordinate * double(std::numeric_limits<float>::epsilon()) * transform.linear().norm();
    const double orientation = transform.linear().determinant() < 0. ? -1. : 1.;
    std::vector<Vec3f> normals(count, Vec3f::Zero());
    std::vector<char> valid(count, 0);
    std::vector<Edge> edges;
    edges.reserve(3 * count);
    for (size_t i = 0; i < count; ++i) {
        const auto &tri = mesh.indices[i];
        bool good = true;
        for (int j = 0; j < 3; ++j)
            good &= tri[j] >= 0 && size_t(tri[j]) < vertices.size();
        if (good) {
            Vec3d normal = (vertices[tri[1]] - vertices[tri[0]]).cross(vertices[tri[2]] - vertices[tri[0]]);
            const double length = normal.norm();
            if (normal.allFinite() && length > 1e-16) {
                normals[i] = (orientation * normal / length).cast<float>();
                valid[i] = 1;
            }
            for (int j = 0; j < 3; ++j) {
                const uint32_t a = uint32_t(std::min(tri[j], tri[(j + 1) % 3]));
                const uint32_t b = uint32_t(std::max(tri[j], tri[(j + 1) % 3]));
                edges.push_back({(uint64_t(a) << 32) | b, int(3 * i + j)});
            }
        }
        if ((i & 4095) == 0) report(float(0.1 + 0.15 * i / count));
    }
    report(0.25f);
    std::sort(edges.begin(), edges.end());
    report(0.3f);
    std::vector<Vec3i> neighbors(count, Vec3i::Constant(-1));
    for (size_t i = 0; i < edges.size();) {
        size_t end = i + 1;
        while (end < edges.size() && edges[end].key == edges[i].key) ++end;
        if (end - i == 2) {
            const int a = edges[i].halfedge, b = edges[i + 1].halfedge;
            // Two oppositely directed, valid incident faces make an ordinary
            // manifold edge. Leave all other edge incidences disconnected.
            if (valid[a / 3] && valid[b / 3] &&
                mesh.indices[a / 3][a % 3] != mesh.indices[b / 3][b % 3]) {
                neighbors[a / 3][a % 3] = b / 3;
                neighbors[b / 3][b % 3] = a / 3;
            }
        }
        i = end;
        if ((i & 4095) < 3) report(float(0.3 + 0.1 * i / edges.size()));
    }
    std::vector<Edge>().swap(edges);
    report(0.4f);
    std::vector<char> processed(count, 0);
    // Failed hypotheses do not consume facets: an edge at a shallow chamfer,
    // or an almost coplanar tessellation diagonal, can suggest the wrong axis.
    // Cache failed axes and permit alternative local edge hypotheses. After
    // three failed walks a face is no longer eligible to seed another walk;
    // it can still belong to a successful candidate started elsewhere. This
    // prevents repeated long walks over spheres and other noncylindrical CAD
    // surfaces without allowing a failed guess to cut a real bore in pieces.
    std::vector<unsigned char> failed_attempts(count, 0);
    std::vector<Vec3f> failed_axis(count, Vec3f::Zero());
    std::vector<unsigned> visited(count, 0);
    unsigned generation = 0;
    for (size_t seed = 0; seed < count; ++seed) {
        if ((seed & 2047) == 0) report(float(0.4 + 0.6 * seed / count));
        if (!valid[seed] || processed[seed] || failed_attempts[seed] >= 3) continue;
        const Vec3d seed_normal = normals[seed].cast<double>();
        std::array<int, 3> seed_edges{{0, 1, 2}};
        std::array<double, 3> turn_strength{{-1., -1., -1.}};
        for (int edge = 0; edge < 3; ++edge) {
            const int other = neighbors[seed][edge];
            if (other >= 0 && seed_normal.dot(normals[other].cast<double>()) >= max_turn_cos)
                turn_strength[edge] = seed_normal.cross(normals[other].cast<double>()).squaredNorm();
        }
        // A real circumferential edge is a better-conditioned axis estimate
        // than a nearly coplanar diagonal within the same tessellated panel.
        std::sort(seed_edges.begin(), seed_edges.end(), [&](int a, int b) { return turn_strength[a] > turn_strength[b]; });
        for (int seed_edge : seed_edges) {
            if (processed[seed] || failed_attempts[seed] >= 3) break;
            const int other = neighbors[seed][seed_edge];
            if (other < 0 || processed[other]) continue;
            const Vec3d other_normal = normals[other].cast<double>();
            const double dot = seed_normal.dot(other_normal);
            Vec3d axis = seed_normal.cross(other_normal);
            // Cross-product magnitude retains tiny real turns on densely
            // tessellated circles; subtracting a float dot product from one
            // would mistake those turns for coplanarity.
            if (dot < max_turn_cos || axis.norm() < 1e-6) continue;
            axis.normalize();
            if (std::abs(axis.dot(failed_axis[seed].cast<double>())) > 0.99999)
                continue;
            const auto &tri = mesh.indices[other];
            const Vec3d other_center = (vertices[tri[0]] + vertices[tri[1]] + vertices[tri[2]]) / 3.;
            if (seed_normal.dot(other_center - vertices[mesh.indices[seed][0]]) <= 0.) continue;
            std::vector<int> queue{int(seed)};
            ++generation;
            visited[seed] = generation;
            for (size_t head = 0; head < queue.size(); ++head) {
                const int face = queue[head];
                const Vec3d normal = normals[face].cast<double>();
                for (int e = 0; e < 3; ++e) {
                    const int other = neighbors[face][e];
                    if (other < 0 || visited[other] == generation || processed[other]) continue;
                    const Vec3d other_normal = normals[other].cast<double>();
                    if (std::abs(axis.dot(other_normal)) > axis_growth_tolerance || normal.dot(other_normal) < max_turn_cos)
                        continue;
                    const auto &tri = mesh.indices[other];
                    const Vec3d other_center = (vertices[tri[0]] + vertices[tri[1]] + vertices[tri[2]]) / 3.;
                    if (normal.dot(other_center - vertices[mesh.indices[face][0]]) < -std::max(precision * 2., 1e-7))
                        continue;
                    visited[other] = generation;
                    queue.push_back(other);
                }
                if ((head & 2047) == 0) report(float(0.4 + 0.6 * seed / count));
            }
            if (auto hole = fit_hole(mesh, vertices, normals, neighbors, queue, precision, origin,
                    [&] { report(float(0.4 + 0.6 * seed / count)); })) {
                for (int face : queue) processed[face] = 1;
                holes.push_back(std::move(*hole));
            } else {
                for (int face : queue) {
                    failed_attempts[face] = std::min<unsigned char>(3, failed_attempts[face] + 1);
                    failed_axis[face] = axis.cast<float>();
                }
            }
        }
    }
    report(1.f);
    return holes;
}
}
