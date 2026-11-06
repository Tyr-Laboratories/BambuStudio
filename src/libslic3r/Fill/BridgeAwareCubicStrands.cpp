#include "BridgeAwareCubicStrands.hpp"

#include <boost/multiprecision/cpp_int.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>

namespace Slic3r { namespace BridgeAwareCubic {
namespace {
constexpr double root3 = 1.7320508075688772935;
// The projection uses binary64 lattice coordinates. Decline indices at which
// integer or half-integer coefficients (including sums below) could alias.
constexpr int64_t index_limit = int64_t(1) << 48;

struct Work {
    StrandBudget &budget;
    const std::function<void()> &cancel;
    size_t until_poll = 256;
    Work(StrandBudget &b, const std::function<void()> &c) : budget(b), cancel(c) { poll(); }
    void poll() const { if (cancel) cancel(); }
    void charge(size_t count = 1) {
        if (count > budget.work_left) {
            budget.work_left = 0;
            throw std::length_error("Canonical cubic strand work limit");
        }
        budget.work_left -= count;
        if (count >= until_poll) { poll(); until_poll = 256; }
        else until_poll -= count;
    }
    void records(size_t &count, size_t n = 1) {
        if (n > budget.max_records || count > budget.max_records - n)
            throw std::length_error("Canonical cubic strand record limit");
        count += n; charge(n);
    }
    void lookup(size_t count) {
        // A red-black tree lookup compares at most twice its binary height.
        size_t cost = 2;
        while (count) { count >>= 1; cost += 2; }
        charge(cost);
    }
};

struct Phase { int64_t floor = 0; bool tied = false; bool ready = false; };
struct Binary { int64_t mantissa; int exponent; };
Binary binary(double x) {
    static_assert(std::numeric_limits<double>::radix == 2 && std::numeric_limits<double>::digits == 53,
                  "Canonical cubic strands require binary64 coordinates");
    int exponent = 0;
    const double fraction = std::frexp(x, &exponent);
    return {int64_t(std::ldexp(fraction, 53)), exponent - 53};
}

// The canonical physical plane uses the same rounded z*sqrt(3) as Plan's
// section calculation. Classify its two source grids exactly, once per used
// depth, rather than deciding a corner tie separately on each producer tile.
Phase phase_at(double height, double edge, Work &work) {
    if (!std::isfinite(height) || !(edge > 0.) || !std::isfinite(edge))
        throw std::overflow_error("Canonical cubic strand elevation or edge is not representable");
    using boost::multiprecision::cpp_int;
    const Binary h = binary(height), e = binary(edge);
    const int exponent = std::min(h.exponent, e.exponent);
    const size_t words = 4 + size_t(std::max(h.exponent, e.exponent) - exponent) / 32;
    work.charge(8 * words * words); // binary64 bounds this temporary to < 2200 bits.
    const cpp_int he = cpp_int(h.mantissa) * (cpp_int(1) << (h.exponent - exponent));
    const cpp_int ee = cpp_int(e.mantissa) << (e.exponent - exponent);
    const cpp_int numerator = 2 * he - ee, denominator = 2 * ee;
    cpp_int floor = numerator / denominator;
    const cpp_int remainder = numerator % denominator;
    if (remainder < 0) --floor; // C++ integer division truncates toward zero.
    if (floor < -index_limit || floor > index_limit)
        throw std::overflow_error("Canonical cubic strand plane index exceeds representable grid");
    return {floor.convert_to<int64_t>(), remainder == 0, true};
}

Point rounded_xy(const Vec3d &world) {
    const double x = world.x() / SCALING_FACTOR, y = world.y() / SCALING_FACTOR;
    constexpr double max_xy = double(std::numeric_limits<coord_t>::max()) - 1.;
    if (!std::isfinite(x) || !std::isfinite(y) || std::abs(x) > max_xy || std::abs(y) > max_xy)
        throw std::overflow_error("Canonical cubic strand exceeds the representable XY range");
    return Point(coord_t(std::llround(x)), coord_t(std::llround(y)));
}
}

struct StrandProgram::Impl {
    struct Edge { StrandEvent last; CellId owner; const Point *a, *b; };
    using Edges = std::map<StrandEvent, Edge>;
    struct Row {
        // Node addresses of this immutable point table survive insertions. A
        // breakpoint is transformed/rounded once, independent of active runs.
        std::map<StrandEvent, Point> vertices;
        Edges active, dormant;
    };
    struct Ref { StrandPlane plane; StrandEvent first; };
    struct Owner { std::array<Ref, 3> edges; size_t count = 0; bool active = true; };
    std::function<void()> cancel;
    std::map<StrandPlane, Row> rows;
    std::set<StrandPlane> live_rows, dormant_rows;
    std::map<CellId, Owner> owners;

    explicit Impl(std::function<void()> c) : cancel(std::move(c)) {}

    void build(double z, double bulk_edge, const Vec3d &origin, const Matrix3d &frame,
               const std::vector<Refinement> &refinements, StrandBudget &budget) {
        Work w(budget, cancel);
        size_t records = 0;
        std::array<Phase, 31> phases{}; // Plan checks max_depth <= 30.
        const double height = z * root3;
        for (const auto &ref : refinements) {
            w.charge();
            if (ref.solid) continue; // Solid masks/dense fallback are a separate emitter.
            const CellId &id = ref.id;
            if (id.depth >= phases.size() || std::any_of(id.address.begin(), id.address.end(),
                    [](int64_t i){ return i < -index_limit || i > index_limit; }))
                throw std::overflow_error("Canonical cubic strand cell address exceeds representable grid");
            const double edge = std::ldexp(bulk_edge, -int(id.depth));
            Phase &phase = phases[id.depth];
            if (!phase.ready) phase = phase_at(height, edge, w);
            const int64_t sum = id.address[0] + id.address[1] + id.address[2];
            const bool first_half = sum == phase.floor;
            if ((!first_half && sum != phase.floor - 1) || (first_half && phase.tied)) continue;
            w.lookup(owners.size()); w.records(records);
            auto inserted_owner = owners.try_emplace(id);
            if (!inserted_owner.second) throw std::logic_error("Duplicate canonical cubic strand owner");
            Owner &owner = inserted_owner.first->second;
            for (unsigned axis = 0; axis != 3; ++axis) {
                const unsigned a = (axis + 1) % 3, b = (axis + 2) % 3;
                const int64_t i = id.address[axis], j = id.address[a], n = phase.floor - i;
                const StrandPlane plane{axis, id.depth + 1, 2 * i + 1};
                // At this depth S/e=n+f. A(j)=j*e, B(j)=S−(n−j)*e.
                // Each tile supplies exactly one elementary interval: [A,B]
                // or [B,A_next]. At f=0 B(j) IS A(j), not a second vertex.
                const StrandEvent first{j, !first_half && !phase.tied};
                const StrandEvent last = first_half ? StrandEvent{j,true} : StrandEvent{j+1,false};
                w.lookup(rows.size());
                auto row_it = rows.find(plane);
                if (row_it == rows.end()) {
                    w.records(records, 2); w.lookup(rows.size()); w.lookup(live_rows.size());
                    row_it = rows.try_emplace(plane).first;
                    live_rows.insert(plane);
                }
                Row &row = row_it->second;
                const double fixed = std::ldexp(bulk_edge * double(plane.numerator), -int(plane.level));
                const double s = height - fixed;
                const auto vertex = [&](StrandEvent key) -> const Point * {
                    w.lookup(row.vertices.size());
                    auto found = row.vertices.find(key);
                    if (found != row.vertices.end()) return &found->second;
                    w.records(records); w.lookup(row.vertices.size());
                    Vec3d p; p[axis] = fixed;
                    if (!key.shifted) { p[a] = double(key.slot) * edge; p[b] = s - p[a]; }
                    else { p[b] = double(n - key.slot) * edge; p[a] = s - p[b]; }
                    return &row.vertices.emplace(key, rounded_xy(origin + frame * p)).first->second;
                };
                const Point *p0 = vertex(first), *p1 = vertex(last);
                w.records(records, 2); w.lookup(row.active.size());
                if (!row.active.emplace(first, Edge{last,id,p0,p1}).second)
                    throw std::logic_error("Distinct split cells claim the same positive canonical cubic atom");
                owner.edges[owner.count++] = {plane,first};
            }
        }
    }
};

StrandProgram::StrandProgram(double z, double bulk_edge, const Vec3d &origin, const Matrix3d &frame,
                             const std::vector<Refinement> &refinements, std::function<void()> cancel,
                             StrandBudget &budget) : impl(std::make_unique<Impl>(std::move(cancel)))
{ impl->build(z, bulk_edge, origin, frame, refinements, budget); }
StrandProgram::~StrandProgram() = default;
StrandProgram::StrandProgram(StrandProgram &&) noexcept = default;
StrandProgram &StrandProgram::operator=(StrandProgram &&) noexcept = default;

std::vector<StrandPlane> StrandProgram::planes(StrandBudget &budget) const {
    Work w(budget, impl->cancel);
    std::vector<StrandPlane> result; size_t count = 0;
    w.records(count, impl->live_rows.size()); result.reserve(count);
    for (const auto &plane : impl->live_rows) { w.charge(); result.push_back(plane); }
    return result;
}
bool StrandProgram::contains_owner(const CellId &id, StrandBudget &budget) const {
    Work w(budget, impl->cancel); w.lookup(impl->owners.size());
    return impl->owners.count(id) != 0;
}
std::vector<StrandRun> StrandProgram::runs(const StrandPlane &plane, StrandBudget &budget) const {
    Work w(budget, impl->cancel); w.lookup(impl->rows.size());
    const auto row = impl->rows.find(plane);
    std::vector<StrandRun> result;
    if (row == impl->rows.end()) return result;
    size_t records = 0;
    for (const auto &entry : row->second.active) {
        const auto &first = entry.first; const auto &edge = entry.second;
        if (result.empty() || !(result.back().atoms.back().last == first)) {
            w.records(records, 2);
            result.emplace_back(); result.back().points.push_back(*edge.a);
        }
        w.records(records, 2);
        result.back().atoms.push_back({first, edge.last, edge.owner});
        result.back().points.push_back(*edge.b);
    }
    return result;
}

std::vector<StrandPlane> StrandProgram::set_owner(const CellId &id, bool active, StrandBudget &budget) {
    Work w(budget, impl->cancel); w.lookup(impl->owners.size());
    auto oi = impl->owners.find(id);
    if (oi == impl->owners.end()) throw std::out_of_range("Canonical cubic strand owner is not in this plane catalog");
    auto &owner = oi->second;
    if (active == owner.active) return {};
    struct Change { Impl::Row *row; Impl::Edges::iterator source; };
    std::array<Change,3> changes{};
    size_t records = 0; w.records(records, owner.count);
    std::vector<StrandPlane> touched; touched.reserve(owner.count);
    // Preflight all throwing work, admission, cancellation and storage before
    // transferring nodes. No allocation or callbacks during publication.
    for (size_t part = 0; part < owner.count; ++part) {
        const auto &ref = owner.edges[part];
        w.lookup(impl->rows.size());
        auto &row = impl->rows.at(ref.plane);
        auto &src = active ? row.dormant : row.active;
        auto &dst = active ? row.active : row.dormant;
        w.lookup(src.size()); w.lookup(dst.size()); w.lookup(dst.size());
        const auto it = src.find(ref.first);
        if (it == src.end() || !(it->second.owner == id) || dst.count(ref.first))
            throw std::logic_error("Inconsistent canonical cubic strand owner incidence");
        w.lookup(impl->live_rows.size()); w.lookup(impl->dormant_rows.size());
        changes[part] = {&row,it}; touched.push_back(ref.plane);
    }
    w.poll();
    for (size_t part = 0; part < owner.count; ++part) {
        auto &row = *changes[part].row;
        auto &src = active ? row.dormant : row.active;
        auto &dst = active ? row.active : row.dormant;
        const bool switch_row = active ? row.active.empty() : row.active.size() == 1;
        dst.insert(src.extract(changes[part].source));
        if (switch_row) {
            auto &from = active ? impl->dormant_rows : impl->live_rows;
            auto &to = active ? impl->live_rows : impl->dormant_rows;
            to.insert(from.extract(touched[part]));
        }
    }
    owner.active = active;
    return touched;
}
}} // namespace Slic3r::BridgeAwareCubic
