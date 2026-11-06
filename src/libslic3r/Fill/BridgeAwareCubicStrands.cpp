#include "BridgeAwareCubicStrands.hpp"

#include <boost/multiprecision/cpp_int.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
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
                             StrandBudget &budget) : impl(std::make_shared<Impl>(std::move(cancel)))
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
namespace strand_clip {
using Wide = boost::multiprecision::int128_t;
using F = StrandFraction;
constexpr size_t none = std::numeric_limits<size_t>::max();
constexpr int64_t coord_limit = std::numeric_limits<int32_t>::max() / 4;

// Endpoints are within +/-2^29, so integer differences <2^30, determinants
// <2^61 and mediant denominators <2^62. An orientation against a rational
// point needs <125 bits. No floating point decides an interval or an XY tie.
void check_xy(const Point &p) {
    if (p.x() < -coord_limit || p.x() > coord_limit || p.y() < -coord_limit || p.y() > coord_limit)
        throw std::out_of_range("Canonical strand clipping coordinate exceeds exact kernel range");
}
struct I { int64_t x,y; };
I delta(const Point &b, const Point &a) { return {int64_t(b.x())-a.x(),int64_t(b.y())-a.y()}; }
int64_t cross(I a,I b) { return a.x*b.y-a.y*b.x; }
F fraction(int64_t n,int64_t d) {
    if (d<0) { n=-n; d=-d; }
    const int64_t divisor=std::gcd(n,d);
    return {n/divisor,d/divisor};
}
bool less(F a,F b) { return Wide(a.numerator)*b.denominator < Wide(b.numerator)*a.denominator; }
F interior(F a,F b) { return {a.numerator+b.numerator,a.denominator+b.denominator}; } // strict mediant
struct Q { Wide x,y; int64_t d; };
Q at(const Point &a,const Point &b,F t) {
    const I v=delta(b,a);
    return {Wide(a.x())*t.denominator+Wide(v.x)*t.numerator,
            Wide(a.y())*t.denominator+Wide(v.y)*t.numerator,t.denominator};
}
struct Box { int64_t x0,y0,x1,y1;
    bool meets(const Point &a,const Point &b)const { return std::max(a.x(),b.x())>=x0 && std::min(a.x(),b.x())<=x1 &&
        std::max(a.y(),b.y())>=y0 && std::min(a.y(),b.y())<=y1; }
    bool meets(const Q &p)const { return p.x>=Wide(x0)*p.d && p.x<=Wide(x1)*p.d && p.y>=Wide(y0)*p.d && p.y<=Wide(y1)*p.d; }
};
// During construction attribute all live logical records, including temporary
// cut arrays, to one cap. Work already spent is never refunded.
struct Memory { Work &w; size_t live=0;
    void take(size_t n) { w.records(live,n); }
    struct Temp { Memory &m; size_t count=0;
        explicit Temp(Memory &mem):m(mem){} ~Temp(){m.live-=count;}
        void take(size_t n=1){m.take(n);count+=n;}
        Temp(const Temp&)=delete; Temp& operator=(const Temp&)=delete;
    };
};
struct Ring {const Polygon *polygon;Box bounds;};
struct Region { Ring outer; std::vector<Ring> holes; };
Ring ring(const Polygon &p,Memory::Temp &space) {
    if(p.size()<3)throw std::invalid_argument("Canonical clip mask ring has fewer than three points");
    space.take(p.size()+1);
    Box box{coord_limit,coord_limit,-coord_limit,-coord_limit}; Wide twice_area=0;
    for(size_t i=0;i<p.size();++i) {
        space.m.w.charge(); const auto &a=p.points[i], &b=p.points[(i+1)%p.size()]; check_xy(a);
        box.x0=std::min<int64_t>(box.x0,a.x()); box.x1=std::max<int64_t>(box.x1,a.x());
        box.y0=std::min<int64_t>(box.y0,a.y()); box.y1=std::max<int64_t>(box.y1,a.y());
        twice_area+=Wide(a.x())*b.y()-Wide(a.y())*b.x();
    }
    if(twice_area==0)throw std::invalid_argument("Canonical clip mask has zero signed area");
    return {&p,box};
}
Region region(const ExPolygon &p,Memory::Temp &space) {
    Region r{ring(p.contour,space),{}}; space.take(p.holes.size()); r.holes.reserve(p.holes.size());
    for(const auto &h:p.holes)r.holes.push_back(ring(h,space));
    return r;
}
struct Hit { bool strict=false, closed=false, half=false; };
Hit locate(const Ring &r,const Q &p,Work &w) {
    w.charge(); if(!r.bounds.meets(p))return {};
    bool boundary=false, inside=false;
    const auto &points=r.polygon->points;
    for(size_t i=0;i<points.size();++i) {
        w.charge();const Point &a=points[i],&b=points[(i+1)%points.size()];const I v=delta(b,a);
        const Wide ay=Wide(a.y())*p.d,by=Wide(b.y())*p.d;
        const Wide det=Wide(v.x)*(p.y-ay)-Wide(v.y)*(p.x-Wide(a.x())*p.d);
        if(det==0 && p.x>=Wide(std::min(a.x(),b.x()))*p.d && p.x<=Wide(std::max(a.x(),b.x()))*p.d &&
           p.y>=std::min(ay,by) && p.y<=std::max(ay,by))boundary=true;
        // Symbolic (+eps,+eps^2) breaks vertex/edge ties identically for
        // neighboring surfaces. No finite tolerance dilates either mask.
        const int sign=det>0?1:det<0?-1:v.y<0?1:v.y>0?-1:v.x>0?1:v.x<0?-1:0;
        if ((ay<=p.y && by>p.y && sign>0) || (by<=p.y && ay>p.y && sign<0))inside=!inside;
    }
    return {!boundary&&inside,boundary||inside,inside};
}
Hit locate(const Region &r,const Q &p,Work &w) {
    Hit h=locate(r.outer,p,w);if(!h.closed)return {};
    for(const auto &ring:r.holes){const Hit hole=locate(ring,p,w); h.strict &= !hole.closed; h.closed &= !hole.strict; h.half &= !hole.half;}
    return h;
}
void intersections(const Ring &ring,const Point &a,const Point &b,std::vector<F> &events,Memory::Temp &space) {
    auto &w=space.m.w; w.charge(); if(!ring.bounds.meets(a,b))return;
    const I d=delta(b,a);
    const auto add=[&](int64_t n,int64_t den){if(den<0){n=-n;den=-den;}if(n>=0&&n<=den){space.take();events.push_back(fraction(n,den));}};
    const auto &points=ring.polygon->points;
    for(size_t i=0;i<points.size();++i) {
        w.charge(); const Point &p=points[i],&q=points[(i+1)%points.size()]; const I v=delta(q,p), r=delta(p,a);
        int64_t den=cross(d,v), t=cross(r,v), u=cross(r,d);
        if(den!=0){if(den<0){den=-den;t=-t;u=-u;}if(u>=0&&u<=den)add(t,den);}
        else if(u==0){ // Collinear boundary: both projected ends are events.
            if(d.x!=0){add(r.x,d.x);add(int64_t(q.x())-a.x(),d.x);}
            else {add(r.y,d.y);add(int64_t(q.y())-a.y(),d.y);}
        }
    }
}
void intersections(const Region &r,const Point &a,const Point &b,std::vector<F> &events,Memory::Temp &space) {
    if(!r.outer.bounds.meets(a,b))return;
    intersections(r.outer,a,b,events,space);for(const auto &h:r.holes)intersections(h,a,b,events,space);
}
void sorted_events(std::vector<F> &e,Work &w) {
    std::sort(e.begin(),e.end(),[&](F a,F b){w.charge();return less(a,b);});
    e.erase(std::unique(e.begin(),e.end(),[&](F a,F b){w.charge();return a==b;}),e.end());
}
struct Part { F lo,hi; size_t material; bool join_a,join_b; };
struct Fragment { ClippedStrandAtom atom; Point a,b; bool join_a,join_b; };
struct Masks {
    const Region &surface; const std::vector<Region> &materials;
    size_t component(const Q &p,const std::vector<size_t> &relevant,Work &w)const {
        size_t first=none, half=none, count=0, strict=0, halves=0;
        for(size_t id:relevant){const Hit hit=locate(materials[id],p,w);if(!hit.closed)continue;
            if(first==none)first=id; ++count; strict+=hit.strict;
            if(hit.half){half=id;++halves;}}
        if(count<2)return first;
        if(strict||halves>1)throw std::domain_error("Canonical clip requires nonoverlapping normalized material components");
        return half==none?first:half;
    }
    bool joins(const Q &p,size_t id,Work &w)const { return locate(surface,p,w).strict && locate(materials[id],p,w).strict; }
};

struct Round { Point p; Wide error; };
struct Rounds { std::array<Round,4> values; size_t count=0; };
Rounds choices(const Q &p) {
    const auto axis=[&](const Wide &n){const Wide d=p.d;Wide q=n/d;if(n%d<0)--q;
        const int64_t a=q.convert_to<int64_t>();return std::pair<int64_t,int64_t>{a,a+(n%d!=0)};};
    const auto xs=axis(p.x),ys=axis(p.y);Rounds out;
    for(int64_t x=xs.first;x<=xs.second;++x)for(int64_t y=ys.first;y<=ys.second;++y){
        const Wide dx=Wide(x)*p.d-p.x,dy=Wide(y)*p.d-p.y;
        out.values[out.count++]={Point(coord_t(x),coord_t(y)),dx*dx+dy*dy};}
    return out;
}
// Recheck the entire proposed integer segment, not just its endpoints. A
// rounded boundary can cross an arbitrarily narrow hole even when both ends
// lie in allowed material. Requiring exact midpoint coverage of every event
// interval proves there is no positive gap. A new interior boundary touch
// introduced by rounding needs a barrier: decline that candidate rather than
// publishing an unmarked join. Original exact touches were already split.
bool rounded_inside(const Point &a,const Point &b,const Masks &m,size_t id,Memory &mem) {
    const auto in_closure=[&](const Point &p){const auto q=at(p,p,{0,1});return locate(m.surface,q,mem.w).closed&&locate(m.materials[id],q,mem.w).closed;};
    if(!in_closure(a)||!in_closure(b))return false;
    if(a==b)return true;
    Memory::Temp scratch(mem);scratch.take(2);std::vector<F> cuts{{0,1},{1,1}};
    intersections(m.surface,a,b,cuts,scratch);intersections(m.materials[id],a,b,cuts,scratch);sorted_events(cuts,mem.w);
    for(size_t i=1;i<cuts.size();++i){
        if(i+1<cuts.size() && !m.joins(at(a,b,cuts[i]),id,mem.w))return false;
        const auto q=at(a,b,interior(cuts[i-1],cuts[i]));
        if(!locate(m.surface,q,mem.w).half||!locate(m.materials[id],q,mem.w).closed)return false;}
    return true;
}
std::pair<Point,Point> rounded_part(const Point &a,const Point &b,const Part &part,const Masks &m,Memory &mem) {
    const auto l=choices(at(a,b,part.lo)),r=choices(at(a,b,part.hi));
    // An unchanged exact subsegment already underwent the interval proof.
    if(l.count==1&&r.count==1)return {l.values[0].p,r.values[0].p};
    struct Pair {size_t a,b;}; std::array<Pair,16> pairs; size_t count=0;
    for(size_t i=0;i<l.count;++i)for(size_t j=0;j<r.count;++j){
        // Lexicographic minimization of endpoint rounding error; the fixed
        // denominators for each endpoint make raw squared residuals comparable
        // within that endpoint. This needs no floating-point tie breaker.
        pairs[count++]={i,j};}
    std::sort(pairs.begin(),pairs.begin()+count,[&](const Pair &u,const Pair &v){mem.w.charge();
        const Round &a0=l.values[u.a],&a1=l.values[v.a],&b0=r.values[u.b],&b1=r.values[v.b];
        if(a0.error!=a1.error)return a0.error<a1.error;
        if(b0.error!=b1.error)return b0.error<b1.error;
        return std::tie(u.a,u.b)<std::tie(v.a,v.b);});
    const I d=delta(b,a);
    for(size_t i=0;i<count;++i){mem.w.charge();const Point &x=l.values[pairs[i].a].p,&y=r.values[pairs[i].b].p;
        const I e=delta(y,x);if(x!=y && d.x*e.x+d.y*e.y<=0)continue;
        if(rounded_inside(x,y,m,part.material,mem))return {x,y};}
    throw std::domain_error("Canonical clip has no legal floor/ceil rounding for a positive source fragment");
}

template<class Emit> void clip_atom(const Point &a,const Point &b,const StrandAtom &source,const Masks &m,Memory &mem,Emit &&emit) {
    check_xy(a);check_xy(b);mem.w.charge();if(!m.surface.outer.bounds.meets(a,b))return;
    Memory::Temp scratch(mem);std::vector<size_t> relevant;
    for(size_t id=0;id<m.materials.size();++id){mem.w.charge();if(m.materials[id].outer.bounds.meets(a,b)){scratch.take();relevant.push_back(id);}}
    if(relevant.empty())return;
    if(a==b){const Q p=at(a,b,{0,1});if(!locate(m.surface,p,mem.w).half)return;
        const size_t id=m.component(p,relevant,mem.w);if(id==none)return; const bool join=m.joins(p,id,mem.w);
        emit(Fragment{{source,{0,1},{1,1},id},a,b,join,join});return;}
    scratch.take(2);std::vector<F> cuts{{0,1},{1,1}};
    intersections(m.surface,a,b,cuts,scratch);for(size_t id:relevant)intersections(m.materials[id],a,b,cuts,scratch);
    sorted_events(cuts,mem.w);std::vector<Part> parts;
    for(size_t i=1;i<cuts.size();++i){mem.w.charge();const F lo=cuts[i-1],hi=cuts[i]; const Q q=at(a,b,interior(lo,hi));
        if(!locate(m.surface,q,mem.w).half)continue;
        const size_t id=m.component(q,relevant,mem.w);if(id==none)continue;
        bool join_a=m.joins(at(a,b,lo),id,mem.w),join_b=m.joins(at(a,b,hi),id,mem.w);
        if(!parts.empty() && parts.back().material==id && parts.back().hi==lo && parts.back().join_b && join_a){parts.back().hi=hi;parts.back().join_b=join_b;}
        else{scratch.take();parts.push_back({lo,hi,id,join_a,join_b});}}
    for(const Part &p:parts){const auto xy=rounded_part(a,b,p,m,mem);emit(Fragment{{source,p.lo,p.hi,p.material},xy.first,xy.second,p.join_a,p.join_b});}
}
} // namespace strand_clip

bool Pilot::inside_material(const Point &a,const Point &b,const ExPolygon &mask,StrandBudget &budget,
                            const std::function<void()> &cancel)
{
    using namespace strand_clip;
    Work work(budget,cancel); Memory mem{work}; Memory::Temp space(mem);
    check_xy(a); check_xy(b); const auto allowed=region(mask,space);
    if (!locate(allowed,at(a,b,{0,1}),work).closed || !locate(allowed,at(a,b,{1,1}),work).closed) return false;
    if (a==b) return true;
    space.take(2);std::vector<F> cuts{{0,1},{1,1}};
    intersections(allowed,a,b,cuts,space);sorted_events(cuts,work);
    for(size_t i=1;i<cuts.size();++i)
        if (!locate(allowed,at(a,b,interior(cuts[i-1],cuts[i])),work).closed) return false;
    return true;
}

struct ClippedStrands::Impl {
    std::shared_ptr<const StrandProgram::Impl> raw;
    std::map<StrandPlane,std::vector<strand_clip::Fragment>> rows;
    bool active(const CellId &id,Work &w)const {w.lookup(raw->owners.size());return raw->owners.at(id).active;}
};
ClippedStrands StrandProgram::clipped(const ExPolygon &surface,const ExPolygons &allowed,StrandBudget &budget)const {
    using namespace strand_clip;
    Work w(budget,impl->cancel);Memory mem{w};Memory::Temp input(mem);
    auto out=std::make_shared<ClippedStrands::Impl>();out->raw=impl;
    if(allowed.empty())return ClippedStrands(std::move(out));
    const Region s=region(surface,input);std::vector<Region> material;input.take(allowed.size());material.reserve(allowed.size());
    for(const auto &p:allowed)material.push_back(region(p,input));const Masks masks{s,material};
    for(const auto &entry:impl->rows){w.charge();const auto &r=entry.second;
        auto a=r.active.begin(),d=r.dormant.begin();std::vector<Fragment> parts;
        while(a!=r.active.end()||d!=r.dormant.end()){
            const auto &e=d==r.dormant.end() || (a!=r.active.end()&&a->first<d->first)?*a++:*d++;
            clip_atom(*e.second.a,*e.second.b,StrandAtom{e.first,e.second.last,e.second.owner},masks,mem,[&](Fragment part){
                mem.take(3);parts.push_back(std::move(part));});}
        if(!parts.empty()){mem.take(1);w.lookup(out->rows.size());out->rows.emplace(entry.first,std::move(parts));}}
    return ClippedStrands(std::move(out));
}
std::vector<StrandPlane> ClippedStrands::planes(StrandBudget &budget)const {
    Work w(budget,impl->raw->cancel);std::vector<StrandPlane> out;size_t records=0;
    for(const auto &e:impl->rows){w.charge();for(const auto &f:e.second)if(impl->active(f.atom.source.owner,w)){
        w.records(records);out.push_back(e.first);break;}}
    return out;
}
std::vector<ClippedStrandRun> ClippedStrands::runs(const StrandPlane &plane,StrandBudget &budget)const {
    using namespace strand_clip;
    Work w(budget,impl->raw->cancel);w.lookup(impl->rows.size());const auto row=impl->rows.find(plane);
    std::vector<ClippedStrandRun> result;if(row==impl->rows.end())return result;
    size_t records=0;const Fragment *previous=nullptr;
    for(const auto &f:row->second){w.charge();if(!impl->active(f.atom.source.owner,w))continue;
        // Connectivity is decided before looking at integer endpoints. Retain
        // all source/material gaps even if two rounded endpoints coincide.
        bool join=previous && previous->join_b && f.join_a && previous->atom.material_component==f.atom.material_component;
        if(join){const auto &p=previous->atom,&c=f.atom;
            join=((p.source.owner==c.source.owner && p.source.first==c.source.first && p.last==c.first) ||
                  (p.source.last==c.source.first && p.last==F{1,1} && c.first==F{0,1})) && previous->b==f.a;}
        if(!join){w.records(records,2);result.emplace_back();result.back().points.push_back(f.a);}
        w.records(records,2);result.back().atoms.push_back(f.atom);result.back().points.push_back(f.b);previous=&f;}
    return result;
}
}} // namespace Slic3r::BridgeAwareCubic
