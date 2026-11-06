#include <catch2/catch.hpp>
#include "libslic3r/Fill/BridgeAwareCubicStrands.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
using namespace Slic3r;
namespace BAC = Slic3r::BridgeAwareCubic;
namespace {
const BAC::StrandPlane horizontal{0,1,1};
const BAC::CellId middle{{{0,-1,-1}},0};
ExPolygon box(coord_t x, coord_t y, coord_t xx, coord_t yy) {
    return ExPolygon(Polygon({Point(x,y),Point(xx,y),Point(xx,yy),Point(x,yy)}));
}
BAC::Plan clip_plan(std::function<void()> stop = {}) {
    BAC::Parameters p; p.bounds=BoundingBoxf3(Vec3d(-4,-4,-2),Vec3d(4,4,3));
    p.flow_spacing=.125; p.density=.25; p.angle=0.; p.throw_if_canceled=std::move(stop);
    const auto d=box(-300000,-300000,300000,300000);
    const double e=std::sqrt(6.)*p.flow_spacing/p.density;
    return BAC::Plan(p, {{{d},0.,.9*e},{{d},e/1.7320508075688772935,.9*e}});
}
struct ClipFixture {
    BAC::Plan plan; BAC::StrandBudget budget; BAC::StrandProgram raw;
    coord_t x=0,y=0,lo=0,hi=0;
    explicit ClipFixture(std::function<void()> stop={}) : plan(clip_plan(std::move(stop))), raw(plan.strand_program(0.,budget)) {
        std::set<BAC::CellId> others; bool found=false;
        for(const auto &run:raw.runs(horizontal,budget)) for(size_t i=0;i<run.atoms.size();++i) {
            if(!(run.atoms[i].owner==middle)) { others.insert(run.atoms[i].owner); continue; }
            const Point a=run.points[i],b=run.points[i+1]; REQUIRE(a.y()==b.y());
            lo=std::min(a.x(),b.x());hi=std::max(a.x(),b.x());y=a.y(); x=lo+(hi-lo)/2; found=true;
        }
        REQUIRE(found); REQUIRE(hi-lo>100); for(const auto &id:others) raw.set_owner(id,false,budget);
    }
    ExPolygon full() const { return box(lo-20,y-30,hi+20,y+30); }
};
void equal_clip(const std::vector<BAC::ClippedStrandRun>& a,const std::vector<BAC::ClippedStrandRun>& b) {
    REQUIRE(a.size()==b.size()); for(size_t i=0;i<a.size();++i){ REQUIRE(a[i].points==b[i].points);
        REQUIRE(a[i].atoms.size()==b[i].atoms.size()); REQUIRE(a[i].points.size()==a[i].atoms.size()+1);
        for(size_t j=0;j<a[i].atoms.size();++j){const auto &x=a[i].atoms[j], &y=b[i].atoms[j];
            CHECK(x.source.owner==y.source.owner); CHECK(x.source.first==y.source.first); CHECK(x.source.last==y.source.last);
            CHECK(x.first==y.first); CHECK(x.last==y.last); CHECK(x.material_component==y.material_component);
        }
    }
}
}
TEST_CASE("Canonical clipped strands preserve sub-grid holes and legal rounded output", "[BridgeAwareCubic][CanonicalStrandClip]") {
    ClipFixture f; const auto full=f.full(); auto holed=full;
    // At y the triangle cuts (x+1/11,x+2/11). Both ideal cuts round to x,
    // but stitching them or extending the right piece from x would cross the hole.
    Polygon hole({Point(f.x,f.y-1),Point(f.x+1,f.y+10),Point(f.x+2,f.y+10)});
    hole.make_clockwise(); holed.holes.push_back(hole);
    const bool material=GENERATE(false,true);
    auto view=f.raw.clipped(material?full:holed,{material?holed:full},f.budget);
    const auto runs=view.runs(horizontal,f.budget); REQUIRE(runs.size()==2);
    std::vector<std::pair<coord_t,coord_t>> spans;
    for(const auto &r:runs){ REQUIRE(r.atoms.size()==1); REQUIRE(r.points.size()==2);
        CHECK(r.atoms[0].source.owner==middle); CHECK(r.points[0].y()==f.y); CHECK(r.points[1].y()==f.y);
        spans.emplace_back(std::min(r.points[0].x(),r.points[1].x()),std::max(r.points[0].x(),r.points[1].x())); }
    std::sort(spans.begin(),spans.end()); CHECK(spans[0].first==f.lo); CHECK(spans[1].second==f.hi);
    CHECK(spans[0].second<=f.x); CHECK(spans[1].first>=f.x+1);
    CHECK_FALSE(runs[0].atoms[0].last==runs[1].atoms[0].first);
}
TEST_CASE("Canonical clipped surfaces assign shared boundary once and retain exact tangent barriers", "[BridgeAwareCubic][CanonicalStrandClip]") {
    ClipFixture f; auto full=f.full();
    auto above=f.raw.clipped(box(f.lo-20,f.y,f.hi+20,f.y+30),{full},f.budget);
    auto below=f.raw.clipped(box(f.lo-20,f.y-30,f.hi+20,f.y),{full},f.budget);
    REQUIRE(above.runs(horizontal,f.budget).size()==1); CHECK(below.runs(horizontal,f.budget).empty());
    auto tangent=full; Polygon hole({Point(f.x,f.y),Point(f.x-4,f.y+9),Point(f.x+4,f.y+9)});
    hole.make_clockwise(); tangent.holes.push_back(hole);
    auto v=f.raw.clipped(tangent,{full},f.budget); const auto runs=v.runs(horizontal,f.budget);
    REQUIRE(runs.size()==2); CHECK(runs[0].points.back()==Point(f.x,f.y)); CHECK(runs[1].points.front()==Point(f.x,f.y));
    CHECK(runs[0].atoms.back().last==runs[1].atoms.front().first);
}
TEST_CASE("Canonical clipped views retain material components and restore exact dormant producers", "[BridgeAwareCubic][CanonicalStrandClip]") {
    ClipFixture f; const auto s=f.full();
    const ExPolygons m{box(f.lo-20,f.y-20,f.x,f.y+20),box(f.x,f.y-20,f.hi+20,f.y+20)};
    auto view=f.raw.clipped(s,m,f.budget); const auto before=view.runs(horizontal,f.budget); REQUIRE(before.size()==2);
    CHECK(before[0].atoms.front().material_component!=before[1].atoms.front().material_component);
    CHECK(before[0].points.back()==before[1].points.front());
    f.raw.set_owner(middle,false,f.budget); CHECK(view.runs(horizontal,f.budget).empty());
    auto dormant=f.raw.clipped(s,m,f.budget); f.raw.set_owner(middle,true,f.budget);
    equal_clip(view.runs(horizontal,f.budget),before); equal_clip(dormant.runs(horizontal,f.budget),before);
    equal_clip(f.raw.clipped(s,m,f.budget).runs(horizontal,f.budget),before);
    CHECK(f.raw.clipped(s,{},f.budget).runs(horizontal,f.budget).empty());
}
TEST_CASE("Canonical clipped stationary source atoms are barriers after removal", "[BridgeAwareCubic][CanonicalStrandClip]") {
    auto plan=clip_plan(); BAC::StrandBudget b; const double tie=(.5*plan.bulk_cell_edge())/1.7320508075688772935;
    auto raw=plan.strand_program(std::nextafter(tie,std::numeric_limits<double>::infinity()),b);
    const BAC::CellId left{{{0,-1,0}},0},tiny{{{0,0,0}},0},right{{{0,0,-1}},0};
    std::set<BAC::CellId> extra;
    for(const auto &r:raw.runs(horizontal,b)) for(const auto &a:r.atoms)
        if(!(a.owner==left)&&!(a.owner==tiny)&&!(a.owner==right))extra.insert(a.owner);
    for(const auto &id:extra)raw.set_owner(id,false,b);
    auto all=box(-400000,-400000,400000,400000); const auto view=raw.clipped(all,{all},b);
    const auto joined=view.runs(horizontal,b); REQUIRE(joined.size()==1); REQUIRE(joined[0].atoms.size()==3);
    REQUIRE(joined[0].points[1]==joined[0].points[2]); raw.set_owner(tiny,false,b);
    const auto cut=view.runs(horizontal,b); REQUIRE(cut.size()==2); CHECK(cut[0].points.back()==cut[1].points.front());
    raw.set_owner(tiny,true,b); equal_clip(joined,view.runs(horizontal,b));
}
TEST_CASE("Canonical clip limits and cancellation leave the reusable view intact", "[BridgeAwareCubic][CanonicalStrandClip]") {
    struct Stop{}; bool stop=false; ClipFixture f([&]{if(stop)throw Stop{};}); const auto full=f.full();
    const auto view=f.raw.clipped(full,{full},f.budget); const auto before=view.runs(horizontal,f.budget); REQUIRE_FALSE(before.empty());
    BAC::StrandBudget no_work{0,200000},no_records{5000000,1};
    CHECK_THROWS_AS(f.raw.clipped(full,{full},no_work),std::length_error);
    CHECK_THROWS_AS(view.runs(horizontal,no_records),std::length_error);
    stop=true; CHECK_THROWS_AS(f.raw.clipped(full,{full},f.budget),Stop); CHECK_THROWS_AS(view.runs(horizontal,f.budget),Stop);
    stop=false; equal_clip(before,view.runs(horizontal,f.budget));
}
