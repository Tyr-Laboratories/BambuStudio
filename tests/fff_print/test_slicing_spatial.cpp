#include <catch2/catch.hpp>
#include "libslic3r/Point.hpp"
#include "libslic3r/KDTreeIndirect.hpp"
#include "libslic3r/WallPainting.hpp"
#include <random>
#include <chrono>
#include <iostream>

using namespace Slic3r;

namespace {
struct CountingCoordinates {
    const std::vector<Vec3f> &points;
    size_t &reads;
    float operator()(size_t index, size_t dimension) const
    {
        ++reads;
        return points[index][dimension];
    }
};
using SampleTree = KDTreeIndirect<3, float, CountingCoordinates>;
}

TEST_CASE("Spherical searches prune corners outside all ancestor bounds", "[SlicingSpatial][KDTree]")
{
    // Each coordinate is within the radius, but every point is outside the
    // sphere. Checking only the current split plane visits the entire tree.
    std::vector<Vec3f> points;
    for (int x=0;x<32;++x) for (int y=0;y<32;++y) for (int z=0;z<32;++z)
        points.emplace_back(10.f+x/8.f,10.f+y/8.f,10.f+z/8.f);
    size_t reads=0;
    SampleTree tree(CountingCoordinates{points,reads},points.size());
    tree.build_radius_index();
    reads=0;
    REQUIRE(find_nearby_points(tree,Vec3f::Zero().eval(),15.f).empty());
    // A deterministic work bound, not a machine-dependent timing assertion.
    REQUIRE(reads < points.size()*2);
}

TEST_CASE("Spherical searches retain membership and tree traversal order", "[SlicingSpatial][KDTree]")
{
    std::mt19937 rng(3421);
    std::vector<Vec3f> points;
    for (size_t i=0;i<2048;++i)
        points.emplace_back(int(rng()%1000)/8.f,int(rng()%1000)/8.f,int(rng()%1000)/8.f);
    size_t reads=0;
    SampleTree tree(CountingCoordinates{points,reads},points.size());
    if (GENERATE(false,true)) tree.build_radius_index();
    std::vector<size_t> preorder;
    auto record=[&](size_t index,size_t) {
        preorder.push_back(index);
        return unsigned(VisitorReturnMask::CONTINUE_LEFT)|unsigned(VisitorReturnMask::CONTINUE_RIGHT);
    };
    tree.visit(record);
    for (const Vec3f &center : {Vec3f(0,0,0),Vec3f(60,45,32),Vec3f(125,125,125)})
        for (float radius : {0.f,0.125f,12.f,75.f,300.f}) {
            std::vector<size_t> expected;
            for (size_t index:preorder)
                if ((center-points[index]).squaredNorm()<radius*radius && index%3!=0)
                    expected.push_back(index);
            REQUIRE(find_nearby_points(tree,center,radius,[](size_t i){return i%3!=0;})==expected);
        }
}

TEST_CASE("Spherical searches preserve strict float radius boundaries", "[SlicingSpatial][KDTree]")
{
    const float inside=std::nextafter(1.f,0.f), outside=std::nextafter(1.f,2.f);
    std::vector<Vec3f> points{{inside,0,0},{1,0,0},{outside,0,0},{0,-inside,0},
                             {0,-1,0},{0,-outside,0},{0,0,0},{0,0,1},{0,0,-1}};
    size_t reads=0;
    SampleTree tree(CountingCoordinates{points,reads},points.size());
    if (GENERATE(false,true)) tree.build_radius_index();
    auto found=find_nearby_points(tree,Vec3f::Zero().eval(),1.f);
    std::sort(found.begin(),found.end());
    REQUIRE(found==std::vector<size_t>{0,3,6});
    REQUIRE(find_nearby_points(tree,Vec3f::Zero().eval(),0.f).empty());
}

TEST_CASE("Radius indexes move with their tree and are invalidated on rebuild", "[SlicingSpatial][KDTree]")
{
    struct Coordinates {
        const std::vector<Vec2d> *points;
        double operator()(size_t i,size_t d) const {return (*points)[i][d];}
    };
    using Tree=KDTreeIndirect<2,double,Coordinates>;
    std::vector<Vec2d> points{{0,0},{3,4},{1,1}}, other{{100,100}};
    Tree tree(Coordinates{&points},points.size());
    tree.build_radius_index();
    Tree moved(std::move(tree));
    Tree assigned(Coordinates{&other},other.size());
    assigned.build_radius_index();
    assigned=std::move(moved);
    auto found=find_nearby_points(assigned,Vec2d::Zero().eval(),5.);
    std::sort(found.begin(),found.end());
    REQUIRE(found==std::vector<size_t>{0,2});
    points={{100,100},{0,0},{200,200}};
    assigned.build(points.size());
    REQUIRE(find_nearby_points(assigned,Vec2d::Zero().eval(),1.)==std::vector<size_t>{1});
    assigned.build_radius_index();
    REQUIRE(find_nearby_points(assigned,Vec2d::Zero().eval(),1.)==std::vector<size_t>{1});
    assigned.clear();
    assigned.build_radius_index();
    REQUIRE(find_nearby_points(assigned,Vec2d::Zero().eval(),1.).empty());
}

TEST_CASE("Local wall patches retain zero counts and segment endpoints", "[SlicingSpatial][WallPainting]")
{
    using namespace WallPainting;
    const int count=GENERATE(0,1,3);
    std::vector<BoundarySpan> spans;
    for (coord_t x=0;x<1000;x+=100)
        spans.push_back({Line(Point(x,0),Point(x+100,0)),count,0});
    spans.push_back({Line(Point(100000,0),Point(101000,0)),count+1,1});
    BoundaryPolicy policy(std::move(spans));
    REQUIRE_FALSE(policy.uniform_count());
    for (const Line &line : {Line(Point(100,10),Point(900,10)),Line(Point(900,10),Point(100,10))}) {
        const auto split=policy.split(line);
        REQUIRE(split.size()==1);
        REQUIRE(split[0].first.a==line.a);
        REQUIRE(split[0].first.b==line.b);
        REQUIRE(split[0].second==count);
    }
    REQUIRE(policy.split(Line(Point(100,10),Point(100,10))).empty());
}

TEST_CASE("Nearby wall transitions and equidistant overrides survive splitting", "[SlicingSpatial][WallPainting]")
{
    using namespace WallPainting;
    BoundaryPolicy transition({{Line(Point(0,0),Point(500,0)),0,0},
                               {Line(Point(500,0),Point(1000,0)),3,0}});
    const auto split=transition.split(Line(Point(0,10),Point(1000,10)));
    REQUIRE(split.size()==2);
    REQUIRE(split[0].second==0);
    REQUIRE(split[0].first.b==Point(500,10));
    REQUIRE(split[1].second==3);
    REQUIRE(split[1].first.a==Point(500,10));
    BoundaryPolicy tied({{Line(Point(0,0),Point(1000,0)),1,0},
                         {Line(Point(0,20),Point(1000,20)),3,1}});
    const auto tie=tied.split(Line(Point(100,10),Point(900,10)));
    REQUIRE(tie.size()==1);
    REQUIRE(tie[0].second==3);
}

TEST_CASE("Benchmark locally uniform wall boundary queries", "[.SlicingBenchmark]")
{
    using namespace WallPainting;
    std::vector<BoundarySpan> spans;
    for (coord_t x=0;x<100000000;x+=10000)
        spans.push_back({Line(Point(x,0),Point(x+10000,0)),1,0});
    spans.push_back({Line(Point(200000000,0),Point(201000000,0)),3,1});
    BoundaryPolicy policy(std::move(spans));
    const auto start=std::chrono::steady_clock::now();
    size_t pieces=0, count=0;
    for (size_t i=0;i<10000;++i) {
        const coord_t x=1000000+coord_t(i%1000)*1000;
        const auto split=policy.split(Line(Point(x,10),Point(x+800000,10)));
        pieces+=split.size();
        for (const auto &piece:split) count+=piece.second;
    }
    const double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    REQUIRE(pieces==10000);
    REQUIRE(count==10000);
    std::cout << "wall_split_10000_seconds=" << seconds << '\n';
}

TEST_CASE("Uniform wall queries preserve endpoint projection rounding", "[SlicingSpatial][WallPainting]")
{
    using namespace WallPainting;
    BoundaryPolicy policy({{Line(Point(0,0),Point(999999999,0)),1,0},
                           {Line(Point(2050000000,0),Point(2100000000,0)),3,1}});
    // The existing cut coalescer keeps the projected endpoint when it is
    // within its 1e-9 parameter tolerance of 1. A performance shortcut must
    // preserve that coordinate as well as the wall count.
    const auto split=policy.split(Line(Point(0,0),Point(1000000000,0)));
    REQUIRE(split.size()==1);
    REQUIRE(split[0].first.a==Point(0,0));
    REQUIRE(split[0].first.b==Point(999999999,0));
    REQUIRE(split[0].second==1);
}
