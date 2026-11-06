#ifndef slic3r_BridgeAwareCubicInteriorShadow_hpp_
#define slic3r_BridgeAwareCubicInteriorShadow_hpp_
#include "../ExPolygon.hpp"
#include "../Polyline.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

namespace Slic3r { namespace BridgeAwareCubic { namespace InteriorShadow {
// Diagnostic-only proposal; no physical verifier consumes its boolean as a verdict.
struct Chord { Point q, a, b; };
constexpr int64_t bound = std::numeric_limits<int32_t>::max()/4;
inline bool safe(const Point &p) { return std::abs(int64_t(p.x()))<=bound && std::abs(int64_t(p.y()))<=bound; }
inline int64_t cross(const Point &a,const Point &b,const Point &p) {
    return (int64_t(b.x())-a.x())*(int64_t(p.y())-a.y())-(int64_t(b.y())-a.y())*(int64_t(p.x())-a.x());
}
inline int sign(int64_t x) { return (x>0)-(x<0); }
inline bool intersects(const Point &a,const Point &b,const Point &c,const Point &d) {
    if (std::max(a.x(),b.x())<std::min(c.x(),d.x()) || std::max(c.x(),d.x())<std::min(a.x(),b.x()) ||
        std::max(a.y(),b.y())<std::min(c.y(),d.y()) || std::max(c.y(),d.y())<std::min(a.y(),b.y())) return false;
    return sign(cross(a,b,c))*sign(cross(a,b,d))<=0 && sign(cross(c,d,a))*sign(cross(c,d,b))<=0;
}
// Proof by separating axes: convex tube = lower segment + closed L-infinity square.
// Use conservative integer radii; a grazing point is not free.
inline bool tube_disjoint(const Point &a,const Point &b,const Point &c,const Point &d,int64_t r) {
    if (!safe(a)||!safe(b)||!safe(c)||!safe(d)||r<0||r>bound) return false;
    const auto separated=[&](int64_t x,int64_t y) {
        auto dot=[&](const Point &p){return x*int64_t(p.x())+y*int64_t(p.y());};
        const int64_t grow=r*(std::abs(x)+std::abs(y));
        return std::max(dot(a),dot(b))<std::min(dot(c),dot(d))-grow ||
               std::min(dot(a),dot(b))>std::max(dot(c),dot(d))+grow;
    };
    return separated(1,0)||separated(0,1)||separated(int64_t(a.y())-b.y(),int64_t(b.x())-a.x())||
        separated(int64_t(c.y())-d.y(),int64_t(d.x())-c.x());
}
inline bool long_enough(const Point &a,const Point &b,double gap) {
    if (!safe(a)||!safe(b)||!std::isfinite(gap)||gap<=0 || gap/SCALING_FACTOR>=bound) return false;
    const int64_t g=int64_t(std::ceil(std::nextafter(gap/SCALING_FACTOR,std::numeric_limits<double>::infinity())));
    const int64_t x=int64_t(a.x())-b.x(), y=int64_t(a.y())-b.y();
    return x*x+y*y>g*g;
}
template<class Charge> bool contains(const ExPolygon &p,const Chord &w,Charge charge) {
    if (!safe(w.q)||!safe(w.a)||!safe(w.b)||p.holes.size()>1024) return false;
    auto ring=[&](const Polygon &r,bool want_inside) {
        if (r.size()<3||r.size()>8192||!charge(r.size()*8+1)) return false;
        bool in=false;
        for (size_t i=0;i<r.size();++i) {
            const auto &u=r.points[i], &v=r.points[(i+1)%r.size()];
            if (!safe(u)||!safe(v)||intersects(w.q,w.a,u,v)||intersects(w.q,w.b,u,v)) return false;
            if ((u.y()>w.q.y())!=(v.y()>w.q.y())) {
                const int64_t t=cross(u,v,w.q);
                if ((v.y()>u.y() && t>0)||(v.y()<u.y() && t<0)) in=!in;
            }
        }
        return in==want_inside;
    };
    if (!ring(p.contour,true)) return false;
    for (const auto &hole:p.holes) if (!ring(hole,false)) return false;
    return true;
}
template<class Charge> bool inside_any(const ExPolygons &p,const Chord &w,Charge charge) {
    if (p.size()>2048) return false;
    for (const auto &poly:p) if (contains(poly,w,charge)) return true;
    return false;
}
// Floating intersections only PROPOSE q. Integer half-plane clearance, exact length,
// and strict per-path mask membership recheck every returned proposal.
template<class Charge> bool propose(const Polygon &parent,const Polyline &path,const ExPolygons &demand,
                                     double gap,int64_t pad, Chord &out,Charge charge) {
    const size_t n=parent.size();
    if (n<3||n>12||path.size()<2||path.size()>512||pad<1||pad>bound||!charge(30*n+1)) return false;
    double cx=0,cy=0;
    for (const Point &p:parent.points) { if (!safe(p)) return false; cx+=p.x(); cy+=p.y(); }
    cx/=n; cy/=n;
    const auto cross_at=[&](size_t i,double x,double y){
        const Point &a=parent.points[i], &b=parent.points[(i+1)%n];
        return (double(b.x())-a.x())*(y-a.y())-(double(b.y())-a.y())*(x-a.x());
    };
    const auto margin=[&](size_t i,int64_t r) {
        const Point &a=parent.points[i], &b=parent.points[(i+1)%n];
        return r*(std::abs(int64_t(b.x())-a.x())+std::abs(int64_t(b.y())-a.y()));
    };
    const auto clear=[&](const Point &p) {
        if (!safe(p)) return false;
        for (size_t i=0;i<n;++i) if (cross(parent.points[i],parent.points[(i+1)%n],p)<=margin(i,pad)) return false;
        return true;
    };
    double shrink=0.;
    for (size_t i=0;i<n;++i) {
        const double c=cross_at(i,cx,cy); if (c<=0.) return false;
        shrink=std::max(shrink,double(margin(i,pad+2))/c);
    }
    if (!(shrink<1.)) return false;
    std::array<Point,12> inner;
    for (size_t i=0;i<n;++i) {
        inner[i]=Point(coord_t(std::llround(cx*shrink+parent.points[i].x()*(1.-shrink))),
                       coord_t(std::llround(cy*shrink+parent.points[i].y()*(1.-shrink))));
        if (!clear(inner[i])) return false;
    }
    int64_t best=-1;
    for (size_t i=0;i<n;++i) for(size_t j=i+1;j<n;++j) {
        if (!long_enough(inner[i],inner[j],gap)) continue;
        const int64_t x=int64_t(inner[i].x())-inner[j].x(),y=int64_t(inner[i].y())-inner[j].y();
        if (x*x+y*y>best) {best=x*x+y*y;out.a=inner[i];out.b=inner[j];}
    }
    if (best<0) return false;
    for (size_t j=1;j<path.size();++j) {
        if (!charge(8*n+1)) return false;
        const auto &a=path.points[j-1], &b=path.points[j];
        if (!safe(a)||!safe(b)) return false;
        if (a==b) continue;
        double lo=0.,hi=1.;
        for (size_t i=0;i<n && lo<hi;++i) {
            const double f=cross_at(i,a.x(),a.y())-margin(i,pad+2), g=cross_at(i,b.x(),b.y())-margin(i,pad+2);
            if (f<=0. && g<=0.) {hi=lo;break;}
            if(f<=0.) lo=std::max(lo,f/(f-g));
            if(g<=0.) hi=std::min(hi,f/(f-g));
        }
        if (lo>=hi) continue;
        const double t=.5*(lo+hi);
        const Point q(coord_t(std::llround((1.-t)*a.x()+t*b.x())),coord_t(std::llround((1.-t)*a.y()+t*b.y())));
        if (clear(q)&&inside_any(demand,Chord{q,q,q},charge)) {out.q=q;return true;}
    }
    return false;
}
}}}
#endif
