#include "WallPainting.hpp"
#include "AABBTreeLines.hpp"
#include "ClipperUtils.hpp"
#include <limits>

namespace Slic3r::WallPainting {
struct BoundaryPolicy::Index {
    Linesf lines;
    AABBTreeIndirect::Tree<2,double> tree;
    explicit Index(const std::vector<BoundarySpan> &spans)
    {
        lines.reserve(spans.size());
        for (const auto &span:spans) lines.emplace_back(span.line.a.cast<double>(),span.line.b.cast<double>());
        tree=AABBTreeLines::build_aabb_tree_over_indexed_lines(lines);
    }
    double nearest_distance(const Point &point, size_t &index) const
    {
        const Vec2d p=point.cast<double>();
        Vec2d nearest;
        return AABBTreeLines::squared_distance_to_indexed_lines(lines,tree,p,index,nearest);
    }
    std::vector<size_t> within(const Point &point,double radius_sq) const
    {
        const Vec2d p=point.cast<double>();
        auto found=AABBTreeLines::all_lines_in_radius(lines,tree,p,radius_sq);
        std::sort(found.begin(),found.end());
        return found;
    }
};

BoundaryPolicy::BoundaryPolicy(std::vector<BoundarySpan> spans) : m_spans(std::move(spans)), m_index(std::make_shared<Index>(m_spans)) {}

namespace {
double distance_sq(const Line &line, const Point &point)
{
    const Vec2d d = (line.b-line.a).cast<double>();
    const Vec2d p = (point-line.a).cast<double>();
    const double t = d.squaredNorm() == 0. ? 0. : std::clamp(p.dot(d)/d.squaredNorm(), 0., 1.);
    return (p-t*d).squaredNorm();
}
Point along(const Line &line, double t) { return Point(line.a + ((line.b-line.a).cast<double>()*t).cast<coord_t>()); }
std::vector<double> endpoints(const Line &line, const std::vector<BoundarySpan> &spans)
{
    std::vector<double> cuts{0., 1.};
    const Vec2d d = (line.b-line.a).cast<double>();
    const double length_sq = d.squaredNorm();
    if (length_sq == 0.) return cuts;
    for (const auto &span : spans) {
        const Vec2d s = (span.line.b-span.line.a).cast<double>();
        if (std::abs(d.dot(s)) < 0.5*std::sqrt(length_sq*s.squaredNorm())) continue;
        for (const Point &p : {span.line.a, span.line.b}) {
            const double t = (p-line.a).cast<double>().dot(d)/length_sq;
            if (t > 0. && t < 1.) cuts.push_back(t);
        }
    }
    std::sort(cuts.begin(), cuts.end());
    cuts.erase(std::unique(cuts.begin(), cuts.end(), [](double a, double b) { return std::abs(a-b)<1.e-9; }), cuts.end());
    return cuts;
}
}

BoundaryPolicy BoundaryPolicy::bind(const Polygons &outline, const LayerPaint &paint,
                                    int normal_count, int painted_count, double tolerance,
                                    const std::vector<CountRegion> &count_regions)
{
    std::vector<BoundarySpan> result;
    const double tolerance_sq = tolerance*tolerance;
    Linesf source_lines;
    source_lines.reserve(paint.size());
    for (const auto &source:paint) source_lines.emplace_back(source.line.a.cast<double>(),source.line.b.cast<double>());
    const auto source_index=AABBTreeLines::build_aabb_tree_over_indexed_lines(source_lines);
    // Layer::make_perimeters expands merged slices by ClipperSafetyOffset.
    // These regions only label the surviving outline; they never create walls.
    std::vector<CountRegion> regions;
    for (const auto &region:count_regions)
        regions.push_back({offset_ex(region.area,2.*ClipperSafetyOffset),region.painted_count,region.slope_threshold});
    for (const Polygon &polygon : outline) {
        for (size_t i=0; i<polygon.size(); ++i) {
            const Line boundary(polygon.points[i], polygon.points[(i+1)%polygon.size()]);
            const Vec2d d = (boundary.b-boundary.a).cast<double>();
            std::vector<BoundarySpan> nearby;
            Eigen::AlignedBox<double,2> box(boundary.a.cast<double>(),boundary.a.cast<double>());
            box.extend(boundary.b.cast<double>());
            box.min().array()-=tolerance;
            box.max().array()+=tolerance;
            std::vector<size_t> candidates;
            AABBTreeIndirect::traverse(source_index,AABBTreeIndirect::intersecting(box),[&](const auto &node) {
                candidates.push_back(node.idx); return true;
            });
            std::sort(candidates.begin(),candidates.end());
            for (size_t candidate:candidates) {
                const auto &source=paint[candidate];
                const Vec2d s = (source.line.b-source.line.a).cast<double>();
                // Orientation distinguishes material on opposite sides of a narrow gap.
                if (d.dot(s) < 0.5*std::sqrt(d.squaredNorm()*s.squaredNorm())) continue;
                if (std::min({distance_sq(boundary, source.line.a), distance_sq(boundary, source.line.b),
                              distance_sq(source.line, boundary.a), distance_sq(source.line, boundary.b)}) > tolerance_sq) continue;
                nearby.push_back({source.line, source.painted ? painted_count : normal_count, source.volume_id,source.painted,source.slope_angle});
            }
            auto cuts = endpoints(boundary, nearby);
            for (const auto &region:regions) for (const auto &polygon:to_polygons(region.area))
                for (size_t edge=0;edge<polygon.size();++edge) {
                    Point hit;
                    if (boundary.intersection(Line(polygon[edge],polygon[(edge+1)%polygon.size()]),&hit)) {
                        const double t=(hit-boundary.a).cast<double>().dot(d)/d.squaredNorm();
                        if (t>0. && t<1.) cuts.push_back(t);
                    }
                }
            std::sort(cuts.begin(),cuts.end());
            cuts.erase(std::unique(cuts.begin(),cuts.end()),cuts.end());
            for (size_t c=1; c<cuts.size(); ++c) {
                const Point mid = along(boundary, (cuts[c-1]+cuts[c])/2.);
                double best = tolerance_sq;
                int count = normal_count;
                size_t id = 0;
                bool painted=false;
                double slope_angle=0., slope_threshold=-1.;
                for (size_t j=0; j<nearby.size(); ++j) {
                    const double dist = distance_sq(nearby[j].line, mid);
                    if (dist < best) {
                        best=dist; count=nearby[j].count; id=nearby[j].source; painted=nearby[j].painted;
                        slope_angle=nearby[j].slope_angle;
                    }
                }
                for (const auto &region:regions)
                    if (std::any_of(region.area.begin(),region.area.end(),[&](const ExPolygon &area){return area.contains(mid);})) {
                        if (painted) count=region.painted_count;
                        slope_threshold=region.slope_threshold;
                    }
                const Line span(along(boundary,cuts[c-1]),along(boundary,cuts[c]));
                if (span.a != span.b) result.push_back({span,count,id,painted,slope_angle,slope_threshold});
            }
        }
    }
    return BoundaryPolicy(std::move(result));
}
int BoundaryPolicy::maximum() const
{
    int maximum = 0;
    for (const auto &span : m_spans) maximum=std::max(maximum,span.count);
    return maximum;
}
std::optional<int> BoundaryPolicy::uniform_count() const
{
    if (empty()) return std::nullopt;
    const int count=m_spans.front().count;
    for (const auto &span : m_spans) if (span.count!=count) return std::nullopt;
    return count;
}
int BoundaryPolicy::count_at(const Point &point) const
{
    if (empty()) return 0;
    size_t nearest=0;
    double best=m_index->nearest_distance(point,nearest);
    int count=m_spans[nearest].count;
    // Resolve shared vertices deterministically without scanning the whole layer.
    for (size_t index:m_index->within(point,best+1.e-4)) {
        const auto &span=m_spans[index];
        const double d=distance_sq(span.line,point);
        if (d<best-1.e-4) { best=d; count=span.count; }
        else if (std::abs(d-best)<=1.e-4) count=std::max(count,span.count);
    }
    return count;
}
BoundaryPolicy BoundaryPolicy::adjusted(int extra, int consumed, bool one_wall) const
{
    auto adjusted=*this;
    for (auto &span : adjusted.m_spans) {
        span.count=std::max(0,span.count+extra);
        if (one_wall) span.count=std::min(1,span.count);
        span.count=std::max(0,span.count-consumed);
    }
    return adjusted;
}

BoundaryPolicy BoundaryPolicy::reinforce_slopes(double threshold, bool one_wall) const
{
    auto reinforced=*this;
    for (auto &span:reinforced.m_spans) {
        const double local_threshold=span.slope_threshold < 0. ? threshold : span.slope_threshold;
        // Eligibility is the requested count, before the top/first-layer cap.
        // A region requesting two or more walls keeps the existing cap behavior.
        span.slope_reinforced=span.count==1 && local_threshold>0. && span.slope_angle>local_threshold+1.e-7;
        if (one_wall) span.count=std::min(1,span.count);
        if (span.slope_reinforced) span.count=2;
    }
    return reinforced;
}

double BoundaryPolicy::transition_distance(const Point &point, size_t depth, double limit) const
{
    if (empty() || limit <= 0.) return limit;
    size_t start=0;
    m_index->nearest_distance(point,start);
    const auto &span=m_spans[start];
    if (int(depth)>=span.count) return 0.;
    const Vec2d delta=(span.line.b-span.line.a).cast<double>();
    const double length=delta.norm();
    const double along=length==0. ? 0. : std::clamp((point-span.line.a).cast<double>().dot(delta)/length,0.,length);
    double result=limit;
    for (int direction:{-1,1}) {
        double distance=direction<0 ? along : length-along;
        size_t current=start;
        while (distance<result) {
            const size_t next=direction<0 ? (current+m_spans.size()-1)%m_spans.size() : (current+1)%m_spans.size();
            if (next==start) break;
            const auto &a=m_spans[current].line, &b=m_spans[next].line;
            if (direction<0 ? a.a!=b.b : a.b!=b.a) break;
            if (int(depth)>=m_spans[next].count) { result=distance; break; }
            distance+=b.length();
            current=next;
        }
    }
    return result;
}

std::vector<std::pair<Line,int>> BoundaryPolicy::split(const Line &line) const
{
    std::vector<std::pair<Line,int>> result;
    if (line.a==line.b) return result;
    if (empty()) return result;
    const Point middle=along(line,0.5);
    size_t nearest=0;
    const double distance=std::sqrt(std::max(0.,m_index->nearest_distance(middle,nearest)));
    // Distance to the nearest boundary is Lipschitz continuous: a source that
    // can be nearest anywhere along this segment is within this search radius.
    const double radius=distance+line.length()+1.;
    std::vector<BoundarySpan> nearby;
    for (size_t index:m_index->within(middle,radius*radius)) nearby.push_back(m_spans[index]);
    const auto cuts=endpoints(line,nearby);
    for (size_t c=1;c<cuts.size();++c) {
        const int count=count_at(along(line,(cuts[c-1]+cuts[c])/2.));
        const Point end=along(line,cuts[c]);
        if (!result.empty() && result.back().second==count) result.back().first.b=end;
        else result.emplace_back(Line(along(line,cuts[c-1]),end),count);
    }
    return result;
}
} // namespace Slic3r::WallPainting

namespace Slic3r::WallPainting {
// Swept variable-width segment, used to derive infill from generated walls.
// Round end caps also close joins between adjacent segments of unequal widths.
void append_coverage(Polygons &out, const Line &line, coord_t width_a, coord_t width_b)
{
    const Vec2d d=(line.b-line.a).cast<double>();
    if (d.squaredNorm()==0. || width_a<=0 || width_b<=0) return;
    const Vec2d n=Vec2d(-d.y(),d.x()).normalized();
    const Vec2d a=line.a.cast<double>(), b=line.b.cast<double>();
    const double ra=width_a/2., rb=width_b/2.;
    out.emplace_back(Points{Point((a+n*ra).cast<coord_t>()),Point((a-n*ra).cast<coord_t>()),
                            Point((b-n*rb).cast<coord_t>()),Point((b+n*rb).cast<coord_t>())});
    for (const auto &end : {std::make_pair(a,ra),std::make_pair(b,rb)}) {
        Points circle;
        for (int i=0;i<16;++i) {
            const double angle=2.*PI*i/16.;
            circle.emplace_back((end.first+end.second*Vec2d(std::cos(angle),std::sin(angle))).cast<coord_t>());
        }
        out.emplace_back(std::move(circle));
    }
}
}
