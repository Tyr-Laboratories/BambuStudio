#ifndef slic3r_WallPainting_hpp_
#define slic3r_WallPainting_hpp_

#include "Line.hpp"
#include "Polygon.hpp"
#include "ExPolygon.hpp"
#include <functional>
#include <memory>
#include <optional>

namespace Slic3r {
class PrintObject;
class PrintRegionConfig;
namespace WallPainting {

// Lines follow the material boundary (solid on their left). All states are
// retained: an unpainted face must occlude paint on a nearby, unrelated face.
struct PaintedLine {
    Line line;
    size_t volume_id = 0;
    bool painted = false;
    // -1 inherits the region setting, including modifier overrides.
    int part_wall_loops = -1;
    // Inclination of the transformed surface, measured from vertical.
    double slope_angle = 0.;
};
using LayerPaint = std::vector<PaintedLine>;
std::vector<LayerPaint> slice_paint(const PrintObject &object, const std::function<void()> &cancel);

void append_coverage(Polygons &out, const Line &line, coord_t width_a, coord_t width_b);

struct BoundarySpan {
    Line line;
    int count = 0;
    size_t source = 0;
    bool painted = false;
    double slope_angle = 0.;
    double slope_threshold = -1.;
    bool slope_reinforced = false;
};

struct CountRegion {
    ExPolygons area;
    int painted_count;
    // A negative value inherits the main region's threshold; zero disables it.
    double slope_threshold = -1.;
};

// The policy describes real material boundaries, never walls at paint borders.
// It is immutable during parallel wall generation.
class BoundaryPolicy {
public:
    BoundaryPolicy() = default;
    explicit BoundaryPolicy(std::vector<BoundarySpan> spans);
    static BoundaryPolicy bind(const Polygons &outline, const LayerPaint &paint,
                               int normal_count, int painted_count, double tolerance,
                               const std::vector<CountRegion> &count_regions = {});
    bool empty() const { return m_spans.empty(); }
    int maximum() const;
    std::optional<int> uniform_count() const;
    int count_at(const Point &point) const;
    // Arc distance along this boundary to a span that does not request depth.
    // Unrelated nearby contours cannot shorten a count-change transition.
    double transition_distance(const Point &point, size_t depth, double limit) const;
    const std::vector<BoundarySpan> &spans() const { return m_spans; }
    BoundaryPolicy adjusted(int extra, int consumed = 0, bool one_wall = false) const;
    BoundaryPolicy reinforce_slopes(double threshold, bool one_wall = false) const;
    // Split a line at projected paint boundaries. Coordinates remain on the
    // candidate line; these are extrusion spans, not closed modifier polygons.
    std::vector<std::pair<Line, int>> split(const Line &line) const;
private:
    std::vector<BoundarySpan> m_spans;
    struct Index;
    std::shared_ptr<const Index> m_index;
};
} // namespace WallPainting
} // namespace Slic3r
#endif
