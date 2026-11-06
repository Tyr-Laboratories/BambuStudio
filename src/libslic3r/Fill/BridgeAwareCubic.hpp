#ifndef slic3r_BridgeAwareCubic_hpp_
#define slic3r_BridgeAwareCubic_hpp_

#include "../BoundingBox.hpp"
#include "../ExPolygon.hpp"
#include "../Polyline.hpp"

#include <cstddef>
#include <functional>
#include <memory>
#include <vector>

namespace Slic3r { namespace BridgeAwareCubic {

struct Demand {
    ExPolygons footprint;
    double z = 0.;
    double gap = 0.;
};

struct Parameters {
    // Common object bounds: their XY center fixes the ordinary Cubic origin.
    BoundingBoxf3 bounds;
    double flow_spacing = 0.;
    double density = 0.; // Fractional sparse density, strictly between 0 and 1.
    double angle = 0.;   // Configured infill direction, in radians.
    std::function<void()> throw_if_canceled;
    size_t max_nodes = 1000000;
    unsigned max_depth = 24;
};

struct Statistics {
    size_t root_cells = 0;
    size_t nodes = 0;
    size_t splits = 0;
    size_t leaves = 0;
    unsigned max_depth = 0;
};

struct LeafSection {
    size_t node_id = 0;
    unsigned depth = 0;
    double edge = 0.;
    // Complete physical cell, before clipping to printable material. A domain
    // edge must never be mistaken for one of the cell's inherited walls.
    Polygon polygon;
};

// Geometric construction, not a certificate of deposited-material support.
// Builds once from current demand; no historical/provisional branches survive.
class Plan {
public:
    Plan(Parameters parameters, std::vector<Demand> demands);
    ~Plan();
    Plan(Plan&&) noexcept;
    Plan& operator=(Plan&&) noexcept;
    Plan(const Plan&) = delete;
    Plan& operator=(const Plan&) = delete;

    // No same-layer anchoring order is promised. The deposition verifier must
    // establish previous-layer support or impose its own dependency ordering.
    Polylines lines(double z, const ExPolygon &domain) const;
    Polylines refinement_lines(double z, const ExPolygon &domain) const;
    // Complete selected-parent sections while their added midplanes exist.
    // Callers add any preceding-layer scaffold required by contact validation.
    ExPolygons refinement_area(double z) const;
    std::vector<LeafSection> leaf_sections(double z) const;

    const Parameters& parameters() const;
    const std::vector<Demand>& demands() const;
    const Statistics& statistics() const;
    double bulk_cell_edge() const;

private:
    struct Impl;
    std::unique_ptr<const Impl> m_impl;
};

}} // namespace Slic3r::BridgeAwareCubic

#endif
