#ifndef slic3r_BridgeAwareCubic_hpp_
#define slic3r_BridgeAwareCubic_hpp_

#include "../BoundingBox.hpp"
#include "../ExPolygon.hpp"
#include "../Polyline.hpp"
#include "BridgeAwareCubicSerialization.hpp"
#include <optional>

#include <cstddef>
#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <limits>
#include <utility>
#include <vector>

namespace Slic3r { namespace BridgeAwareCubicValidation { struct WorkBudget; }}

namespace Slic3r { namespace BridgeAwareCubic {

class StrandProgram;
struct StrandBudget;

// Stable only within one lattice's bounds/origin, spacing, density and angle.
struct CellId {
    std::array<int64_t, 3> address {{ 0, 0, 0 }};
    unsigned depth = 0;
    bool operator==(const CellId &other) const { return address == other.address && depth == other.depth; }
    bool operator<(const CellId &other) const { return depth == other.depth ? address < other.address : depth < other.depth; }
};

struct Demand {
    ExPolygons footprint;
    double z = 0.;
    double gap = 0.;
    bool dependency = false;
    // Alternative actual producers of this dependent strand. A containing
    // ancestor is not an owner unless it produces the strand itself.
    std::vector<CellId> owners;
};

struct LayerClosure {
    double z = 0.;
    Polylines paths;
    ExPolygons reservation;
    // Empty means a persistent direct closure, otherwise alternative owners.
    std::vector<CellId> owners;
    size_t emitting_region = std::numeric_limits<size_t>::max(); // Default: plan's storage region.
};

struct LayerAttachment {
    double z = 0.;
    size_t emitting_region = 0;
    Polyline original, replacement;
    double width = 0., height = 0.;
    std::vector<CellId> owners;
    bool solid_cut = false;
    // Empty selects the whole actual path; otherwise preserve the original
    // terminal-only obligation when a solid region clipped inherited infill.
    ExPolygons demand_selection;
};

struct Refinement {
    CellId id;
    bool solid = false;
    double bottom_z = 0., top_z = 0.;
    std::vector<size_t> demand_indices;
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
    // Solid regions are construction obligations, not deposited material.
    // The consuming filler/validator must construct and certify them.
    bool allow_solid_fallback = false;
    std::optional<SerializedXY> canonical_phase; // immutable pilot placement, absent in normal slicing
};

struct Statistics {
    size_t root_cells = 0;
    size_t nodes = 0;
    size_t splits = 0;
    size_t leaves = 0;
    unsigned max_depth = 0;
    size_t solid_cells = 0;
};

struct LeafSection {
    size_t node_id = 0;
    unsigned depth = 0;
    double edge = 0.;
    // Complete physical cell, before clipping to printable material. A domain
    // edge must never be mistaken for one of the cell's inherited walls.
    Polygon polygon;
    CellId cell_id;
    bool solid = false;
};

struct ProducerInterval {
    CellId owner;
    // Finite [first,last] parameters on the original merged source axis,
    // before domain clipping or XY serialization. A touching owner is not
    // necessarily an alternative producer of the complete emitted path.
    double first = 0., last = 0.;
};

struct LineSource {
    enum class Kind { Bulk, Partition };
    Kind kind = Kind::Bulk;
    Point canonical_a, canonical_b;
    std::vector<ProducerInterval> producers;
};

struct TaggedLineBatch {
    Polylines paths;
    std::vector<size_t> source_for_path;
    std::vector<LineSource> sources;
    // Complete requires exact source IDs for every output plus ALL original
    // merged refinement sources at this Z, including those wholly clipped out
    // of this domain. Source kind is generation provenance, not legacy added-
    // mask classification or a deposited-material support certificate.
    bool complete = false;
};

struct ReferenceLineBatch {
    std::shared_ptr<const void> plan_identity;
    double z = 0.;
    BoundingBox bounds;
    // Exact broad-reference refinement output and original source attribution.
    // The complete raw source table includes rows clipped out of these bounds.
    Polylines paths;
    std::vector<size_t> source_for_path;
    std::vector<LineSource> sources;
    // Independently grouped finite fragments, identical to partition_lines().
    std::vector<std::pair<CellId, Polylines>> partitions;
    bool complete = false;
};

// Geometric construction, not a certificate of deposited-material support.
// Builds once from current demand; no historical/provisional branches survive.
class Plan {
public:
    Plan(Parameters parameters, std::vector<Demand> demands,
         std::vector<LayerClosure> closures = {}, std::vector<LayerAttachment> attachments = {});
    ~Plan();
    Plan(Plan&&) noexcept;
    Plan& operator=(Plan&&) noexcept;
    Plan(const Plan&) = delete;
    Plan& operator=(const Plan&) = delete;

    // Cubic paths only: solid reservations and layer closures emit separately.
    // No same-layer anchoring order is promised. The deposition verifier must
    // establish previous-layer support or impose its own dependency ordering.
    Polylines lines(double z, const ExPolygon &domain) const;
    // Raw added partition strands, before clipping, flow and physical validation.
    StrandProgram strand_program(double z, StrandBudget &budget) const;
    Polylines inherited_bulk(double z, const ExPolygon &domain) const;
    // Optional provenance for the identical lines() geometry and order.
    // Metadata admission is bounded by capacity_units (at most 200000,
    // measured in sizeof(Point) storage units) and 20000 records. Whole vector
    // capacities and temporary relocation storage count toward admission;
    // failure returns ordinary geometry with complete=false
    // and empty metadata, retaining any shared work already spent. Active
    // solid/closure layers are ineligible in this initial implementation.
    // Canonical refinement axes and finite owner ranges remain in the source
    // frame: form finite fragment endpoints BEFORE pointwise serialization.
    TaggedLineBatch tagged_lines(double z, const ExPolygon &domain, size_t capacity_units = 200000,
                                BridgeAwareCubicValidation::WorkBudget *budget = nullptr) const;
    // Optional-only broad reference for classification/ownership. Uses the
    // same rectangle, clipping batch and finite-owner arithmetic as the legacy
    // refinement_lines()/partition_lines() queries. All generated buffers are
    // admitted before allocation. The quota covers those explicit buffers,
    // not Clipper's internal allocator or total process RSS; native clipping
    // retains its existing finite input/work/coordinate limits and runs
    // sequentially with ordinary fill clipping. Failure returns an empty
    // incomplete batch; spent shared work remains charged, cancellation propagates.
    ReferenceLineBatch reference_lines(double z, const BoundingBox &bounds, size_t capacity_units = 200000,
                                      BridgeAwareCubicValidation::WorkBudget *budget = nullptr) const;
    Polylines refinement_lines(double z, const ExPolygon &domain) const;
    Polylines partition_lines(const CellId &owner, double z, const ExPolygon &domain) const;
    // Compute the complete emitted batch once before assigning its surviving
    // finite intervals to their active producers. Prefer this for layer loops.
    std::vector<std::pair<CellId, Polylines>> partition_lines(double z, const ExPolygon &domain) const;
    Polylines closure_lines(double z) const;
    std::vector<LayerClosure> closures(double z) const;
    bool has_closures(double z) const;
    std::vector<LayerAttachment> attachments(double z) const;
    // Borrow matching records in their exact storage order without allocating
    // a copy. False means the callback requested an early stop.
    bool visit_attachments(double z, const std::function<bool(const LayerAttachment &)> &visitor) const;
    // Complete selected-parent sections while their added midplanes exist.
    // Callers add any preceding-layer scaffold required by contact validation.
    ExPolygons refinement_area(double z) const;
    std::vector<LeafSection> leaf_sections(double z) const;
    std::vector<LeafSection> solid_sections(double z) const;
    ExPolygons solid_area(double z) const;

    const std::vector<Refinement>& refinements() const;
    const std::vector<size_t>& active_demands() const;
    // Exact immutable trial copy, including accepted collapses.
    Plan clone() const;
    // Retains an opaque lifetime-owned nonce after the Plan is destroyed,
    // without retaining its geometry or callback captures. Clones share the
    // token; every rebuild receives a new identity. A token identifies exact
    // construction geometry, not a deposited-material certificate.
    std::shared_ptr<const void> immutable_identity() const noexcept;
    // Rebuild after external dependency filtering while preserving accepted
    // collapses and all unreplaced payloads. These remain uncertified trials.
    // Use a fresh constructor for new target geometry that may need regrowth.
    Plan with_demands(std::vector<Demand> demands) const;
    Plan with_payloads(std::vector<LayerClosure> closures, std::vector<LayerAttachment> attachments) const;
    // Uncertified trial: remove this exact split/solid and all descendants,
    // then rebuild dependency reachability from direct demand. Inherited
    // walls remain. Previous accepted collapses persist across trials.
    Plan collapsed(const CellId &id) const;
    // The callback must validate this exact trial's final deposited paths,
    // gaps, closure, contact and ordering without repairing the trial.
    // A work-limit failure cannot be reported as an irreducible result.
    Plan coarsened(const std::function<bool(const Plan&)> &validate, size_t max_trials = 10000) const;

    const Parameters& parameters() const;
    const std::vector<Demand>& demands() const;
    const Statistics& statistics() const;
    double bulk_cell_edge() const;

private:
    struct Impl;
    explicit Plan(std::shared_ptr<const Impl> implementation);
    std::shared_ptr<const Impl> m_impl;
};

}} // namespace Slic3r::BridgeAwareCubic

#endif
