#ifndef slic3r_BridgeAwareCubicValidation_hpp_
#define slic3r_BridgeAwareCubicValidation_hpp_

#include "../ExPolygon.hpp"
#include "../Polyline.hpp"

#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace Slic3r { namespace BridgeAwareCubicContact { struct PathContact; }}

namespace Slic3r { namespace BridgeAwareCubicValidation {

// Share across a complete verification pass, including its paths and layers.
// Exhaustion means the check is incomplete, never geometrically infeasible.
enum class WorkCategory { ContactInput, ContactSections, ContactCandidates, ContactMasks, Validation, Count };
struct WorkBudget {
    size_t remaining_work = 50000000;
    // Diagnostic totals only; these never alter admission or replenish work.
    std::array<size_t, size_t(WorkCategory::Count)> charged_work{};
    size_t unspent_at_limit = 0, rejected_request = 0;
    std::string describe_work(WorkCategory active = WorkCategory::Count) const;
};

struct PreparedGeometry;
using PreparedGeometryPtr = std::shared_ptr<const PreparedGeometry>;

enum class UnsupportedRunKind { TwoEndedBridge, OneSidedOverhang, Floating };

struct UnsupportedRun {
    size_t path_index = 0;
    // Approximate positions for diagnostics. length_mm is measured locally and
    // remains authoritative even if a microscopic run at a large accumulated
    // position makes these two double-precision positions round identically.
    double begin_mm = 0.;
    double end_mm = 0.;
    double length_mm = 0.;
    // Contact mode adds a conservative bound for anchor movement and length
    // changes of every traversed original segment. Zero for raw geometry.
    double uncertainty_mm = 0.;
    UnsupportedRunKind kind = UnsupportedRunKind::Floating;
    bool exceeds_gap = false;
};

struct VoidCell {
    ExPolygon geometry;
    double diameter_mm = 0.;
    bool exceeds_gap = false;
    bool closed = false;
};

struct Result {
    bool input_valid = true;
    bool completed = true;
    bool resource_limited = false;
    std::string failure_reason;
    double max_unsupported_mm = 0.;
    double max_void_diameter_mm = 0.;
    std::vector<UnsupportedRun> unsupported_runs;
    std::vector<VoidCell> void_cells;

    bool passed() const;
};

// Inputs describe actual material geometry at one physical contact plane.
// deposited_support must already exclude future, too-low, separated, or
// otherwise unusable deposition. The caller supplies any required bead-contact
// or material anchoring margins. Polygon edges alone are not usable anchors.
// material_domain is the complete actual model domain, not a demand or modifier
// clip; empty disables cell checks. demand only selects entire runs and cells,
// never clips a path or closes a void. Paths retain their emitted endpoints.
// The tolerance (at least one scaled-coordinate unit) is subtracted from
// max_gap_mm, never added to it. Unsupported one-sided paths are reported for
// the caller's overhang rule, and fail here. Fitted G2/G3 arcs are rejected;
// callers must supply segmented geometry with a known displacement bound.
Result validate(const ExPolygons &material_domain,
                const ExPolygons &deposited_support,
                const Polylines &required_paths,
                const ExPolygons &demand,
                double max_gap_mm,
                double geometry_tolerance_mm);

// The same verifier with direction-dependent bead contact certificates, one
// per full required path. Qualified finite physical contact lines may supply
// isolated arclength anchors; arbitrary polygon-boundary points still may not.
// deposited_support remains the undilated physical material at the common
// contact plane. Contact masks and line events never participate in cell walls.
Result validate_contacts(const ExPolygons &material_domain,
                         const ExPolygons &deposited_support,
                         const Polylines &required_paths,
                         const ExPolygons &demand,
                         double max_gap_mm,
                         double geometry_tolerance_mm,
                         const std::vector<BridgeAwareCubicContact::PathContact> &contacts,
                         WorkBudget *budget = nullptr);

// Immutable contact-mode cell geometry. Cache only for exactly the same
// deposited physical material, domain and contact plane; invalidate whenever
// emitted paths change. Selection and the requested gap are never cached.
PreparedGeometryPtr prepare_geometry(const ExPolygons &material_domain,
                                     const ExPolygons &deposited_support,
                                     WorkBudget *budget = nullptr);
Result validate_prepared_contacts(const PreparedGeometry &prepared,
                                  const Polylines &required_paths,
                                  const ExPolygons &demand,
                                  double max_gap_mm,
                                  double geometry_tolerance_mm,
                                  const std::vector<BridgeAwareCubicContact::PathContact> &contacts,
                                  WorkBudget *budget = nullptr);

}} // namespace Slic3r::BridgeAwareCubicValidation

#endif
