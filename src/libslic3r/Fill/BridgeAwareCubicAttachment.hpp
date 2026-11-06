#ifndef slic3r_BridgeAwareCubicAttachment_hpp_
#define slic3r_BridgeAwareCubicAttachment_hpp_

#include "../ExPolygon.hpp"
#include "../Polyline.hpp"
#include "BridgeAwareCubic.hpp"

#include <cstddef>
#include <optional>
#include <vector>

namespace Slic3r { namespace BridgeAwareCubicAttachment {

struct TerminalAirLengths {
    // Complete terminal runs from the caller's direction-aware verifier.
    // An absent value means that endpoint already has qualified support.
    std::optional<double> start_mm;
    std::optional<double> end_mm;
};

struct TerminalAnchorIntervals {
    // Skip this many positive interior contact intervals along each outgoing
    // terminal ray. Callers bound retries and certify every serialized path.
    size_t start_skip = 0;
    size_t end_skip = 0;
};

// Extend actual free endpoints along their terminal tangents into usable
// preceding deposition. The caller supplies contact-qualified support and the
// complete permitted footprint of the emitting region, including its holes.
// Support is never enlarged. Every original point is retained, and only newly
// deposited bead area is checked against allowed_bead_domain. The complete
// repaired path must pass the ordinary two-anchor/gap validator.
//
// Call only for independent open paths, never a loop or an interior multipath
// join. Fitted arcs, zero-length terminal segments, and paths with more than
// 1024 points are rejected; the point cap bounds footprint construction before
// the verifier checks its generated geometry and remaining work. nullopt
// means either no repair was necessary or no bounded valid repair was found;
// the caller must still validate its final emitted paths.
// preferred_anchor_inset_mm changes only candidate placement: it requests at
// least this inset beyond first contact, capped at the interval midpoint. The
// existing coordinate margin still applies. If the deeper footprint exceeds
// the allowed domain, the original shallow placement is tried once. It does
// not enlarge support, override the gap, or replace final serialized checks.
// Directional contact callers supply the first/last segment's support masks
// separately. A mask from another segment direction cannot anchor an endpoint
// search. They must also supply terminal_air_lengths from the existing full
// direction-aware verification; the helper does not reclassify bent paths
// using a single endpoint's mask. The union is a preliminary path check only.
// anchor_intervals allows a caller to try a later real anchor when an earlier
// interval cannot retain contact after output rounding. Skipping never widens
// support or increases the complete terminal air-run gap budget.
std::optional<Polyline> attach_endpoints(const Polyline &original,
                                         const ExPolygons &qualified_prior_support,
                                         const ExPolygons &allowed_bead_domain,
                                         double width_mm,
                                         double max_gap_mm,
                                         double geometry_tolerance_mm,
                                         double preferred_anchor_inset_mm = 0.,
                                         const ExPolygons *start_anchor_support = nullptr,
                                         const ExPolygons *end_anchor_support = nullptr,
                                         const TerminalAirLengths *terminal_air_lengths = nullptr,
                                         const TerminalAnchorIntervals *anchor_intervals = nullptr);

struct ResolvedChain {
    Polyline polyline;
    // Input indices in application order. Callers retain the corresponding
    // cut-selection and terminal-owner metadata instead of flattening it into
    // alternative owners for the entire final strand.
    std::vector<size_t> applied_indices;
};

// Resolve records already filtered to the current layer and active owners,
// gathered from every storage plan. Point
// order, emitting region, width and height must match exactly; dimensions must
// therefore use the actual emitted precision, including in closure previews.
// Storage order does not choose a branch or limit the number of replay steps.
// No match returns the original path and an empty lineage. Relevant duplicate
// decisions are rejected even with equal replacements: their provenance may
// differ, so selecting one record would discard metadata. Matched chains must
// contain linear paths with at least two points.
//
// Throws std::invalid_argument for invalid dimensions or matched geometry,
// ambiguity, no-ops and cycles; std::runtime_error when the work cap is exceeded.
// The cap includes records, point comparisons, fitting data and output copying.
// Failure never returns a partially resolved path or changes an input record.
ResolvedChain resolve_chain(const Polyline &original,
                            double width,
                            double height,
                            size_t emitting_region,
                            const std::vector<BridgeAwareCubic::LayerAttachment> &attachments,
                            size_t max_work = 1000000);

}} // namespace Slic3r::BridgeAwareCubicAttachment

#endif
