#ifndef slic3r_BridgeAwareCubicSupport_hpp_
#define slic3r_BridgeAwareCubicSupport_hpp_

#include "../Polyline.hpp"
#include "../ExPolygon.hpp"

#include <optional>
#include <set>
#include <functional>
#include <string>

namespace Slic3r {
class Layer;
class PrintObject;
namespace BridgeAwareCubicValidation { struct UnsupportedRun; }
namespace BridgeAwareCubic {
struct SerializedXY;
struct LayerAttachment;

// Reset generation-local provenance before any previous fill entity is freed.
// Optional capture is bounded across the object and never changes deposition.
void begin_layer_emissions(Layer &layer);

// Apply immutable, once-per-layer closure deposition and saved model-path
// attachments after the ordinary fill groups have finished generating.
void apply_layer_paths(Layer &layer);

// Diagnostic comparison against the established construction classifier on
// the same final geometry. This performs both collections and is deliberately
// separate from slicing timings. Exceptions and cancellation propagate.
struct EmissionCollectionAudit {
    bool equal = false;
    // Counts on the optimized traversal only; reference_emitters excludes
    // the audit's mandatory second, independent reference traversal.
    size_t ledger_emitters = 0, reference_emitters = 0, ledger_construction_paths = 0;
    std::string mismatch;
};
EmissionCollectionAudit audit_construction_collection(const PrintObject &object);

// Choose only automatic bridge orientations from regenerated candidate paths
// before BAC plans and model-support demands are frozen. The callback rebuilds
// fill paths after each bounded candidate; explicit bridge controls are kept.
void orient_automatic_bridges(PrintObject &object, const std::function<void()> &regenerate);

// The sparse anchoring preview must see the same extra physical partitions.
Polylines layer_closure_lines(const Layer &layer);

// Endpoint additions retain their original emitter. Adjacent modifier
// material is usable only when it participates and has equal material/flow.
ExPolygons endpoint_attachment_domain(const Layer &layer, size_t emitting_region);

// Match clipped source fragments to their actual partition axes without
// extrapolating a rounded short fragment over a much longer owner interval.
Polylines matching_partition_segments(const Polylines &candidates, const Polylines &partitions);

// The matched pieces must be actual intersections from this source path.
// Rounding allowance may complete positive overlap, never create a producer
// for an entirely unmatched short ordinary strand.
bool complete_construction_match(const Polyline &path, const Polylines &matched, double tolerance_mm);

// Provenance only: include the integer polygon-clipping roundoff around a
// finite-width owned strand. When original source segments are supplied, use
// their directions for the projected owned intervals, never a rounded short
// fragment's direction. This envelope never supplies physical support.
ExPolygons partition_provenance_footprint(const Polylines &owned, double width_mm, double tolerance_mm,
                                         const Polylines *source_segments = nullptr);

// Retain logical demand components under output rounding. The source demand
// is expanded only by the exact maximum vertex displacement of its full
// paths, plus integer polygon roundoff, in the phase(Point(0))-local frame.
ExPolygons serialized_demand(const ExPolygons &source, const Polylines &paths, const SerializedXY &phase);

// A finite computation window for the actual complement of deposited
// support. Its boundary stays open; model/region seams never become walls.
ExPolygons support_topology_domain(const ExPolygons &support, const ExPolygons &all_demands);

// nullopt retains the complete path obligation. A restricted cut mask is
// valid only when every applicable producer requires that restricted scope.
std::optional<ExPolygons> merged_attachment_selection(bool ordinary_full_path,
    const std::vector<LayerAttachment> &active, const std::set<size_t> &applicable_lineage);

// Diagnostics use arclength on the verified emitted path, not on the source
// path whose segment lengths and retained vertices can differ after export.
ExPolygons unsupported_interval_demand(const Polyline &verified_path, double begin_mm, double end_mm,
                                       double width_mm, double tolerance_mm, const ExPolygons &original_demand,
                                       const Polyline *source_path = nullptr);

// Select only violating complete air runs, retaining each source path's own
// finite-width demand. An empty verified_paths list uses source arclength.
// Returned fragments remain separate until the caller combines cell repairs.
// complete_selected_runs uses each demand only to select its intersecting run;
// the returned footprint then covers that complete run on its original bead.
ExPolygons select_unsupported_run_demands(const Polylines &source_paths, const Polylines &verified_paths,
    const std::vector<double> &widths_mm, const std::vector<ExPolygons> &path_demands,
    const std::vector<BridgeAwareCubicValidation::UnsupportedRun> &runs, double tolerance_mm,
    bool complete_selected_runs = false);

// Select only newly cut bulk endpoints beside an emitted solid patch. Callers
// retain the full extrusion path when checking every selected air interval.
ExPolygons solid_cut_endpoint_demand(const Polyline &path, const ExPolygons &solid_area,
                                     double width_mm, double tolerance_mm);

// Provenance for a solid-induced cut is exactly its selected terminal part
// of the real source bead, plus two integer units of Boolean roundoff. The
// selector may be retained from an attachment's original cut endpoint.
ExPolygons solid_cut_provenance_footprint(const Polyline &path, const ExPolygons &selection, double width_mm);

} // namespace BridgeAwareCubic
} // namespace Slic3r

#endif
