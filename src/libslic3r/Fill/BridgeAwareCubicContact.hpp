#ifndef slic3r_BridgeAwareCubicContact_hpp_
#define slic3r_BridgeAwareCubicContact_hpp_

#include "../ExtrusionEntity.hpp"
#include "../ExPolygon.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace Slic3r { namespace BridgeAwareCubicValidation { struct WorkBudget; }}

namespace Slic3r { namespace BridgeAwareCubicContact {

enum class FootprintMode { Full, ContactsOnly };

struct Bead {
    const ExtrusionPath *path = nullptr;
    double print_z = 0.;
};

// Parameters on the original segment, in [0,1]. Equal endpoints represent a
// finite transverse contact line, not an isolated physical contact point.
struct ExactInterval {
    int64_t begin_numerator = 0;
    int64_t begin_denominator = 1;
    int64_t end_numerator = 0;
    int64_t end_denominator = 1;
    // Maximum movement along the upper path relative to this nominal event.
    // Nonzero only for a contact witness certified through XY perturbations.
    double position_uncertainty_mm = 0.;
};

struct SegmentContact {
    size_t segment_index = 0;
    ExPolygons centerline_support;
    std::vector<ExactInterval> line_contacts;
};

struct PathContact {
    bool input_valid = true;
    bool completed = true;
    bool resource_limited = false;
    std::string failure_reason;
    double contact_z = 0.;
    // Per original emitted endpoint; the verifier bounds accumulated segment
    // length changes separately from line-anchor position uncertainty.
    double coordinate_uncertainty_mm = 0.;
    // Undilated lower-bead sections at contact_z. These alone describe cell
    // walls. Zero-width sections remain lines, never inflated material.
    ExPolygons physical_footprint;
    Polylines physical_lines;
    // One entry for each original positive-length upper segment. These are
    // direction-dependent contact certificates and must never close cells.
    std::vector<SegmentContact> segments;
};

// Rounded-rectangle bead section, using actual emitted width and height.
// The top and bottom flat widths are width-height; width==height is circular.
// Returns zero outside the closed Z interval; invalid dimensions return NaN.
double section_width_mm(double width, double height, double print_z, double z);

// Immutable owned lower sections for one exact (upper_print_z, upper_height)
// pair. Validate every deposited bead before selecting eligible sections. A
// failed preparation retains only its failure status, never partial evidence.
// Keep at most one prepared entry alive per checking pass: each query admits
// its allocations together with this entry's owned geometry and conservative
// validated-input admission under the existing vertex bound. Preparation and
// every query share the pass budget; input admission is not charged twice.
struct PreparedLower;
using PreparedLowerPtr = std::shared_ptr<const PreparedLower>;
PreparedLowerPtr prepare_lower(const std::vector<Bead> &deposited,
                               double upper_print_z, float upper_height,
                               BridgeAwareCubicValidation::WorkBudget *budget = nullptr);

// The upper elevation and float height must exactly match the prepared key.
// Upper path validity, width, direction and XY tolerance remain per query.
PathContact footprint(const PreparedLower &deposited,
                      const ExtrusionPath &upper,
                      double upper_print_z,
                      double xy_tolerance_mm,
                      FootprintMode mode = FootprintMode::Full,
                      BridgeAwareCubicValidation::WorkBudget *budget = nullptr);

// All physical material in one result belongs to one common plane. Heights
// differing only by float representation error are snapped at a tangent plane;
// xy_tolerance_mm is never used to bridge a vertical separation. The supplied
// paths must already have been emitted before upper, and are not retained.
// Positive tolerance requires line events to survive endpoint rounding: they
// must be interior to an upper segment, and zero-width collinearity alone is
// insufficient. Zero tolerance exposes the exact geometric line-contact rule.
// Piecewise straight, finite stadium/circular sections are supported. Bounds
// and work exhaustion fail closed. Upper w<h and fitted arcs fail closed;
// valid lower gap-fill beads with w<h and stationary serialized deposition
// contribute no support evidence and cannot select the contact plane.
// ContactsOnly preserves every contact certificate but omits physical wall
// polygons/lines; use only when those outputs will not participate in cells.
// A shared budget bounds a complete collection/check pass across all paths.
PathContact footprint(const std::vector<Bead> &deposited,
                      const ExtrusionPath &upper,
                      double upper_print_z,
                      double xy_tolerance_mm,
                      FootprintMode mode = FootprintMode::Full,
                      BridgeAwareCubicValidation::WorkBudget *budget = nullptr);

}} // namespace Slic3r::BridgeAwareCubicContact

#endif
