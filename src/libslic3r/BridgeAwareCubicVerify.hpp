#ifndef slic3r_BridgeAwareCubicVerify_hpp_
#define slic3r_BridgeAwareCubicVerify_hpp_

// Bridge aware cubic: the development checks (rewrite architecture, section 5). The verifier measures K1 on the final paths of
// an object, plane by plane; plan_warnings formats the plan's own statistics. Both are compiled into the slice only under
// SLIC3R_BAC_DEVELOPMENT_CHECKS, as warnings that change no path and stop no slice; the tests call them directly in any build.

#include "libslic3r.h"
#include "Point.hpp"

#include <functional>
#include <string>
#include <vector>

namespace Slic3r {

class PrintObject;

namespace BridgeAwareCubic {

class ObjectPlan;

struct PlaneReport
{
    size_t roof_layer = 0, plane_layer = 0;
    double z = 0.;                 // print_z of the plane's layer
    double gap = 0.;               // G
    double bead = 0.;              // r, the roof's narrowest bead over the void
    size_t openings = 0;
    double max_opening = 0.;       // the widest opening under the roof: max(hull diameter, strip length)
    size_t runs = 0;
    double max_run = 0.;           // the longest unsupported run of a roof path over the void
    size_t growth_runs = 0;        // information: runs of bridges over the void beyond the plan's footprint
    double max_growth_run = 0.;
    size_t bridge_free_ends = 0;   // K1(b): ends of internal bridge infill hanging over the void
    size_t other_free_ends = 0;    // reported
    bool   passed = true;
    std::string failure;           // the first failure, with its place
    Vec2d  where = Vec2d::Zero();  // mm, in the object's frame
};

struct VerificationReport
{
    std::vector<PlaneReport> planes;
    bool passed() const;
};

// K1 on the final paths of the object, on the planes of its plan (none when the object has no plan).
VerificationReport verify(const PrintObject &object, const std::function<void()> &cancel);
// The same, on the planes of a plan built separately (the test oracle measures a Cubic object with the planes its surfaces define).
VerificationReport verify(const PrintObject &object, const ObjectPlan &plan, const std::function<void()> &cancel);

// The plan checks of 5.2, formatted: one message per finding, naming the object, the region's settings, the heights and the place.
std::vector<std::string> plan_warnings(const PrintObject &object);

} // namespace BridgeAwareCubic
} // namespace Slic3r

#endif // slic3r_BridgeAwareCubicVerify_hpp_
