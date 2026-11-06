#ifndef slic3r_BridgeAwareCubicDomainProbe_hpp_
#define slic3r_BridgeAwareCubicDomainProbe_hpp_
// Diagnostic experiment only. Each make_fills atomically acquires one shared
// observer lease. Deinstallation prevents new leases without invalidating an
// already running foreign fill. No observer selects or produces geometry.
#include "BridgeAwareCubic.hpp"
#include <atomic>
#include <memory>
namespace Slic3r {
class Layer;
namespace BridgeAwareCubic {
struct DomainProbe {
    virtual ~DomainProbe() = default;
    virtual bool begin_layer(const Layer &) = 0;
    virtual void surface(const Layer &, size_t emitter, const Plan &, double z, double width,
        const ExPolygons &overlap_components, const ExPolygons *actual_inset_material,
        const Polylines &actual_mixed_output) = 0;
};
// Atomically accessed ONLY through the C++17 shared_ptr free functions.
extern std::shared_ptr<DomainProbe> diagnostic_domain_probe;
}}
#endif
