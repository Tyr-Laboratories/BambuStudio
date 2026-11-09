#include "FillBridgeAwareCubic.hpp"

namespace Slic3r {

void FillBridgeAwareCubic::bind(const BridgeAwareCubic::ObjectPlan &plan, const Layer &layer)
{
    m_plan  = &plan;
    m_layer = &layer;
}

Polylines FillBridgeAwareCubic::fill_surface(const Surface *surface, const FillParams &params)
{
    // Stage 1 of the build order: no plan exists yet, so every island prints as Cubic.
    return FillCubic::fill_surface(surface, params);
}

} // namespace Slic3r
