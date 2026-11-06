#ifndef slic3r_FillBridgeAwareCubic_hpp_
#define slic3r_FillBridgeAwareCubic_hpp_

#include "FillRectilinear.hpp"
#include "BridgeAwareCubic.hpp"
#include "../ClipperUtils.hpp"

namespace Slic3r {

class FillBridgeAwareCubic : public FillCubic
{
public:
    const BridgeAwareCubic::Plan *plan = nullptr;
    const ExPolygons *material_domain = nullptr;
    double extrusion_width = 0.;

    Fill *clone() const override { return new FillBridgeAwareCubic(*this); }
    bool no_sort() const override { return plan != nullptr; }

    Polylines fill_surface(const Surface *surface, const FillParams &params) override
    {
        // The preliminary bridge-direction pass runs before support demand is
        // known. It uses the identical ordinary Cubic lattice as the plan.
        if (plan == nullptr)
            return FillCubic::fill_surface(surface, params);

        Polylines result;
        // Ordinary Cubic adds boundary anchors after its half-spacing inset.
        // These complete partition faces need their straight ends to reach the
        // actual fill boundary instead, so they join the deposited inner wall.
        for (const ExPolygon &domain : offset_ex(surface->expolygon, scale_(overlap)))
            append(result, plan->lines(z, domain));
        // A fill boundary can also be a real exterior or cavity, including
        // models without perimeters. Keep the whole bead inside that material.
        if (material_domain != nullptr)
            result = intersection_pl(result, offset_ex(*material_domain, -scale_(.5 * extrusion_width)));
        return result;
    }
};

} // namespace Slic3r

#endif
