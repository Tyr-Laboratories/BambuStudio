#ifndef slic3r_FillBridgeAwareCubic_hpp_
#define slic3r_FillBridgeAwareCubic_hpp_

// Bridge aware cubic: Cubic's lattice, plus the lines of the object's plan under roofs. The filler prints Cubic's lines
// through FillCubic where the plan has nothing for an island's group, and otherwise the closed-form lattice with the plan's
// strands and courses (rewrite architecture, section 3.8).

#include "FillRectilinear.hpp"

namespace Slic3r {

class Layer;

namespace BridgeAwareCubic { class ObjectPlan; }

class FillBridgeAwareCubic : public FillCubic
{
public:
    Fill* clone() const override { return new FillBridgeAwareCubic(*this); }
    ~FillBridgeAwareCubic() override = default;

    // The object's plan and the layer being filled; set by Layer::make_fills and by the anchoring preview before fill_surface.
    void bind(const BridgeAwareCubic::ObjectPlan &plan, const Layer &layer);

    Polylines fill_surface(const Surface *surface, const FillParams &params) override;

private:
    const BridgeAwareCubic::ObjectPlan *m_plan  = nullptr;
    const Layer                        *m_layer = nullptr;
};

} // namespace Slic3r

#endif // slic3r_FillBridgeAwareCubic_hpp_
