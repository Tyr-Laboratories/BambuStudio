#ifndef slic3r_BridgeAwareCubicPilot_hpp_
#define slic3r_BridgeAwareCubicPilot_hpp_
#include "BridgeAwareCubicStrands.hpp"
#include "../ExtrusionEntity.hpp"

namespace Slic3r { namespace BridgeAwareCubic {
struct PilotCourse {
    StrandPlane plane;
    Points points;
    std::vector<ClippedStrandAtom> atoms;
};
// Source incidence travels with the actual emitted child through a trial swap
// or collection clone. Optional metadata cannot change what this object prints.
struct PilotPath final : ExtrusionPath {
    const PilotCourse source;
    const std::shared_ptr<const void> plan_identity;
    PilotPath(const ExtrusionPath &p, PilotCourse s, std::shared_ptr<const void> identity) :
        ExtrusionPath(p), source(std::move(s)), plan_identity(std::move(identity)) {}
    bool partition() const { return !source.atoms.empty(); }
    ExtrusionEntity *clone() const override { return new PilotPath(*this); }
    ExtrusionEntity *clone_move() override { return new PilotPath(std::move(*this)); }
};
inline std::vector<PilotCourse> pilot_courses(StrandPlane plane, const ClippedStrandRun &run, const SerializedXY &phase)
{
    if (run.points.size() != run.atoms.size()+1) throw std::logic_error("Invalid clipped strand incidence");
    std::vector<PilotCourse> result;
    bool restart = true;
    for (size_t i=0; i<run.atoms.size(); ++i) {
        if (phase(run.points[i]) == phase(run.points[i+1])) { restart=true; continue; }
        if (restart) { result.push_back({plane,{run.points[i]}, {}}); restart=false; }
        result.back().points.push_back(run.points[i+1]);
        result.back().atoms.push_back(run.atoms[i]);
    }
    return result;
}
}} // namespace
#endif
