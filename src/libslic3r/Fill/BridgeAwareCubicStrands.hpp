#ifndef slic3r_BridgeAwareCubicStrands_hpp_
#define slic3r_BridgeAwareCubicStrands_hpp_
#include "BridgeAwareCubic.hpp"
#include <tuple>

namespace Slic3r { namespace BridgeAwareCubic {

struct StrandBudget {
    size_t work_left = 5000000;
    // Bound logical catalog and query records separately; not a byte or RSS bound.
    size_t max_records = 200000;
};
struct StrandPlane {
    unsigned axis = 0, level = 0;
    int64_t numerator = 0;
    bool operator<(const StrandPlane &o) const { return std::tie(axis, level, numerator) < std::tie(o.axis, o.level, o.numerator); }
    bool operator==(const StrandPlane &o) const { return axis == o.axis && level == o.level && numerator == o.numerator; }
};
struct StrandEvent {
    int64_t slot = 0;
    bool shifted = false;
    bool operator<(const StrandEvent &o) const { return std::tie(slot, shifted) < std::tie(o.slot, o.shifted); }
    bool operator==(const StrandEvent &o) const { return slot == o.slot && shifted == o.shifted; }
};
struct StrandAtom {
    StrandEvent first, last;
    // A positive interval on an added dyadic face is interior to one tile.
    // Adjacent producers are never interchangeable owners of a whole run.
    CellId owner;
};
struct StrandRun {
    // Increasing local a=(axis+1)%3. points[i:i+2] belongs to atoms[i].
    // Keep stationary rounded atoms as source incidence, not extrusion claims.
    Points points;
    std::vector<StrandAtom> atoms;
};

class StrandProgram {
public:
    ~StrandProgram();
    StrandProgram(StrandProgram &&) noexcept;
    StrandProgram &operator=(StrandProgram &&) noexcept;
    StrandProgram(const StrandProgram &) = delete;
    StrandProgram &operator=(const StrandProgram &) = delete;
    std::vector<StrandPlane> planes(StrandBudget &) const;
    std::vector<StrandRun> runs(const StrandPlane &, StrandBudget &) const;
    bool contains_owner(const CellId &, StrandBudget &) const;
    // Remove or restore a catalog owner, touching at most three rows. Unknown
    // owners throw out_of_range. Exceptions leave the previous program intact.
    std::vector<StrandPlane> set_owner(const CellId &, bool active, StrandBudget &);
private:
    friend class Plan;
    struct Impl;
    std::unique_ptr<Impl> impl;
    StrandProgram(double z, double bulk_edge, const Vec3d &origin, const Matrix3d &to_world,
                  const std::vector<Refinement> &, std::function<void()> cancel, StrandBudget &);
};

}} // namespace Slic3r::BridgeAwareCubic
#endif
