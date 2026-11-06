#ifndef slic3r_BridgeAwareCubicDependencies_hpp_
#define slic3r_BridgeAwareCubicDependencies_hpp_

#include "BridgeAwareCubic.hpp"

#include <cstddef>
#include <functional>
#include <vector>

namespace Slic3r { namespace BridgeAwareCubic { namespace Dependencies {

// Indices belong to a caller-owned, stable demand table, not a compacted
// Plan::demands() vector. Remap refinement reasons after filtering a plan.
struct DemandKey {
    size_t region = 0;
    size_t index = 0;
    bool operator==(const DemandKey &other) const { return region == other.region && index == other.index; }
    bool operator<(const DemandKey &other) const
    { return region == other.region ? index < other.index : region < other.region; }
};

// Cell identities are local to a region's lattice. Only the exact surviving
// physical producer counts; a surviving ancestor cannot replace its child.
struct SourceOwner {
    size_t region = 0;
    CellId cell;
    bool operator==(const SourceOwner &other) const { return region == other.region && cell == other.cell; }
    bool operator<(const SourceOwner &other) const
    { return region == other.region ? cell < other.cell : region < other.region; }
};

struct Producer {
    SourceOwner owner;
    // Alternative demand reasons for this exact producer, in owner.region.
    std::vector<size_t> demand_indices;
};

struct Obligation {
    DemandKey demand;
    // Alternative producers, EACH of which produces the complete failed
    // geometric fragment. Split demands for jointly supplied fragments.
    // Missing producers and an empty list never activate an obligation.
    std::vector<SourceOwner> sources;
};

struct Reachability {
    // Sorted unique keys, independent of input order.
    std::vector<DemandKey> active_demands;
    std::vector<SourceOwner> active_sources;
    bool is_active(const DemandKey &demand) const;
    bool is_active(const SourceOwner &source) const;
};

struct Limits {
    // Nodes count distinct demand keys plus surviving source owners.
    size_t max_nodes = 1000000;
    // All input demand-reason and source references consume this budget,
    // including duplicate edges, absent sources and unreachable components.
    size_t max_edges = 4000000;
    std::function<void()> throw_if_canceled;
};

// The least fixed point starts ONLY at direct roots, then follows active
// demands to their surviving producers and producers to their obligations.
// Cycles without a direct-root path therefore disappear together. Persistent
// closures and other unconditional obligations must be explicit direct roots.
// Duplicate producer identities or obligation target keys are invalid input;
// duplicate roots and edges are harmless. Work exhaustion throws rather than
// returning a partial result. This proves ownership, not physical support.
Reachability resolve(const std::vector<DemandKey> &direct_roots,
                     const std::vector<Producer> &surviving_producers,
                     const std::vector<Obligation> &obligations,
                     const Limits &limits = {});

}}} // namespace Slic3r::BridgeAwareCubic::Dependencies

#endif
