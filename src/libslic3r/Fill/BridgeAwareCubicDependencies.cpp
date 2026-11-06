#include "BridgeAwareCubicDependencies.hpp"

#include <algorithm>
#include <map>
#include <set>
#include <stdexcept>

namespace Slic3r { namespace BridgeAwareCubic { namespace Dependencies {

bool Reachability::is_active(const DemandKey &demand) const
{
    return std::binary_search(active_demands.begin(), active_demands.end(), demand);
}

bool Reachability::is_active(const SourceOwner &source) const
{
    return std::binary_search(active_sources.begin(), active_sources.end(), source);
}

namespace {

struct Node {
    std::vector<size_t> outgoing;
    bool active = false;
};

class Graph {
public:
    explicit Graph(const Limits &limits) : m_limits(limits) {}

    Reachability resolve(const std::vector<DemandKey> &roots,
                         const std::vector<Producer> &producers,
                         const std::vector<Obligation> &obligations)
    {
        checkpoint();
        for (const DemandKey &root : roots)
            demand_id(root);

        for (const Producer &producer : producers) {
            checkpoint();
            if (m_source_ids.find(producer.owner) != m_source_ids.end())
                throw std::invalid_argument("Bridge-aware cubic ownership has a duplicate producer.");
            add_node();
            const size_t source_id = m_sources.size();
            m_source_ids.emplace(producer.owner, source_id);
            m_sources.emplace_back();
            for (size_t index : producer.demand_indices) {
                add_edge();
                const size_t reason = demand_id({producer.owner.region, index});
                m_demands[reason].outgoing.push_back(source_id);
            }
        }

        std::set<DemandKey> obligation_keys;
        for (const Obligation &obligation : obligations) {
            checkpoint();
            const size_t target = demand_id(obligation.demand);
            if (!obligation_keys.insert(obligation.demand).second)
                throw std::invalid_argument("Bridge-aware cubic ownership has a duplicate obligation demand.");
            for (const SourceOwner &source : obligation.sources) {
                add_edge();
                const auto found = m_source_ids.find(source);
                if (found != m_source_ids.end())
                    m_sources[found->second].outgoing.push_back(target);
            }
        }

        // A source is queued only after one of its own demand reasons becomes
        // reachable. Merely existing in the trial cannot bootstrap a cycle.
        for (const DemandKey &root : roots) {
            checkpoint();
            activate(true, m_demand_ids.at(root));
        }
        for (size_t head = 0; head < m_queue.size(); ++head) {
            checkpoint();
            const QueueEntry entry = m_queue[head];
            const Node &node = entry.demand ? m_demands[entry.id] : m_sources[entry.id];
            for (size_t next : node.outgoing) {
                checkpoint();
                activate(!entry.demand, next);
            }
        }

        Reachability result;
        for (const auto &entry : m_demand_ids) {
            checkpoint();
            if (m_demands[entry.second].active)
                result.active_demands.push_back(entry.first);
        }
        for (const auto &entry : m_source_ids) {
            checkpoint();
            if (m_sources[entry.second].active)
                result.active_sources.push_back(entry.first);
        }
        if (m_limits.throw_if_canceled)
            m_limits.throw_if_canceled();
        return result;
    }

private:
    struct QueueEntry { bool demand; size_t id; };

    void checkpoint()
    {
        if ((m_operations++ & size_t(1023)) == 0 && m_limits.throw_if_canceled)
            m_limits.throw_if_canceled();
    }

    void add_node()
    {
        if (m_node_count >= m_limits.max_nodes)
            throw std::runtime_error("Bridge-aware cubic ownership exceeded its node limit.");
        ++m_node_count;
    }

    void add_edge()
    {
        checkpoint();
        if (m_edge_count >= m_limits.max_edges)
            throw std::runtime_error("Bridge-aware cubic ownership exceeded its edge limit.");
        ++m_edge_count;
    }

    size_t demand_id(const DemandKey &key)
    {
        checkpoint();
        const auto found = m_demand_ids.find(key);
        if (found != m_demand_ids.end())
            return found->second;
        add_node();
        const size_t id = m_demands.size();
        m_demand_ids.emplace(key, id);
        m_demands.emplace_back();
        return id;
    }

    void activate(bool demand, size_t id)
    {
        Node &node = demand ? m_demands[id] : m_sources[id];
        if (!node.active) {
            node.active = true;
            m_queue.push_back({demand, id});
        }
    }

    const Limits &m_limits;
    size_t m_node_count = 0, m_edge_count = 0, m_operations = 0;
    std::map<DemandKey, size_t> m_demand_ids;
    std::map<SourceOwner, size_t> m_source_ids;
    std::vector<Node> m_demands, m_sources;
    std::vector<QueueEntry> m_queue;
};

} // namespace

Reachability resolve(const std::vector<DemandKey> &direct_roots,
                     const std::vector<Producer> &surviving_producers,
                     const std::vector<Obligation> &obligations,
                     const Limits &limits)
{
    return Graph(limits).resolve(direct_roots, surviving_producers, obligations);
}

}}} // namespace Slic3r::BridgeAwareCubic::Dependencies
