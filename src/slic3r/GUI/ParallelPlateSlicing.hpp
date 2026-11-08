#ifndef slic3r_GUI_ParallelPlateSlicing_hpp_
#define slic3r_GUI_ParallelPlateSlicing_hpp_

#include <algorithm>
#include <cstdint>
#include <map>
#include <vector>

namespace Slic3r::GUI {

// UI-thread-owned admission and event attribution. Workers never mutate this queue.
class ParallelPlateSlicing
{
    enum class State { Pending, Running, Finished };
    struct Plate { State state = State::Pending; int percent = 0; };
    std::map<int, Plate> m_plates;
    std::uint64_t m_generation = 0;
    size_t m_limit = 1;
    bool m_active = false;
    bool m_failed = false;

public:
    static size_t concurrency(size_t plates, unsigned cpus, std::uint64_t memory)
    {
        constexpr std::uint64_t gib = std::uint64_t(1024) * 1024 * 1024;
        const auto memory_slots = memory > 2 * gib ? (memory - 2 * gib) / (3 * gib) : 1;
        return std::max<size_t>(1, std::min<size_t>({plates, 4, std::max(1u, cpus / 2),
            size_t(std::max<std::uint64_t>(1, memory_slots))}));
    }

    std::uint64_t begin(const std::vector<int> &plates, size_t limit)
    {
        m_plates.clear();
        for (int id : plates) m_plates.emplace(id, Plate{});
        m_limit = std::max<size_t>(1, limit);
        m_active = true;
        m_failed = false;
        return ++m_generation;
    }
    std::uint64_t generation() const { return m_generation; }
    bool active() const { return m_active; }
    bool failed() const { return m_failed; }
    bool accepts(int id, std::uint64_t generation) const
    {
        auto it = m_plates.find(id);
        return m_active && generation == m_generation && it != m_plates.end() &&
            it->second.state == State::Running;
    }
    std::vector<int> take_available()
    {
        std::vector<int> admitted;
        if (!m_active) return admitted;
        size_t running = 0;
        for (const auto &[id, plate] : m_plates)
            running += plate.state == State::Running;
        for (auto &[id, plate] : m_plates) {
            if (running == m_limit) break;
            if (plate.state == State::Pending) {
                plate.state = State::Running;
                admitted.push_back(id);
                ++running;
            }
        }
        return admitted;
    }
    bool progress(int id, std::uint64_t generation, int percent)
    {
        if (!accepts(id, generation)) return false;
        auto &plate = m_plates.at(id);
        plate.percent = std::max(plate.percent, std::clamp(percent, 0, 100));
        return true;
    }
    bool finish(int id, std::uint64_t generation, bool success)
    {
        if (!accepts(id, generation)) return false;
        m_plates.at(id) = {State::Finished, 100};
        m_failed |= !success;
        return true;
    }
    bool done() const
    {
        return std::all_of(m_plates.begin(), m_plates.end(), [](const auto &entry) {
            return entry.second.state == State::Finished;
        });
    }
    double percent() const
    {
        double sum = 0;
        for (const auto &[id, plate] : m_plates) sum += plate.percent;
        return m_plates.empty() ? 100. : sum / m_plates.size();
    }
    void cancel() { m_active = false; ++m_generation; }
};

} // namespace Slic3r::GUI
#endif
