#ifndef slic3r_FillBridgeAwareCubic_hpp_
#define slic3r_FillBridgeAwareCubic_hpp_

#include "FillRectilinear.hpp"
#include "BridgeAwareCubic.hpp"
#include "BridgeAwareCubicEmissionLedger.hpp"
#include "BridgeAwareCubicValidation.hpp"
#include "../ClipperUtils.hpp"
#include "../ExtrusionEntityCollection.hpp"

#include <exception>
#include <limits>
#include <new>
#include <typeinfo>

namespace Slic3r {

class FillBridgeAwareCubic : public FillCubic
{
public:
    const BridgeAwareCubic::Plan *plan = nullptr;
    const ExPolygons *material_domain = nullptr;
    double extrusion_width = 0.;
    BridgeAwareCubic::EmissionLedgerBuilder *emission_builder = nullptr;
    BridgeAwareCubicValidation::WorkBudget *emission_budget = nullptr;

    FillBridgeAwareCubic() = default;
    // A clone carries ordinary fill configuration, never a pending generation
    // event or borrowed binding handle from another filler invocation.
    FillBridgeAwareCubic(const FillBridgeAwareCubic &other)
        : FillCubic(other), plan(other.plan), material_domain(other.material_domain),
          extrusion_width(other.extrusion_width) {}

    Fill *clone() const override { return new FillBridgeAwareCubic(*this); }
    bool no_sort() const override { return plan != nullptr; }

    Polylines fill_surface(const Surface *surface, const FillParams &params) override
    {
        try { return fill_surface_impl(surface, params); }
        catch (const CaptureCanceled &canceled) { std::rethrow_exception(canceled.exception); }
    }

    void fill_surface_extrusion(const Surface *surface, const FillParams &params, ExtrusionEntitiesPtr &out) override
    {
        clear_capture();
        const size_t before = out.size();
        m_capture = emission_builder != nullptr && plan != nullptr && !params.use_arachne;
        if (emission_builder != nullptr && !m_capture) emission_builder->decline();
        if (m_capture) {
            m_limit = emission_builder->available_scratch_bytes();
            m_local_work = BridgeAwareCubicValidation::WorkBudget{};
            if (m_limit < sizeof(BridgeAwareCubic::TaggedLineBatch)) decline_capture();
        }
        try {
            if (m_capture && emission_builder->needs_reference()) {
                // No pending surface tags or child vectors coexist with this
                // optional reference. Transfer ownership, then refresh the
                // quota before the ordinary fill batches begin.
                auto reference = plan->reference_lines(z, emission_builder->reference_bounds(),
                    m_limit / sizeof(Point), work_budget());
                if (!reference.complete || !emission_builder->bind_reference(std::move(reference),
                        work_budget(), [&] { poll(); }, 0)) decline_capture();
                if (m_capture) {
                    m_limit = emission_builder->available_scratch_bytes();
                    if (m_limit < sizeof(BridgeAwareCubic::TaggedLineBatch)) decline_capture();
                }
            }
            // Keep the production conversion, flow calculation, validity
            // filtering, collection construction and reversal policy intact.
            Fill::fill_surface_extrusion(surface, params, out);
            if (m_capture) optional_capture([&] { bind_children(before, out); });
            clear_capture();
        } catch (const CaptureCanceled &canceled) {
            decline_capture();
            std::rethrow_exception(canceled.exception);
        } catch (...) {
            decline_capture();
            throw;
        }
    }

private:
    struct CaptureUnavailable {};
    struct CaptureCanceled { std::exception_ptr exception; };
    bool m_capture = false;
    size_t m_limit = 0, m_used = 0, m_since_poll = 0;
    BridgeAwareCubicValidation::WorkBudget m_local_work;
    BridgeAwareCubic::TaggedLineBatch m_pending;

    BridgeAwareCubicValidation::WorkBudget *work_budget()
    {
        return emission_budget == nullptr ? &m_local_work : emission_budget;
    }
    void poll() const
    {
        if (plan != nullptr && plan->parameters().throw_if_canceled) {
            try { plan->parameters().throw_if_canceled(); }
            catch (...) { throw CaptureCanceled{std::current_exception()}; }
        }
    }
    void charge(size_t count = 1)
    {
        auto *budget = work_budget();
        if (count > budget->remaining_work) throw CaptureUnavailable{};
        budget->remaining_work -= count;
        budget->charged_work[size_t(BridgeAwareCubicValidation::WorkCategory::Validation)] += count;
        m_since_poll += count;
        if (m_since_poll >= 256) { m_since_poll = 0; poll(); }
    }
    void admit(size_t bytes)
    {
        charge();
        if (bytes > m_limit - m_used) throw CaptureUnavailable{};
        m_used += bytes;
    }
    template<class T> void reserve(std::vector<T> &items, size_t count)
    {
        if (count <= items.capacity()) return;
        if (count > (m_limit - m_used) / sizeof(T)) throw CaptureUnavailable{};
        // Charge the complete new allocation while the old buffer still lives.
        // Retired optional buffers remain charged for this one fill call.
        admit(count * sizeof(T));
        charge(count);
        items.reserve(count);
        if (items.capacity() > count) admit((items.capacity() - count) * sizeof(T));
    }
    void clear_capture() noexcept
    {
        m_capture = false;
        m_pending = BridgeAwareCubic::TaggedLineBatch{};
        m_limit = m_used = m_since_poll = 0;
    }
    void decline_capture() noexcept
    {
        if (emission_builder != nullptr) emission_builder->decline();
        clear_capture();
    }
    template<class Function> void optional_capture(Function &&function)
    {
        if (!m_capture) return;
        try { function(); }
        catch (const CaptureUnavailable &) { decline_capture(); }
        catch (const std::bad_alloc &) { decline_capture(); }
    }
    void append_sources(BridgeAwareCubic::TaggedLineBatch &batch)
    {
        if (!batch.complete || batch.source_for_path.size() != batch.paths.size()) throw CaptureUnavailable{};
        // tagged_lines already bounded this metadata's peak construction by
        // the available quota. Account its live capacities before merging.
        admit(sizeof(batch));
        admit(batch.sources.capacity() * sizeof(BridgeAwareCubic::LineSource));
        admit(batch.source_for_path.capacity() * sizeof(size_t));
        for (const auto &source : batch.sources) {
            charge();
            admit(source.producers.capacity() * sizeof(BridgeAwareCubic::ProducerInterval));
        }
        const size_t source_offset = m_pending.sources.size();
        if (batch.sources.size() > std::numeric_limits<size_t>::max() - source_offset)
            throw CaptureUnavailable{};
        reserve(m_pending.sources, source_offset + batch.sources.size());
        if (batch.source_for_path.size() > std::numeric_limits<size_t>::max() - m_pending.source_for_path.size())
            throw CaptureUnavailable{};
        reserve(m_pending.source_for_path, m_pending.source_for_path.size() + batch.source_for_path.size());
        for (size_t source : batch.source_for_path) {
            charge();
            if (source >= batch.sources.size()) throw CaptureUnavailable{};
            m_pending.source_for_path.push_back(source_offset + source);
        }
        for (auto &source : batch.sources) {
            charge();
            m_pending.sources.push_back(std::move(source));
        }
    }
    Polylines clip_material(const Polylines &paths, const ExPolygons &material)
    {
        if (!m_capture) return intersection_pl(paths, material);
        // This is exactly intersection_pl's one mixed open-subject batch,
        // with the same flags and PolyTree traversal. Metadata cannot alter
        // the subjects, masks, contour ordering or emitted points.
        ClipperLib::Clipper clipper;
        clipper.AddPaths(ClipperUtils::PolylinesProvider(paths), ClipperLib::ptSubject, false);
        clipper.AddPaths(ClipperUtils::ExPolygonsProvider(material), ClipperLib::ptClip, true);
        ClipperLib::PolyTree tree;
        clipper.Execute(ClipperLib::ctIntersection, tree, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
        Polylines clipped;
        clipped.reserve(tree.Total());
        std::vector<size_t> source_ids;
        optional_capture([&] { reserve(source_ids, tree.Total()); });
        for (auto *node = tree.GetFirst(); node != nullptr; node = node->GetNext()) {
            if (node->Contour.empty()) continue;
            optional_capture([&] {
                charge();
                if (!node->IsOpen() || node->OpenPathIndex() >= m_pending.source_for_path.size())
                    throw CaptureUnavailable{};
                source_ids.push_back(m_pending.source_for_path[node->OpenPathIndex()]);
            });
            clipped.emplace_back(std::move(node->Contour));
        }
        if (m_capture) m_pending.source_for_path = std::move(source_ids);
        return clipped;
    }
    void retain_paths(const Polylines &paths)
    {
        if (m_pending.source_for_path.size() != paths.size()) throw CaptureUnavailable{};
        reserve(m_pending.paths, paths.size());
        for (const Polyline &path : paths) {
            charge();
            if (!path.fitting_result.empty()) throw CaptureUnavailable{};
            Polyline copy;
            reserve(copy.points, path.size());
            for (const Point &point : path.points) { charge(); copy.points.push_back(point); }
            m_pending.paths.push_back(std::move(copy));
        }
        poll();
        m_pending.complete = true;
    }
    void bind_children(size_t before, const ExtrusionEntitiesPtr &out)
    {
        if (!m_pending.complete) throw CaptureUnavailable{};
        const bool had_polylines = !m_pending.paths.empty();
        size_t valid = 0;
        for (size_t i = 0; i < m_pending.paths.size(); ++i) {
            charge();
            if (!m_pending.paths[i].is_valid()) continue;
            if (valid != i) {
                m_pending.paths[valid] = std::move(m_pending.paths[i]);
                m_pending.source_for_path[valid] = m_pending.source_for_path[i];
            }
            ++valid;
        }
        m_pending.paths.resize(valid);
        m_pending.source_for_path.resize(valid);
        std::vector<const ExtrusionPath *> children;
        reserve(children, valid);
        if (out.size() != before + size_t(had_polylines)) throw CaptureUnavailable{};
        if (had_polylines) {
            const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(out[before]);
            if (collection == nullptr || typeid(*collection) != typeid(ExtrusionEntityCollection) ||
                collection->entities.size() != valid || collection->no_sort != no_sort()) throw CaptureUnavailable{};
            for (size_t i = 0; i < valid; ++i) {
                charge(m_pending.paths[i].size() + 1);
                const auto *path = dynamic_cast<const ExtrusionPath *>(collection->entities[i]);
                if (path == nullptr || typeid(*path) != typeid(ExtrusionPath) ||
                    !path->polyline.fitting_result.empty() || path->polyline != m_pending.paths[i])
                    throw CaptureUnavailable{};
                children.push_back(path);
            }
        }
        poll();
        // Other per-call optional buffers have been destroyed. bind_batch
        // admits this borrowed batch and pointer vector alongside its copies.
        if (!emission_builder->bind_batch(m_pending, children, work_budget(), [&] { poll(); }, 0))
            decline_capture();
    }

    Polylines fill_surface_impl(const Surface *surface, const FillParams &params)
    {
        // The preliminary bridge-direction pass runs before support demand is
        // known. It uses the identical ordinary Cubic lattice as the plan.
        if (plan == nullptr)
            return FillCubic::fill_surface(surface, params);

        Polylines result;
        // Ordinary Cubic adds boundary anchors after its half-spacing inset.
        // These complete partition faces need their straight ends to reach the
        // actual fill boundary instead, so they join the deposited inner wall.
        for (const ExPolygon &domain : offset_ex(surface->expolygon, scale_(overlap))) {
            if (m_capture) {
                const size_t remaining = m_limit - m_used;
                const size_t units = remaining > sizeof(BridgeAwareCubic::TaggedLineBatch) ?
                    (remaining - sizeof(BridgeAwareCubic::TaggedLineBatch)) / sizeof(Point) : 0;
                // This call propagates Plan cancellation directly, including
                // allocation-shaped exceptions; only our own optional copies
                // are covered by optional_capture's allocation fallback.
                auto tagged = plan->tagged_lines(z, domain, units, work_budget());
                optional_capture([&] { append_sources(tagged); });
                append(result, std::move(tagged.paths));
            } else append(result, plan->lines(z, domain));
        }
        // Two perpendicular half-density families preserve one nominal layer
        // volume while closing the long top-plane slots of parallel solid
        // fill. Their masks remain construction obligations: the final verifier
        // checks every emitted line and its preceding-layer support.
        std::array<FillAlignedRectilinear, 2> solid;
        for (size_t family = 0; family < solid.size(); ++family) {
            solid[family].copy_fill_data(this);
            solid[family].spacing = params.flow.spacing();
            solid[family].angle = float(plan->parameters().angle + family * .5 * PI);
        }
        FillParams solid_params = params;
        solid_params.density = .5f;
        solid_params.multiline = 1;
        solid_params.dont_adjust = true;
        solid_params.anchor_length = solid_params.anchor_length_max = 0.f;
        for (const ExPolygon &domain : intersection_ex(plan->solid_area(z),
                 offset_ex(surface->expolygon, scale_(overlap)))) {
            Surface patch = *surface;
            patch.expolygon = domain;
            // This internal construction follows the cubic lattice directions;
            // a model bridge hint must not override both families to one angle.
            patch.bridge_angle = -1.;
            for (FillAlignedRectilinear &family : solid)
                append(result, family.fill_surface(&patch, solid_params));
        }
        // A fill boundary can also be a real exterior or cavity, including
        // models without perimeters. Keep the whole bead inside that material.
        if (material_domain != nullptr)
            result = clip_material(result, offset_ex(*material_domain, -scale_(.5 * extrusion_width)));
        optional_capture([&] { retain_paths(result); });
        return result;
    }
};

} // namespace Slic3r

#endif
