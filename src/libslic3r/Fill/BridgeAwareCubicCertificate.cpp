#include "BridgeAwareCubicCertificate.hpp"

#include "BridgeAwareCubic.hpp"
#include "../Layer.hpp"
#include "../Print.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <new>
#include <type_traits>
#include <typeinfo>

namespace Slic3r { namespace BridgeAwareCubic {
namespace {

// Used only during optional capture allocation accounting. Live comparison
// uses a status flag, so even cache misses need no exception allocation.
struct Declined {};

using Token = std::shared_ptr<const void>;
using Bytes = std::vector<unsigned char>;
using Limits = DepositionStateSnapshot::Limits;
using Cancel = DepositionStateSnapshot::Cancel;

// One encoder drives counting, capture, and comparison. Scalars are encoded
// individually (never struct padding); float bits preserve nullable NaNs and
// signed zero without decimal conversion or approximate comparisons. These
// bytes are process-local and are deliberately not a persistence format.
class Stream {
public:
    enum class Mode { Count, Write, Compare };
    Stream(Mode mode, const Limits &limits, const Cancel &cancel,
           Bytes *output = nullptr, std::vector<Token> *tokens = nullptr,
           const Bytes *expected = nullptr, const std::vector<Token> *expected_tokens = nullptr,
           bool *inside_cancel = nullptr)
        : mode(mode), limits(limits), cancel(cancel), output(output), tokens(tokens),
          expected(expected), expected_tokens(expected_tokens), inside_cancel(inside_cancel) {}

    void item(size_t count = 1)
    {
        if (!ok) return;
        if (count > limits.items - items) return decline();
        items += count;
        // Large strings use the same cancellation cadence as point arrays.
        if (items - last_cancel >= 256) {
            if (cancel) {
                if (inside_cancel) *inside_cancel = true;
                cancel();
                if (inside_cancel) *inside_cancel = false;
            }
            last_cancel = items;
        }
    }

    void raw(const void *data, size_t size)
    {
        if (!ok) return;
        if (size > limits.bytes - bytes) return decline();
        if (mode == Mode::Write) {
            if (bytes > output->size() || size > output->size() - bytes) return decline();
            std::memcpy(output->data() + bytes, data, size);
        } else if (mode == Mode::Compare) {
            if (bytes > expected->size() || size > expected->size() - bytes ||
                std::memcmp(expected->data() + bytes, data, size) != 0) return decline();
        }
        bytes += size;
    }

    template<class T> void scalar(T value)
    {
        static_assert(std::is_arithmetic<T>::value, "Encode individual scalar values only");
        item();
        raw(&value, sizeof(value));
    }
    void boolean(bool value) { scalar(uint8_t(value ? 1 : 0)); }
    void count(size_t value)
    {
        // Reject impossible collection sizes before any traversal/allocation.
        if (!ok) return;
        if (value > limits.items - items) return decline();
        scalar(uint64_t(value));
    }
    void string(const std::string &value)
    {
        count(value.size());
        for (size_t offset = 0; ok && offset < value.size();) {
            const size_t chunk = std::min(size_t(256), value.size() - offset);
            item(chunk);
            raw(value.data() + offset, chunk);
            offset += chunk;
        }
    }
    void token(const Token &value)
    {
        item();
        boolean(bool(value));
        if (!ok) return;
        if (mode == Mode::Write) tokens->push_back(value);
        else if (mode == Mode::Compare) {
            if (token_count >= expected_tokens->size()) return decline();
            const Token &old = (*expected_tokens)[token_count];
            if (value.get() != old.get() || value.owner_before(old) || old.owner_before(value)) return decline();
        }
        ++token_count;
    }
    void finish()
    {
        if (mode == Mode::Compare && (bytes != expected->size() || token_count != expected_tokens->size()))
            decline();
    }
    void decline() { ok = false; }

    Mode mode;
    const Limits &limits;
    const Cancel &cancel;
    size_t bytes = 0, items = 0, token_count = 0, last_cancel = 0;
    Bytes *output;
    std::vector<Token> *tokens;
    const Bytes *expected;
    const std::vector<Token> *expected_tokens;
    bool *inside_cancel;
    bool ok = true;
    bool trace_miss = false, traced_miss = false;
};

void value(Stream &out, double v) { out.scalar(v); }
void value(Stream &out, float v) { out.scalar(v); }
void value(Stream &out, int v) { out.scalar(v); }
void value(Stream &out, unsigned char v) { out.scalar(v); }
void value(Stream &out, const std::string &v) { out.string(v); }
void value(Stream &out, const Vec2d &v) { out.scalar(v.x()); out.scalar(v.y()); }
void value(Stream &out, const Vec3d &v) { out.scalar(v.x()); out.scalar(v.y()); out.scalar(v.z()); }
void value(Stream &out, const FloatOrPercent &v) { out.scalar(v.value); out.boolean(v.percent); }

template<class T> void value(Stream &out, const std::vector<T> &values)
{
    out.count(values.size());
    for (const auto &v : values) {
        if (!out.ok) return;
        value(out, v);
    }
}

template<class T> void option_vector(Stream &out, const ConfigOption &option)
{
    const auto *typed = dynamic_cast<const ConfigOptionVector<T> *>(&option);
    if (typed == nullptr) return out.decline();
    value(out, typed->values);
}

// These are the concrete static configs stored by Print, PrintObject and
// PrintRegion. Their keys_ref() is complete and allocation-free, and each key
// addresses its concrete member. Dynamic/derived config stores are declined.
template<class Config> void config(Stream &out, const Config &config)
{
    if (!out.ok) return;
    if (typeid(config) != typeid(Config)) return out.decline();
    const auto &keys = config.keys_ref();
    out.count(keys.size());
    for (const auto &key : keys) {
        if (!out.ok) return;
        out.string(key);
        const ConfigOption *option = config.optptr(key);
        if (option == nullptr) return out.decline();
        out.scalar(int(option->type()));
        out.boolean(option->nullable());
        // On a value mismatch, bytes still points to the start of the
        // failed scalar. Retain the option value start independently.
        const size_t value_begin = out.bytes;
        const bool reached_value = out.ok;
        switch (option->type()) {
        case coFloat: case coPercent: out.scalar(option->getFloat()); break;
        case coInt: case coEnum: out.scalar(option->getInt()); break;
        case coBool: out.boolean(option->getBool()); break;
        case coString: value(out, static_cast<const ConfigOptionString *>(option)->value); break;
        case coFloatOrPercent: {
            const auto *v = dynamic_cast<const ConfigOptionFloatOrPercent *>(option);
            if (v == nullptr) return out.decline();
            out.scalar(v->value); out.boolean(v->percent); break;
        }
        case coPoint: value(out, static_cast<const ConfigOptionPoint *>(option)->value); break;
        case coPoint3: value(out, static_cast<const ConfigOptionPoint3 *>(option)->value); break;
        case coFloats: case coPercents: option_vector<double>(out, *option); break;
        case coInts: case coEnums: option_vector<int>(out, *option); break;
        case coBools: option_vector<unsigned char>(out, *option); break;
        case coStrings: option_vector<std::string>(out, *option); break;
        case coFloatsOrPercents: option_vector<FloatOrPercent>(out, *option); break;
        case coPoints: option_vector<Vec2d>(out, *option); break;
        case coPointsGroups: option_vector<Vec2ds>(out, *option); break;
        case coIntsGroups: option_vector<std::vector<int>>(out, *option); break;
        default: return out.decline();
        }
        if (!out.ok && out.trace_miss && !out.traced_miss) {
            out.traced_miss = true;
            const char *scope = std::is_same<Config, PrintConfig>::value ? "PrintConfig" :
                                std::is_same<Config, PrintObjectConfig>::value ? "PrintObjectConfig" : "PrintRegionConfig";
            // Report names only for arbitrary config options. Restrict
            // value reporting to the two known integer mapping fields; no
            // machine snippets, credentials or other user data are emitted.
            if (reached_value && option->type() == coInts &&
                (key == "filament_map_2" || key == "filament_nozzle_map") &&
                value_begin <= out.expected->size() && out.expected->size() - value_begin >= sizeof(uint64_t)) {
                uint64_t old_size = 0;
                std::memcpy(&old_size, out.expected->data() + value_begin, sizeof(old_size));
                const auto *now = dynamic_cast<const ConfigOptionVector<int> *>(option);
                if (now && old_size == 1 && now->values.size() == 1 &&
                    out.expected->size() - value_begin - sizeof(old_size) >= sizeof(int)) {
                    int old_value = 0;
                    std::memcpy(&old_value, out.expected->data() + value_begin + sizeof(old_size), sizeof(old_value));
                    std::fprintf(stderr, "BAC_CERT_FIRST_MISS scope=%s key=%s type=coInts index=0 previous=%d current=%d byte=%zu\n",
                        scope, key.c_str(), old_value, now->values.front(), out.bytes);
                    return;
                }
            }
            std::fprintf(stderr, "BAC_CERT_FIRST_MISS scope=%s key=%s option_type=%d part=%s byte=%zu\n",
                scope, key.c_str(), int(option->type()), reached_value ? "value" : "metadata", out.bytes);
        }
    }
}

void points(Stream &out, const Points &points)
{
    out.count(points.size());
    for (const Point &point : points) {
        if (!out.ok) return;
        out.scalar(point.x()); out.scalar(point.y());
    }
}
void polygon(Stream &out, const ExPolygon &polygon)
{
    points(out, polygon.contour.points);
    out.count(polygon.holes.size());
    for (const Polygon &hole : polygon.holes) {
        if (!out.ok) return;
        points(out, hole.points);
    }
}
void polygons(Stream &out, const ExPolygons &polygons)
{
    out.count(polygons.size());
    for (const auto &poly : polygons) {
        if (!out.ok) return;
        polygon(out, poly);
    }
}
void surfaces(Stream &out, const Surfaces &surfaces)
{
    out.count(surfaces.size());
    for (const Surface &surface : surfaces) {
        if (!out.ok) return;
        out.scalar(int(surface.surface_type));
        polygon(out, surface.expolygon);
        out.scalar(surface.thickness);
        out.scalar(surface.thickness_layers);
        out.scalar(surface.bridge_angle);
        out.scalar(surface.extra_perimeters);
        out.boolean(surface.bridge_followup);
        out.boolean(surface.counter_circle_compensation);
        value(out, surface.holes_circle_compensation);
    }
}

void entity(Stream &out, const ExtrusionEntity &entity, size_t depth)
{
    if (!out.ok) return;
    if (depth > out.limits.tree_depth) return out.decline();
    // Inspect exact dynamic type before touching base-only state. In
    // particular, Sloped paths/loops must not be sliced by a base clone.
    const auto &kind = typeid(entity);
    uint8_t tag = 0;
    if (kind == typeid(ExtrusionPath)) tag = 1;
    else if (kind == typeid(ExtrusionMultiPath)) tag = 2;
    else if (kind == typeid(ExtrusionLoop)) tag = 3;
    else if (kind == typeid(ExtrusionEntityCollection)) tag = 4;
    else return out.decline();
    out.scalar(tag);
    out.scalar(int(entity.get_customize_flag()));
    out.scalar(entity.get_cooling_node());
    out.boolean(entity.can_reverse());
    if (tag == 1) {
        const auto &path = static_cast<const ExtrusionPath &>(entity);
        if (!path.polyline.fitting_result.empty()) return out.decline();
        out.scalar(int(path.role()));
        out.scalar(path.width); out.scalar(path.height); out.scalar(path.mm3_per_mm);
        out.scalar(path.overhang_degree); out.scalar(path.curve_degree); out.scalar(path.smooth_speed);
        out.boolean(path.is_force_no_extrusion());
        points(out, path.polyline.points);
    } else if (tag == 2 || tag == 3) {
        const ExtrusionPaths &paths = tag == 2 ? static_cast<const ExtrusionMultiPath &>(entity).paths :
            static_cast<const ExtrusionLoop &>(entity).paths;
        if (tag == 3) out.scalar(int(static_cast<const ExtrusionLoop &>(entity).loop_role()));
        out.count(paths.size());
        for (const auto &path : paths) {
            if (!out.ok) return;
            BridgeAwareCubic::entity(out, path, depth + 1);
        }
    } else {
        const auto &collection = static_cast<const ExtrusionEntityCollection &>(entity);
        out.boolean(collection.no_sort);
        // The private is_reverse flag is consumed solely by can_reverse().
        // no_sort=true masks it completely; switching no_sort later itself
        // invalidates this record before the hidden bit can affect behavior.
        out.scalar(collection.loop_node_range.first); out.scalar(collection.loop_node_range.second);
        out.count(collection.entities.size());
        for (const auto *child : collection.entities) {
            if (!out.ok) return;
            if (child == nullptr) return out.decline();
            BridgeAwareCubic::entity(out, *child, depth + 1);
        }
    }
}

void region_policy(Stream &out, const PrintObject &object)
{
    if (!out.ok) return;
    if (object.shared_regions() == nullptr || object.num_printing_regions() > size_t(std::numeric_limits<int>::max()))
        return out.decline();
    out.count(object.num_printing_regions());
    for (size_t i = 0; i < object.num_printing_regions(); ++i) {
        if (!out.ok) return;
        if (!object.shared_regions()->all_regions[i]) return out.decline();
        const auto &region = object.printing_region(i);
        out.scalar(region.print_region_id());
        out.scalar(region.print_object_region_id());
        config(out, region.config());
        const Plan *plan = object.bridge_aware_cubic_plan(i);
        out.boolean(plan != nullptr);
        const Token identity = plan == nullptr ? Token{} : plan->immutable_identity();
        if (plan != nullptr && !identity) return out.decline();
        out.token(identity);
    }
}

void state(Stream &out, const PrintObject &geometry, const PrintObject &caller)
{
    if (!out.ok) return;
    if (geometry.print() == nullptr || geometry.print() != caller.print() ||
        geometry.get_shared_object() != nullptr ||
        (&caller != &geometry && caller.get_shared_object() != &geometry)) return out.decline();
    const BoundingBox bounds = geometry.bounding_box();
    out.boolean(bounds.defined);
    out.scalar(bounds.min.x()); out.scalar(bounds.min.y());
    out.scalar(bounds.max.x()); out.scalar(bounds.max.y());
    config(out, geometry.print()->config()); // Includes the full nozzle-offset vector.
    config(out, geometry.config());
    config(out, caller.config());
    region_policy(out, geometry);
    region_policy(out, caller);
    if (geometry.layer_count() > size_t(std::numeric_limits<int>::max())) return out.decline();
    out.count(geometry.layer_count());
    for (size_t i = 0; i < geometry.layer_count(); ++i) {
        if (!out.ok) return;
        const Layer *layer = geometry.get_layer(int(i));
        if (layer == nullptr || layer->object() != &geometry ||
            layer->regions().size() != geometry.num_printing_regions()) return out.decline();
        out.scalar(uint64_t(layer->id()));
        out.scalar(layer->print_z); out.scalar(layer->slice_z); out.scalar(layer->height);
        polygons(out, layer->lslices);
        out.count(layer->regions().size());
        for (size_t j = 0; j < layer->regions().size(); ++j) {
            if (!out.ok) return;
            const LayerRegion *region = layer->get_region(int(j));
            if (region == nullptr || region->layer() != layer ||
                &region->region() != &geometry.printing_region(j)) return out.decline();
            surfaces(out, region->slices.surfaces);
            surfaces(out, region->fill_surfaces.surfaces);
            polygons(out, region->fill_expolygons);
            polygons(out, region->bridge_fill_expolygons);
            out.scalar(region->bridge_fill_overlap_delta);
            polygons(out, region->fill_no_overlap_expolygons);
            polygons(out, geometry.bridge_aware_cubic_reservations(i, j));
            entity(out, region->perimeters, 0);
            entity(out, region->fills, 0);
        }
    }
    out.finish();
}

bool same_double(double a, double b)
{
    return std::memcmp(&a, &b, sizeof(a)) == 0;
}
bool same_phase(const SerializedXY &a, const SerializedXY &b)
{
    return same_double(a.shift.x(), b.shift.x()) && same_double(a.shift.y(), b.shift.y()) &&
        same_double(a.nozzle_offset.x(), b.nozzle_offset.x()) && same_double(a.nozzle_offset.y(), b.nozzle_offset.y()) &&
        same_double(a.plate_origin.x(), b.plate_origin.x()) && same_double(a.plate_origin.y(), b.plate_origin.y());
}
bool phase_guard(const std::vector<SerializedXY> &phases, const Limits &limits)
{
    if (phases.empty() || phases.size() > limits.phases || phases.size() > limits.items) return false;
    for (const auto &phase : phases)
        if (!phase.shift.allFinite() || !phase.nozzle_offset.allFinite() || !phase.plate_origin.allFinite()) return false;
    return true;
}

size_t add_storage(size_t used, size_t count, size_t size, size_t limit)
{
    if (used > limit || count > (limit - used) / size) throw Declined{};
    return used + count * size;
}
} // namespace

struct DepositionStateSnapshot::Impl {
    Limits limits;
    Bytes bytes;
    std::vector<Token> tokens;
    std::vector<SerializedXY> phases;
    size_t retained_bytes = 0;
};

DepositionStateSnapshot::DepositionStateSnapshot(std::unique_ptr<const Impl> impl) : m_impl(std::move(impl)) {}
DepositionStateSnapshot::~DepositionStateSnapshot() = default;

std::shared_ptr<const DepositionStateSnapshot> DepositionStateSnapshot::capture(
    const PrintObject &geometry, const PrintObject &caller, const std::vector<SerializedXY> &phases,
    const Cancel &cancel, const Limits &limits)
{
    if (cancel) cancel();
    bool inside_cancel = false;
    try {
        if (!phase_guard(phases, limits)) return {};
        Stream count(Stream::Mode::Count, limits, cancel, nullptr, nullptr, nullptr, nullptr, &inside_cancel);
        count.item(phases.size());
        state(count, geometry, caller);
        if (!count.ok) return {};
        size_t retained = sizeof(Impl) + sizeof(DepositionStateSnapshot);
        retained = add_storage(retained, count.bytes, 1, limits.bytes);
        retained = add_storage(retained, count.token_count, sizeof(Token), limits.bytes);
        retained = add_storage(retained, phases.size(), sizeof(SerializedXY), limits.bytes);
        auto impl = std::make_unique<Impl>();
        impl->limits = limits;
        impl->bytes.resize(count.bytes);
        impl->tokens.reserve(count.token_count);
        impl->phases = phases;
        Stream write(Stream::Mode::Write, limits, cancel, &impl->bytes, &impl->tokens, nullptr, nullptr, &inside_cancel);
        write.item(phases.size());
        state(write, geometry, caller);
        if (!write.ok || write.bytes != count.bytes || write.token_count != count.token_count) return {};
        // Count actual vector capacities too, rather than assuming reserve and
        // assignment retained exactly the requested payload sizes.
        retained = sizeof(Impl) + sizeof(DepositionStateSnapshot);
        retained = add_storage(retained, impl->bytes.capacity(), 1, limits.bytes);
        retained = add_storage(retained, impl->tokens.capacity(), sizeof(Token), limits.bytes);
        retained = add_storage(retained, impl->phases.capacity(), sizeof(SerializedXY), limits.bytes);
        impl->retained_bytes = retained;
        return std::shared_ptr<const DepositionStateSnapshot>(new DepositionStateSnapshot(std::move(impl)));
    } catch (const Declined &) {
        return {};
    } catch (const std::bad_alloc &) {
        if (inside_cancel) throw;
        return {};
    }
}

bool DepositionStateSnapshot::matches(const PrintObject &geometry, const PrintObject &caller,
    const std::vector<SerializedXY> &phases, const Cancel &cancel) const
{
    if (cancel) cancel();
    const char *setting = std::getenv("BAC_TRACE_FINAL_PROOF_MISS");
    const bool trace = setting && setting[0] == '1' && setting[1] == '\0';
    if (!phase_guard(phases, m_impl->limits)) {
        if (trace) std::fprintf(stderr, "BAC_CERT_FIRST_MISS scope=phase_admission\n");
        return false;
    }
    for (const auto &phase : phases) {
        if (cancel) cancel();
        if (std::none_of(m_impl->phases.begin(), m_impl->phases.end(),
                [&](const SerializedXY &retained) { return same_phase(phase, retained); })) {
            if (trace) std::fprintf(stderr, "BAC_CERT_FIRST_MISS scope=phase_subset\n");
            return false;
        }
    }
    Stream compare(Stream::Mode::Compare, m_impl->limits, cancel, nullptr, nullptr, &m_impl->bytes, &m_impl->tokens);
    compare.trace_miss = trace;
    compare.item(m_impl->phases.size());
    state(compare, geometry, caller);
    if (trace && !compare.ok && !compare.traced_miss)
        std::fprintf(stderr, "BAC_CERT_FIRST_MISS scope=nonoption_or_admission byte=%zu token=%zu\n",
            compare.bytes, compare.token_count);
    return compare.ok;
}

size_t DepositionStateSnapshot::retained_bytes() const noexcept { return m_impl->retained_bytes; }

}} // namespace Slic3r::BridgeAwareCubic
