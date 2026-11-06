#include "Print.hpp"
#include "Layer.hpp"
#include "ClipperUtils.hpp"
#include "Exception.hpp"
#include "Fill/BridgeAwareCubicValidation.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <map>
#include <sstream>
#include <tuple>

namespace Slic3r {
namespace {

// No path simplification is allowed after nominal-footprint verification.
// This allowance covers polygon operations plus paired XYZ export rounding:
// 0.001 mm output can displace two relative XY endpoints by sqrt(2)*0.001.
// Maximum XY bead width does not establish physical cross-section contact.
constexpr double geometry_tolerance = .002 + 16. * SCALING_FACTOR;
constexpr unsigned repair_limit = 6;

bool participates(const PrintRegionConfig &config)
{
    return config.sparse_infill_pattern == ipBridgeAwareCubic &&
        config.sparse_infill_density.value > 0. && config.sparse_infill_density.value < 100.;
}

template<class Visitor> void visit_paths(const ExtrusionEntity &entity, const Visitor &visitor)
{
    if (const auto *path = dynamic_cast<const ExtrusionPath*>(&entity)) {
        if (!path->is_force_no_extrusion() && path->mm3_per_mm > 0. && path->polyline.size() >= 2)
            visitor(*path);
    } else if (const auto *multi = dynamic_cast<const ExtrusionMultiPath*>(&entity)) {
        for (const ExtrusionPath &path : multi->paths) visit_paths(path, visitor);
    } else if (const auto *loop = dynamic_cast<const ExtrusionLoop*>(&entity)) {
        for (const ExtrusionPath &path : loop->paths) visit_paths(path, visitor);
    } else if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection*>(&entity)) {
        for (const ExtrusionEntity *child : collection->entities) visit_paths(*child, visitor);
    }
}

template<class Visitor> void visit_layer(const Layer &layer, const Visitor &visitor)
{
    for (const LayerRegion *region : layer.regions()) {
        visit_paths(region->perimeters, visitor);
        visit_paths(region->fills, visitor);
    }
}

bool has_circle_scarf(const ExtrusionEntity &entity)
{
    if (const auto *loop = dynamic_cast<const ExtrusionLoop*>(&entity))
        return loop->get_customize_flag() == cfCircleCompensation && !loop->has_overhang_paths();
    if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection*>(&entity))
        for (const ExtrusionEntity *child : collection->entities)
            if (has_circle_scarf(*child)) return true;
    return false;
}

ExPolygons sparse_domain(const LayerRegion &region)
{
    ExPolygons result;
    for (const Surface &surface : region.fill_surfaces.surfaces)
        if (surface.surface_type == stInternal || surface.surface_type == stInternalVoid)
            result.push_back(surface.expolygon);
    return union_ex(result);
}

struct Target {
    size_t upper_layer = 0;
    size_t supporting_region = 0;
    double gap = 0.;
    double bottom_z = 0.;
    bool dependency = false;
    bool bridge_followup = false;
    Polylines paths;
    std::vector<double> path_widths;
    ExPolygons footprint;
};

// Group equal contact planes, but retain complete emitted paths and endpoints.
// Only the demand mask is clipped to the supplying region; it never closes a
// cell or supplies an artificial endpoint to the validator.
std::vector<Target> collect_targets(const PrintObject &object, bool dependencies, bool unsupported_only = true)
{
    std::vector<Target> targets;
    const auto &layers = object.layers();
    const BoundingBox bounds = object.bounding_box();
    const ExPolygon broad_domain(Polygon{Points{
        bounds.min, Point(bounds.max.x(), bounds.min.y()), bounds.max,
        Point(bounds.min.x(), bounds.max.y())}});
    for (size_t index = 1; index < layers.size(); ++index) {
        if (object.print()->canceled()) throw CanceledException();
        const Layer &upper = *layers[index];
        const Layer &lower = *layers[index - 1];
        Polygons support_polygons;
        visit_layer(lower, [&](const ExtrusionPath &path) {
            path.polygons_covered_by_width(support_polygons, 0.f);
        });
        // Reserve positive overlap after relative support/target coordinate
        // rounding. Erode the union, so real crossing/joining beads remain
        // connected while a vanishing overlap is never accepted as an anchor.
        const ExPolygons previous_material = offset_ex(union_ex(support_polygons), -scale_(geometry_tolerance));
        std::vector<ExPolygons> eligible(lower.regions().size());
        std::vector<ExPolygons> bridge_eligible(lower.regions().size());
        for (size_t region_id = 0; region_id < lower.regions().size(); ++region_id) {
            if (participates(lower.get_region(int(region_id))->region().config()))
                eligible[region_id] = sparse_domain(*lower.get_region(int(region_id)));
        }
        if (!dependencies) {
            // Carry the originating sparse region's policy through bridge
            // planes, including across a modifier ending below a bridge. The
            // bridge's own region is not necessarily the support owner.
            ExPolygons projection;
            for (const LayerRegion *region : lower.regions())
                for (const Surface &surface : region->fill_surfaces.surfaces)
                    if (surface.surface_type == stInternalBridge)
                        projection.push_back(surface.expolygon);
            for (size_t bridge_index = index - 1; bridge_index > 0 && !projection.empty(); --bridge_index) {
                ExPolygons bases, continuing;
                for (const LayerRegion *region : layers[bridge_index]->regions())
                    for (const Surface &surface : region->fill_surfaces.surfaces)
                        if (surface.surface_type == stInternalBridge)
                            append(surface.bridge_followup ? continuing : bases,
                                   intersection_ex(surface.expolygon, projection));
                if (!bases.empty()) {
                    const Layer &beneath_base = *layers[bridge_index - 1];
                    for (size_t region_id = 0; region_id < beneath_base.regions().size(); ++region_id)
                        if (participates(beneath_base.get_region(int(region_id))->region().config()))
                            append(bridge_eligible[region_id], intersection_ex(bases,
                                sparse_domain(*beneath_base.get_region(int(region_id)))));
                }
                projection = std::move(continuing);
            }
        }
        for (ExPolygons &domain : eligible) domain = union_ex(domain);
        for (ExPolygons &domain : bridge_eligible) domain = union_ex(domain);

        std::map<std::tuple<size_t, double, bool>, size_t> groups;
        auto add = [&](const ExtrusionPath &path, bool dependency, size_t emitting_region) {
            // A swept bead edge alone is not an unsupported extrusion. Select
            // actual air runs first and then use the real target bead width.
            // The original path below retains all endpoints and identities.
            const bool contact = upper.print_z - path.height <= lower.print_z + geometry_tolerance;
            const Polylines unsupported = unsupported_only && contact ?
                diff_pl(Polylines{path.polyline}, previous_material) : Polylines{path.polyline};
            if (unsupported.empty()) return;
            const ExPolygons footprint = union_ex(offset(unsupported, scale_(.5 * path.width)));
            for (size_t region_id = 0; region_id < eligible.size(); ++region_id) {
                for (bool followup : {false, true}) {
                    const ExPolygons &domain = followup ? bridge_eligible[region_id] : eligible[region_id];
                    if (domain.empty()) continue;
                    ExPolygons demand = intersection_ex(footprint, domain);
                    if (demand.empty()) continue;
                    const double bottom_z = upper.print_z - path.height;
                    const auto key = std::make_tuple(region_id, bottom_z, followup);
                    auto found = groups.find(key);
                    if (found == groups.end()) {
                        const auto &config = lower.get_region(int(region_id))->region().config();
                        found = groups.emplace(key, targets.size()).first;
                        targets.push_back({index, region_id, config.bridge_aware_cubic_max_gap.value,
                                           bottom_z, dependency, followup, {}, {}, {}});
                    }
                    Target &target = targets[found->second];
                    target.paths.push_back(path.polyline);
                    target.path_widths.push_back(path.width);
                    append(target.footprint, std::move(demand));
                }
            }
            // An added strand outside every eligible lower domain must still
            // have real prior material. A forbidden region is never filled to
            // hide an infeasible construction.
            if (dependency) {
                ExPolygons all_eligible;
                for (const ExPolygons &domain : eligible) append(all_eligible, domain);
                ExPolygons outside = diff_ex(footprint, all_eligible);
                if (!outside.empty()) {
                    const auto &config = upper.get_region(int(emitting_region))->region().config();
                    targets.push_back({index, emitting_region, config.bridge_aware_cubic_max_gap.value,
                                       upper.print_z - path.height, true, false, {path.polyline},
                                       {double(path.width)}, std::move(outside)});
                }
            }
        };

        if (!dependencies) {
            visit_layer(upper, [&](const ExtrusionPath &path) {
                if (path.role() != erInternalInfill && path.role() != erIroning)
                    add(path, false, 0);
            });
        } else {
            for (size_t region_id = 0; region_id < upper.regions().size(); ++region_id) {
                const auto *plan = object.bridge_aware_cubic_plan(region_id);
                if (plan == nullptr) continue;
                const Polylines added = plan->refinement_lines(upper.print_z, broad_domain);
                if (added.empty()) continue;
                const Polygons added_mask = offset(added, scale_(geometry_tolerance));
                visit_paths(upper.get_region(int(region_id))->fills, [&](const ExtrusionPath &path) {
                    if (path.role() != erInternalInfill) return;
                    const Polylines matched = intersection_pl(Polylines{path.polyline}, added_mask);
                    double matched_length = 0.;
                    for (const Polyline &line : matched) matched_length += line.length();
                    // Full emitted strand identity, not a clipped fragment.
                    if (matched_length + scale_(geometry_tolerance) >= path.polyline.length())
                        add(path, true, region_id);
                });
            }
        }
        for (const auto &group : groups)
            targets[group.second].footprint = union_ex(targets[group.second].footprint);
    }
    return targets;
}

struct Failure {
    Target target;
    BridgeAwareCubicValidation::Result result;
};

ExPolygons repair_footprint(const Failure &failure)
{
    ExPolygons affected;
    for (const auto &run : failure.result.unsupported_runs) {
        if (!run.exceeds_gap && run.kind == BridgeAwareCubicValidation::UnsupportedRunKind::TwoEndedBridge)
            continue;
        const Polyline &original = failure.target.paths.at(run.path_index);
        const double total = unscale<double>(original.length());
        // Diagnostic arclengths are approximate doubles. Expand along the
        // path before clipping, and radially after clipping, so a subresolution
        // failing interval cannot disappear during scaled-coordinate rounding.
        const double begin = std::max(0., run.begin_mm - geometry_tolerance);
        const double end = std::min(total, run.end_mm + geometry_tolerance);
        Polyline interval = original;
        interval.clip_end(scale_(std::max(0., total - end)));
        if (interval.size() >= 2) interval.clip_start(scale_(begin));
        if (interval.size() >= 2)
            append(affected, union_ex(offset(Polylines{std::move(interval)},
                scale_(.5 * failure.target.path_widths.at(run.path_index) + geometry_tolerance))));
    }
    for (const auto &cell : failure.result.void_cells)
        if (!cell.closed || cell.exceeds_gap)
            append(affected, offset_ex(cell.geometry, scale_(geometry_tolerance)));
    // The grouped target may contain many unrelated passing paths. Only a
    // violating run or void selects repair, and the original finite-width
    // demand remains the outer limit of that repair selection.
    return intersection_ex(union_ex(affected), failure.target.footprint);
}

std::vector<Failure> check_targets(const PrintObject &object, const std::vector<Target> &targets)
{
    std::vector<Failure> failures;
    size_t cached_lower = size_t(-1);
    ExPolygons support;
    ExPolygons domain;
    for (const Target &target : targets) {
        if (object.print()->canceled()) throw CanceledException();
        const Layer &lower = *object.layers()[target.upper_layer - 1];
        if (cached_lower != target.upper_layer - 1) {
            Polygons material;
            visit_layer(lower, [&](const ExtrusionPath &path) {
                path.polygons_covered_by_width(material, 0.f);
            });
            support = offset_ex(union_ex(material), -scale_(geometry_tolerance));
            domain = union_ex(lower.lslices);
            cached_lower = target.upper_layer - 1;
        }
        // Only preceding deposition is admissible: no reliance on same-layer
        // scheduling, a future wall, or a lower strand separated in Z.
        const bool contact = target.bottom_z <= lower.print_z + geometry_tolerance;
        // Model roof/wall targets require closed cubic cells in every
        // direction. Construction strands and follow-up bridge planes instead
        // bridge actual preceding deposition. Their complete unsupported runs
        // need two real anchors and must satisfy the inherited gap, but a short
        // growing partition tip does not require the adjacent coarse parent
        // cell to reach the final model-target cell size. Empty domain disables
        // only the cell test for these two explicit construction target kinds.
        auto result = BridgeAwareCubicValidation::validate(target.bridge_followup || target.dependency ? ExPolygons{} : domain,
            contact ? support : ExPolygons{},
            target.paths, target.footprint, target.gap, geometry_tolerance);
        if (!result.passed()) failures.push_back({target, std::move(result)});
    }
    return failures;
}

[[noreturn]] void fail(const PrintObject &object, const Failure &failure)
{
    const Layer &layer = *object.layers()[failure.target.upper_layer];
    std::ostringstream message;
    message << "Bridge aware cubic cannot satisfy the requested support gap for object \""
            << object.model_object()->name << "\", layer " << layer.id() + 1
            << " (Z " << std::fixed << std::setprecision(3) << layer.print_z << " mm), region "
            << failure.target.supporting_region + 1 << ": requested " << failure.target.gap
            << " mm; longest unsupported run " << failure.result.max_unsupported_mm << " mm";
    if (!failure.target.dependency && !failure.target.bridge_followup)
        message << ", largest opening " << failure.result.max_void_diameter_mm << " mm";
    message << ". ";
    double longest_one_sided = 0., longest_floating = 0.;
    for (const auto &run : failure.result.unsupported_runs) {
        if (run.kind == BridgeAwareCubicValidation::UnsupportedRunKind::OneSidedOverhang)
            longest_one_sided = std::max(longest_one_sided, run.length_mm);
        else if (run.kind == BridgeAwareCubicValidation::UnsupportedRunKind::Floating)
            longest_floating = std::max(longest_floating, run.length_mm);
    }
    if (longest_one_sided > 0.)
        message << "Longest one-sided run: " << longest_one_sided << " mm. ";
    if (longest_floating > 0.)
        message << "Longest run without either end anchored: " << longest_floating << " mm. ";
    if (!failure.result.failure_reason.empty()) {
        message << failure.result.failure_reason;
        const char last = failure.result.failure_reason.back();
        if (last != '.' && last != '!' && last != '?') message << '.';
        message << ' ';
    }
    message << (failure.target.dependency ? "An added partition lacks valid preceding-layer support. " :
                failure.target.bridge_followup ? "A follow-up bridge layer lacks adequately anchored preceding-layer support. " :
                "The final model extrusion lacks a closed, adequately anchored support cell. ");
    if (failure.target.bridge_followup)
        message << "Check Bridge Layer Density and bridge overlap, or provide additional printable support beneath this feature.";
    else
        message << "Increase Unsupported gap size or provide additional printable support beneath this feature.";
    throw SlicingError(message.str(), object.model_object()->id().id);
}

} // namespace

bool PrintObject::has_bridge_aware_cubic() const
{
    for (size_t region_id = 0; region_id < num_printing_regions(); ++region_id)
        if (participates(printing_region(region_id).config())) return true;
    return false;
}

const BridgeAwareCubic::Plan *PrintObject::bridge_aware_cubic_plan(size_t region_id) const
{
    if (m_shared_object != nullptr) return m_shared_object->bridge_aware_cubic_plan(region_id);
    return region_id < m_bridge_aware_cubic_plans.size() ? m_bridge_aware_cubic_plans[region_id].get() : nullptr;
}

const ExPolygons &PrintObject::bridge_aware_cubic_reservations(size_t layer_id, size_t region_id) const
{
    if (m_shared_object != nullptr) return m_shared_object->bridge_aware_cubic_reservations(layer_id, region_id);
    static const ExPolygons empty;
    return layer_id < m_bridge_aware_cubic_reservations.size() &&
        region_id < m_bridge_aware_cubic_reservations[layer_id].size() ?
        m_bridge_aware_cubic_reservations[layer_id][region_id] : empty;
}

void PrintObject::prepare_bridge_aware_cubic()
{
    m_bridge_aware_cubic_plans.clear();
    m_bridge_aware_cubic_plans.resize(num_printing_regions());
    m_bridge_aware_cubic_reservations.clear();
    if (!has_bridge_aware_cubic() || m_layers.empty()) return;

    if (config().seam_gap.value != 0.)
        throw SlicingError("Bridge aware cubic currently requires Seam gap = 0 because support validation precedes seam placement.", model_object()->id().id);
    for (const PrintRegion &region : all_regions()) {
        const auto &cfg = region.config();
        const bool scarf = cfg.override_filament_scarf_seam_setting.value ?
            cfg.seam_slope_type.value != SeamScarfType::None :
            std::any_of(print()->config().filament_scarf_seam_type.values.begin(),
                        print()->config().filament_scarf_seam_type.values.end(),
                        [](int value) { return value != int(SeamScarfType::None); });
        if (scarf)
            throw SlicingError("Bridge aware cubic currently requires Scarf seam type = None because support validation precedes scarf generation.", model_object()->id().id);
    }
    if (config().apply_scarf_seam_on_circles.value)
        for (const Layer *layer : m_layers)
            for (const LayerRegion *region : layer->regions())
                if (has_circle_scarf(region->perimeters))
                    throw SlicingError("Bridge aware cubic currently requires Apply scarf seam on circles disabled for compensated circular walls.", model_object()->id().id);

    const auto generate = [this] {
        for (Layer *layer : m_layers) {
            throw_if_canceled();
            layer->make_fills(m_adaptive_fill_octrees.first.get(), m_adaptive_fill_octrees.second.get(), m_lightning_generator.get());
        }
    };
    // Freeze the actual bridge angles/anchor geometry selected by the ordinary
    // bridge passes. The hierarchy retains the bulk scaffold and only adds
    // partitions; these exact upper paths are the planning targets.
    generate();
    const std::vector<Target> roofs = collect_targets(*this, false);
    std::vector<std::vector<BridgeAwareCubic::Demand>> demands(num_printing_regions());
    for (const Target &target : roofs) {
        if (target.bridge_followup) continue;
        const Layer &lower = *m_layers[target.upper_layer - 1];
        ExPolygons refinable = intersection_ex(target.footprint,
            sparse_domain(*lower.get_region(int(target.supporting_region))));
        if (!refinable.empty())
            demands[target.supporting_region].push_back({std::move(refinable), lower.print_z,
                                                        target.gap - geometry_tolerance});
    }

    const BoundingBox bounds = bounding_box();
    std::vector<BridgeAwareCubic::Parameters> parameters(num_printing_regions());
    for (size_t region_id = 0; region_id < num_printing_regions(); ++region_id) {
        const auto &config = printing_region(region_id).config();
        if (!participates(config)) continue;
        if (config.fill_multiline.value != 1 || config.conformal_infill.value)
            throw SlicingError("Bridge aware cubic requires Fill multiline = 1 and Conformal infill disabled.", model_object()->id().id);
        bool has_sparse = false;
        for (const Layer *layer : m_layers)
            if (!sparse_domain(*layer->get_region(int(region_id))).empty()) {
                has_sparse = true;
                break;
            }
        if (!has_sparse) continue;
        const double spacing = printing_region(region_id).flow(*this, frInfill, this->config().layer_height, false).spacing();
        auto &params = parameters[region_id];
        params.bounds = BoundingBoxf3(Vec3d(unscale<double>(bounds.min.x()), unscale<double>(bounds.min.y()), m_layers.front()->bottom_z()),
                                     Vec3d(unscale<double>(bounds.max.x()), unscale<double>(bounds.max.y()), m_layers.back()->print_z));
        params.flow_spacing = spacing;
        params.density = config.sparse_infill_density.value * .01;
        params.angle = Geometry::deg2rad(config.infill_direction.value);
        params.throw_if_canceled = [this] { throw_if_canceled(); };
    }

    std::vector<Failure> failures;
    for (unsigned pass = 0; pass < repair_limit; ++pass) {
        throw_if_canceled();
        for (size_t region_id = 0; region_id < num_printing_regions(); ++region_id) {
            if (parameters[region_id].flow_spacing <= 0.) continue;
            const auto contextual_error = [&](const std::exception &error) {
                std::ostringstream support_context;
                if (!demands[region_id].empty()) {
                    const auto z_range = std::minmax_element(demands[region_id].begin(), demands[region_id].end(),
                        [](const auto &a, const auto &b) { return a.z < b.z; });
                    const auto support_layer = [this](double z) {
                        auto found = std::lower_bound(m_layers.begin(), m_layers.end(), z - EPSILON,
                            [](const Layer *layer, double value) { return layer->print_z < value; });
                        return found == m_layers.end() ? m_layers.back()->id() + 1 : (*found)->id() + 1;
                    };
                    support_context << ", support layers " << support_layer(z_range.first->z)
                                    << '-' << support_layer(z_range.second->z) << " (Z "
                                    << std::fixed << std::setprecision(3) << z_range.first->z
                                    << '-' << z_range.second->z << " mm)";
                }
                return SlicingError("Bridge aware cubic for object \"" + model_object()->name + "\", region " +
                    std::to_string(region_id + 1) + support_context.str() + ", requested gap " +
                    std::to_string(printing_region(region_id).config().bridge_aware_cubic_max_gap.value) +
                    " mm: " + error.what(), model_object()->id().id);
            };
            try {
                m_bridge_aware_cubic_plans[region_id] = std::make_unique<BridgeAwareCubic::Plan>(parameters[region_id], demands[region_id]);
            } catch (const std::logic_error &error) {
                throw contextual_error(error);
            } catch (const std::runtime_error &error) {
                throw contextual_error(error);
            }
        }
        generate();
        failures = check_targets(*this, roofs);
        const auto dependencies = collect_targets(*this, true);
        auto dependency_failures = check_targets(*this, dependencies);
        failures.insert(failures.end(), std::make_move_iterator(dependency_failures.begin()),
                        std::make_move_iterator(dependency_failures.end()));
        if (failures.empty()) {
            // Reservation is separate from refinement demand. An added strand
            // already supported by a retained parent face must not split that
            // parent's otherwise coarse cells, but combination must retain the
            // actual predecessor scaffold. Store complete supplying leaf cells.
            m_bridge_aware_cubic_reservations.resize(m_layers.size());
            for (auto &regions : m_bridge_aware_cubic_reservations)
                regions.resize(num_printing_regions());
            auto reserve = collect_targets(*this, false, false);
            auto partition_support = collect_targets(*this, true, false);
            reserve.insert(reserve.end(), std::make_move_iterator(partition_support.begin()),
                           std::make_move_iterator(partition_support.end()));
            std::map<std::pair<size_t, size_t>, ExPolygons> selected;
            for (const Target &target : reserve)
                append(selected[{target.upper_layer - 1, target.supporting_region}], target.footprint);
            for (auto &entry : selected) {
                const size_t layer_index = entry.first.first;
                const size_t region_id = entry.first.second;
                const auto *plan = bridge_aware_cubic_plan(region_id);
                if (plan == nullptr) continue;
                const ExPolygons footprint = union_ex(entry.second);
                ExPolygons &reserved = m_bridge_aware_cubic_reservations[layer_index][region_id];
                for (const auto &cell : plan->leaf_sections(m_layers[layer_index]->print_z))
                    if (!intersection_ex(ExPolygon(cell.polygon), footprint).empty())
                        reserved.emplace_back(cell.polygon);
                reserved = union_ex(reserved);
            }
            return;
        }

        bool repaired = false;
        for (const Failure &failure : failures) {
            if (!failure.result.input_valid || !failure.result.completed) fail(*this, failure);
            // Changing sparse cubic partitions cannot repair spacing between
            // already emitted bridge planes. Report this setting conflict.
            if (failure.target.bridge_followup) fail(*this, failure);
            const size_t lower_index = failure.target.upper_layer - 1;
            const Layer &lower = *m_layers[lower_index];
            const ExPolygons affected = repair_footprint(failure);
            // Propagate the specific failed footprint into allowed sparse
            // material. Do not refine a roof's bounding box or a whole ancestor.
            for (size_t region_id = 0; region_id < lower.regions().size(); ++region_id) {
                if (parameters[region_id].flow_spacing <= 0.) continue;
                ExPolygons local = intersection_ex(affected, sparse_domain(*lower.get_region(int(region_id))));
                if (local.empty()) continue;
                const double gap = std::min(failure.target.gap,
                    printing_region(region_id).config().bridge_aware_cubic_max_gap.value) /
                    std::pow(2., failure.target.dependency ? pass : pass + 1);
                if (gap <= parameters[region_id].flow_spacing + geometry_tolerance) continue;
                demands[region_id].push_back({std::move(local), lower.print_z, gap - geometry_tolerance});
                repaired = true;
            }
        }
        if (!repaired) break;
    }
    if (!failures.empty()) fail(*this, failures.front());
}

void PrintObject::validate_bridge_aware_cubic() const
{
    if (!has_bridge_aware_cubic()) return;
    if (m_shared_object != nullptr) {
        m_shared_object->validate_bridge_aware_cubic();
        return;
    }
    for (size_t region_id = 0; region_id < num_printing_regions(); ++region_id) {
        if (!participates(printing_region(region_id).config()) || bridge_aware_cubic_plan(region_id) != nullptr)
            continue;
        for (const Layer *layer : m_layers)
            if (!sparse_domain(*layer->get_region(int(region_id))).empty())
                throw SlicingError("Bridge aware cubic support plan is missing for object \"" + model_object()->name +
                    "\". Reslice the object to rebuild and verify its support geometry.", model_object()->id().id);
    }
    auto failures = check_targets(*this, collect_targets(*this, false));
    if (!failures.empty()) fail(*this, failures.front());
    failures = check_targets(*this, collect_targets(*this, true));
    if (!failures.empty()) fail(*this, failures.front());
}

} // namespace Slic3r
