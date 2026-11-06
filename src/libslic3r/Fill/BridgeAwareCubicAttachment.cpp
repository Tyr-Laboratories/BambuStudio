#include "BridgeAwareCubicAttachment.hpp"

#include "BridgeAwareCubicValidation.hpp"
#include "../ClipperUtils.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace Slic3r { namespace BridgeAwareCubicAttachment {
namespace {

using Validation = BridgeAwareCubicValidation::Result;
using RunKind = BridgeAwareCubicValidation::UnsupportedRunKind;
constexpr coord_t coordinate_limit = std::numeric_limits<coord_t>::max() / 4;
constexpr size_t max_path_points = 1024;

ExPolygons bounding_demand(const Polyline &path, double width_mm)
{
    Point low = path.first_point(), high = low;
    for (const Point &point : path.points)
        for (size_t axis = 0; axis < 2; ++axis) {
            low[axis] = std::min(low[axis], point[axis]);
            high[axis] = std::max(high[axis], point[axis]);
        }
    const coord_t margin = coord_t(std::ceil(scale_(.5 * width_mm)));
    low -= Point(margin, margin);
    high += Point(margin, margin);
    return {ExPolygon(Polygon{Points{low, Point(high.x(), low.y()), high, Point(low.x(), high.y())}})};
}

ExPolygons footprint(const Polyline &path, double width_mm)
{
    return union_ex(offset(Polylines{path}, float(scale_(.5 * width_mm))));
}

bool strictly_inside(const ExPolygons &support, const Point &point)
{
    return std::any_of(support.begin(), support.end(), [&](const ExPolygon &polygon) {
        return polygon.contains(point, false);
    });
}

std::optional<double> terminal_air_length(const Validation &result, bool at_start)
{
    for (const auto &run : result.unsupported_runs) {
        if (run.kind == RunKind::Floating)
            return run.length_mm;
        if (run.kind == RunKind::OneSidedOverhang && (run.begin_mm == 0.) == at_start)
            return run.length_mm;
    }
    return std::nullopt;
}

// A new endpoint cannot repair a pre-existing overlong interior bridge.
bool unrepairable(const Validation &result)
{
    if (!result.input_valid || !result.completed)
        return true;
    return std::any_of(result.unsupported_runs.begin(), result.unsupported_runs.end(), [](const auto &run) {
        return run.kind == RunKind::TwoEndedBridge && run.exceeds_gap;
    });
}

std::optional<Polyline> extend_terminal(const Polyline &path, bool at_start, double air_length_mm,
                                      const ExPolygons &support, const ExPolygons &allowed_domain,
                                      const ExPolygons &original_footprint, double width_mm,
                                      double max_gap_mm, double tolerance_mm,
                                      double preferred_anchor_inset_mm, size_t skip_intervals)
{
    const Point &endpoint = at_start ? path.points.front() : path.points.back();
    const Point &neighbor = at_start ? path.points[1] : path.points[path.size() - 2];
    const Vec2d delta = endpoint.cast<double>() - neighbor.cast<double>();
    const double terminal_length = delta.norm();
    if (!std::isfinite(terminal_length) || terminal_length == 0.)
        return std::nullopt;
    const Vec2d direction = delta / terminal_length;

    const double margin_mm = std::max(tolerance_mm, SCALING_FACTOR);
    const double remaining_mm = max_gap_mm - margin_mm - air_length_mm;
    if (remaining_mm <= 0.)
        return std::nullopt;

    // Search slightly into the candidate anchor. Only the unsupported portion
    // spends the gap budget; the final validator measures it independently.
    double ray_length = scale_(remaining_mm + 4. * margin_mm);
    for (size_t axis = 0; axis < 2; ++axis) {
        if (direction[axis] > 0.)
            ray_length = std::min(ray_length, (double(coordinate_limit) - endpoint[axis]) / direction[axis]);
        else if (direction[axis] < 0.)
            ray_length = std::min(ray_length, (-double(coordinate_limit) - endpoint[axis]) / direction[axis]);
    }
    if (!std::isfinite(ray_length) || ray_length <= scale_(2. * margin_mm))
        return std::nullopt;
    const Vec2d ray_end = endpoint.cast<double>() + direction * ray_length;
    const Point far_point(coord_t(std::llround(ray_end.x())), coord_t(std::llround(ray_end.y())));
    if (far_point == endpoint)
        return std::nullopt;

    const Polyline ray(Points{endpoint, far_point});
    // Clipper may return a boundary-coincident prefix and a genuine interior
    // anchor as one collinear polyline. Derive strict interior intervals from
    // the verifier instead; its unsupported runs retain boundary transitions.
    const Validation ray_result = BridgeAwareCubicValidation::validate({}, support, {ray},
        bounding_demand(ray, width_mm), max_gap_mm, tolerance_mm);
    if (!ray_result.input_valid || !ray_result.completed)
        return std::nullopt;
    std::vector<std::pair<double, double>> intervals;
    double cursor = 0.;
    for (const auto &run : ray_result.unsupported_runs) {
        if (run.begin_mm > cursor)
            intervals.emplace_back(scale_(cursor), scale_(run.begin_mm));
        cursor = std::max(cursor, run.end_mm);
    }
    const double actual_ray_length = ray.length();
    if (scale_(cursor) < actual_ray_length)
        intervals.emplace_back(scale_(cursor), actual_ray_length);

    const double scaled_margin = scale_(margin_mm);
    for (const auto &interval : intervals) {
        if (interval.first > scale_(remaining_mm))
            break;
        if (interval.second - interval.first <= 2. * scaled_margin)
            continue;
        if (skip_intervals > 0) {
            --skip_intervals;
            continue;
        }
        // Exact contact can retain a real anchor thinner than output rounding.
        // Prefer its midpoint when requested, without enlarging that interval
        // or traversing a large supported island to its far edge. The caller
        // must still certify the actual serialized candidate in every instance.
        const double shallow_insertion = std::min(2. * scaled_margin, .5 * (interval.second - interval.first));
        const double preferred_insertion = std::min(std::max(2. * scaled_margin, scale_(preferred_anchor_inset_mm)),
                                                   .5 * (interval.second - interval.first));
        const std::array<double, 2> insertions{preferred_insertion, shallow_insertion};
        const size_t attempts = preferred_insertion == shallow_insertion ? 1 : 2;
        for (size_t attempt = 0; attempt < attempts; ++attempt) {
            const Vec2d position = endpoint.cast<double>() + direction * (interval.first + insertions[attempt]);
            const Point attached(coord_t(std::llround(position.x())), coord_t(std::llround(position.y())));
            const double attached_distance = (attached.cast<double>() - endpoint.cast<double>()).dot(direction);
            if (attached == endpoint || attached_distance <= interval.first || attached_distance >= interval.second ||
                !strictly_inside(support, attached))
                continue;

            Polyline candidate = path;
            if (at_start)
                candidate.append_before(attached);
            else
                candidate.append(attached);
            const ExPolygons candidate_footprint = footprint(candidate, width_mm);
            if (candidate_footprint.empty())
                return std::nullopt;
            const ExPolygons added_material = diff_ex(candidate_footprint, original_footprint);
            if (!diff_ex(added_material, allowed_domain).empty()) {
                // A preference must not discard the shallower original
                // placement when only that footprint fits the owning region.
                if (attempt + 1 < attempts) continue;
                return std::nullopt;
            }
            return candidate;
        }
    }
    return std::nullopt;
}

} // namespace

std::optional<Polyline> attach_endpoints(const Polyline &original,
                                         const ExPolygons &qualified_prior_support,
                                         const ExPolygons &allowed_bead_domain,
                                         double width_mm, double max_gap_mm,
                                         double geometry_tolerance_mm,
                                         double preferred_anchor_inset_mm,
                                         const ExPolygons *start_anchor_support,
                                         const ExPolygons *end_anchor_support,
                                         const TerminalAirLengths *terminal_air_lengths,
                                         const TerminalAnchorIntervals *anchor_intervals)
{
    if (!std::isfinite(width_mm) || width_mm < 2. * SCALING_FACTOR ||
        width_mm > double(coordinate_limit) * SCALING_FACTOR || original.size() < 2 || original.size() > max_path_points ||
        !std::isfinite(preferred_anchor_inset_mm) || preferred_anchor_inset_mm < 0. ||
        preferred_anchor_inset_mm > double(coordinate_limit) * SCALING_FACTOR ||
        original.first_point() == original.last_point() ||
        original.points[0] == original.points[1] ||
        original.points[original.size() - 1] == original.points[original.size() - 2] ||
        qualified_prior_support.empty() || allowed_bead_domain.empty())
        return std::nullopt;
    if ((start_anchor_support != nullptr || end_anchor_support != nullptr) && terminal_air_lengths == nullptr)
        return std::nullopt;
    if (terminal_air_lengths != nullptr)
        for (const auto &length : {terminal_air_lengths->start_mm, terminal_air_lengths->end_mm})
            if (length && (!std::isfinite(*length) || *length < 0.)) return std::nullopt;

    try {
        // Check numeric/coordinate/vertex bounds and arcs before constructing
        // even the broad demand polygon. Then use nonempty demand to invoke
        // the verifier's boolean-work guard before our unions and offsets.
        const Validation input = BridgeAwareCubicValidation::validate(allowed_bead_domain,
            qualified_prior_support, {original}, {}, max_gap_mm, geometry_tolerance_mm);
        if (!input.input_valid || !input.completed)
            return std::nullopt;
        const ExPolygons broad_demand = bounding_demand(original, width_mm);
        Validation result = BridgeAwareCubicValidation::validate({}, qualified_prior_support,
            {original}, broad_demand, max_gap_mm, geometry_tolerance_mm);
        if (unrepairable(result) || (result.passed() && terminal_air_lengths == nullptr))
            return std::nullopt;
        const Validation domain_check = BridgeAwareCubicValidation::validate({}, allowed_bead_domain,
            {original}, broad_demand, max_gap_mm, geometry_tolerance_mm);
        if (!domain_check.input_valid || !domain_check.completed)
            return std::nullopt;
        const ExPolygons support = union_ex(qualified_prior_support);
        const ExPolygons allowed_domain = union_ex(allowed_bead_domain);
        const ExPolygons original_footprint = footprint(original, width_mm);
        if (original_footprint.empty())
            return std::nullopt;
        const auto check = [&](const Polyline &path) {
            // Include original finite-width demand even after extension. Empty
            // model domain disables cells, never full-path anchor/gap checks.
            return BridgeAwareCubicValidation::validate({}, support, {path},
                union_ex(original_footprint, footprint(path, width_mm)), max_gap_mm, geometry_tolerance_mm);
        };
        result = check(original);
        if (unrepairable(result) || (result.passed() && terminal_air_lengths == nullptr))
            return std::nullopt;
        Polyline repaired = original;
        bool changed = false;
        for (bool at_start : {true, false}) {
            const ExPolygons *directional = at_start ? start_anchor_support : end_anchor_support;
            const ExPolygons &anchor_support = directional == nullptr ? support : *directional;
            // The union can contain certificates for unrelated directions.
            // Use the caller's existing complete-run classification instead
            // of inferring either terminal need or air length from that union.
            const auto air_length = terminal_air_lengths == nullptr ? terminal_air_length(result, at_start) :
                (at_start ? terminal_air_lengths->start_mm : terminal_air_lengths->end_mm);
            if (!air_length)
                continue;
            auto extended = extend_terminal(repaired, at_start, *air_length, anchor_support, allowed_domain,
                                            original_footprint, width_mm, max_gap_mm, geometry_tolerance_mm,
                                            preferred_anchor_inset_mm, anchor_intervals == nullptr ? 0 :
                                            (at_start ? anchor_intervals->start_skip : anchor_intervals->end_skip));
            if (!extended)
                return std::nullopt;
            repaired = std::move(*extended);
            changed = true;
            result = check(repaired);
            if (unrepairable(result))
                return std::nullopt;
        }
        if (changed && result.passed())
            return repaired;
    } catch (const std::exception &) {
        // Geometry failures are an unsuccessful repair, not an exemption from
        // the caller's final validation of its unchanged emitted geometry.
    }
    return std::nullopt;
}

ResolvedChain resolve_chain(const Polyline &original, double width, double height,
                            size_t emitting_region,
                            const std::vector<BridgeAwareCubic::LayerAttachment> &attachments,
                            size_t max_work)
{
    if (!std::isfinite(width) || width <= 0. || !std::isfinite(height) || height <= 0.)
        throw std::invalid_argument("Bridge aware cubic attachment chains require finite positive dimensions.");

    size_t remaining_work = max_work;
    const auto consume = [&](size_t amount) {
        if (amount > remaining_work)
            throw std::runtime_error("Bridge aware cubic attachment chain exceeded its work limit.");
        remaining_work -= amount;
    };
    const auto same_points = [&](const Polyline &first, const Polyline &second) {
        consume(1);
        if (first.size() != second.size()) return false;
        for (size_t i = 0; i < first.size(); ++i) {
            consume(1);
            if (first.points[i] != second.points[i]) return false;
        }
        return true;
    };
    const auto require_linear_path = [&](const Polyline &path) {
        consume(1);
        if (path.size() < 2)
            throw std::invalid_argument("Bridge aware cubic attachment chain has an invalid matched path.");
        for (const auto &fit : path.fitting_result) {
            consume(1);
            if (fit.path_type != EMovePathType::Linear_move)
                throw std::invalid_argument("Bridge aware cubic attachment chains require linear matched paths.");
        }
    };

    // Bound allocation as well as repeated scans; irrelevant decisions still
    // consume work, but cannot introduce ambiguity into the selected chain.
    consume(attachments.size());
    std::vector<bool> applied(attachments.size(), false);
    std::vector<size_t> indices;
    const Polyline *current = &original;
    for (;;) {
        std::optional<size_t> selected;
        for (size_t index = 0; index < attachments.size(); ++index) {
            consume(1);
            const auto &attachment = attachments[index];
            if (attachment.emitting_region != emitting_region || attachment.width != width ||
                attachment.height != height || !same_points(*current, attachment.original))
                continue;
            if (selected)
                throw std::invalid_argument("Bridge aware cubic attachment chain has ambiguous matching decisions.");
            selected = index;
        }
        if (!selected) {
            consume(current->points.size());
            consume(current->fitting_result.size());
            return {*current, std::move(indices)};
        }

        const size_t index = *selected;
        if (applied[index])
            throw std::invalid_argument("Bridge aware cubic attachment chain contains a cycle.");
        const auto &attachment = attachments[index];
        require_linear_path(*current);
        require_linear_path(attachment.original);
        require_linear_path(attachment.replacement);
        if (same_points(*current, attachment.replacement))
            throw std::invalid_argument("Bridge aware cubic attachment chain contains a no-op decision.");
        applied[index] = true;
        indices.push_back(index);
        current = &attachment.replacement;
    }
}

}} // namespace Slic3r::BridgeAwareCubicAttachment
