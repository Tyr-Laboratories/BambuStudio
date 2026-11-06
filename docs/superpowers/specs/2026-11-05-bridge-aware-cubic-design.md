# Bridge aware cubic infill design

Bridge aware cubic retains the requested sparse cubic density through the bulk of a part and recursively subdivides cells beneath upcoming extrusions. At every recursion, only children intersecting actual support demand are candidates for further subdivision; their siblings remain coarse unless a specific support or closure dependency requires refinement. The new **Unsupported gap size** setting bounds the air span those extrusions must cross. Refinement starts far enough below each target for the added partitions to grow from existing material.

Status: approved design, 5 November 2026; implementation in progress on `codex/bridge-aware-cubic`. This specifies the intended slicing behavior and acceptance criteria. Current implementation limits are recorded in `docs/bridge-aware-cubic.md`; the feature is not yet complete or print-qualified. Integration references describe the Tyr fork, including its existing bridge-layer changes.

## User controls and behavior

| Control | Meaning |
| --- | --- |
| Sparse infill pattern | New choice: **Bridge aware cubic**. |
| Sparse infill density | Nominal bulk density, using ordinary Cubic's spacing semantics. Local support refinement can exceed this value. |
| Unsupported gap size | Positive distance in millimeters: maximum straight air span across a closed opening beneath an extrusion requiring support. The emitted upper path is also checked for consecutive unsupported travel. |

Show the gap immediately below the pattern in Strength > Sparse infill. Serialize the pattern as `bridgeawarecubic` and the region option as `bridge_aware_cubic_max_gap`. Use normal process, object, part, and modifier inheritance. This is independent of the existing `max_bridge_length` support setting.

Proposed tooltip: “Subdivides cubic infill below upcoming solid layers and walls to keep unsupported spans within this distance. Smaller values add material and print time near those features. Choose a distance your filament and bridge settings can print reliably.”

A provisional setting default of 5 mm is a product-design choice, not a qualified bridge distance for any material. Existing projects keep their existing pattern; this value has no effect until the new pattern is selected. Zero density continues to mean no sparse infill, with the gap control inactive and no gap claim. At 100% retain the slicer's existing solid-infill behavior. Reject zero, negative, non-finite, or unrepresentable gap values.

The gap applies where non-sparse model extrusions are supported by eligible internal sparse space: the first solid layer above sparse infill, internal ledges and roofs, hole closures, and inward-moving walls. It also applies to unsupported bridge segments introduced by refinement. The coarse bulk lattice retains ordinary cubic behavior away from these demands. The setting does not impose a global short-span requirement on every ordinary bulk-infill move.

Refinement remains inside printable model material. Intentional holes, cavities, external air, zero-density regions, and nonparticipating infill regions are not automatically filled. Thin parts can require refinement through their entire available sparse depth. The part's average infill percentage can therefore exceed its bulk setting.

## Approaches and selected design basis

The user's requested structure is recursive closed cubic subdivision. The recommended implementation is a uniform bulk cubic lattice plus an independently controlled local subdivision hierarchy and final geometry verification.

| Approach | Tradeoff |
| --- | --- |
| Add a gap-derived spacing to existing Support cubic | Reuses most code, but its density determines the finest spacing and leaves the interior coarser. Its triangle proximity rule and path cleanup do not establish the requested bulk density or maximum gap. |
| Insert dense horizontal bands below roofs | Simple planning, but density changes can introduce unsupported strands abruptly and do not preserve the requested nested cubic cells. |
| Preserve bulk cubic and add nested, verified partitions | Matches the requested structure and separates bulk density from roof support. Requires new demand planning, support dependencies, and final gap verification. Recommended. |

Reuse the adaptive generator's rotation, recursive traversal, line extraction, and spatial indexing where useful. Give this pattern its own policy and plan. Existing Cubic, Adaptive cubic, and Support cubic behavior remains governed by their current generators.

Support cubic is relevant prior art: Prusa describes increasing its density toward top layers, while Adaptive cubic refines near surfaces. Neither description specifies this two-control contract or an emitted-path maximum-gap check. [Prusa infill documentation](https://help.prusa3d.com/article/infill-patterns_177130)

## Cell geometry and subdivision depth

Let `a` be the edge of an actual cubic cell, oriented with a body diagonal vertical. A horizontal section can be triangular or hexagonal. Its largest possible straight span is:

```text
Dmax = sqrt(2) × a
```

This is the horizontal face-diagonal bound, rather than the spatial body diagonal. To see it, a horizontal displacement in cube coordinates satisfies `du + dv + dw = 0`, with each coordinate difference at most `a`. The maximum squared length is `2a²`, attained by `(a, -a, 0)`.

After `k` subdivisions of a bulk cell with edge `a0`:

```text
ak = a0 / 2^k
Dk = sqrt(2) × a0 / 2^k
k_sizing = max(0, ceil(log2(sqrt(2) × a0 / G)))
```

Here `G` is the gap limit. `k_sizing` estimates sufficient depth for an ideal complete cell at any section height; it is not a command to refine the parent's entire subtree to that depth. Each selected split produces eight child volumes, after which demand and the gap condition are evaluated separately for each child. Where a region is uniformly refined, wall area per unit volume approximately doubles per level. Nominal local density there progresses approximately as `d`, `2d`, `4d`, and so on, until overlapping beads and geometric clipping make this approximation inappropriate. Actual deposited volume is computed from generated paths.

For ordinary single-line Cubic, with flow spacing `s` and fractional density `d`:

```text
horizontal pitch of each line family: p = 3s / d
physical cube edge:                  a0 = p × sqrt(2/3)
worst ideal horizontal span:         D0 = 2p / sqrt(3)
```

For example, assuming `s = 0.45 mm`, `d = 0.05`, and `G = 5 mm`:

| Refinement level | Nominal local density | Worst ideal horizontal span |
| --- | ---: | ---: |
| Bulk | 5% | 31.18 mm |
| First split | about 10% | 15.59 mm |
| Second split | about 20% | 7.79 mm |
| Third split | about 40% | 3.90 mm |

This table describes the levels along refined branches, not the average density of the original parent cell. Only children meeting demand or required support dependencies continue to the next level. This is a sizing example; 0.45 mm is assumed flow spacing, not an asserted nozzle diameter or bead width. Real bead widths normally reduce the air opening; final path geometry determines whether a particular leaf can stop earlier than the conservative sizing estimate.

The adaptive implementation has an important unit distinction: a fully populated node of stored edge `E` contributes three central partition planes, leaving physical child cells of edge `E/2`. Its minimum stored edge is twice its `line_spacing` parameter. Do not feed the ordinary Cubic family pitch directly into this parameter or mistake node edge for final cell edge. Establish the conversion with geometry tests.

## Find the extrusions that require support

Build support demand from sliced material and extrusion geometry, including perimeter widths and the final locations of internal bridge planes. Do not use only raw mesh triangles or only surfaces already classified as bridges.

For each target deposition plane:

1. Gather the actual swept footprints of non-sparse extrusion paths that project onto eligible sparse space below. Include small solid islands and narrow strips; there is no area threshold for exemption. Preserve gaps between separate paths. A line's axis-aligned bounding box or the containing roof polygon may accelerate discovery but must not replace the line footprint as final refinement demand.
2. Subtract coverage with adequate existing solid support. Ordinary vertical walls and a fully supported bottom need no extra infill.
3. Record the actual supporting deposition elevation, ownership, effective gap limit, target footprint, and relevant extrusion geometry. Account for thick-bridge cross sections and variable layer heights.
4. Select cubic cells whose actual horizontal sections intersect this demand on the supporting plane. At every split, intersect the demand again with each child's section and recurse only where the gap condition still fails. Include neighboring cells only for identified inherited-wall or connection dependencies. A clipped demand-mask edge is never counted as material or as a closing wall.

Use the lowest bridge in an internal bridge stack as the direct sparse-support target. Follow-up bridge layers are checked against what is actually printed below them. If a deliberately sparse first bridge layer leaves an excessive span in a later layer, that is a setting conflict to diagnose; refinement cannot assume that the intervening bridge plane is solid or silently change its density.

Demands are collected across region boundaries before support is assigned to the sparse regions allowed to supply it. The supporting region's gap setting governs its cells. An unsupported interval spanning multiple participating regions uses the strictest applicable limit. Nonparticipating regions remain outside the guarantee and are identified where they prevent certification of an otherwise covered target.

## Select the minimum required branches

Subdivision is a decision for each individual cell, repeated at every level. Passing a target depth from a selected parent to all descendants is prohibited. For a fixed lattice and fixed final demand, a split must have one of two recorded reasons: the leaf intersects demand and fails the gap condition, or an identified support/closure dependency cannot otherwise be satisfied.

The primary selection rule is:

```text
visit(cell, target_fragments):
    local_demand = intersect actual cell sections with target_fragments
    if local_demand is empty:
        keep cell as a leaf
    else if the cell satisfies the gap condition at every local demand plane:
        keep cell as a leaf
    else:
        split this cell once into eight children
        for each child:
            visit(child, the local_demand intersecting that child)
```

Retain original path identities and endpoints alongside clipped fragments. These intersections decide which cells need attention; they do not introduce new extrusion endpoints, anchors, or physical cell boundaries. Include finite bead width and the established geometry tolerance in intersection tests, without an arbitrary padding band. Multiple target fragments share one node and use the strictest applicable gap, without filling the spaces between them.

For a single line crossing a parent, the first necessary split creates eight children. Only children touched by the line's required support footprint are examined for another split. The same rule applies to their children, producing a narrow sequence of fine cells following the line and progressively coarser siblings around it. A child already within the gap limit stops even when its sibling needs another level. A line already supported by a retained wall creates no additional demand.

One split still introduces complete partition faces in that selected parent so its eight children remain closed. Their natural births and inherited faces are retained. This unavoidable partitioning cost does not authorize further subdivision of untouched siblings or cropping partition walls to the width of the supported line.

Support construction uses the same local selection discipline: propagate the specific added strand or missing connection that needs support, not its entire ancestor's footprint. Every extra split outside direct target demand records the dependent strand or closure constraint that requires it. Do not impose global 2:1 neighbor balancing; retain unequal neighboring depths when their common boundaries and print support are valid. Add a neighboring split only when a concrete geometry or deposition check requires it.

After target paths and their support elevations are stable, remove provisional branches without an active demand or dependency. Coarsen a split if replacing its descendants with the parent leaf still passes all affected gap, closure, and print-support checks. Remove orphaned dependencies and repeat until no such collapse succeeds. This establishes an irreducible tree for the chosen lattice and the stated checks; it does not claim a global material optimum across different lattice origins or alternative support structures.

## Grow subdivisions from existing material

Keep a stable cubic origin and orientation for each compatible lattice group. Bulk cells follow ordinary Cubic density and angle. Each selected refinement adds three midplanes to a physical parent cell and retains every required parent face. Only individually selected children are divided further.

Emit complete added faces over their available geometric extent. Do not turn fine lines on at an arbitrary height or crop them to a vertical projection of the roof. For a parent `[0,a]^3`, the new plane `u = a/2` naturally begins at `(a/2,0,0)` on an inherited edge. Relative to the parent's lowest corner, its birth elevation is `a/(2sqrt(3))`; the plane spans `2a/sqrt(3)` vertically. Its section grows continuously from that birth.

Starting at this natural birth gives a geometric route to the parent scaffold. Actual layer sampling, extrusion widths, and clipping must still establish support. Build the dependency closure downward through the specific needed faces and ancestors until it reaches valid existing material. Include neighboring refinement only when a recorded connection or deposition check requires it; a difference in refinement level alone is not a reason to split a neighboring cell.

The look-ahead depth is derived from cell geometry and dependencies in millimeters. It is not a fixed number of layers. The vertical extent of a bulk cube is `sqrt(3) × a0`; this is a useful scale for the transition, not a universal depth allowance after object clipping and neighboring dependencies.

Check each added strand against preceding deposited material. A strand may continue with valid bead overlap or form a bridge with real anchors at both ends and an unsupported run within `G`. Point contact, a future path, or an unprinted modifier boundary is not an anchor. Use the engine's extrusion cross-section and contact geometry, retaining the applicable anchoring requirements; any additional material-dependent overlap margin remains an implementation calibration parameter, not a third density control.

Where same-layer parent material is used as an anchor, emit it before dependent child material. Preserve this order through path connection, reversal, travel optimization, and G-code scheduling. If this dependency cannot be enforced, require support from a preceding layer instead. The existing adaptive filler's unrestricted reordering cannot be reused as proof of this property.

## Verify the actual remaining openings

The analytic cube formula supplies a sizing estimate for each selected branch. The authoritative check runs on the actual emitted geometry after clipping, hooks, line deletion, connection, width assignment, and infill combination decisions.

At every demand plane, build the footprint of material that will exist when that extrusion is deposited. Count supporting walls, solid fill, and sparse partitions only at physically valid contact elevations and in valid print order. Exclude material that is too low, deliberately separated, or not yet printed.

Two complementary checks are required:

1. **Direction-independent cell check.** Identify the actual closed void cells meeting demand. Require their polygon diameter to be at most `G`, using a conservative tolerance. For convex cells this equals the maximum straight chord. For concave clipped cells, the convex-hull diameter is a conservative bound. Check the whole affected cell rather than closing it artificially at the demand boundary. An opening through a missing partition joins the adjacent voids and must be measured as such.
2. **Actual extrusion check.** Intersect each required upper path with the usable support footprint. Measure every consecutive unsupported interval along the path; require its length to be at most `G` and the applicable end anchors to exist. For curved or segmented paths use path length, since a short cell diameter does not bound a curve's travel inside it. One-sided overhangs require the existing overhang-support rule and must not pass merely because they are shorter than `G`.

Retain cell ownership through generation, and use polygon booleans plus convex hulls for final measurements. This permits local checks instead of repeatedly polygonizing the entire layer. Protect required partition segments from cleanup or restore and revalidate them. One can choose more exact chord calculations later for efficiency; a conservative diameter bound is the initial contract.

Use a geometry tolerance derived from scaled-coordinate resolution and every path operation's maximum allowed displacement. Plan below the requested limit by that tolerance; never add tolerance to the user's maximum to make an oversized gap pass. Recheck after any downstream operation that can move or delete a support segment.

If a check fails, add the responsible subdivision and its support dependencies, regenerate affected paths, and recheck. Keep refinement monotone during a repair pass with fixed target paths, so repairs do not oscillate. Once demand is stable, perform the validated coarsening pass described above; monotonicity must not preserve obsolete exploratory branches. Cap refinement at representable and printable spacing, and cap allocated work. A cap stops with a precise unsatisfied-constraint result, not a successful slice.

At dense saturation, allow a local solid-filled region using the existing solid-fill machinery, confined to the necessary demanded cells and their construction dependencies. Do not replace the original bulk parent or a whole roof envelope merely because one descendant reaches saturation. Plan the solid region from a supported lower boundary and subject it to the same support checks. This is the limiting case of removing the remaining cells. It cannot be inserted as an unsupported dense cap. If no supported construction fits, stop G-code export for this pattern with the object, layer, requested gap, measured failure, and reason. Preview may show the affected area. Never silently increase `G` or report compliance after a failed check.

## Slicing pipeline and ownership

Introduce an immutable `BridgeAwareCubicPlan` owned by `PrintObject`, with per-region lattice parameters, target planes and path fragments, individually selected refinement nodes and their reasons, support dependencies, and reserved deposition domains. A dedicated filler consumes it for both anchoring previews and final infill.

The intended dependency order is:

```text
Slice model and generate walls
  -> classify shells and determine internal bridge-stack geometry
  -> discover potential support areas and choose provisional upper paths
  -> intersect actual path demand at every recursion and build support dependencies
  -> reserve required sparse deposition from layer combination/removal
  -> generate candidate supports and choose/reconstruct bridge anchors
  -> generate final candidate paths and verify cells, paths, and order
  -> refine/regenerate affected data until valid or explicitly infeasible
  -> freeze target paths, coarsen unnecessary branches, and revalidate
  -> freeze the plan and use the certified geometry for output
```

Separate bridge footprint/base placement from angle/anchor reconstruction. Conservative surface masks can discover possible support areas before a direction is known, but final subdivision must follow actual emitted path footprints. Generate provisional paths using the bulk support and current direction rules, refine against those paths, then reconstruct anchors using the resulting support. If reconstruction changes a path or its supporting elevation, update its identified demand and rerun affected branches and anchors. Freeze the chosen upper geometry before final coarsening; do not reselect its angle merely because coarsening changes the candidate supports. Anchor extensions may change only if the retained geometry passes the same support checks, and any resulting demand change is revalidated. Bound this solve and report non-convergence instead of keeping a permanent union of every abandoned direction or exploratory roof mask. Unaffected layers may run in parallel; a plan under refinement must not be read as immutable final data.

Exclude the complete transition and its dependency layers from infill combination. Also preserve these domains from “infill only where needed” trimming. Other bulk regions can continue to use their usual optimization rules. Candidate and final generation must share the same clipping, spacing, hooks, extrusion widths, and support geometry. G-code scheduling must preserve recorded dependencies; output transformations that change relevant geometry invalidate its certificate.

Preserve each region's density, gap, angle, width, and material ownership. Do not average them. Compatible groups can share a lattice. Arbitrary density or angle changes can require distinct lattices with a common object coordinate origin; they cannot all share one dyadic hierarchy. At their interfaces, close the demanded cells using actual allowed extrusion within the adjacent domains, with shared segments deduplicated and assigned to one region. These boundary cells can be clipped or irregular. A modifier boundary alone is not a physical partition. If closure requires material in a forbidden region, report the affected target as infeasible.

For the first implementation, use one line per cubic wall and reject a conflicting multiline setting with an actionable message. Multiline changes physical openings and contact geometry and needs its own certification before being enabled. Honor infill direction explicitly; the current adaptive filler ignores the passed direction.

## Integration locations

Paths below are relative to the repository root. Existing uncommitted changes at these locations must be retained when implementation begins.

| Area | Required integration |
| --- | --- |
| `src/libslic3r/PrintConfig.hpp`, `PrintConfig.cpp` | Append the new enum without changing existing values; register the serialized pattern and positive gap option. |
| `src/libslic3r/Preset.cpp`; GUI `Tab.cpp`, `GUI_Factories.cpp`, `ConfigManipulation.cpp` | Persist the scalar setting, expose inheritance, control visibility, and validate incompatible combinations. |
| `src/libslic3r/Print.hpp`, `PrintObject.cpp` | Own the plan, separate demand preparation from bridge reconstruction, protect required layers from combination, and invalidate affected stages. |
| `src/libslic3r/Fill/FillBase.cpp` | Instantiate a dedicated filler. |
| `src/libslic3r/Fill/FillAdaptive.cpp` | Reuse geometric machinery where appropriate; retain explicit physical-cell units, per-child demand tests, refinement reasons, and dependency-aware coarsening. |
| `src/libslic3r/Fill/Fill.cpp` | Pass the same plan to anchor and final generation; include region/plan identity in fill grouping; protect necessary partition segments. |
| Extrusion ordering and G-code generation | Preserve parent-before-child dependencies and any geometry certificate assumptions. |
| `tests/fff_print` | Measure final paths and void geometry, extend bridge-stack/modifier/reslice fixtures, and test legacy-pattern preservation. |

Relevant source anchors at the inspected checkout are ordinary Cubic's three sweeps in `FillRectilinear.cpp:3163`; adaptive region averaging at `FillAdaptive.cpp:315`; center-plane extraction at `FillAdaptive.cpp:449`; short-line cleanup at `FillAdaptive.cpp:883`; cube properties at `FillAdaptive.cpp:1431`; filtered bridge candidates at `PrintObject.cpp:2941`; initial tree preparation at `PrintObject.cpp:3105`; bridge-stack demand rebuilding at `PrintObject.cpp:3694`; and infill combination at `PrintObject.cpp:4475`.

Changing the gap invalidates infill preparation and everything downstream. Changes to layer heights, geometry, wall/shell layout, extrusion widths, infill angle/density/pattern, bridge stacking or density, and region ownership also invalidate the relevant plan. Do not leave a setting at an infill-only invalidation stage when it changes support demand. Cache keys include all effective inputs; determinism must not depend on worker scheduling.

## Acceptance criteria

| Case | Required evidence |
| --- | --- |
| 5% cube with a flat roof | Bulk pitch matches ordinary Cubic outside the transition; each demanded final cell and upper unsupported path passes the selected gap. |
| One supported line crossing a large cell | Repeat the footprint intersection at every level; untouched siblings remain leaves unless an identified support/closure dependency requires splitting. Compare node count and material with uniform refinement to the same terminal level. |
| Diagonal, curved, boundary-aligned, and separated lines | Refinement follows actual finite-width footprints rather than their bounding boxes; shared nodes are deduplicated, spaces between lines stay coarse, and numerical boundary cases preserve coverage. |
| Minimality in a fixed lattice | Every split has an active gap failure or support/closure reason. Removing a target releases orphaned branches; after coarsening, no remaining split can be collapsed with its resulting orphaned dependencies while retaining all required checks. |
| Unequal neighboring depths and early stopping | A coarse neighbor stays coarse when its interface and support are valid. A demanded child whose actual section already passes stops before the worst-case sizing depth. |
| Replaced provisional direction or elevation | The final hierarchy matches a fresh deterministic solve for the final target geometry, with no subdivisions retained solely for abandoned paths. |
| Several gap values and lattice phases | Smaller gaps cause adequate extra refinement; limits hold at every tested roof Z and XY phase, including triangle/hexagon sections. |
| Partial roofs, tiny islands, ribs, sloped walls, and hole closures | Demand is local, omitted bridge-classifier candidates still receive support, and intentional voids remain empty. |
| Subdivision birth and print order | Added strands have prior support or valid short bridges; no child depends on material printed later. |
| Variable layer height and thick bridges | Contact uses real deposition height and cross section; transition paths retain sufficient support. |
| Combined infill and infill trimming | Required support and dependency planes survive; skipped-layer paths are never counted as deposited material. |
| Internal bridge layers 1, 2, and 3 | Sparse refinement targets the actual lowest plane; follow-up layers use actual lower coverage and conflicting bridge density is diagnosed. |
| Region and modifier changes | Each region retains its bulk density and gap; seams are physically closed or reported; forbidden volumes remain unchanged. |
| Cleanup, connection, and export | The gap still holds after every path-changing operation and final scheduling. Preview and output describe the same supports. |
| Tiny gaps, very low density, and insufficient depth | Bounded runtime, saturation confined to necessary cells and dependencies where feasible, and explicit failure where the limit cannot be met. |
| Save, reopen, and reslice | Settings survive presets/3MF, invalidation is complete, and deterministic regeneration preserves the result. |
| Existing infill patterns | Their geometry and settings remain unaffected by the new pattern. |

Use independent measurement helpers rather than comparing the planner to its own depth calculation. Sweep roof heights and lattice offsets; test intentionally broken walls to show the validator rejects false closure. Validate representative exported extrusion geometry as well as internal path objects. Record time, peak memory, additional material, and localized refinement extent on small and large fixtures; optimize after correctness is established.

Physical qualification is separate: print low-density coupons with identical material and process settings, compare ordinary Cubic and Bridge aware cubic, and inspect the first roof layer as well as the finished top. The feature can enforce a geometric bridge limit. Consistent solid tops additionally depend on a usable selected gap, anchoring, flow, cooling, and material behavior. It must not present geometric compliance as proof of print quality.
