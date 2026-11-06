# Bridge aware cubic implementation plan

> **For agentic workers:** Use superpowers:subagent-driven-development or superpowers:executing-plans for implementation. Preserve unrelated working changes and use the approved spec as authority. One controller owns shared CMake changes and builds.

**Goal:** Add Bridge aware cubic with ordinary Cubic bulk spacing, selective recursive subdivision under actual extrusion footprints, and checks on emitted support gaps and construction.

**Status:** The source work below has passed an automated correctness checkpoint. Delivery is held for the user's additional mandatory performance limit: end-to-end slicing must be no more than twice stock official Bambu Studio using regular Cubic. The current implementation is substantially slower and requires optimization plus renewed correctness and performance verification.

**Architecture:** `PrintObject` owns per-region immutable plans. A physical cubic hierarchy supplies ordinary bulk walls, local partitions, and saturated local solid domains. Candidate non-sparse paths establish support demand after bridge-stack placement. Separate contact, validation, attachment, and dependency modules check actual deposition, repair local failures, and remove unnecessary construction before the final plan is used for output.

**Tech stack:** C++17, Eigen, Clipper, Catch2, CMake/Ninja, and the existing wxWidgets configuration UI.

**Spec:** [Approved design](../specs/2026-11-05-bridge-aware-cubic-design.md). Keep the approved specification unchanged unless a separate change is authorized.

## Global constraints

- Serialize the pattern as `bridgeawarecubic` and the positive scalar millimeter option as `bridge_aware_cubic_max_gap`; the provisional default is 5 mm.
- Preserve ordinary Cubic bulk spacing. Zero density disables sparse fill; 100% routes to Rectilinear fill.
- Re-evaluate the actual finite-width demand in every child. Unneeded siblings remain coarse, and a neighboring depth difference alone never requires refinement.
- Keep natural partition births, inherited faces, region ownership, real support elevations, and explicit dependencies. A logical modifier edge or clipped demand edge cannot supply material or an anchor.
- Use only preceding deposition for support evidence; do not rely on same-layer sorting to make a future parent strand available.
- Preserve the independently released bridge controls and unrelated working changes. Treat source verification, application installation, release publication, and physical printing as separate actions and evidence levels. Delivery follows the user's explicit instructions recorded below; physical printer work remains outside this task.
- Meet the user's subsequent 2× stock-official regular-Cubic end-to-end slicing limit using identical persistent model/settings fixtures. Measure actual slicing through G-code export separately from application startup and project packaging. Do not satisfy the limit by weakening geometry, anchoring, ownership, work-limit, or irreducibility requirements.

## Implemented source work

### 1. Settings and pattern integration

**Files:** `PrintConfig.{hpp,cpp}`, `Preset.cpp`, `Fill/FillBase.cpp`, `Fill/Fill.cpp`, `Layer.cpp`, and GUI `Tab.cpp`, `GUI_Factories.cpp`, `ConfigManipulation.cpp`, `ParamTooltip.cpp`.

- [x] Append the enum without renumbering existing patterns; register the scalar setting, persistence, inheritance, and conditional UI.
- [x] Instantiate a dedicated filler and keep distinct region plans from merging during fill grouping.
- [x] Preserve zero/full-density behavior and reject unrepresentable positive densities and invalid gap values.
- [x] Invalidate preparation when settings change demand, contact, ownership, or emitted geometry; reject reuse of extrusion caches without a plan.
- [ ] Record the final configuration, persistence, UI-build, and legacy-pattern verification results.

### 2. Physical cubic hierarchy and local solid saturation

**Files:** `Fill/BridgeAwareCubic.{hpp,cpp}`, `Fill/FillBridgeAwareCubic.hpp`.

- [x] Use actual cubic cell edge `a`, with ordinary Cubic family pitch `p = 3s/d`, `a = p sqrt(2/3)`, and ideal horizontal diameter bound `sqrt(2) a`.
- [x] Intersect each child's actual section with current finite-width demand; stop locally and retain untouched siblings.
- [x] Emit complete selected midplanes from their natural births, retain ancestor faces, and deduplicate shared collinear intervals.
- [x] Expose stable lattice-scoped cell identities, refinement reasons, solid sections, and immutable closure/attachment payloads.
- [x] Detect local density saturation before emitting overlapping cubic walls beyond the supported density. The integrated filler emits two perpendicular half-density line families within the affected local leaf sections, at a combined nominal density of 100%, and clips beads to actual material. Final contact-plane openings and supporting predecessors are still verified.
- [x] Bound coordinates, allocation, recursion, work, and cancellation. Solid masks remain construction obligations subject to the same deposition checks.
- [ ] Record final geometry, phase, saturation, and construction acceptance results.

### 3. Actual contact and emitted-geometry validation

**Files:** `Fill/BridgeAwareCubicContact.{hpp,cpp}`, `Fill/BridgeAwareCubicValidation.{hpp,cpp}`, `BridgeAwareCubicSupport.cpp`.

- [x] Derive contact from actual emitted width, height, and serialized deposition elevation using rounded-rectangle or circular bead sections at one common plane.
- [x] Keep undilated physical lower sections separate from direction-dependent upper-path contact evidence. Finite line contacts do not become inflated cell walls, and isolated points do not become anchors.
- [x] Restrict vertical snapping to representation roundoff; an XY margin cannot close a real vertical gap. Unsupported shapes, fitted arcs, invalid input, and exhausted contact work fail closed.
- [x] Measure complete unsupported path intervals, preserving endpoints across demand masks and region seams, with explicit two-ended, one-sided, and floating classifications.
- [x] Measure whole actual void components selected by roof/wall demand. An opening through a missing face joins the adjacent voids; material and cavity boundaries are never invented from demand clipping.
- [x] Check construction strands and follow-up bridges against actual preceding deposition without imposing final roof-cell sizing on their surrounding coarse cells.
- [ ] Record final contact, anchoring, void-topology, variable-height, and thick-bridge verification results.

### 4. Local repair, interface ownership, and deposition reservations

**Files:** `Fill/BridgeAwareCubicAttachment.{hpp,cpp}`, `Fill/BridgeAwareCubicSupport.hpp`, `BridgeAwareCubicSupport.cpp`, `Fill/Fill.cpp`, `PrintObject.cpp`.

- [x] Freeze candidate upper geometry after ordinary bridge placement. Collect non-sparse path demand and carry the supplying sparse region's gap policy through follow-up bridge planes.
- [x] Extend only genuine free endpoints along their terminal tangents into contact-qualified preceding material. Keep original points, enforce the emitting region's permitted bead footprint, and revalidate the whole replacement path.
- [x] Store repairs by original path identity and replay them during final generation; do not extend closed-loop joins or interior multipath joins.
- [x] Build physical interface partitions for incompatible modifier lattices using explicit storage/emitting ownership. Limit construction to compatible positive-density regions using the same sparse-infill filament; deduplicate coincident courses without deleting crossing joints.
- [x] Continue interface courses downward until preceding material passes the contact/span check. If a valid construction cannot be found within the solve, reject it.
- [x] Restrict refinement repair to failing strand fragments or affected voids, preserving each fragment's producing cell or explicit closure identity.
- [x] Reserve both actual transition paths and their predecessor scaffold from combination and trimming. An already-supported partition does not need extra refinement merely to preserve its supporting bulk cells.
- [ ] Record final modifiers, endpoint repair, ribs/inward walls, cavities, combination, and reslice verification results.

### 5. Dependency pruning and final coarsening

**Files:** `Fill/BridgeAwareCubicDependencies.{hpp,cpp}`, `Fill/BridgeAwareCubic.{hpp,cpp}`, `BridgeAwareCubicSupport.cpp`.

- [x] Resolve dependencies across region-scoped lattice identities from direct demand roots and actual surviving producers. A different region's same-numbered cell or a surviving ancestor cannot impersonate the removed producer.
- [x] Remove unreachable obligations, including unrooted cycles, and rebuild affected plan demand without retaining obsolete exploratory branches.
- [x] Trial collapsing active subdivisions and solid leaves, regenerate affected layers, and check the affected frozen roof paths plus construction strands against their actual preceding deposition.
- [x] Trial removing complete interface construction stacks and release their dependent obligations. Repeat accepted reductions until no candidate collapse succeeds.
- [x] Reject work-limit exhaustion before claiming the chosen plan is irreducible. This procedure is bounded to the chosen lattice and available collapse operations, not a global material optimum.
- [ ] Record final ownership, orphan-pruning, coarsening, determinism, and runtime evidence.

### 6. Output precision and post-verification constraints

**Files:** `GCodeWriter.{hpp,cpp}`, `GCode.cpp`, `Print.cpp`, `PrintObject.cpp`, `BridgeAwareCubicSupport.cpp`.

- [x] Use the same serialized deposition Z values in contact checks and motion output. Current source selects 12 fractional Z digits for prints containing Bridge aware cubic and retains the ordinary three-digit path otherwise.
- [x] Preserve sufficient emitted width/height annotation precision for exported-geometry checks. Check actual XY output conversion, rounding, and skipped source segments at each instance placement, with a fresh check before export.
- [x] Reject different configured XY nozzle offsets until tool-specific physical contact handling is available.
- [x] Reject Spiral vase, nonzero seam gap, scarf transformations, incompatible Multiline, and Conformal infill when this pattern participates. Reject compensated circular loops that would still receive scarf seams.
- [x] Preserve participating objects' wall/infill paths instead of applying subsequent simplification or arc fitting that would invalidate the checked geometry.
- [ ] Record the final build and exported-coordinate verification after the precision changes settle.

## Qualification and delivery work

The test sources cover configuration/persistence, independent lattice geometry, validator topology, rounded-bead contact, attachment constraints, dependency ownership, local repair, modifiers, combination, reslicing, bridge stacks, demand phases, inward walls, variable heights, cavities, and actual G-code coordinates. Test presence does not establish their final result.

- [ ] Run the final feature suite, including acceptance and export tests, after all source changes land.
- [ ] Run relevant existing bridge, wall, flow, and serialization regressions; distinguish feature regressions from separately reproduced baseline failures.
- [ ] Finish independent review and rerun checks affected by any correction.
- [ ] Record runtime, refinement/material extent, and any work-limit behavior on representative fixtures. Small acceptance fixtures alone do not qualify large-model performance.
- [ ] Update the user documentation and this record with the final bounded evidence, without implying application installation or printer acceptance.
- [ ] Perform physical coupon qualification separately with the intended printer, filament, temperatures, flow, cooling, and bridge settings, inspecting the first roof layer and finished top.

The geometric bead model does not establish adhesion, sag resistance, or production print quality. A checked source path, an exported G-code file, an installed application, and a successful physical print are separate evidence levels.

## Execution record

- The feature branch began at `66a45f8f7271420dd0c75681b5b7ea4aa3598298` with the existing dirty state retained and snapshotted.
- The user approved the design and instructed implementation. Routine source completion and verification do not require another design-approval round.
- The user subsequently instructed committing all changes, testing them, merging the feature into the main branch after tests pass, and directing the release agent to build a release containing everything. The active delivery objective also authorizes replacing the local application and publishing the new release to the fork. These actions remain conditional on successful source verification.
- The branch was advanced to the released bridge-controls baseline `28aed765070b663ae4938b93435b87c9680c2a24` (`v2.8.4.57-tyr.17`), preserving the feature changes and unrelated artifacts.
- Earlier reviews identified saturation, modifier closure, endpoint attachment, physical contact, stale ownership, export precision, and cache gaps. The source now contains the corresponding construction and verification mechanisms described above; final combined qualification remains pending.
- A Lightning repeat-slice anchor-ratio failure was also reproduced using the unmodified release-baseline executable. Its tree ordering has since been made deterministic; the final combined verification below must include the bridge-controls suite.
- Shared builds and final test execution remain under one controller. The release agent will handle the authorized application and release delivery after the tested merge; physical printer qualification remains separate.

## Final verification

The 6 November correctness checkpoint passed the full application build, all 243 default feature cases (245 executions), both opt-in qualification cases, all 17 existing CTest targets, and all 44 packaging tests. Feature tests totaled 290,500 passing assertions. The detailed fixture timings, material, refinement extent, and limits are recorded in [the feature documentation](../../bridge-aware-cubic.md#correctness-checkpoint-and-outstanding-performance-gate).

This is not release acceptance. The small and large fixture process-plus-export times were approximately 34.74 and 673.64 seconds, compared with about 0.026 seconds for ordinary Cubic in the same fork. Stock-official timing has not yet been established. The user added the 2× stock-official limit before merge, and it remains unsatisfied.

- [ ] Establish a verified stock-official baseline and exact model/settings parity.
- [ ] Profile complete planning, repair, coarsening, verification, and export costs.
- [ ] Eliminate repeated whole-plan and geometry work while retaining the approved support and irreducibility contracts.
- [ ] Repeat all affected correctness checks and prove the 2× end-to-end performance gate before merging, installing, or publishing.
