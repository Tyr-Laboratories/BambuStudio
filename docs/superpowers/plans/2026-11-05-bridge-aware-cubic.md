# Bridge aware cubic implementation plan

> **For agentic workers:** Use superpowers:subagent-driven-development or superpowers:executing-plans to implement these tasks. Preserve the pre-existing working changes and use the approved spec as authority.

**Goal:** Add Bridge aware cubic with bulk-density preservation, selective recursive subdivision under actual extrusion footprints, and checks on emitted support gaps.

**Architecture:** A region-owned immutable cubic hierarchy supplies both sparse toolpaths and the transition domains that must survive infill combination. Demand comes from actual candidate upper extrusion paths; every recursive child intersects that demand independently. Final generated paths are checked before successful slicing.

**Tech Stack:** Existing C++17 slicer, Eigen geometry, Clipper polygon operations, Catch2, CMake/Ninja, wxWidgets configuration UI.

**Spec:** `docs/superpowers/specs/2026-11-05-bridge-aware-cubic-design.md`

## Global constraints

- Pattern `bridgeawarecubic`, label `Bridge aware cubic`, positive scalar millimeter option `bridge_aware_cubic_max_gap`, provisional default 5 mm.
- Sparse density retains ordinary Cubic bulk spacing. Zero density disables sparse fill; 100% uses existing solid handling.
- Every recursion independently tests actual finite-width extrusion footprint and its gap requirement. Unneeded siblings remain coarse; global neighbor balancing is prohibited.
- Retain natural partition births, inherited walls, region ownership, required deposition planes, and explicit support dependencies.
- Preserve existing bridge controls and all unrelated working changes. No printer, installed-app, cloud, or deployment actions.
- The branch is `codex/bridge-aware-cubic`. Pre-existing changed files are snapshotted outside the repository; do not commit unrelated work.

## Review focus

- A line or two separated lines must not refine their entire containing roof rectangle.
- Modifier seams and negative-volume holes are not support walls; ownership and actual deposition govern clipping.
- Bridge-stack base relocation and infill combination must not remove presumed supports.
- Tiny gaps, thin parts, absent anchors, and topology cleanup must fail explicitly when the bound cannot be established.
- Changing gap, direction, geometry, or region settings must remove stale branches and regenerate deterministic output.

### Task 1: Persistent settings and user controls

**Files:** `src/libslic3r/PrintConfig.{hpp,cpp}`, `src/libslic3r/Preset.cpp`, `src/slic3r/GUI/{Tab,GUI_Factories,ConfigManipulation}.cpp`, `tests/fff_print/test_bridge_aware_cubic_config.cpp`.

**Interfaces:** Append `ipBridgeAwareCubic` before `ipCount`, register `bridgeawarecubic`, and add `ConfigOptionFloat bridge_aware_cubic_max_gap` to region config and its macro registration. The setting is a scalar across nozzle variants. Integration consumes these names verbatim.

- [x] Add failing real config round-trip and invalid-value tests.
- [x] Register the enum, setting, preset/inheritance lists, conditional UI, and multiline incompatibility.
- [x] Run the focused tests and review configuration consumers.

### Task 2: Selective cubic hierarchy and geometry

**Files:** new `src/libslic3r/Fill/BridgeAwareCubic.{hpp,cpp}` and `tests/fff_print/test_bridge_aware_cubic_geometry.cpp`; source/test CMake registration coordinated by the parent implementer.

**Interfaces:** Namespace `Slic3r::BridgeAwareCubic`; immutable `Plan` built from per-region `Parameters` and `std::vector<Demand>`. A demand carries physical support Z, finite-width `ExPolygons` footprint, and gap. Parameters carry bulk flow spacing, fractional density, infill angle in radians, world bounds, and cancellation callback. Expose `Polylines lines(double z, const ExPolygon &domain) const` for complete bulk plus selected partitions; `ExPolygons refinement_area(double z) const` for protecting transition material; and inspectable geometric leaf/split statistics for diagnostics. Agree precise header types with the integrating implementer before coding.

- [x] Write focused tests for no-demand bulk pitch, one line vs uniform subdivision, separate lines, actual-section early stopping, replaced demand, variable Z and angle, and invalid/bounded inputs.
- [x] Implement physical cubic cells and full midplane births, recursive per-child demand clipping, stopping at the local cell gap, deduplicated shared partitions, cancellation and bounded work.
- [x] Preserve coarse siblings and expose exact transition domains; rebuild from current demand to discard historical branches.
- [x] Run geometry tests with independently computed span/coverage assertions and review the hierarchy.

### Task 3: Slicing integration and final support verification

**Files:** new inline filler `src/libslic3r/Fill/FillBridgeAwareCubic.hpp` and `src/libslic3r/BridgeAwareCubicSupport.cpp`; modify `FillBase.{hpp,cpp}`, `Fill.cpp`, `Print.{hpp,cpp}`, `PrintObject.cpp`, `Layer.cpp`, source/test CMake; new `tests/fff_print/test_bridge_aware_cubic.cpp`.

**Interfaces:** Dedicated filler consumes `const BridgeAwareCubic::Plan*` for its surface's region and uses ordinary Cubic during provisional generation. `PrintObject` owns region plans and prepares them from actual upper fill/perimeter footprints at final bridge base elevations. The same plan supplies anchoring and final generation. Protected domains are subtracted from infill-combination candidates.

- [ ] Add failing full-slicing fixtures for 5% cubes, partial roofs, inward walls, bridge stacks, gap changes, modifiers, and combination.
- [x] Generate provisional non-sparse paths, collect precise demands across eligible regions, build plans, and reserve required transition/support material before layer combination.
- [x] Wire factory, grouping identity, flow/angle/region settings, anchoring and final filling. Keep other patterns unchanged.
- [ ] Verify final actual support footprints, relevant closed cells, consecutive unsupported path intervals, anchors, and added-partition deposition support. Repair/refine and regenerate locally or reject unsatisfied geometry; do not silently certify a sizing estimate.
- [x] Preserve dependencies through emission or require prior-layer support. Enforce explicit incompatibilities and invalidate all demand-affecting settings.
- [ ] Run integration and pre-existing bridge regression tests, then review the combined implementation.

### Task 4: Verification and delivery

**Files:** feature tests, user documentation, and this plan's completion record.

- [x] Build the native slicing tests and affected GUI translation units in the existing configured build. A complete application bundle and installation remain outside this increment.
- [ ] Run geometry/config/integration tests and relevant existing bridge, wall, flow, and serialization tests. Record failures separately from baseline results.
- [ ] Obtain an independent feature review, resolve actionable findings, and re-run affected checks.
- [x] Document settings and bounded behavior. Report the branch, changes, evidence, and any remaining qualification gap.

## Execution record

- Branch created at `66a45f8f7271420dd0c75681b5b7ea4aa3598298`; existing dirty state retained and snapshotted.
- User approved the design and explicitly instructed implementation. Continue without another plan-approval round.
- Use disjoint file ownership for parallel work; one controller owns CMake, builds, and integration to avoid concurrent build mutations.

- Advanced the feature branch to the released bridge-controls baseline `28aed765070b663ae4938b93435b87c9680c2a24` (`v2.8.4.57-tyr.17`). A three-way merge against the original dirty-state snapshot retained the cubic additions and incorporated the independently completed baseline changes. Existing unrelated artifacts were preserved.
- Independent geometry and integration reviews identified saturation, seam/scarf transformations, export rounding, bridge-follow-up ownership, and cache hazards. The current increment rejects unsupported saturation and export transformations, reserves positive geometric overlap, carries support-region policy through bridge stacks, and rejects caches without a plan.
- Local repairs now select only violating run footprints and failed void cells, intersected with original finite-width demand. A new full-slicing regression verifies that a failed strand does not request refinement under passing strands elsewhere in the same group.
- Final feature run: **58 of 62 declared test cases pass**. All 48 configuration, geometry, and validator cases pass; 10 of 14 full-slicing cases pass, including the low-density cube, 8-to-3 mm reslice, bridge stack, zero/full density behavior, cache/export guards, sparse follow-up rejection, empty-neighbor preservation, and repair locality. Runtime was 86.1 seconds on the development build. JUnit expands one test into two section entries, producing 63 XML entries.
- Four full-slicing cases remain failing: mixed density/angle/gap modifier interfaces; modifier reslicing blocked by a 0.023 mm one-sided follow-up tail; stepped cavity blocked by a 0.025 mm tail; and tall combined infill reaching density saturation during construction repair. The tests retain their required success expectations.
- Relevant legacy checks: bridge acceleration **4 cases / 68 assertions passed**; overhang walls **18 cases / 5,585 assertions passed**. The bridge-controls suite has one failed Lightning repeat-slice anchor-ratio case. A copied executable from the unmodified `28aed7650` release build reproduces the same assertion without the cubic feature, with a different stack-count parameter; this baseline failure is recorded separately and remains unresolved.
- Core/test builds and the four affected GUI translation units compile. Whitespace checks pass. Feature changes remain on the feature branch pending successful verification before merge; no installed application or printer was changed.

## Open implementation work

- [ ] Construct real shared interface partitions for incompatible modifier lattices, with explicit material ownership, deduplication, and supported deposition.
- [ ] Repair short one-sided construction and follow-up endpoints using actual supported attachment geometry. Do not waive anchor checks merely because a tail is shorter than the requested gap.
- [ ] Implement supported local solid-fill fallback at cubic saturation, confined to affected cells and construction dependencies.
- [ ] Coarsen against final deposited geometry and remove orphaned repair dependencies until no valid collapse remains.
- [ ] Qualify contact from actual bead cross sections and deposition heights; nominal projected width is not a physical-contact proof.
- [ ] Complete partial-roof/inward-wall, variable-height/thick-bridge, phase-sweep, export-geometry, performance, and physical-print acceptance evidence.

This is an initial implementation increment. The unchecked acceptance work and known failing fixtures prevent calling the approved design complete.
