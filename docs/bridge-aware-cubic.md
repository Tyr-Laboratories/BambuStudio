# Bridge aware cubic

Bridge aware cubic adds a sparse infill pattern with two controls:

- **Sparse infill density** sets the ordinary Cubic lattice away from support demand.
- **Unsupported gap size**, in millimeters, sets the requested maximum opening beneath upcoming extrusions. Its initial default is 5 mm; this is not a qualified bridge distance for a particular filament.

The source implementation is on `codex/bridge-aware-cubic`. The automated correctness checkpoint below passes, but the feature is not ready for release: the required end-to-end slicing limit of twice stock official Bambu Studio with regular Cubic has not been met. Performance optimization and fresh verification remain required before merging or delivery.

Each recursive child independently intersects the actual finite-width extrusion demand. A child that does not need further subdivision remains coarse. Complete added partition faces grow from their natural beginnings and retain their inherited walls. The transition can extend well below a roof, and a thin part may need refinement through its entire available sparse depth. Average infill can therefore exceed the selected bulk percentage.

The setting applies to model roofs and walls supplied by eligible sparse material, and to the added construction strands needed to support them. It does not impose the same short-span requirement on every ordinary bulk-infill move. Intentional holes, external air, and zero-density regions remain outside the material that refinement may create. Zero percent disables sparse infill; 100 percent uses Rectilinear fill.

## Construction and verification

The slicer keeps separate plans for regions with different density, gap, angle, or flow settings. It collects demand from generated non-sparse paths after bridge-stack placement, preserving complete path identities and endpoints. Each selected subdivision is local to those footprints. Its required deposition and supporting material are protected from infill combination and “infill only where needed” trimming.

Before fixing that demand, automatically angled internal bridge stacks can be rotated when the next layer would otherwise span too far between their strands. A bounded set of angles is evaluated against regenerated paths, preserving the relative angles within each connected stack. An explicit bridge angle remains authoritative. The selected paths still undergo the complete support checks below.

Angle trials preserve the clipping priority of neighboring regions, so changing one participating bridge stack cannot change an untouched neighbor's seam geometry through fill-group ordering.

Support is checked against preceding deposited material using the emitted bead width, height, and deposition elevation. The contact model uses rounded-rectangle or circular bead cross sections at one common contact plane. It distinguishes physical lower-bead sections, which can bound openings, from direction-dependent contact evidence used to anchor an upper path. A maximum-width XY projection alone cannot establish contact, and XY tolerance cannot bridge a real vertical separation. Finite line contact is handled separately from an isolated point.

Prepared lower-bead sections are reused only within an unchanged layer snapshot and an exact upper elevation and bead-height pair. Each upper path still receives its own contact query. The geometry, combined allocation bound, and aggregate work limits remain unchanged; preparation is charged when performed rather than repeated for each path.

Two checks govern model roof and wall demand: the actual closed opening must fit within the gap in every direction, and every consecutive unsupported run along the upper path must fit within the gap with the required anchors. A demand-mask or modifier boundary is not a wall. Construction strands and follow-up bridge layers are checked against their actual predecessors for span and anchoring; their surroundings need not already have the final roof's cubic cell size.

The implementation can repair a failure by:

- Extending a real free endpoint along its terminal direction into usable previous material, while keeping the new bead inside permitted material. Compatible neighboring regions may supply that material only when their participation, filament assignments, and extrusion settings match. Closed loops and interior joins are not treated as free endpoints.
- Connecting a selected open infill endpoint to the actual wall cross section when a perimeter channel leaves an oversized opening. The added connection stays inside permitted material and must pass the same predecessor checks as other construction.
- Constructing an owned, deduplicated partition along a modifier interface, with lower supporting courses. This is limited to compatible participating regions using the same sparse-infill filament and flow. Their density, gap, and angle can differ.
- Closing a selected sparse/solid interface with a real deposited course when slots between existing bridge strands open into an oversized sparse cell. The course and any supporting courses must remain inside permitted material and pass the same support checks.
- Placing a short lower course through an unsupported fixed perimeter seam, without moving the seam or joining the initial and final unsupported runs. An added partition can also locate a lower landing at its exact producer's upper sparse/solid interface. These locations only propose construction; actual material ownership, full-bead containment, and preceding support still govern admission.
- Refining the particular failing footprint and propagating the support obligation to its actual producing partitions. A failing unsupported run is followed across participating region boundaries, while passing runs on the same path and unrelated paths remain excluded. Each receiving child still tests its own intersection before subdividing.
- Replacing saturated local leaves with two perpendicular line families at a combined nominal density of 100%, confined to the needed cells. This avoids the long contact-plane slots that a single family of parallel strands can leave. The deposition and its predecessors are checked; a solid reservation is never assumed to be an already supported sheet.

Repairs are regenerated and rechecked, including the last accepted repair batch. Support obligations can propagate downward through the actual layers; each local obligation begins at its own initial refinement strength. After target paths are stable, the slicer removes dependencies that no longer lead to a surviving producer. It then trials collapsing subdivisions and removing interface construction stacks against regenerated deposition, retaining a collapse only when the affected roof and construction checks pass. This reduces unnecessary material for the chosen lattice and the available collapse operations; it does not claim a global minimum over other lattice origins or support structures.

Each interface course is admitted with its complete predecessor stack. An invalid course does not discard independent courses whose ownership, material containment, and preceding support can be established. Final verification still checks the resulting deposition together.

Before coarsening, the slicer checks the complete model and construction target set. Any failure exposed by expanding the original planning selection returns to the bounded repair loop. Coarsening and export retain that complete scope and the unchanged model paths.

If the bounded construction, repair, contact, or coarsening work cannot establish the requested geometry, slicing stops with a diagnostic identifying the object, region, and relevant layer or support range. It does not silently enlarge the requested gap or count future material as support.

## Current constraints

- Multiline must be 1, Conformal infill must be disabled, and Spiral vase must be off.
- Seam gap must be zero and scarf seam transformations must be disabled. Compensated circular loops that would receive scarf seams are rejected as well, because these operations would change support after validation.
- Objects using this pattern retain their wall and infill paths instead of applying subsequent simplification or arc fitting.
- Extrusion caches are not reused for this pattern because they do not contain its support plan and construction dependencies.
- Configured extruders must use the same finite XY nozzle offset. Different physical nozzle offsets require additional tool-specific contact handling and are currently rejected.
- Mixed-filament sublayers that change the certified deposition height are rejected. Layer grouping must also preserve each actual Bridge aware cubic deposition plane; close but unequal object or support heights cannot be combined into a different plane.
- The contact model supports straight, planar extrusion segments with valid rounded-rectangle or circular cross sections. Unsupported upper-bead shapes and fitted arcs are rejected; narrow lower gap-fill shapes outside the model do not contribute support evidence.
- Cross-region repair cannot treat an empty neighbor, cavity, or incompatible material assignment as permission to add a wall. Bridge-layer density can also leave a follow-up layer that cannot be anchored within the selected gap. These cases require a valid construction or an explicit failure.
- Geometry and work limits remain finite. A rejected solve can reflect those limits or unavailable construction, and is not proof that every possible support arrangement would be unprintable.

The export path uses additional Z precision when a print contains Bridge aware cubic, and contact validation uses the same serialized deposition elevations. Width and height annotations also retain enough precision for exported-geometry checks. XY checks use the actual coordinate conversion and output rounding for every placed instance. Bridge aware cubic preserves nonzero source segments through export and uses additional extrusion precision, so very short segments cannot silently change the deposited path when it is reversed. For moving segments, extrusion volume follows the emitted XY distance and retains the configured flow calibration. Before each model path it restores the configured extrusion mode, and rejects a move with changed XY coordinates if its emitted extrusion increment would be zero. Export repeats verification at the current placement, since moving a part can reuse its infill plan. The checkpoint below includes these exported-coordinate checks.

An added standalone construction path that collapses entirely to one emitted XY point in any placed instance is removed from the actual generated paths before certification. Ordinary bulk paths, ambiguous matches to the original bulk lattice, and joined paths are retained. The remaining geometry must still pass every instance's contact, opening, and dependency checks; the removed path never supplies support evidence.

## Qualification

The source includes configuration, geometry, contact, dependency, attachment, slicing, modifier, variable-height, phase, bridge-stack, and exported-coordinate tests. The emitted-geometry tests measure actual paths and openings rather than treating refinement depth as a certificate. Sliced-path recertification starts from deposited paths and sliced material, independently of the planner's demands and subdivision state, while sharing the bead-contact model. Separate synthetic geometry, broken-wall controls, cavity-exclusion checks, and nominal-footprint measurements of exported paths provide additional checks.

The bead-contact model is a geometric model, not a material adhesion or sag test. Physical qualification must still use the intended printer, filament, flow, temperature, cooling, and bridge settings, inspecting the first roof layer as well as the finished top. No installed application, printer, or physical print result is implied by source implementation or automated tests.

The [approved design](superpowers/specs/2026-11-05-bridge-aware-cubic-design.md) and [implementation plan](superpowers/plans/2026-11-05-bridge-aware-cubic.md) describe the detailed contracts and verification work.

The [official slicing benchmark procedure](bridge-aware-cubic-benchmark.md) documents the reproducible timing inputs, configuration review, isolation, and required performance gate.

## Correctness checkpoint and outstanding performance gate

The 6 November 2026 checkpoint passed the complete arm64 RelWithDebInfo application build, all 17 existing CTest targets, and all 44 packaging tests. The Bridge aware cubic binary passed all 243 default cases (245 executions because the three modifier-reslice sections ran independently), totaling 176,972 assertions. Exact-name manifests were replay-listed, checked for complete non-overlapping coverage, and tied to one unchanged binary. Both opt-in qualification cases also passed, bringing the feature total to 290,500 assertions across 247 case executions. Independent source reviews covered the construction, contact, ownership, output, and repair changes. This is automated geometric evidence, not physical print qualification.

Measurements used 5% bulk density, a 5 mm gap, 0.2 mm layers, two walls, and three top/bottom layers on an Apple M4 Max with 128 GiB memory. They were collected alongside other verification processes and are observed runtimes, not isolated performance benchmarks. The comparison below uses regular Cubic in this fork; it must not be represented as a stock-official timing result.

| Fixture | Fork Cubic process + export | Bridge aware process | Bridge aware export | Pair peak RSS | Total moving extrusion increase |
| --- | ---: | ---: | ---: | ---: | ---: |
| 32 × 28 × 18 mm | 0.0258 s | 34.2955 s | 0.4419 s | 53.08 MiB | 39.83% |
| 64 × 56 × 18 mm | 0.0257 s | 671.1511 s | 2.4840 s | 122.64 MiB | 58.76% |

Total moving extrusion was 3,419.175 → 4,780.949 mm³ for the small fixture and 10,591.149 → 16,814.490 mm³ for the large fixture. Corresponding sparse extrusion was 860.763 → 2,222.537 mm³ and 3,148.450 → 9,371.757 mm³. These measurements use actual positive moving extrusion and exclude retraction, priming, and stationary extrusion.

The small plan had 475 nodes and 56 splits; the large plan had 1,749 nodes and 203 splits. Both reached depth three, used no saturated solid cells or interface courses, and selected parent envelopes on all 90 layers from Z 0.2 to 18 mm. The maximum selected areas were 890.679 and 3,572.555 mm²; their layer-integrated envelopes were 14,050.264 and 53,392.308 mm³. Those envelopes describe selected parent cells, not uniformly dense material. The plans retained six and five endpoint attachments respectively.

The user subsequently made **end-to-end slicing no more than 2× stock official Bambu Studio with regular Cubic** a mandatory release condition. The current checkpoint fails that objective by a wide margin. A verified official executable, matching persistent project fixtures, actual per-plate slicing timers, and emitted-setting parity are required for acceptance. Startup and packaging overhead must be reported separately rather than used to conceal slicing cost. The feature remains unmerged while the repeated planning, coarsening, and verification work is optimized without relaxing gap, anchoring, ownership, or completeness requirements.

### Subsequent performance-work checkpoint

The subsequent 6 November checkpoint passes the full application build, all 17 existing CTest targets, 44 packaging tests, and 70 benchmark-tool tests. Bridge aware cubic passes 298 default cases across 300 executions, with 226,276 assertions. The small and large qualification fixtures and the exhaustive-versus-optimized comparison also pass, bringing the total to 303 executions and 353,357 assertions. These results refer to test executable SHA-256 `e2ef3bdfc9c0c9faca5ecd7131771b72adbd3d01b5d1b3fcfcaec2e3f7fee1fd`.

The changes reuse immutable lower-section preparation, contact masks, unchanged model proofs and exact export-state snapshots; retain unaffected plan data during collapse; and avoid copying passing-cell geometry when only failure evidence is needed. Indexed contact queries reuse temporary candidate storage instead of cumulatively charging it again for every segment. Independent export tests reject changed deposited paths, bead dimensions and elevations, and verify renewal after phase or Plan replacement, including shared objects. Continuous line-contact proofs apply only to construction and bridge-followup span checks: model roof and wall targets retain their independent physical-cell checks.

Small and large fixture material totals, subdivision counts and refinement extent remain unchanged. A diagnostic checkpoint measured approximately 13.49 seconds of processing for the small cube and 192.63 seconds for the large cube, with export taking approximately 0.024 and 0.047 seconds respectively. These are internal diagnostic observations, not a new official paired performance qualification. The required 2× stock-official limit remains unmet, so this checkpoint does not authorize merging, installation or release. The performance qualification scope remains the existing small cube, large cube, rib and modifier fixtures.
