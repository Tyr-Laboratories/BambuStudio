# Bridge aware cubic

Bridge aware cubic is under development on `codex/bridge-aware-cubic`. It adds a sparse infill pattern with two controls:

- **Sparse infill density** sets the ordinary Cubic lattice away from support demand.
- **Unsupported gap size**, in millimeters, sets the requested maximum opening beneath upcoming extrusions. Its initial default is 5 mm.

Each recursive child independently intersects the finite-width extrusion demand. A child that does not need further subdivision remains coarse. The generator retains complete partition faces and their natural beginnings, rather than switching a dense horizontal pattern on immediately below a roof.

The slicer builds separate plans for regions with different settings. It reserves the material needed to construct the subdivisions, checks generated extrusion paths against preceding-layer material, and reports an error when its bounded solve cannot meet the requested geometry. Zero percent disables sparse infill; 100 percent uses Rectilinear fill.

## Current development constraints

- Multiline must be 1, and Conformal infill must be disabled.
- Seam gap must be zero and scarf seam transformations must be disabled. These operations alter perimeter support after the current validation stage. Compensated circular loops that would receive scarf seams are rejected as well.
- Objects using this pattern retain their original wall and infill paths instead of applying subsequent path simplification or arc fitting.
- Extrusion caches are not reused for this pattern because they do not contain its support plan.
- Reaching cubic density saturation, the subdivision limit, or the verification work limit is an explicit failure. A supported local solid-fill fallback is not implemented yet.
- Different lattices on opposite sides of a modifier boundary do not yet receive a shared physical closing partition. Some modifier fixtures therefore fail even after additional subdivision.
- Some short, one-sided partition starts and bridge-follow-up endpoints cannot yet be repaired. This also affects the stepped-cavity and tall combined-infill fixtures. These are known implementation gaps, not evidence that the requested part is inherently unprintable.

## Qualification still required

The current validator measures nominal deposited-width geometry. It does not establish a physical contact or adhesion model for different layer heights, circular bridge beads, material temperatures, flow, or cooling. No physical print qualification is implied.

The approved design also requires final coarsening against deposited material and pruning of obsolete repair dependencies. Rebuilding from current target paths removes obsolete original targets, but it does not yet establish that every remaining split is necessary after all repairs.

These limits must be resolved before treating the feature as a completed implementation of the approved design. The detailed design and execution plan are in `docs/superpowers/specs/2026-11-05-bridge-aware-cubic-design.md` and `docs/superpowers/plans/2026-11-05-bridge-aware-cubic.md`.

## Development verification — 5 November 2026

The branch includes the `v2.8.4.57-tyr.17` bridge-controls baseline (`28aed7650`). Native slicing tests and the affected GUI sources compile.

| Check | Result |
| --- | --- |
| Configuration, recursive geometry, and nominal support validator | 48 test cases pass. |
| Full slicing | 10 of 14 test cases pass, including the 5% cube, gap changes, bridge stacking, and localized repair. |
| Known feature failures | Mixed modifier interfaces, modifier reslicing with a short follow-up tail, stepped cavity, and tall combined infill. |
| Existing bridge acceleration and overhang walls | 22 test cases pass. |
| Existing bridge controls | One Lightning repeat-slice anchor-ratio case fails; the same assertion also reproduces in the released baseline executable without this feature. |

The full feature suite has 58 passing and four failing declared test cases. The failing success expectations remain in the suite to track the unfinished behavior.
