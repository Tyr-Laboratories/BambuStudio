# Full slicing pipeline optimization

This pass follows Tyr.22 and profiles the complete CLI pipeline: import, layer
slicing, walls, infill preparation and generation, supports, path planning,
G-code export, and the G-code processing used by preview. It does not benchmark
interactive 3D rendering. The private saved project supplies the workloads;
its meshes, paint, and settings are not published.

## Changes retained

Seam searches can build immutable, tight bounding boxes for KD-tree subtrees.
Empty subtrees are excluded before fetching point coordinates. Membership and
preorder are preserved, including strict floating-point radius boundaries.
Only the heavily reused mesh-visibility sample index opts into this cache.
Layer candidate indexes continue using the original traversal. Rebuilding
or clearing a tree invalidates it; queries do not modify shared state.

Wall-painting splits skip repeated interior count lookups when every span in
a conservatively padded local neighborhood has the same wall count. The
original endpoint projection and coalescing remain intact. Nearby differing,
zero, or slope-painted counts continue through the original splitter.

The global 30,000-sample seam tree adds approximately 0.75 MiB of bounds, freed
with the visibility model before alignment. Each allocated tree node uses two coordinate arrays;
for the three-dimensional float indexes this is 24 bytes per node. Peak process
memory is reported separately because allocator and scheduling variation also
influence it.

## Measurement method

The laptop remains in low power mode, with independent development jobs running.
No conclusion assumes constant frequency, idle CPUs, or equal wall-time cost.
The primary evidence is deterministic coordinate-access counts and final
per-process retired instructions across all threads. The macOS counter runner
reads `proc_pid_rusage(RUSAGE_INFO_V4)` after exit but before reaping the child.
Its tests verify sleep independence, work scaling, inclusion of completed worker
threads, and propagation of the child exit code.

Warmup precedes five alternating baseline/candidate pairs for each fixture.
The baseline and candidate use the same compiler, dependencies, build flags,
resources, and input bytes. Wall times, stage timings, and CPU cycles are
observations, not fixed-frequency speedup claims. Instruction counts can still
vary with scheduling and synchronization; ranges and paired ratios matter.

Qualification compares every ordered G-code command, interleaved preview tags
(role, width, height, layer, and other processor controls), and complete selected
filament/time/warning metadata. No geometry tolerance is used.

The original Tyr.22 comparison exposed pre-existing floating-shell instability
and failed exact-output qualification. Isolated probes compiling the actual
production translation unit under AddressSanitizer reproduced reads after vector
erase and outside a clipped path when interpolation anchors were missing.
Focused expected-value cases also exposed a lost preceding-path flag at a width
transition and an extra/stale closing flag after loop rebasing. Continued diagnosis
found the self-intersection grid borrowing points from a temporary container
that had already been destroyed when the grid was queried. AddressSanitizer
poison checks at that boundary fail before the fix and pass after it; classification
regressions cover both intersecting and nonintersecting paths.

The corrections preserve source flags before vector mutation, retain the
accumulated path's classification, use actual endpoints for existing default-width
fallbacks, assign the closing flag in its existing slot, and keep borrowed grid
points alive through the query. Valid-anchor
interpolation and clipping topology remain unchanged. All sanitizer and expected
value cases pass. Final measurement uses Tyr.22 plus these same corrections as
the baseline, isolating optimization cost from undefined-behavior correction.
The earlier unfixed measurements are diagnostic evidence only.

Before the borrowed-grid correction, baseline controls produced different complete
streams and two candidate streams remained unmatched. That campaign failed;
its measurements remain diagnostic evidence. A fresh campaign with the grid
correction in both binaries is required. Each fixture's direct
calibration and counter-wrapped baseline controls establish exact references. The harness can run at most three extra baseline-only
diagnostics if a candidate remains unmatched; these never enter paired performance
statistics. A candidate-only stream or a mixture of different reference components
fails qualification.

A separate diagnostic build checked both optimized algorithms against independent
copies of their original implementations on identical live inputs. Across all
three fixtures, all 49,476,931 wall splits (34,361,561 fast-path uses) and 4,021,681
indexed seam queries matched exactly, in order. An intentional mismatch confirmed
that the KD checker aborts independently of release assertions. These runs were
excluded from performance statistics and did not make the failed full-stream
campaign pass. Four traced slices and repeated Arachne-input probes did not
reproduce the variation; the subsequent lifetime assertion identified the
expired grid storage at its actual production query boundary.

The next campaign passed the foundation fixture but failed on one larger-model
support stream after five pairs and three baseline-only controls. A separate
checked detector then found an invalid read beyond the trimmed painted-enforcer
array: 1,545 object layers, 1,227 enforcer entries, and an attempted read at index
1,351. A synthetic two-shelf fixture reproduces this invalid access in the checked
original detector. Bounding the lookup by the array size passes that regression
and retains automatic supports above the painted shelf. The full support-base
suite passes 539 assertions in 22 cases.

A guarded diagnostic slice matches the previous baseline's complete command,
preview, and metadata record. Its detector never reaches the existing time or
complexity cutoffs (maximum 14 regions per layer); the arena reports 16 threads.
These observations establish the bounds defect and its correction, not that it
caused the single unmatched stream. Fresh qualification includes this same
correction in both binaries. Diagnostic logs now live outside the CLI output
directory so the slicer's directory reset cannot unlink them.

## Focused work measurements

| Probe | Baseline instructions | Candidate instructions | Reduction |
| --- | ---: | ---: | ---: |
| 10,000 uniform wall splits | 3,801,367,022 | 1,330,736,883 | 64.99% |
| 100,000 seam queries, 30,000 planar samples | 2,399,264,341 | 2,012,244,808 | 16.13% |

The seam instruction probe uses three alternating pairs without coordinate-read
instrumentation. It finds the same 3,931,085 points; the native tests verify
membership and order. A separate instrumented run counts query coordinate reads:
88,275,308 versus 32,966,667. These focused results are not whole-model speedup
estimates.

Per-layer caching was also rejected after construction-inclusive planar probes:
200 fresh indexes with 32 queries each required 10.69%, 13.04%, and 14.20% more
instructions at 1,024, 3,000, and 30,000 points respectively. At 3,000 points and
12,000 queries, caching reduced instructions 11.23%. This confirms that reuse
must amortize construction; only the proven high-reuse visibility index opts in.

Two other prototypes were rejected. An ancestor-only KD bound reduced work on an
empty-corner synthetic test but slowed realistic planar-sample queries. The
retained cache uses tight subtree bounds. Travel-edge deduplication reduced
instructions only 1.03% on a 250-vertex annulus and increased them 18.74% on a
2,000-vertex annulus, despite identical distances; it is absent from the change.

## Coverage and verification

The stage survey completed 15 of the saved project's 16 plates. Plate 6 has an
existing G-code conflict in the baseline; later plates were profiled individually.
Most observed work was in G-code export, supports, and wall generation. Infill
preparation, infill generation, layer slicing, and brim/skirt generation were
included in the survey. This project did not exercise ironing. Low-cost stages
were left unchanged rather than receiving speculative optimizations.

Eight affected native suites passed after the shared fixes: spatial queries, wall painting, overhang
walls, bridge controls, parallel plates, organic foundations, support bases,
and GUI job ownership. The 53 Python packaging/measurement tests passed. The standalone sanitizer
probe checks segment flags, interpolation boundaries and closed-loop flag rotation. A fresh
whole-change review found a preview-equivalence gap in the harness; the fix now
checks semantic comments in their exact position among commands. This is not a
claim that the full upstream suite or physical printing was qualified.

Whole-model results and delivery details are added after qualification completes.
