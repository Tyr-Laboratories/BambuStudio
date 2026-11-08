# Full slicing performance

Continue the user's slicing-speed work across the complete pipeline, using
Tyr.22 (`f7c37853`) as the baseline. Measure import, mesh slicing, walls,
infill preparation/generation, supports, path planning, G-code export and
preview processing. Prioritize measured hotspots, rather than rewriting
functions that contribute little to elapsed time.

The saved Full Heimdall2 project is the representative workload. Preserve its
mesh, painting and settings. The baseline CLI stops at plate 6 with an existing
G-code conflict; profile later plates individually and retain that limitation.

Initial native samples identify repeated wall-boundary queries, seam visibility
radius queries, and repeated edge-distance calculations in travel planning.
Optimize these by skipping provably redundant work. Preserve spatial-query
membership and traversal order, wall counts and transitions, and travel paths.
No reduced resolution, changed settings, altered support geometry, new global
mutable caches, or new dependencies.

The laptop remains in low power mode. Analytical conclusions must not assume
constant CPU frequency or background load. Use deterministic operation counts
and per-process retired instructions as primary work evidence, with repeated
pairs to expose scheduling variation; elapsed time and cycles are observational.

Qualification requires focused native regressions, matching-toolchain baseline
and candidate runs, warmup followed by alternating paired measurements, and
identical normalized G-code commands on representative plates. Record timings
and stage coverage, including stages with no material hotspot. Keep only
changes that show a useful gain without a material regression. Verify parallel
plate isolation remains intact. Follow the existing authorization to update the
Mac installation, merge and publish after qualification; preserve user state.
Remove this pass's bulky temporary artifacts after delivery.
