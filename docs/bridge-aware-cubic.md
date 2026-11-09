# Bridge aware cubic

A sparse infill pattern: Cubic's lattice everywhere, plus lines under roofs, ledges and other material that would otherwise span
too far over the sparse void, so that nothing above spans more than the Unsupported gap size (`bridge_aware_cubic_max_gap`,
default 5 mm). It exists for single-wall parts at 3% infill whose first roof layer fails on ordinary Cubic.

**Status (9 November 2026).** Being rewritten from `master` against the
[rewrite architecture](superpowers/specs/2026-11-08-bridge-aware-cubic-rewrite-design.md), which replaces the earlier
constructive branch, following the [implementation plan](superpowers/plans/2026-11-08-bridge-aware-cubic-rewrite.md). The
benchmark tooling (`scripts/bridge_aware_cubic_benchmark/`, [procedure](bridge-aware-cubic-benchmark.md)) and the acceptance
scripts (`scripts/bridge_aware_cubic_acceptance/`, see its README) are in place.

| Stage (architecture 8) | State | Gate |
| --- | --- | --- |
| 0 | done | benchmark-tool tests pass; official 2.9.1.70 app present; master's ctest baseline: 20 of 21 suites, one pre-existing failing assertion in `fff_print_tests` (test_printgcode.cpp:190). Master did not compile until two misplaced hunks of 8fddaaa51 were fixed (0568b9dad). |
| 1 | done (fedc2d147) | the four Cubic fixture projects' G-code equals master's byte for byte but for the embedded `; bridge_aware_cubic_max_gap = 5`; ctest as on master |
| 2 | done (e040f2ffb) | L1, L2 pass |
| 3 | done (material d04dd8e56; plan with the next commit) | M1, P1 pass |
| 4-5a | built; committing | P2, K1 (openings and runs) on the cubes, phases, instances, densities and gaps; K2 on the cubes, the rib and the 40 mm box at 3 %; M1, P1; K1n (the nozzle-floor gap) still fails on the solid columns' clipped faces. Roofs with an edge over the void (partial roof, stepped cavity, the rib's rib footprints) and the two-lattice seam (modifier) fail K1 and K2 until the rails of 6b and the seam walls of 6a: cases `[K1edges]`, `[K2seam]`. Anchors: every pattern line goes through the stock connector and gets a nub where it meets another line, with the stock sparse infill anchor settings (user, 9 November 2026). |
| 5b-10 | not started | |

Cubic baselines for the acceptance: the 16 Heimdall plates sliced with master's build (0568b9dad): 4,255.4 g, 308.0 h of
model print time (`$BAC_DATA/baselines/cubic/`).

What a released build will do: plan once per object before the bridge generator, from the sliced surfaces; print Cubic's lines plus
the plan's strands and courses through the ordinary fill machinery; run no check. What a development build adds: the verifier and
the plan checks of the architecture's section 5, as warnings that stop nothing.

How it is judged: the contract of the architecture's section 1 (K1 roofs, K2 the pattern's own lines, no change to any other
setting), the four fixtures through an observer of exported G-code, all 16 plates of the user's parts through the acceptance
scripts, and the release gate of at most 2.0 times official Cubic in instructions retired.
