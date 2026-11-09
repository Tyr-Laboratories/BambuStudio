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
| 3 | done (d04dd8e56, f9c4b13d5) | M1, P1 pass |
| 4-5a | done (d10faed9d, c9c197be3) | P2, K1 (openings and runs) on the cubes, phases, instances, densities and gaps; K2 on the cubes, the rib and the 40 mm box at 3 %; V1. Anchors: every pattern line goes through the stock connector and gets a nub where it meets another line, with the stock sparse infill anchor settings (user, 9 November 2026). |
| 5b-6c | done (01fe19e87, 64ace2a9a, 38c1ee3b2, 8a66896e7) | the pass (3.8), seam walls (W1, M1), rails (R1), K1 on roofs with edges over the void and on the modifier, K2 across the seam; columns land |
| 7a | done | S3 (infill combination): the surfaces combine as Cubic's; outside the single-layer zones a combined island prints Cubic's own combined fill, inside them the plan's lines print at layer height on every layer |
| 7b-10 | not started | |

Known limitations (findings the construction does not yet meet, 9 November 2026; the local debug dump `[BACDebug]` measures them):

- Thick bases (`internal_bridge_layers` 2 and 3, this fork's feature; the default and the user's parts use 1): the stock generator's first pass makes the one-bead vertical-shell strips along the walls into bridge surfaces, and `add_internal_bridge_layers` re-anchors them as bases on a preview of the layer below from which the big base has already taken the sparse area; they grow up to a cell over the coarse lattice beside the roof, and their ends hang. K1 fails on the partial roof's planes at 9.0 and 9.4 mm (openings of 5.1 and 12.6 mm) and R1's overgrowth measure reads 67 and 91 mm² against 0.05 allowed.
- The rib (vertical-shell thickening off, a stress fixture): the plane under the rib's first layer reads an opening of 7.6 mm from the void band between the body's top fill, which stops half a spacing short of its surface's edge, and the one-bead solid ring the stock pipeline lays under the rib's walls; the plan counts the top surface as printing to its edge (3.3) and sees no void there. A limitation of the measure on this fixture.
- K1n (the nozzle-floor gap, 0.4 mm at 3 %): the solid columns print, the roof openings read 0.76 and 1.65 mm at two island corners, and K2 reports stretches of 0.5 mm (G_b is 0.3 mm there) where nubs fold back at face junctions.
- The candidate-test pin (hook 7): on Heimdall plate 1 sliced as Cubic the new app's moves equal the baseline's but for 22 lines of one Arachne outer-wall loop's start at z 214.2 mm (the loop's seam moved about 1 mm); two slices with the new app agree with each other, and every bridge and infill move matches, so the extraction left the bridging unchanged. The baseline was made by an earlier build of the branch.
- R1 does not try a roof the candidate test rejects (one the generator fills with solid infill); rails are checked where bridges exist.

Cubic baselines for the acceptance: the 16 Heimdall plates sliced with master's build (0568b9dad): 4,255.4 g, 308.0 h of
model print time (`$BAC_DATA/baselines/cubic/`).

What a released build will do: plan once per object before the bridge generator, from the sliced surfaces; print Cubic's lines plus
the plan's strands and courses through the ordinary fill machinery; run no check. What a development build adds: the verifier and
the plan checks of the architecture's section 5, as warnings that stop nothing.

How it is judged: the contract of the architecture's section 1 (K1 roofs, K2 the pattern's own lines, no change to any other
setting), the four fixtures through an observer of exported G-code, all 16 plates of the user's parts through the acceptance
scripts, and the release gate of at most 2.0 times official Cubic in instructions retired.
