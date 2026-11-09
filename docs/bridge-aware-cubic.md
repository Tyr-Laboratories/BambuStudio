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
| 5c | done (Task 10) | the family change of 3.7 step 6, after Heimdall run 2 measured +19.1 % time; K2 unchanged on the fixtures |
| 7a | done, corrected after review | S3 (infill combination): the surfaces combine as Cubic's; outside the single-layer zones a combined island prints the plan's lattice at the combined thickness, inside them the plan's lines print at layer height on every layer of the group, resting on lines of their own planes; S3 reads only the surfaces and the export |
| 7b | done (Task 16) | S4: at N = 2 the bands equal Cubic's away from the roof, and K1 and K2 hold with bands; at N = 1 the four fixtures' G-code is byte-identical |
| 8a | done (Task 17) | S5 (instances, a shared object and its plan), S6 (settings unchanged but for the pattern and the gap; bridge angles as Cubic's), D1 (byte-identical exports, reslices as fresh slices) |
| 8b-10 | in progress | |

Known limitations (findings the construction does not yet meet, 9 November 2026; the local debug dump `[BACDebug]` measures them):

- Thick bases (`internal_bridge_layers` 2 and 3, this fork's feature; the default and the user's parts use 1): the stock generator's first pass makes the one-bead vertical-shell strips along the walls into bridge surfaces, and `add_internal_bridge_layers` re-anchors them as bases on a preview of the layer below from which the big base has already taken the sparse area; they grow up to a cell over the coarse lattice beside the roof, and their ends hang. K1 fails on the partial roof's planes at 9.0 and 9.4 mm (openings of 5.1 and 12.6 mm) and R1's overgrowth measure reads 67 and 91 mm² against 0.05 allowed.
- The rib (vertical-shell thickening off, a stress fixture): the plane under the rib's first layer reads an opening of 7.6 mm from the void band between the body's top fill, which stops half a spacing short of its surface's edge, and the one-bead solid ring the stock pipeline lays under the rib's walls; the plan counts the top surface as printing to its edge (3.3) and sees no void there. A limitation of the measure on this fixture.
- K1n (the nozzle-floor gap, 0.4 mm at 3 %): the part slices and the solid columns print, which is what its test checks; K1
  and K2 do not hold. The plane below z 17.4 mm reads an opening of 0.49 mm against the 0.4 mm gap, and K2 reports 594 free ends
  and 191 stretches longer than G_b (0.3 mm there). Before Task 10 they were placed at the columns' lowest layers, where nubs fold
  back at face junctions; printing a face shorter than a bead (lengthened to one) moved the counts from 599 and 221 only.
- The candidate-test pin (hook 7): on Heimdall plate 1 sliced as Cubic the new app's moves equal the baseline's but for 22 lines of one Arachne outer-wall loop's start at z 214.2 mm (the loop's seam moved about 1 mm); two slices with the new app agree with each other, and every bridge and infill move matches, so the extraction left the bridging unchanged. The baseline was made by an earlier build of the branch.
- R1 does not try a roof the candidate test rejects (one the generator fills with solid infill); rails are checked where bridges exist.
- At the fuselage top's apex the generator bridges one layer above the roof the candidate test predicted: rails lie at 23.8 and
  24.0 mm under predicted bridges at 24.0 and 24.2 mm, the generator prints solid at 24.2 and bridges at 24.4 mm, whose inner edges
  have no rail; no bridge end hangs and nothing overgrows. The cause: with thick bridges (the user's setting) `bridge_over_infill`
  narrows the candidates to the sparse area deep enough for a thick bridge, less what its own thick bridges on the layers within
  that height already took; the second part is the generator's own output, which the plan, made before the generator (user
  decision), cannot read. R1 therefore checks the fuselage's rails per fixture, every other fixture's per bridge layer, and
  scans the fixtures whose roofs end at the walls for rail-like beads on every layer.
- The stock generator grows a bridge along its direction until both ends reach an anchor. Where the pattern's refined lines lie
  close it bridges less than over Cubic's coarse lattice, but a single bridge line can run on across a sparse area to a distant
  anchor: on Heimdall plate 10 at z 186.4 one 38 mm line, which holds nothing up (the area above it is sparse too). The bridge
  check counts such lines among its free stretches.

Cubic baselines for the acceptance: the 16 Heimdall plates sliced with master's build (0568b9dad): 4,255.4 g, 308.0 h of
model print time (`$BAC_DATA/baselines/cubic/`).

Heimdall acceptance runs (all 16 plates, `scripts/bridge_aware_cubic_acceptance/measure/measure.sh`; Cubic: 4,255.4 g, 312.7 h
estimated, 492 k sparse paths, 0.99 M retractions; budgets of the architecture's 1.4):

| Run | App | Grams | Estimated time | Sparse paths (≤ 0.6 mm) | Retractions | K2 on plates 1-5: free added ends, long added stretches |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 8a66896e7 | 15 plates (plate 4 crashed, fixed in f2e2dc288): +5.9 % | +18 % on 15 plates | 473 k on 15 plates | 1.65 M on 15 plates | 1,387, 169 |
| 2 | 6e823809f | 4,522.9 g (+6.3 %) | 372.4 h (+19.1 %) | 1.44 M (533 k) | 1.79 M | 1,791, 187 |
| 3 | + family change | 4,533.7 g (+6.5 %) | 365.7 h (+16.9 %) | 1.38 M (379 k) | 1.64 M | 1,825, 119 |
| 4 | 06deb8b89 | 4,479.5 g (+5.3 %) | 360.2 h (+15.2 %) | 1.34 M (618 k, nearly all column pieces) | 1.62 M | 1,791, 59 |

The bridge check of runs 1 to 3 measured nothing: it read the internal bridge layers from the old branch's probe dumps. It now
finds them in the G-code (a bridge path inside the outline of the layer below) and measures, as the export observer does, the
free stretches and hanging ends over the sparse void below. On plate 4 (run 2's app): Cubic 3,120 free stretches over 5 mm
(longest 79 mm), 7 hanging ends (longest hanging stretch 38 mm); the pattern 1 (7.0 mm), 11 hanging ends (longest 0.27 mm).

Run 4, all 16 plates: internal bridge stretches over 5 mm free over the void, Cubic 41,564 (longest 181 mm), the pattern 81
(longest 36.6 mm on plate 10 at z 186.4, 34.4 mm on plate 13 at z 186.4, 29.9 mm on plate 12 at z 15.6, 26.2 mm on plate 2 at
z 145.0); hanging bridge ends, Cubic 150 (longest 177 mm), the pattern 137 (longest 5.1 mm, most under 0.3 mm). The long free
stretches and the hanging ends over 1 mm are under investigation.

What a released build will do: plan once per object before the bridge generator, from the sliced surfaces; print Cubic's lines plus
the plan's strands and courses through the ordinary fill machinery; run no check. What a development build adds: the verifier and
the plan checks of the architecture's section 5, as warnings that stop nothing.

How it is judged: the contract of the architecture's section 1 (K1 roofs, K2 the pattern's own lines, no change to any other
setting), the four fixtures through an observer of exported G-code, all 16 plates of the user's parts through the acceptance
scripts, and the release gate of at most 2.0 times official Cubic in instructions retired.
