# Bridge aware cubic

A sparse infill pattern: Cubic's lattice everywhere, plus lines under roofs, ledges and other material that would otherwise span
too far over the sparse void, so that nothing above spans more than the Unsupported gap size (`bridge_aware_cubic_max_gap`,
default 5 mm). It exists for single-wall parts at 3% infill whose first roof layer fails on ordinary Cubic.

**Status (9 November 2026).** Rewritten from `master` against the
[rewrite architecture](superpowers/specs/2026-11-08-bridge-aware-cubic-rewrite-design.md), which replaces the earlier
constructive branch, following the [implementation plan](superpowers/plans/2026-11-08-bridge-aware-cubic-rewrite.md): every
stage is built, the whole-branch review's fix round is in (501020f91), the tests pass and the release gate passes, and the branch
is merged into master (the user's decisions of 9 November 2026: the source and shared-file budgets raised, the test budget held
and the test set reduced to meet it, the shared-code changes approved). Not met: the Heimdall gates of stages 5a and 6 (K2's free
ends on plates 1-5, hanging bridge ends), below; the next work waits for the user's review of the app. The benchmark tooling (`scripts/bridge_aware_cubic_benchmark/`,
[procedure](bridge-aware-cubic-benchmark.md)) and the acceptance scripts (`scripts/bridge_aware_cubic_acceptance/`, see its
README) are in place.

| Stage (architecture 8) | State | Gate |
| --- | --- | --- |
| 0 | done | benchmark-tool tests pass; official 2.9.1.70 app present; master's ctest baseline: 20 of 21 suites, one pre-existing failing assertion in `fff_print_tests` (test_printgcode.cpp:190). Master did not compile until two misplaced hunks of 8fddaaa51 were fixed (0568b9dad). |
| 1 | done (fedc2d147) | the four Cubic fixture projects' G-code equals master's byte for byte but for the embedded `; bridge_aware_cubic_max_gap = 5`; ctest as on master |
| 2 | done (e040f2ffb) | L1, L2 pass |
| 3 | done (d04dd8e56, f9c4b13d5) | M1, P1 pass |
| 4-5a | built (d10faed9d, c9c197be3); fixture gates met, Heimdall gate not met | P2, K1 (openings and runs) on the cubes, phases, instances, densities and gaps; K2 on the cubes, the rib and the 40 mm box at 3 %, and with the user's settings on the large cube and the partial roof; V1. Anchors: every pattern line goes through the stock connector and gets a nub where it meets another line, with the stock sparse infill anchor settings (user, 9 November 2026). Heimdall: K2 counts 1,662 free added ends and 59 long stretches on plates 1-5 (run 5) against the stage gate of 86 (below). |
| 5b-6c | built (01fe19e87, 64ace2a9a, 38c1ee3b2, 8a66896e7); fixture gates met, Heimdall gate not met | the pass (3.8), seam walls (W1, M1), rails (R1), K1 on roofs with edges over the void and on the modifier, K2 across the seam; columns land. Heimdall: 135 hanging bridge ends over the sparse void on the 16 plates (run 5) against 0, and the longest hanging stretch within Cubic's on 14 of the 16 plates (below). No fixture reproduces a hanging end of the pattern. |
| 5c | done (Task 10) | the family change of 3.7 step 6, after Heimdall run 2 measured +19.1 % time; K2 unchanged on the fixtures |
| 7a | done, corrected after review | S3 (infill combination): the surfaces combine as Cubic's; outside the single-layer zones a combined island prints the plan's lattice at the combined thickness, inside them the plan's lines print at layer height on every layer of the group, resting on lines of their own planes; S3 reads only the surfaces and the export |
| 7b | done (Task 16) | S4: at N = 2 the bands equal Cubic's away from the roof, and K1 and K2 hold with bands; at N = 1 the four fixtures' G-code is byte-identical |
| 8a | done (Task 17) | S5 (instances, a shared object and its plan), S6 (settings unchanged but for the pattern and the gap; bridge angles as Cubic's), D1 (byte-identical exports, reslices as fresh slices) |
| 8b | done (Task 18) | S7 (nine stress settings on the small cube, a pocket block and a blind cavity, variable layer height, arc fitting); the hidden X1 (timing, information only) and X2 (the benchmark's fixture projects, carried over unchanged) |
| 8c | done (Task 19) | S2 at 1 %, with conformal infill, on a raft, in spiral vase mode and at 100 % with and without combination (fills equal Cubic's); the 100 % configuration check says of the pattern what it says of Cubic; S1 was complete |
| 9 | done (Task 20) | the release gate passes (below) after a construction fix of the plan's cost |
| 10 | review done, one fix round (below); merged | the whole-branch review (Opus): one construction defect fixed (a saturated cell's lines printed past the island, and a band's at N ≥ 2), the shared-file change outside the hook list reverted, five test gaps closed; then the test set reduced to 18 cases within the held test budget (architecture 7.2) |

Known limitations (findings the construction does not yet meet, 9 November 2026; the local debug dump `[BACDebug]` measures them):

- Thick bases (`internal_bridge_layers` 2 and 3, this fork's feature; the default and the user's parts use 1): the stock generator's first pass makes the one-bead vertical-shell strips along the walls into bridge surfaces, and `add_internal_bridge_layers` re-anchors them as bases on a preview of the layer below from which the big base has already taken the sparse area; they grow up to a cell over the coarse lattice beside the roof, and their ends hang. K1 fails on the partial roof's planes at 9.0 and 9.4 mm (openings of 5.1 and 12.6 mm) and R1's overgrowth measure reads 67 and 91 mm² against 0.05 allowed.
- The rib (vertical-shell thickening off, a stress fixture): the plane under the rib's first layer reads an opening of 7.6 mm from the void band between the body's top fill, which stops half a spacing short of its surface's edge, and the one-bead solid ring the stock pipeline lays under the rib's walls; the plan counts the top surface as printing to its edge (3.3) and sees no void there. A limitation of the measure on this fixture.
- The nozzle-floor gap (0.4 mm at 3 %, once K1n, now rows of S7): the part slices, the solid columns print and their lines stay
  inside the infill area, which is what the test checks; K1 and K2 do not hold. The plane below z 17.4 mm reads an opening of 0.49 mm against the
  0.4 mm gap, and K2 reports 592 free ends and 191 stretches longer than G_b (0.3 mm there; 594 and 191 before the columns' lines
  were clipped to the fill boundary). Before Task 10 they were placed at the columns' lowest layers, where nubs fold
  back at face junctions; printing a face shorter than a bead (lengthened to one) moved the counts from 599 and 221 only.
- K2 with the user's settings (one Arachne wall at 75 % overlap, narrow solid infill, thick bridges, 1 mm anchors, 3 %) holds on
  the small and large cubes, the 40 mm box, the partial roof and the fuselage top, and leaves one free end on the rib (at
  102.7, 84.6 mm on the bed), the stress fixture with vertical-shell thickening off. Not yet examined.
- Recorded at the final review, not fixed: the rails take the thick bridge flow's spacing whatever `thick_bridges` says (about
  0.07 mm further out without it; the user's parts have it on, the fixtures off, and R1 measures them with the same flow); a
  course that a narrow surface laid after the plan cuts in its middle runs on from both sides and joins across it; the filler's
  carry of a strand end to the boundary may cross another line on the way; a bulk line's run-on into a rail's zone is undone
  only near walls, so with no walls at a steep angle (about 70 degrees or more at 0.12 mm layers) an end could land half a
  spacing past the island (S7's no-walls row stays inside). The reduced test set no longer checks the rails' and seam walls'
  presence layer by layer, a 0 % neighbour beside a seam, or the plan's plane list; K1, K2, R1's bridge measures and W1's gaps
  judge what those mechanisms are for.
- R1 does not try a roof the candidate test rejects (one the generator fills with solid infill); rails are checked where bridges exist.
- At the fuselage top's apex the generator bridges one layer above the roof the candidate test predicted: rails lie at 23.8 and
  24.0 mm under predicted bridges at 24.0 and 24.2 mm, the generator prints solid at 24.2 and bridges at 24.4 mm, whose inner edges
  have no rail; no bridge end hangs and nothing overgrows. The cause: with thick bridges (the user's setting) `bridge_over_infill`
  narrows the candidates to the sparse area deep enough for a thick bridge, less what its own thick bridges on the layers within
  that height already took; the second part is the generator's own output, which the plan, made before the generator (user
  decision), cannot read. R1 judges the rails by the bridges' growth and hanging ends, which hold there.
- The stock generator grows a bridge along its direction until both ends reach an anchor. Where the pattern's refined lines lie
  close it bridges less than over Cubic's coarse lattice, but a single bridge line can run on across a sparse area to a distant
  anchor: on Heimdall plate 10 at z 186.4 one 38 mm line, which holds nothing up (the area above it is sparse too). The bridge
  check counts such lines among its free stretches.

Release gate (`scripts/bridge_aware_cubic_benchmark/run_benchmark.py`, official 2.9.1.70 against this branch's app on the
benchmark generator's projects, slice instructions retired, a warm-up and five alternating pairs, 9 November 2026). The first complete run (app at
5be6edb42) failed on the large cube; the sweep's boundary queries and a stock config copy per path were most of the cost, and
ad1a10e16 cut them without changing a path. The config copy was a shared file outside the hook list (2.3, 10.5), so the final
review reverted it, and the final app (501020f91) passes without it:

| Case | 5be6edb42: median, worst | ad1a10e16: median, worst | 501020f91 (final): median, worst | Build parity (fork Cubic over official), final |
| --- | --- | --- | --- | --- |
| small cube | 1.38, 1.39 | 1.17, 1.17 | 1.21, 1.22 | 0.98 |
| large cube | 2.94, 2.94 | 1.76, 1.76 | 1.95, 1.96 | 0.99 |
| rib | 1.11, 1.12 | 1.09, 1.09 | 1.10, 1.11 | 0.99 |
| modifier | 1.47, 1.48 | 1.26, 1.27 | 1.33, 1.33 | 0.99 |

The large cube's margin under 2.0 is small (2 %); the config copy (`LayerRegion::simplify_path` and its two siblings copy the
print config for every path) was worth about 0.19 of its ratio and is a candidate for the legacy branch, for every pattern.

The gate needed one new reviewed exception: `flow_above_internal_bridges`, which the fork's master added after the baseline
review, at exactly 100 % (neutral: `bridge_flow_mask` stays empty).

Budgets of the architecture's 1.4 at the merge (9 November 2026). The user raised the source and shared-file limits to these sizes
and held the test limit; the less important cases were deleted to meet it (architecture 7.2):

| Quantity | Limit | Now |
| --- | --- | --- |
| Source lines in the pattern's units (`Fill/BridgeAwareCubic*`, `Fill/FillBridgeAwareCubic*`, `BridgeAwareCubicMaterial`, `BridgeAwareCubicVerify`) | 4,833 (raised from 4,000) | 4,833 |
| Test lines (the five test files, the fixtures and the observer) | 2,500 | 2,425 |
| Test cases | 40 | 18, all run by `[BridgeAwareCubic]` (the benchmark's project generator, once the hidden X2, builds as its own executable `bridge_aware_cubic_benchmark_fixtures`) |
| Lines added in shared files, hook 7's moved candidate test (about 65 lines) apart | about 185 (raised from 150) | about 185 (250 added and 76 removed in all; 118 of the added outside `PrintObject.cpp`, 26 of them build registration) |

Cubic baselines for the acceptance: the 16 Heimdall plates sliced with master's build (0568b9dad, master with the two hunks
fixed that it needed to compile): 4,255.4 g, 312.7 h estimated (308.0 h of model print time plus the printer's preparation time on
each plate; the tables compare estimated times) (`$BAC_DATA/baselines/cubic/`). Hook 7 and the other shared changes leave Cubic
alone on the user's parts. Heimdall plate 1 sliced as Cubic: the app at 3664d9ee8 equals the baseline move for move (2,921,978
moves); the final app (501020f91) and an earlier build (9 November, 09:19) differ from it in one Arachne outer-wall loop's seam at
z 214.2 mm (22 lines, the start moved about 1 mm). That seam is the stock slicer's, not the branch's: master's own baseline app
prints exactly the final app's moves (the same move stream, checksum for checksum) when macOS fills new or freed heap memory with a
pattern (`MallocPreScribble=1`, `MallocScribble=1`), and the baseline otherwise, so some stock code on that path reads memory it has
not initialised or has freed; which seam a build prints depends on its heap. A finding for master, outside the pattern.

Heimdall acceptance runs (all 16 plates, `scripts/bridge_aware_cubic_acceptance/measure/measure.sh`; Cubic: 4,255.4 g, 312.7 h
estimated, 492 k sparse paths, 0.99 M retractions; budgets of the architecture's 1.4):

| Run | App | Grams | Estimated time | Sparse paths (≤ 0.6 mm) | Retractions | K2 on plates 1-5: free added ends, long added stretches |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 8a66896e7 | 15 plates (plate 4 crashed, fixed in f2e2dc288): +5.9 % | +18 % on 15 plates | 473 k on 15 plates | 1.65 M on 15 plates | 1,387, 169 |
| 2 | 6e823809f | 4,522.9 g (+6.3 %) | 372.4 h (+19.1 %) | 1.44 M (533 k) | 1.79 M | 1,791, 187 |
| 3 | + family change | 4,533.7 g (+6.5 %) | 365.7 h (+16.9 %) | 1.38 M (379 k) | 1.64 M | 1,825, 119 |
| 4 | 06deb8b89 | 4,479.5 g (+5.3 %) | 360.2 h (+15.2 %) | 1.34 M (618 k, nearly all column pieces) | 1.62 M | 1,791, 59 |
| 5 | ad1a10e16 | 4,479.9 g (+5.3 %) | 360.3 h (+15.2 %) | 1.34 M (618 k) | 1.62 M | 1,662, 59 |
| 6 | 501020f91 (final) | 4,479.9 g (+5.3 %) | 360.3 h (+15.2 %) | 1.34 M (618 k) | 1.62 M | 1,662, 59 |

The bridge check of runs 1 to 3 measured nothing: it read the internal bridge layers from the old branch's probe dumps. It now
finds them in the G-code (a bridge path inside the outline of the layer below) and measures, as the export observer does, the
free stretches and hanging ends over the sparse void below. On plate 4 (run 2's app): Cubic 3,120 free stretches over 5 mm
(longest 79 mm), 7 hanging ends (longest hanging stretch 38 mm); the pattern 1 (7.0 mm), 11 hanging ends (longest 0.27 mm).

Runs 4 to 6, all 16 plates: internal bridge stretches over 5 mm free over the void, Cubic 41,564 (longest 181 mm), the pattern 81
(longest 36.6 mm on plate 10 at z 186.4, 34.4 mm on plate 13 at z 186.4, 29.9 mm on plate 12 at z 15.6, 26.2 mm on plate 2 at
z 145.0); hanging bridge ends, Cubic 150 (longest 177 mm), the pattern 135 in runs 5 and 6 (longest 5.1 mm, most under 0.3 mm). The
36.6 mm stretch is one bridge line that the stock generator ran across a sparse area to a distant anchor, holding nothing up (known
limitations); the other long stretches are not yet examined. Run 6 (the final app; the fix round changes nothing at one line and
gap 5, where the refinement never saturates) per plate against Cubic, as the architecture's 7.4 asks (instructions retired by the
whole slice and export, measured while the batch slices the plates in parallel):

| Plate | Instructions G | Ratio | Peak RSS MB | Hanging bridge ends | Longest hanging stretch mm | Within Cubic's |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 2,486 / 3,032 | 1.22 | 4,577 / 6,407 | 16 / 15 | 41.78 / 0.93 | yes |
| 2 | 2,826 / 3,673 | 1.30 | 5,467 / 7,881 | 7 / 4 | 17.38 / 0.13 | yes |
| 3 | 2,030 / 2,347 | 1.16 | 4,767 / 5,618 | 17 / 15 | 176.62 / 2.38 | yes |
| 4 | 1,332 / 1,669 | 1.25 | 4,601 / 4,608 | 7 / 10 | 38.20 / 0.27 | yes |
| 5 | 2,337 / 2,606 | 1.11 | 4,653 / 6,270 | 9 / 9 | 0.81 / 0.81 | yes |
| 6 | 2,123 / 2,612 | 1.23 | 4,700 / 4,889 | 10 / 9 | 25.42 / 5.11 | yes |
| 7 | 1,533 / 1,696 | 1.11 | 4,623 / 6,416 | 8 / 4 | 4.24 / 0.02 | yes |
| 8 | 1,126 / 1,498 | 1.33 | 4,583 / 4,613 | 3 / 3 | 3.80 / 3.80 | yes |
| 9 | 1,566 / 2,012 | 1.28 | 4,638 / 4,927 | 4 / 7 | 4.73 / 0.84 | yes |
| 10 | 2,586 / 2,960 | 1.14 | 4,921 / 5,698 | 19 / 13 | 32.25 / 0.99 | yes |
| 11 | 2,611 / 2,984 | 1.14 | 4,951 / 5,746 | 12 / 10 | 12.38 / 0.73 | yes |
| 12 | 2,286 / 2,773 | 1.21 | 4,722 / 5,858 | 13 / 15 | 45.52 / 2.68 | yes |
| 13 | 1,420 / 1,648 | 1.16 | 4,564 / 4,482 | 0 / 2 | 0.00 / 0.12 | no |
| 14 | 1,063 / 1,251 | 1.18 | 4,528 / 4,568 | 10 / 7 | 9.31 / 0.37 | yes |
| 15 | 3,720 / 4,458 | 1.20 | 6,037 / 7,467 | 0 / 1 | 0.00 / 0.08 | no |
| 16 | 1,263 / 1,582 | 1.25 | 4,518 / 4,591 | 15 / 11 | 14.64 / 1.87 | yes |

Each pair is Cubic / the pattern. The budget of 1.4 is zero hanging bridge ends over the pattern's void and a longest hanging
stretch within Cubic's on every plate: the first is met on no plate (the pattern hangs 1 to 15 ends per plate, 135 in all, against Cubic's
150), the second on 14 of 16 (plates 13 and 15, where Cubic hangs nothing, have stretches of 0.12 and 0.08 mm). Peak RSS rises to 7.9 GB (plate 2).

K2 on plates 1 to 5, runs 5 and 6 (equal): 1,662 free added ends, 59 long added stretches and 59 free posts, by the geometry of the free piece (a local classification of
the checker's report): 908 end in a connector hook along an island edge that faces same-layer material (a floating vertical
shell, a bridge, a top surface, gap fill), the anchors the user accepted as Cubic's boundary behaviour, which the checker counts
because Cubic has no hook on that line; 643 are lattice-direction ends beside a solid surface of the layer (hooks along edges
parallel to a lattice direction among them, and ends of lines a later surface cut more than a spacing from its edge); 159 lie
with nothing within 1 mm, of which those checked on plate 1 are places the sweep records as unheld ("no admissible piece for
the column"); 11 end beside walls. The target is zero (R20); the stage gate of 86 is not met.

What a released build will do: plan once per object before the bridge generator, from the sliced surfaces; print Cubic's lines plus
the plan's strands and courses through the ordinary fill machinery; run no check. What a development build adds: the verifier and
the plan checks of the architecture's section 5, as warnings that stop nothing.

How it is judged: the contract of the architecture's section 1 (K1 roofs, K2 the pattern's own lines, no change to any other
setting), the four fixtures through an observer of exported G-code, all 16 plates of the user's parts through the acceptance
scripts, and the release gate of at most 2.0 times official Cubic in instructions retired.
