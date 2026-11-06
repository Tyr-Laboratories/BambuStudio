# Bridge aware cubic sprint retrospective

**6 November 2026 · Development paused · Release held**

The feature achieved the requested 5 mm roof geometry on the measured cube exports. Slicing time prevents release on every existing fixture. The team closed the sprint on the feature branch. The release tracker owns the decision to resume qualification and records the evidence required before merging. [Release tracker #1](https://github.com/Tyr-Laboratories/BambuStudio/issues/1)

## What shipped on the feature branch

Users can select Bridge aware cubic and set sparse infill density and unsupported gap size independently. The planner intersects actual upcoming extrusion footprints with each child at every recursion. A thin line can therefore refine just the children it crosses. It retains construction required to print those subdivisions, checks actual prior-layer material and attempts to remove surplus cells. Unsupported geometry produces a slicing diagnostic. The tests cover cases that a flat cube alone cannot establish, including narrow lines, holes, interfaces, partial roofs, placement changes and reslicing. [Approved design and acceptance matrix](superpowers/specs/2026-11-05-bridge-aware-cubic-design.md#acceptance-criteria) · [Implementation and supported settings](bridge-aware-cubic.md)

On the first roof at Z 17.6 mm, independent measurements from the timed CLI exports showed the following results. The observer measured unsupported runs along the upper paths and entire selected openings in the nominal footprint of the preceding beads. The feature exports had zero unanchored ends and zero open selected cells. Physical print and material qualification remains a separate exercise.

| Existing 5% cube | Worst unsupported run, official → feature | Widest opening, official → feature |
| --- | ---: | ---: |
| 32 × 28 × 18 mm | 21.027 → 4.101 mm | 24.280 → 4.977 mm |
| 64 × 56 × 18 mm | 21.027 → 4.176 mm | 24.280 → 4.925 mm |

The last full-suite and timing baseline is `67a1cf755`. It passed 443 feature case executions and 426,060 assertions, 17 legacy CTest targets, 44 packaging tests, 70 benchmark-tool tests and the 260-assertion fixture writer. The latest source checkpoint, `467ea713b`, adds opt-in diagnostic comparisons. It passed three focused controls and both cube qualification cases. Full-suite and four-fixture timing results for that later source remain open. [Measurement record and hashes](bridge-aware-cubic-sprint-evidence.json)

## Why the release is held

| Fixture | Official regular Cubic | Bridge aware cubic | Ratio | Feature infill preparation |
| --- | ---: | ---: | ---: | ---: |
| Small cube | 0.305 s | 9.076 s | 29.76× | 8.515 s |
| Large cube | 0.270 s | 130.962 s | 485.04× | 128.895 s |
| Rib | 0.278 s | 31.888 s | 114.71× | 31.300 s |
| Modifier | 0.260 s | 122.716 s | 471.98× | 121.688 s |

Each entry is one development pair with no warm-up against official 2.9.1.70. The eight slices passed the fixture, configuration and G-code checks. Release requires at least one warm-up pair and five alternating measured pairs on **each** fixture. Both the ratio of median times and the worst paired ratio must be at most 2.0. Use the end-to-end CLI slicing metric across the existing four fixtures. [Benchmark procedure](bridge-aware-cubic-benchmark.md)

## What to carry forward

- **Check the output a printer would receive.** A union of nominal footprints can hide an extra line deposited over an existing wall. The experimental emitter compares moving segment multiplicity and calibrated volume and checks rounded edges against inset material. Its two cube exports passed 46,029 and 136,730 assertions, including deliberately introduced duplicate and orphan repairs. This is a reference for further work on local edits. Its current scope covers one region, one admissible placement and model-only deposition. [Tested plane prototype](https://github.com/Tyr-Laboratories/BambuStudio/blob/f9aaafd0677d3a8e94bd6da9d5970aeb9234e683/docs/bridge-aware-cubic-plane-program.md)
- **Remove work only after identifying who consumes it.** The feature now computes reservation polygons only for actual sparse restoration or enabled infill combination. On the large cube the previous reservation/snapshot block consumed 20.344 s. All 117,963 executable commands across the four timed exports matched the previous feature build after the change. Repeated support verification still dominates preparation. [Reservation change](https://github.com/Tyr-Laboratories/BambuStudio/commit/67a1cf7552798c8cc2734508445317ce86a72c63)
- **An exact failed roof path can identify a necessary wall.** On the first sweep, the geometric diagonal test proved zero removals unsafe. On the same staged layers, checking the actual roof path found 19 failures in 21 attempts on the small cube and 41 in 48 on the large cube. All 60 matched the same failure after full regeneration. The diagnostic left all ordinary decisions in charge. This gives the team a concrete shortcut to qualify. [First-sweep diagnostic checkpoint](https://github.com/Tyr-Laboratories/BambuStudio/commit/467ea713bf6b0e6ebe0d78395c5267959c1de311)
- **Profile the cases separately.** Modifier interfaces and rib closures can prevent reuse that works on a cube. Full-plane verification remains the reference when ownership, topology or placement is uncertain. The earlier direct-roof cache experiment and phase diagnostics are available for comparison. [Direct-roof experiment](https://github.com/Tyr-Laboratories/BambuStudio/commit/932e94d320409e9581100e3db0e3d82aa9b8cc28) · [Phase diagnostics](https://github.com/Tyr-Laboratories/BambuStudio/commit/e50f7aefd092108ad37b71f3f5285c2bf02f821d)

## Next sprint

| Order | Work | Exit evidence |
| --- | --- | --- |
| 1 | Qualify first-sweep rejection from one actual predecessor layer. | Each early failure agrees with full regeneration. Plans, decisions and output match. Report skipped work and elapsed time per fixture. [#2](https://github.com/Tyr-Laboratories/BambuStudio/issues/2) |
| 2 | Localize verification of surviving paths and openings, including rib and modifier boundaries. | Incremental edits and rollback agree with fresh geometry and preserve the required gap, anchoring and minimality. Report fallback and memory costs. [#3](https://github.com/Tyr-Laboratories/BambuStudio/issues/3) |
| 3 | Reduce startup and final proof costs. This can proceed alongside local verification. | Demonstrate safe reuse on actual inputs and ordered G-code. Retain full checks for ambiguous placement, flow and tool assignments. [#4](https://github.com/Tyr-Laboratories/BambuStudio/issues/4) |
| 4 | Freeze one release candidate and run all functional and timing gates. | Attach the full test results and a passing four-fixture release benchmark tied to the same source and executable. [#5](https://github.com/Tyr-Laboratories/BambuStudio/issues/5) |
| 5 | Hand off the accepted merge to the release agent, publish and verify the local installation. | Record the merge SHA, release tag, package checksum and installed version on the tracker. This step is deferred pending acceptance. [#6](https://github.com/Tyr-Laboratories/BambuStudio/issues/6) |

Start from `codex/bridge-aware-cubic`. Compare a proposed change with the most recent timed build before claiming a speedup. Use reduced diagnostic runs for iteration, then the full benchmark for release. The implementation plan records the cancellation, memory and fallback requirements for new shortcuts. The current public release at sprint close is `v2.8.4.57-tyr.17`. [Local verification plan](superpowers/plans/2026-11-06-bridge-aware-cubic-local-proof-ledger.md) · [Existing public release](https://github.com/Tyr-Laboratories/BambuStudio/releases/tag/v2.8.4.57-tyr.17)
