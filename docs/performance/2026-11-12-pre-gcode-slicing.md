# Slicing before G-code export

This pass follows the full-pipeline pass of 2026-11-08 (which mostly sped up G-code
export) and the support pass released in Tyr.26. It measures every stage of slicing
on the private aircraft project, 16 plates, and speeds up the stages before G-code
export: walls, infill preparation and supports.

Every change leaves the G-code byte-identical. The comparison baseline is master
`5b593090b` (Tyr.26 plus the painted support base fix). Plate 6 stops at the G-code
conflict check in every version, so 15 plates are compared.

## Results

All 15 plates give byte-identical G-code. Instructions are counted for the whole
command-line run, which includes loading the project (unchanged at about 245 G per
plate). "Net" leaves the load out. Wall time is information only, because the
laptop's clock speed and background load vary.

| Plate | Instructions G, master | Final | Change | Net change | Wall s, master | Final |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 2,490 | 2,284 | -8.2% | -9.1% | 31.9 | 22.9 |
| 2 | 2,969 | 2,684 | -9.6% | -10.5% | 46.9 | 32.4 |
| 3 | 2,206 | 2,060 | -6.6% | -7.4% | 53.5 | 30.0 |
| 4 | 1,184 | 1,095 | -7.5% | -9.4% | 16.2 | 11.1 |
| 5 | 1,767 | 1,617 | -8.5% | -9.8% | 28.4 | 18.2 |
| 7 | 1,141 | 1,052 | -7.8% | -9.9% | 16.2 | 11.1 |
| 8 | 1,072 | 980 | -8.6% | -11.2% | 12.7 | 9.6 |
| 9 | 1,356 | 1,275 | -5.9% | -7.2% | 18.8 | 15.2 |
| 10 | 2,299 | 2,106 | -8.4% | -9.4% | 23.4 | 19.2 |
| 11 | 2,297 | 2,104 | -8.4% | -9.4% | 23.4 | 19.3 |
| 12 | 2,072 | 1,896 | -8.5% | -9.7% | 33.1 | 25.8 |
| 13 | 1,234 | 1,168 | -5.4% | -6.7% | 16.2 | 15.1 |
| 14 | 870 | 819 | -5.8% | -8.1% | 11.0 | 8.2 |
| 15 | 3,584 | 2,994 | -16.5% | -17.7% | 42.0 | 30.4 |
| 16 | 1,087 | 1,005 | -7.6% | -9.8% | 16.6 | 9.5 |
| **Total** | **27,628** | **25,141** | **-9.0%** | **-10.4%** | **390.3** | **277.9** |

Plate 6 (no G-code in either version): 1,599 G and 17.1 s on master, 1,475 G and
14.3 s final.

By stage, over the 15 plates, measured with the stage probes:

| Stage | Instructions G, master | Final | Change | Wall s, master | Final |
| --- | ---: | ---: | ---: | ---: | ---: |
| **Slicing (`Print::process`)** | **21,121** | **18,657** | **-11.7%** | **282.3** | **184.5** |
| Mesh slicing | 624 | 579 | -7.2% | 7.6 | 7.1 |
| Wall painting projection | 497 | 462 | -7.0% | 29.1 | 7.0 |
| Walls (Arachne) | 8,089 | 7,093 | -12.3% | 48.8 | 43.0 |
| Wall internal extension | 834 | 755 | -9.5% | 6.4 | 5.4 |
| Infill preparation and infill | 3,285 | 2,620 | -20.2% | 30.5 | 24.0 |
| of which `process_external_surfaces` | 1,167 | 677 | -42.0% | 10.3 | 6.3 |
| **Supports** | **7,485** | **6,858** | **-8.4%** | **155.7** | **94.1** |
| G-code export (not changed) | 2,781 | 2,770 | -0.4% | 55.7 | 54.5 |

## Where the time went

The stage probes count instructions retired, CPU time and wall time for the whole
process at the boundaries of each stage. Stages run one after another, except
supports, which run for all objects at once. In this table, wall time is
information only. "Parallel" is CPU time divided by wall time: 16 means all cores
were busy.

Master `5b593090b`, 15 plates. Instructions are in billions (G); the percentage is of
slicing plus export.

| Stage | Instructions G | % | Wall s | Parallel |
| --- | ---: | ---: | ---: | ---: |
| Load the project (per command-line slice) | 3,670 | | 38.5 | 6.2 |
| **Slicing (`Print::process`)** | **21,121** | **88%** | **282.3** | **7.1** |
| Mesh slicing (`PrintObject::slice`) | 624 | 2.6% | 7.6 | 10.4 |
| Wall painting projection | 497 | 2.1% | 29.1 | 1.8 |
| Walls (`Layer::make_perimeters`, Arachne) | 8,089 | 33.8% | 48.8 | 14.8 |
| Wall internal extension | 834 | 3.5% | 6.4 | 13.6 |
| Infill preparation and infill | 3,285 | 13.7% | 30.5 | 11.0 |
| of which `process_external_surfaces` | 1,167 | 4.9% | 10.3 | 12.5 |
| of which `discover_vertical_shells` | 624 | 2.6% | 4.4 | 14.4 |
| of which infill toolpaths | 838 | 3.5% | 7.7 | 10.5 |
| **Supports (all objects at once)** | **7,485** | **31.3%** | **155.7** | **4.5** |
| G-code export | 2,781 | 11.6% | 55.7 | 3.6 |

Findings, largest first by wall time:

1. **Supports.** This is the slowest stage on the wall clock (156 s over the 15
   plates) and uses only 4.5 cores on average. On plates with painted support
   bases, much of the organic tree work ran serially on the object's support
   thread. On plate 3 that serial work was 27 s of a 54 s slice, on 4 to 5 cores
   (Tyr.26 profile). The serial work was:
   - the foundation validation in `render_foundation()` and `assemble_foundation()`,
     which tests every part of every layer against the whole adjacent layer with a
     Clipper intersection;
   - the four per-layer passes of `organic_lightning_infill()`;
   - `tree_anchor()`.
2. **Walls.** `Layer::make_perimeters` is the largest stage by work: 34% of
   instructions, already spread over 15 cores. Inside Arachne, the wall painting
   policy took up to 45% of the skeletal trapezoidation on painted plates.
   `boundaryBeadCount()` split the same boundary segment for every Voronoi node
   along it.
3. **Wall painting projection.** `WallPainting::slice_paint` cut every painted facet
   at every layer and labelled every layer's boundary on one thread: 29 s over the
   15 plates on 1.8 cores.
4. **Infill preparation.** `process_external_surfaces` was 5% of the work, and 19% on
   plate 15. On painted layers it looked up the nearest slice border for every point
   of every wall, only to keep the largest distance.
5. **Clipper.** The polygon library takes 40 to 60% of all CPU cycles, spread over
   every stage.
6. **Load and export, for context.** Reading the 250 MB project costs about 245 G
   instructions (2.6 s) per command-line slice; the GUI pays this once per project.
   G-code export is 2.8 T instructions and 56 s, with a mostly serial pipeline.

## Changes

All are exact: the same polygons, in the same order, reach every later step.

| Change | Where | Effect |
| --- | --- | --- |
| **Wall painting projection in parallel.** Facets are cut in fixed blocks whose lines are appended in facet order, and each layer is labelled on its own. | `WallPaintingSlice.cpp` | 29.1 s to 7.0 s of wall time over 15 plates. |
| **Painted wall depth measured only where it can grow.** The distance to the line nearest the last measured point bounds a point's distance from above, so points that cannot raise the maximum are skipped. | `LayerRegion.cpp` | `process_external_surfaces` -42% instructions; plate 15 -8%. |
| **Each painted boundary segment is split once per trapezoidation.** `boundaryBeadCount()` caches the largest wall count per segment. | `SkeletalTrapezoidation` | Plate 15 -5.4%, plate 1 -2.7%. |
| **Arachne infill contour offset once.** `offset2_ex(E, d1, d2)` is `offset_ex(offset(E, d1), d2)`, so the shared first stage is done once. | `PerimeterGenerator.cpp` | Small; part of the walls' -12.3%. |
| **Clipper: no sort pass when nothing crosses.** Clipper also no longer recomputes each edge's X at the top of the scanbeam. | `clipper.cpp` | 6 to 10% fewer instructions in stages where nothing else changed (mesh slicing, surface detection, vertical shells, infill toolpaths). |
| **Clipper: the sorted local minima are read from an array.** Only edge tops go through the scanbeam heap. | `clipper.{hpp,cpp}` | 0.2% overall, 1.6% in mesh slicing. |
| **Exact early-out in the support overlap test.** Clipper is skipped when no polygon's box touches the part. | `TreeSupport3D.cpp` | Skips the whole-layer Clipper runs for parts far from the layer. |
| **Organic lightning infill's per-layer passes in parallel.** The tree growth itself stays serial. | `TreeSupport3D.cpp` | Most of its 16 G serial cycles on plate 3 now run on all cores. |
| **Foundation checks in parallel.** Upward checks run across layers. The walks back from the seam test every layer speculatively and recheck only below a layer that dropped a part. A rejected foundation stops above its first failure. | `TreeSupport3D.cpp` | With the tree anchors: plate 3's serial support work 82 to 26 G cycles. |
| **Tree anchors in parallel.** | `TreeSupport3D.cpp` | Plate 3: 34.1 to 32.0 s of wall time. |
| **Support collision unions reused** across radii and layers. | `TreeModelVolumes.cpp` | 0.3 to 0.5% on support plates. |

On plate 3, the plate with the most support, the work left on the support thread
fell from about 105 G CPU cycles (Tyr.26) to 26 G. The plate's wall time went from
53.5 s to 30.0 s.

## Not done

- **Y-band filter for the support overlap test.** Running Clipper only on the polygons
  near the tested part saved a further 5 s on plate 3 (before the foundation checks
  ran in parallel). It is not exact, so it was reverted. Removing polygons can reorder
  Clipper's local minima at equal Y, because the sort is unstable. Where a part and a
  layer touch with zero area, that tie-break decides whether a sliver survives.
- **Arachne and boost Voronoi** are algorithmic work, already spread over all cores.
- **Support volumes.** The collision and avoidance volumes of `TreeModelVolumes` are
  also algorithmic work across all cores. Two redundant unions were removed.
- **G-code export** was outside the scope of this pass.

## Method

- **Cost is instructions retired, all threads** (`/usr/bin/time -l` for whole runs;
  `proc_pid_rusage` in the probes). The laptop's clock speed and background load vary,
  so wall and CPU time are information only.
- **Byte identity:** every plate's G-code SHA-256 matches the baseline.
- **Stage probes and profiles.**
  - The stage probes are a temporary patch (`BambuStudio-bac-data/tools/slice-perf/`)
    that is never committed.
  - Hot spots come from Instruments' CPU Profiler, which samples on cycles on each core,
    so parallel stages are weighted fairly. `sample` under-counts them.
  - Instruments' `--launch` runs the installed app with the same bundle id, so profiles
    attach by process id.
- **Clipper differential test.** The old and new `clipper.cpp` ran 5,000 seeded random
  cases. Each case ran every clip type, fill rule, Paths and PolyTree output,
  StrictlySimple mode and offset join type, over random, star, grid, tiny and rectangle
  inputs. The outputs were identical, and the assertions held in a non-NDEBUG build.
- **Review.** Each group of commits was reviewed independently for exactness and
  thread safety. The review rejected the Y-band filter.
- **Unit tests.** All slicer suites pass, except the known legacy failure at
  `test_printgcode.cpp:190`.
