# Organic foundation support profiling

The saved Assembly fixture preserves the two original mesh volumes and support
paint, with only an XY translation onto plate 1. The input project SHA-256 is
`a2aa6761c3feea3aade34e6807fbbf039f22f8b9b4e2bfd55ec4feb33fc281c2`;
the extracted fixture SHA-256 is
`00be6b13f6e6a69dbd27d3c0b1f524584b2ef60b851171e85151f163d67cfb1e`.
The model/paint is private and is not included in the public release.

Use `scripts/extract_support_benchmark.py SOURCE OUTPUT --object-id 120 --plate 12
--offset-x -1224 --offset-y 768` to reproduce extraction from that saved project.
Build `benchmark_organic_support`, then run it with the fixture, repository
resources directory, and optional output JSON path. It measures object slicing
and support generation separately and hashes the complete ordered support paths
and their extrusion metadata. Temporary profile data is isolated and removed.

## Profile and correction

A macOS sample of the installed build showed repeated polygon offsets beneath
`foundation_for_seam -> generate_initial_areas_from_contacts ->
support_tip_forbidden -> getAvoidanceImpl -> calculateAvoidance ->
getCollisionHolefree -> calculateCollisionHolefree`.

Reversed collision views populated rows lazily, but each request through layer N
recomputed previously cached rows 0..N. Slow and FastSafe avoidance tasks also
repeated this work concurrently. The correction skips existing rows and prepares
the shared hole-free rows once before the parallel avoidance tasks. It retains
the existing offset operations, geometry and clearance values.

The work-count regression warms the physical source and queries cold reversed
views at 20 and 40 layers. The original code performed 439 and 1338 work callbacks;
the corrected code performs 83 and 163. Cold and incremental polygons match for
all three avoidance modes.

## Saved-project measurements

Three rounds used alternating runner order on an ARM64 Mac with 16 logical CPUs
and 128 GiB RAM. Other development activity was present; the third round in
particular overlapped independent CLI slicing jobs. These are diagnostic paired
measurements, not an idle-machine throughput guarantee.

| Organic algorithm | Support seconds, three rounds | Median seconds |
| --- | --- | --- |
| Before foundations, `2cde1365` | 10.1644, 10.9063, 10.9143 | 10.9063 |
| Earlier foundations, `1e95089e` | 45.5956, 41.2745, 42.5160 | 42.5160 |
| Installed fixes, `522d59b0` | 48.7008, 42.2326, 84.2275 | 48.7008 |
| Cache correction | 13.7891, 14.3147, 21.4681 | 14.3147 |

The corrected median is 70.6% lower than the installed algorithm and 66.3% lower
than the earlier foundation algorithm. The historical comparison replaces only
the Organic translation units in the same common core and harness; it does not
claim to benchmark complete historical application builds.

Every installed/corrected run produced 559 support layers, 12,286 paths, zero
interface paths, and fingerprint `13287933617984265538`. The correction therefore
preserves the contact/interface and midpoint-path fixes exactly on this fixture.

A separate end-to-end CLI comparison used official Bambu Studio 02.08.04.57 and
the optimized app with the same extracted model and build-plate-only disabled,
because stock does not understand Tyr base paint. Official support times were
10.9568, 27.5186 and 17.5012 s; corrected-app times were 14.8916, 11.8310 and
14.4190 s. Whole slice/export times were 34.32, 64.54, 44.98 s and 47.66, 44.32,
49.25 s respectively. Concurrent compilation affected these runs and the two
apps generate different foundation geometry, so these numbers do not establish
an exact stock-relative speed ratio. The downloaded official DMG SHA-256 was
`6d0099f60bb4449ee9066160cb65a8a1f639650a3f48cddf13566fe0c520ac66`.


## Parallel whole-project GUI measurements

Four independent copies of the extracted Assembly were placed on four plates,
with the original mesh, paint and embedded printer/filament/process settings.
The fixture SHA-256 is `863847c51b14fee8f5e3db0700d0dd8039ce0ffed7afa283b16c5b06dbfa623d`.
Implementation revision: `f08730589`. Both GUI executables use the same
native core and UI code; the serial control forces the admission limit in
`Plater::priv::start_parallel_slicing` to one instead of the automatic four.
Each run starts a fresh process with an isolated data directory and the unsliced
3MF, then invokes the normal All plates handler through Cmd+Shift+R. Timing is
from batch admission through the last successful UI result commit, including
G-code export, processing, thumbnails and per-plate finalization.

| Final GUI run | Plates / active slots | Wall seconds |
| --- | --- | --- |
| Serial admission | 4 / 1 | 197.526 |
| Parallel admission | 4 / 4 | 89.311 |

This pair measured **2.21x faster** whole-project generation. Other independent
slicing jobs were active during both runs, so it is a measured diagnostic pair,
not a general speed guarantee. All four ordered command streams match exactly
after removing comments, surrounding whitespace, and object labels (`M486`,
`M624`). Per-plate command counts and hashes, fixture identity and executable
hashes are retained in [the compact comparison record](2026-11-08-parallel-plates.json).

The large-model comparison exposed shared-nozzle E/retraction variables that
were still process-wide. The corrected writer shares these axes only among its
own filaments. Deterministic interleaving failed before the correction in both
absolute and relative extrusion modes; the native export fixture now explicitly
uses Bambu shared-nozzle mode. The earlier mismatching parallel output was
rejected and is not used as release qualification.

## Verification scope

The final native build passed `parallel_plate_tests` (70 assertions / 8 cases),
`organic_foundation_tests` (7,159 / 27), `support_base_tests` (534 / 21), and
`gui_job_tests` (144 / 14). All 44 Python packaging checks passed. Reproduce the
focused checks with the corresponding CMake targets and CTest names, and
`python3 -m unittest discover -s tests/packaging -p 'test_*.py'`.

GUI checks exercised four simultaneous jobs, whole-project statistics, per-plate
Preview, cancellation on a configuration edit, retry, and cached completion
without launching workers. An invalid fourth plate reported its layer-height
error while all three valid plates completed. The final build repeated the
four-plate command/timing comparison and confirmed that selecting All plates
again admitted zero new jobs. Manual source review covered result ownership,
worker termination, stale events, custom G-code lifetime, printer context,
post-process environments and shared support/extrusion settings. CodeRabbit
could not authenticate and is not counted as a completed review.

This is macOS source, native-test and GUI slicing evidence. It does not establish
physical print strength, Windows runtime behavior, comprehensive multi-material
prime-tower qualification, or a passing full upstream test suite.
