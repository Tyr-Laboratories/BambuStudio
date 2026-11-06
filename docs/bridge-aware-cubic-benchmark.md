# Bridge aware cubic slicing benchmark

This procedure compares Bridge aware cubic with regular Cubic in the unmodified official Bambu Studio application. It covers the four approved cases: small cube, large cube, rib, and modifier. A successful benchmark establishes software timing and configuration checks for those cases. It does not establish physical print quality or performance on other models, machines, or slicer versions.

The feature remains subject to the performance gate described in [Bridge aware cubic](bridge-aware-cubic.md). This document and the checked-in parser examples are not a passing benchmark result.

## Timing and acceptance

The primary measurement is `result.json.sliced_plates[0].times.sliced_time`, in integer milliseconds. In the reviewed official CLI, the interval starts before per-plate setup, configuration application, and validation, and ends after slicing and G-code export. Process startup, initial project import, and 3MF packaging are outside this interval. The runner records whole-process wall time separately. It neither subtracts slicing work nor adds startup time to reduce the ratio.

Final acceptance requires all of the following for **each** case:

- At least one warm-up pair and five measured pairs, with the official/fork order alternating between pairs.
- Both the ratio of median slicing times and the worst observed paired ratio at or below **2.0**.
- Successful CLI and result codes, no reused sliced objects, correct model bounds and triangle counts, and actual moving extrusion on all expected deposition layers.
- Exact intended configuration and paired configuration checks, apart from the explicitly reviewed transformations and the intended pattern/gap differences.
- Unchanged source identity, executables, inputs, runner, archive auditor, and configuration guards throughout the run.

The rib has 75 layers; the other cases have 90. The modifier archive contains 24 triangles across its model and modifier volumes, while the CLI's model-only triangle counts must be 12. Development mode and partial case coverage can never pass the release gate. Integer-millisecond measurements are retained without a denominator floor; short official intervals are flagged in the report.

The similarly named `sliced_time_with_cache` field is not the primary metric: the CLI can add G-code export time to that accumulator even when no object cache was used.

## Requirements and official baseline

The slicing launcher currently supports an Apple silicon macOS session with `/usr/bin/arch` and `/usr/bin/sandbox-exec`. The Python code uses only the standard library and requires Python 3.11 or later. The pure parser and guard tests do not require macOS, a slicer build, an installed application, network access, or printer access.

The reviewed baseline is official **2.9.1.70**. Its [provenance record](../scripts/bridge_aware_cubic_benchmark/baselines/official-2.9.1.70/provenance.json) identifies the public release asset, asset checksum, executable checksum, and vendor signing team. Obtain that asset from the recorded official URL, extract it into an isolated directory, and independently check its checksum and app signature before using it. The runner records the executable hash; it does not download the application or establish vendor provenance by itself. No official executable or vendor application is included in this repository.

Use the actual fork app bundle being evaluated. Freeze builds and source edits before timing. Keep a build receipt tying its executable hash to the source revision, configuration, compiler, and build inputs. `--build-provenance PATH` embeds a supplied JSON receipt in the report; it does not infer or independently validate that source-to-binary relationship.

The configuration exceptions are specific to the recorded baseline and guarded fixture values. Their [review evidence](../scripts/bridge_aware_cubic_benchmark/baselines/official-2.9.1.70/review_evidence.json) links to the reviewed official tag and fork checkpoint. Changed slicer versions, active feature controls, or relevant implementation semantics require a fresh review. An exception does not exempt any work from the timer.

## Generate fresh timing inputs

The existing hidden C++ test `BACBenchmarkFixtures` in [test_bridge_aware_cubic_export.cpp](../tests/fff_print/test_bridge_aware_cubic_export.cpp) is the input authority. It writes eight native 3MF projects, JSON and INI snapshots of their completed configuration, and a manifest. It does not slice. Each pair has identical model geometry, placement, and common global/local settings; Cubic omits the unsupported BAC gap setting. It also completes the CLI inheritance and filament metadata vectors.

Generate into a new absolute directory using the test binary built from the source under evaluation. For example, from the repository root, with the test executable path adjusted to the actual build:

```sh
benchmark_root="$(pwd)/build/bridge-aware-cubic-benchmark"
mkdir -p "$benchmark_root"
BAC_BENCHMARK_FIXTURE_DIR="$benchmark_root/projects" \
  build/wall-painting/tests/fff_print/bridge_aware_cubic_tests \
  '[BACBenchmarkFixtures]'
```

The generator refuses to overwrite its outputs. The tracked files under [tests/data/bridge_aware_cubic_benchmark](../tests/data/bridge_aware_cubic_benchmark) are **parser regression snapshots**, including example output schemas. Use fresh native generation for final timing; a parser snapshot's stored time or successful parsing is not performance evidence for a new executable.

## Run the comparison

The runner requires explicit official and fork executable paths, a repository path, fixture directory, and a fresh output directory. It launches both executables as arm64 with the same slicing options and validation enabled. No 3MF export, cache import, estimate mode, or validation bypass is used.

The example below runs only the small case once for a development check. Replace both executable paths with verified local paths:

```sh
python3 scripts/bridge_aware_cubic_benchmark/run_benchmark.py \
  --official /path/to/official/BambuStudio.app/Contents/MacOS/BambuStudio \
  --fork build/wall-painting/src/BambuStudio.app/Contents/MacOS/BambuStudio \
  --repo . \
  --fixtures "$benchmark_root/projects" \
  --output "$benchmark_root/development-small" \
  --intended-exceptions scripts/bridge_aware_cubic_benchmark/baselines/official-2.9.1.70/intended_exceptions.json \
  --parity-exceptions scripts/bridge_aware_cubic_benchmark/baselines/official-2.9.1.70/parity_exceptions.json \
  --development --sizes small
```

For final acceptance, use a new output directory, omit `--development --sizes small`, and supply the build receipt with `--build-provenance`. The defaults then run all four cases with one warm-up pair and five measured pairs each. Use `--check-inputs-only` to audit the projects without launching an executable. `--official-only` is an explicit development preflight and cannot qualify the fork.

Each process receives a separate data directory, output directory, temporary directory, and Cocoa home. `HOME` remains unchanged. A process-local macOS sandbox denies networking, access to normal Bambu profile directories, live Cocoa state/cache/container paths, and the app's preference domains. The Cocoa isolation prevents unrelated saved-state recovery prompts from interfering with the CLI. The runner reaps its own child process group on timeout or interruption; it does not operate on an installed application's process.

The main result is `benchmark.json`. Preserve it with its per-run CLI result, emitted G-code, configuration comparisons, logs, and build receipt. It records each sample, paired order and ratio, medians, worst observed ratio, whole-process times, input/module/executable hashes, source snapshots, and the exact accepted configuration transformations. Review any invalid or incomplete status before interpreting timings. Keep generated outputs under the ignored build directory or outside the checkout.

## Test the benchmark tooling

Run the standard-library tests from any checkout location:

```sh
python3 -m unittest discover -s tests/benchmarking -v
```

These tests exercise corrupted archives, global/local configuration guards, result schemas, actual deposition-layer parsing, sample accounting, timing ratios, and incomplete evidence. Their subprocess guard prevents a default test run from launching a slicer. The normal C++ test targets remain separate; the timing benchmark is an opt-in release check and is not added to automatic CTest execution.
