# Bridge aware cubic slicing benchmark

This procedure compares Bridge aware cubic with regular Cubic in the unmodified official Bambu Studio application. It covers the four approved cases: small cube, large cube, rib, and modifier. A successful benchmark establishes software cost, measured in instructions retired, and configuration checks for those cases. It does not establish physical print quality or performance on other models, machines, or slicer versions.

The feature remains subject to the performance gate described in [Bridge aware cubic](bridge-aware-cubic.md). This document and the checked-in parser examples are not a passing benchmark result.

## Measurement and acceptance

**The metric is instructions retired, not time.** Wall-clock and CPU time change with the machine's power state and with whatever else it is running; the number of instructions a process retires barely does. Every figure that gates is therefore the instructions retired by the process, all threads, which macOS `/usr/bin/time -l` prints on its `instructions retired` line. Wall and CPU time are reported as information and nothing gates on them. Spin-waiting worker threads also retire instructions, so idle parallel phases add a little to both sides (a few tenths of a percent on a quiet machine, more on a loaded one). Measure with spare cores and compare only counts taken the same way.

**Two processes per run.** Every measured run launches two processes of the same application on the same fixture, in this order, each under `/usr/bin/time -l` and each with its own empty data, output, temporary, and Cocoa directories:

1. `--slice 1` is the full slice: startup, project load, slicing, G-code export, and `result.json`.
2. `--slice 99` is startup and project load only. The fixtures have one plate, so the CLI loads the project, logs `invalid plate id 99, total 1`, writes a `result.json` that reports return code -2 (`Invalid parameters to the slicer.`), prints `run found error, exit`, and exits with status 254. It slices nothing, exports nothing, and opens no window.

The run's **slice instructions** are the first process's instructions retired minus the second's. Startup, project load, and process exit, which both processes execute, drop out of the difference; so does the fixed cost of the sandbox and `arch` wrappers that launch each process. The runner keeps both raw counts in `benchmark.json` beside the difference, and keeps each process's `time -l` report as `time.txt` in its directory.

The runner accepts the second process only if it ends exactly as described: exit status 254; `invalid plate id 99, total N` (with the fixture's plate count) and `run found error, exit` on stdout; a `result.json` with return code -2, that message, plate index 0 and no sliced plates; and nothing but `result.json` in its output directory. It rejects anything else, because a crash or failed load would retire fewer instructions and inflate the difference. The slice process must exit 0 and pass every result, G-code, and configuration check. The difference must be positive.

If `/usr/bin/time -l` prints no `instructions retired` line, which is the case off macOS and for virtual machines without performance counters, the runner stops with a message that says so.

**Sides.** Each interleaved iteration runs three measured runs: official Cubic (the official application on the Cubic project), the fork's BAC (the fork on the BAC project), and the fork's Cubic (the fork on the same Cubic project as the official run). Odd iterations run them in that order and even iterations in the reverse order, so the gate's pair alternates and so does the build-parity pair.

**The gate.** For each interleaved pair, the ratio is the fork's BAC slice instructions over official Cubic's. Final acceptance requires all of the following for **each** case:

- At least one warm-up iteration and five measured iterations, with the order alternating between iterations.
- Both the median and the worst of the five pair ratios at or below **2.0**. The median is taken over the pair ratios, not over the counts.
- Successful CLI and result codes, no reused sliced objects, correct model bounds and triangle counts, and actual moving extrusion on all expected deposition layers.
- Exact intended configuration and paired configuration checks, apart from the explicitly reviewed transformations and the intended pattern/gap differences.
- Unchanged source identity, executables, inputs, runner, archive auditor, and configuration guards throughout the run.

**Build parity** is the fork's Cubic slice instructions over official Cubic's, per iteration, with its median and worst value. It is information only: it shows how much of a ratio comes from the build rather than from the pattern, and nothing gates on it. The fork's Cubic run passes the same result, G-code, and configuration checks as the others. The fork writes its own default for the BAC gap, 5 mm, into the configuration of a Cubic input, which omits it; that value, and no other, is expected there.

**Information only.** `sliced_time` (`result.json.sliced_plates[0].times.sliced_time`, wall-clock milliseconds), the whole-process wall times, and the ratio of the sliced times stay in `benchmark.json`, labelled as information only. `sliced_time` must still be a positive integer of milliseconds, as the result check has always required, but nothing gates on the value of any of these figures. The similarly named `sliced_time_with_cache` field is not used either: the CLI can add G-code export time to that accumulator even when no object cache was used.

The rib has 75 layers; the other cases have 90. The modifier archive contains 24 triangles across its model and modifier volumes, while the CLI's model-only triangle counts must be 12. Development mode and partial case coverage can never pass the release gate. Counts are exact integers and the ratios have no denominator floor.

## Requirements and official baseline

The slicing launcher currently supports an Apple silicon macOS session with `/usr/bin/arch`, `/usr/bin/sandbox-exec`, and the macOS `/usr/bin/time`, whose `-l` report includes the `instructions retired` line. The Python code uses only the standard library and requires Python 3.11 or later. The pure parser, guard, and measurement tests do not require macOS, a slicer build, an installed application, network access, or printer access.

The reviewed baseline is official **2.9.1.70**. Its [provenance record](../scripts/bridge_aware_cubic_benchmark/baselines/official-2.9.1.70/provenance.json) identifies the public release asset, asset checksum, executable checksum, and vendor signing team. Obtain that asset from the recorded official URL, extract it into an isolated directory, and independently check its checksum and app signature before using it. The runner records the executable hash; it does not download the application or establish vendor provenance by itself. No official executable or vendor application is included in this repository.

Use the actual fork app bundle being evaluated. Freeze builds and source edits before timing. Keep a build receipt tying its executable hash to the source revision, configuration, compiler, and build inputs. `--build-provenance PATH` embeds a supplied JSON receipt in the report; it does not infer or independently validate that source-to-binary relationship.

The configuration exceptions are specific to the recorded baseline and guarded fixture values. Their [review evidence](../scripts/bridge_aware_cubic_benchmark/baselines/official-2.9.1.70/review_evidence.json) links to the reviewed official tag and fork checkpoint. Changed slicer versions, active feature controls, or relevant implementation semantics require a fresh review. An exception does not exempt any work from the measurement.

## Generate fresh timing inputs

The existing hidden C++ test `BACBenchmarkFixtures` in [test_bridge_aware_cubic_export.cpp](../tests/fff_print/test_bridge_aware_cubic_export.cpp) is the input authority. It writes eight native 3MF projects, JSON and INI snapshots of their completed configuration, and a manifest. It does not slice. Each pair has identical model geometry, placement, and common global/local settings; Cubic omits the unsupported BAC gap setting. It also completes the CLI inheritance and filament metadata vectors.

All four families use empty machine start and end scripts. The generator sets them after loading the family defaults and checks both values after configuration resolution on each side. This prevents a default startup Z move from preceding the slicer's absolute-coordinate and millimeter declarations. Keep the G-code mode and moving-extrusion checks enabled.

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

The runner requires explicit official and fork executable paths, a repository path, fixture directory, and a fresh output directory. It launches every measured process as arm64, under `/usr/bin/time -l`, with the same slicing options and validation enabled; the two processes of a run differ only in the plate they are asked for. No 3MF export, cache import, estimate mode, or validation bypass is used.

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

For final acceptance, use a new output directory, omit `--development --sizes small`, and supply the build receipt with `--build-provenance`. The defaults then run all four cases with one warm-up iteration and five measured iterations each. A development run defaults to one iteration without a warm-up: three runs, so six processes, per case. Use `--check-inputs-only` to audit the projects without launching an executable. `--official-only` is an explicit development preflight, which still runs two processes per official run, and cannot qualify the fork.

Each process receives a separate data directory, output directory, temporary directory, and Cocoa home. `HOME` remains unchanged. A process-local macOS sandbox denies networking, access to normal Bambu profile directories, live Cocoa state/cache/container paths, and the app's preference domains. The Cocoa isolation prevents unrelated saved-state recovery prompts from interfering with the CLI. The runner reaps its own child process group on timeout or interruption; it does not operate on an installed application's process.

The main result is `benchmark.json`. Preserve it with its per-run CLI results, emitted G-code, `time.txt` reports, configuration comparisons, logs, and build receipt. A run's slice process keeps the run directory (`<case>-<warmup|sample>-<iteration>-<side>`, with the sides `official`, `fork`, and `fork_cubic`); its load process lives in the `load` directory inside it. The report records, for each run, the exit status and raw instructions retired of both processes and their difference, the slice process's checks, and the wall times; for each pair, the execution order, the gate ratio, the build parity, and the configuration comparisons; for each case, the medians and the worst pair; and the input, module, and executable hashes, source snapshots, and exact accepted configuration transformations. Review any invalid or incomplete status before interpreting a count. Keep generated outputs under the ignored build directory or outside the checkout.

The report's `schema_version` is 3. It keeps every field of version 2, but the fields that gate now hold slice-instruction figures: `slice_ratio`, `all_pair_ratios`, `worst_observed_pair_ratio`, `median_slice_ratio`, and `within_2x`. `median_slice_ratio` is now the median of the pair ratios rather than a ratio of medians. The wall-clock fields (`slice_ms`, `slice_s`, `whole_process_wall_s`, `all_slice_ms`, `median_slice_ms`, and the rest) keep their meaning and are information only; the ratio of sliced times moved to the `sliced_time_ratio_information_only` fields (prefixed `median_` and `worst_` in a case summary). New fields hold the raw counts (`slice_process`, `load_process`, `slice_instructions`) and the build parity (`build_parity_ratio`, `all_build_parity_ratios`, `median_build_parity_ratio`, `worst_build_parity_ratio`).

## Test the benchmark tooling

Run the standard-library tests from any checkout location:

```sh
python3 -m unittest discover -s tests/benchmarking -v
```

These tests exercise corrupted archives, global/local configuration guards, result schemas, actual deposition-layer parsing, sample accounting, and incomplete evidence. They also cover the instruction measurement: parsing `/usr/bin/time -l` output (including a missing `instructions retired` line), the pairing of each slice process with its load process, the subtraction, what a load process must look like, the gate ratio with its median and worst pair, the build parity, and that `sliced_time` and wall time gate nothing. Runs and whole benchmark invocations are tested against a fake process launcher that leaves what the real command leaves behind, so a default test run never launches a slicer. The normal C++ test targets remain separate; the benchmark is an opt-in release check and is not added to automatic CTest execution.
