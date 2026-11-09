# Bridge aware cubic parser regression references

These files are fixed parser and configuration regression references. They are not release speed qualification inputs or acceptance measurements. Tests use them without launching Bambu Studio, accessing the network, or generating new projects.

`projects/` contains the eight native version-2 fixture archives, eight INI snapshots, eight project JSON snapshots, and their manifest, copied without modification. The four approved cases are small cube, large cube, inward ribs, and the right-half modifier. Both Cubic and bridge aware cubic inputs are retained for every case. All paths in the manifest are relative to this directory's `projects/` subdirectory.

`official-emitted-config.json` records the serialized configuration from an official Bambu Studio 2.9.1.70 small-Cubic diagnostic run. `fork-emitted-config.json` records the feature build's small-BAC diagnostic configuration. These maps exercise the reviewed, exact configuration differences and their inactive-feature guards. They do not establish current runtime performance or source-to-binary build provenance.

`official-result-schema.json` contains the unchanged values of only the result fields consumed by the validator, captured from the same official diagnostic. Its recorded timing is parser input, not a performance expectation. Motion-parser tests construct synthetic 75- and 90-layer G-code independently; no historical G-code file is required.

`snapshot-provenance.json` identifies the reference sources and artifact hashes without depending on workstation paths. Reference captures may remain useful after implementation defaults change, but updates require review of the intended parser/configuration contract.

For release qualification, build the native `bridge_aware_cubic_benchmark_fixtures` target from the intended source revision and explicitly run its hidden `[BACBenchmarkFixtures]` case with `BAC_BENCHMARK_FIXTURE_DIR` set to a fresh absolute output directory. That generator in `tests/fff_print/bridge_aware_cubic_benchmark_fixtures.cpp` (until 9 November 2026 in `test_bridge_aware_cubic_export.cpp`, which the snapshots' provenance names) remains authoritative and performs configuration application and native export without slicing. Record the generator binary and source/build provenance, then audit the new inputs before timing. Do not silently reuse these regression snapshots as fresh qualification inputs.

A fresh native export can contain new dates and part identifiers. Preserve each qualification run's archive hashes, and compare regenerated fixtures using the strict geometry/configuration audit rather than requiring byte-identical archives across generations.
