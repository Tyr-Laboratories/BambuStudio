# Bridge aware cubic benchmark tooling tests

Run the pure Python suite from the repository root with Python 3.11 or later:

```sh
python3 -m unittest discover -s tests/benchmarking -v
```

The tests use the standard library, temporary directories, and the labeled parser reference snapshots in `tests/data/bridge_aware_cubic_benchmark`. They do not require an app installation, a native build, network access, or successful slicing on the current workstation. Process launches are blocked in test cases that exercise preflight and archive validation, and the source-provenance test mocks its repository queries. The cases that run the tool's measured runs (`run_one`, `main`) replace the process launcher with a fake that leaves what the real `/usr/bin/time -l ... --slice N` command leaves behind, built from outputs captured from the official and fork apps; nothing is ever killed or launched for real.

The suite checks archive/configuration parity, real layer motion parsing, runtime result schema, exact reviewed exception guards, the instruction measurement (parsing `/usr/bin/time -l`, the pairing of each full-slice process with its startup-and-load process, the subtraction, and what a load process must look like), the gate and build-parity ratios, and rejection of incomplete or invalid speed-gate evidence. Synthetic instruction counts test the acceptance rule; they are not benchmark measurements. Native fixture generation and actual macOS arm64 qualification are separate opt-in workflows documented with the benchmark tool.
