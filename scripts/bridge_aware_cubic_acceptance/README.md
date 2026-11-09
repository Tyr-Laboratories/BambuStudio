# Bridge aware cubic: acceptance and measurement tooling

Copied out of the session scratchpad on 8 November 2026 (ruling in the SDD ledger) and re-pointed at the repository and a data
directory. To be committed under `scripts/bridge_aware_cubic_acceptance/` on the new branch (specification, 7.4 and stage 0).

Layout:
- `acc/`: `batch.sh VARIANT_DIR` slices all 16 Heimdall plates with `VARIANT_DIR/app/BambuStudio.app` (four at a time), `run.sh` one
  plate, `analyse2.sh` the K2 and bridge-end analyses. `measure/measure.sh LABEL APP` drives all of it and writes `table.md`.
- `metrics/metrics.py`: per-G-code grams by role, estimated time, path and retraction counts (one line per plate in `summary.txt`).
- `k2/`: `k2check.py` (the K2 checker, aligned to the hold rule R of specification 3.7: round-ended reach of w/2 from sparse centre lines, w/2 + 0.01 from other footprints, free ends unless fused alongside a bead of the same layer, long runs against G_b = G − 0.1), `causes.py`, `onbulk.py`,
  `classes.py`.
- `tools/`: `bridgecheck.py` (bridge ends over sparse void against Cubic), `table.py` (the summary tables), the G-code readers `gcr.py`,
  `fp.py`, `dump.py`, and the one-off analyses of the old batches.
- `checkM/`, `instr/`: older run and instruction-count scripts, kept for reference.
- `branch/`: the benchmark tooling, fixture projects and procedure document from the old head ef9c17704 (`git archive`), for stage 0.

Environment:
- `BAC_DATA` (default `/Users/jforbes/Documents/Development.nosync/BambuStudio-bac-data`): holds `Heimdall2 - BAC review.3mf`,
  `Heimdall2 - Cubic (original).3mf` and `baselines/cubic/pN/output/plate_N.gcode` (16 Cubic plates, regenerated with the branch's
  build: `batch.sh` on a variant dir whose app slices the Cubic project, then copy the `pN` dirs).
- `BAC_PYTHON` (default `python3`): a Python with shapely 2 and numpy (the session used a venv with Python 3.14 and shapely 2.2).
- `BAC_WORK` (default `$BAC_DATA/work`): scratch for the one-off analyses.
- The slicer is always run with `--slice N`; without it the binary opens the GUI.
- Instruction counts come from `/usr/bin/time -l` (instructions retired); never use wall time.
