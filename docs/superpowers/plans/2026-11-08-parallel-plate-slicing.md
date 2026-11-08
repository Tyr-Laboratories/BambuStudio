# Parallel Plate Slicing Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans to implement this plan task-by-task. Root owns builds and integration.

**Goal:** Generate whole-project G-code and previews concurrently without changing individual plate output.

**Architecture:** Prepare independent existing Print contexts on the UI thread and admit bounded BackgroundSlicingProcess workers. Route tagged events through one batch queue; reuse existing slice/export/thumbnail paths and join workers before mutation.

**Tech Stack:** C++17, wxWidgets, existing TBB/native slicing, Catch2, CMake/Ninja.

**Spec:** [Parallel whole-project slicing](../specs/2026-11-08-parallel-plate-slicing-design.md)

## Global constraints

- Same geometry, extrusion roles, printer parameters, filament mapping, post-processing and preview as individual slicing.
- Maximum four active plate workers, reduced by two hardware threads per slot and 3 GiB per slot after a 2 GiB reserve; minimum one.
- UI preparation/rendering/commit remain on the UI thread. Worker events carry plate id and generation.
- Join all active workers before model/plate/config destruction or mutation; retain completed valid plates on cancellation.
- Preserve single-plate and imported-G-code workflows; no new printer/network actions.

## Review focus

- Two concurrent processors interleave flush tags or use different printer types.
- Cancel/restart delivers old completion events after a new batch starts.
- Invalid or empty first/last plates and cached middle plates affect batch success.
- Thumbnail callbacks are pending when a project is closed or modified.
- Post-processing changes preview text while computed nozzle/filament metadata must survive.

## Task 1: Make core processing state independent

Files: `src/libslic3r/PrintBase.{hpp,cpp}`, `src/libslic3r/GCode/GCodeProcessor.{hpp,cpp}`, `src/libslic3r/GCode.cpp`, `tests/fff_print/test_parallel_plate_slicing.cpp`, relevant CMake target.

- [x] Add failing interleaved flush-role and printer-context regressions; native two-Print serial/parallel output comparison.
- [x] Run the targeted cases and confirm failures attributable to shared state.
- [x] Move role/printer state into processor/context instances; make timestamp/result counters atomic.
- [x] Replace Lightning global random traversal with deterministic geometry-local choices; compare repeated and concurrent conversion.
- [x] Run focused processor and actual slicing comparisons, then commit.

## Task 2: Add bounded batch admission and worker event identity

Files: new `src/slic3r/GUI/ParallelPlateSlicing.hpp`, `src/slic3r/GUI/BackgroundSlicingProcess.{hpp,cpp}`, `tests/gui_jobs/test_parallel_plate_slicing.cpp`, `tests/gui_jobs/CMakeLists.txt`.

Interfaces: UI-owned `ParallelPlateSlicing` queue with generation, pending/running/complete plate ids, concurrency limit, progress, completion and cancel operations; worker event context includes plate id and generation.

- [x] Add tests for bounded initial admission, slot replacement, out-of-order events, duplicate/stale completion, cancellation, and concurrency policy limits.
- [x] Confirm tests fail, then implement the small scheduler and explicit worker event identity.
- [x] Replace orphan-on-timeout cancellation with stop-before-reuse for every worker, including the prior automatic/single-plate job. Cancel all batch jobs before waiting for any one of them.
- [x] Isolate post-process child environments; test cancellation and metadata-preserving finalization paths.
- [x] Run focused queue/job tests and commit.

## Task 3: Wire Preview All plates to the batch

Files: `src/slic3r/GUI/Plater.cpp`, relevant Plater declarations, PartPlate integration if needed.

- [ ] Snapshot printer/mapping settings and eagerly allocate unique output paths on the UI thread; commit filament/nozzle maps on completion.
- [ ] Prepare/apply/validate each eligible plate through the existing configuration path; retain cached results and report invalid plate ids.
- [ ] Launch bounded independent workers; bind plate/generation-aware progress and completion events.
- [ ] Update only completed plate results; retain aggregate progress until every admitted plate finishes.
- [ ] Cancel/join on Cancel, edit/apply, job replacement, project load/clear, publishing cancel and destruction. Restore selection and suppress stale events.
- [ ] Retain post-process consent and cached/imported-G-code behavior; prevent publishing an incomplete batch.
- [ ] Build app, exercise multi-plate completion and cancellation in UI, measure serial versus parallel whole-project time, and commit.

## Task 4: Release the complete optimization

- [ ] Finish paired Organic benchmarks and full geometry/interface regressions; document exact artifacts, metrics and evidence limits.
- [ ] Re-run affected GUI/core checks and packaging validation after integration review.
- [ ] Package matching app and corresponding source; install with profile/session preservation and verify the launched build.
- [ ] Merge into default master, publish next unused Tyr release with verified assets, and remove temporary profiling/rollback artifacts.
