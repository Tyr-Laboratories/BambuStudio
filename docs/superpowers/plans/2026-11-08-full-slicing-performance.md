# Full Slicing Performance Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** Reduce computational work throughout slicing without changing output;
measure elapsed time separately because CPU speed and background load vary.

**Architecture:** Retain the existing generators and remove redundant spatial
work in their hot paths. Keep query state local and preserve visitation order
where floating-point accumulation depends on it.

**Tech Stack:** C++17, TBB, Catch2, CMake/Ninja, macOS sample, Python CLI harness.

**Spec:** `docs/superpowers/specs/2026-11-08-full-slicing-performance.md`

## Global Constraints

- Low power mode remains enabled. Primary metrics: deterministic work counts and
  per-process retired instructions; wall time and CPU cycles are observational.
- Baseline: Tyr.22 (`f7c37853`) plus the same floating-shell and support-enforcer correctness fixes
  as the candidate; same source toolchain for paired runs.
- Preserve mesh, paint, settings, normalized G-code commands and preview roles.
- No global mutable caches, new dependencies or reduced resolution.
- User has authorized continued implementation, installation, merge and release.
- Reuse this checkout and one build tree; preserve other actors' work.

## Review Focus

- Floating-point boundary queries retain strict radius membership and order.
- A locally uniform wall patch does not hide nearby zero-count or reinforced spans.
- Grid deduplication resets for every point and does not skip another contour.
- Concurrent plates cannot share query scratch or mutate an index.
- Measurements use deterministic work and retired instructions despite load/frequency variation;
  timings remain observational and pre-existing output variation is recorded.

### Task 1: Seam neighborhood queries

Files: `src/libslic3r/KDTreeIndirect.hpp`, `src/libslic3r/GCode/SeamPlacer.cpp`,
`tests/fff_print/test_slicing_spatial.cpp`, `tests/fff_print/CMakeLists.txt`.

Interface: preserve both `find_nearby_points` overloads; add a radius-aware
tree traversal used only by the spherical overload.

- [x] Add membership/order/float-boundary tests and a coordinate-access budget
  for an empty spherical query against a dense distant corner.
- [x] Run baseline tests; observe the work-budget failure, preserve baseline binary.
- [x] Build optional immutable subtree boxes for the mesh visibility sample index, conservatively
  pruning empty subtrees while retaining preorder. Leave layer candidate and other KD users uncached.
- [x] Run native spatial tests and measure export on the saved assembly; retain
  the change only if it improves measured work/time with identical commands.
- [x] Commit the verified change and evidence.

### Task 2: Wall policy lookup

Files: `src/libslic3r/WallPainting.cpp`, `tests/fff_print/test_slicing_spatial.cpp`.

Interface: unchanged `BoundaryPolicy::split(const Line&)`.

- [x] Pin local uniform, zero-count, reversed, transition and near-tie behavior.
- [x] Capture baseline query timing and existing wall regression results.
- [x] Skip interior count queries when every candidate in a conservatively padded
  neighborhood has the same count; retain the original cuts and endpoint rounding.
- [x] Run wall/spatial regressions and compare wall time and complete G-code.
- [x] Commit the verified change and evidence.

### Task 3: Travel planning duplicate edges

Files: `src/libslic3r/GCode/AvoidCrossingPerimeters.cpp`, performance harness.

Interface: unchanged travel planning; scratch storage belongs to one visitor.

- [x] Capture baseline travel-heavy G-code and export profile.
- [x] Mark grid edges visited for each contour point so a segment present in
  multiple cells is evaluated once, in its original first-visit order.
- [x] Reject the prototype before full-model qualification: instruction counts fell
  only 1.03% on a 250-vertex annulus and rose 18.74% on a 2,000-vertex annulus.
  Restore the original production path and record the probe evidence.

### Task 4: Stabilize the floating-shell control path

- [x] Reproduce the vector lifetime error under AddressSanitizer using the actual
  production translation unit.
- [x] Preserve both source flags before vector erase/reallocation.
- [x] Reproduce the following-segment width-transition classification error.
- [x] Use the accumulated path's flag and pass all sanitizer regression cases.
- [x] Reproduce missing-anchor buffer reads under AddressSanitizer, preserving
  fallback widths at valid path endpoints.
- [x] Reproduce the incorrect closing flag count/rotation after rebasing.
- [x] Reproduce the borrowed EdgeGrid container lifetime error at the actual
  self-intersection query; keep the point container alive through that query.
- [x] Verify intersecting and nonintersecting path classification under the fix.
- [x] Pass the complete sanitizer probe after all narrow corrections.
- [x] Rebuild baseline and candidate with the same fixes; discard earlier
  whole-model qualification as release evidence.

### Task 5: Whole-pipeline qualification and delivery

Files: `scripts/benchmark_slicing_pipeline.py`, `scripts/measure_macos_process.c`, `docs/performance/` report and
compact result data; release packaging/version files per existing workflow.

- [x] Complete stage survey of all available plates, recording baseline errors.
- [x] Warm each executable, then run five alternating paired rounds across
  representative fixtures using retired instruction counters and recording variation independently of
  background load or clock frequency.
- [x] Compare every normalized G-code command and key preview/filament/time
  metadata; report median/range instructions, observed times and memory.
- [x] Run affected wall, overhang, bridge, support, parallel-plate and packaging
  checks; obtain one fresh whole-branch review and resolve material findings.
- [ ] Build release artifacts, preserve user state, install/verify, merge/push,
  publish and verify downloads, then remove this pass's temporary artifacts.
