# Organic foundations implementation plan

**Goal:** Generate converging foundations for organic trees rooted on the model, reusing organic branching and preserving build-plate trees.

**Architecture:** Run the existing generator against a reversed physical collision view constrained to a midpoint seam. Keep upper and lower forests separate and combine their sliced tube geometry per connected tree.

**Tech Stack:** C++17, libslic3r, Clipper polygons, TBB, Catch2, native CMake/Ninja.

**Spec:** `docs/superpowers/specs/2026-11-07-organic-foundations-design.md`

## Global constraints

- Reuse existing branch sampling, propagation, merging, settings and extrusion.
- Preserve build-plate rooted trees and physical model-contact gaps.
- Retain existing support-base painting permissions.
- Work from verified origin/master 2cde13653; preserve the unrelated bridge-aware cubic checkout.

## Review focus

- Multiple stems crossing one midpoint: replace only when each seam has a valid foundation.
- Short/narrow cavities: retain valid original geometry when replacement is impossible.
- Blocked or painted contact surfaces: every new foot must have an allowed landing.
- Rafts and unequal contact gaps: reverse layer mapping must preserve physical coordinates.
- Midpoint joins: no disconnected shells or lost overhang contacts.

## Tasks

- [x] Add an end-to-end foundation regression and record its failure on the original generator. Cover it in `tests/fff_print/test_organic_foundations.cpp` with a registered native test target.
- [x] Add `TreeModelVolumes(const TreeModelVolumes&, LayerIndex, const Polygons&, std::function<void()>)`, with reversed collision lookup and existing avoidance propagation constrained at the seam. Cover it in `test_organic_foundation_volumes.cpp`.
- [x] In `TreeSupport3D.cpp`, reuse the existing sampler/path generator for permitted foundation masks. Reuse smoothing with a pinned virtual root and tube extrusion, then reflect the mesh into physical Z.
- [x] Select model-rooted trees and their midpoint crossings, replace only validated lower geometry, and preserve upper contact coverage and physical bottom interfaces.
- [x] Run focused foundation tests plus the existing support-base suite; inspect geometry and seam continuity. Review the full diff independently and address findings.
- [x] Update the feature guide with behavior and verified limitations. Record final checks and leave the feature branch reviewable.

## Verification result

- Native `organic_foundation_tests`: 17 cases, 2,587 assertions passed.
- Existing `support_base_tests`: 21 cases, 534 assertions passed.
- The final foundation, spacing, and angle regressions fail against the original generator and pass with the new foundations.
- The test cavity has 21 lower branches converging to four midpoint stems, retaining 35 upper branches. Layer-to-layer extrusion footprints remain connected.
- Collision-view tests cover asymmetric model gaps, raft offsets, blockers, seam reachability, cancellation, and retry geometry. Repeated cancellation checks pass after correcting the nested-cache retry path.
- Independent source review completed without remaining correctness findings.
- These checks establish generated geometry and native software behavior; physical print stability remains untested.
