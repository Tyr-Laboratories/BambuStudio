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

## Initial implementation verification

- Native `organic_foundation_tests`: 17 cases, 2,587 assertions passed.
- Existing `support_base_tests`: 21 cases, 534 assertions passed.
- The final foundation, spacing, and angle regressions fail against the original generator and pass with the new foundations.
- The test cavity has 21 lower branches converging to four midpoint stems, retaining 35 upper branches. Layer-to-layer extrusion footprints remain connected.
- Collision-view tests cover asymmetric model gaps, raft offsets, blockers, seam reachability, cancellation, and retry geometry. Repeated cancellation checks pass after correcting the nested-cache retry path.
- Independent source review completed without remaining correctness findings.
- These checks establish generated geometry and native software behavior; physical print stability remains untested.

## Contact-role and midpoint-path corrections

- [x] Add a failing printed-output regression for top interfaces disabled with a nonzero bottom count. Keep foundation grounding masks internal unless the existing renderer enables bottom interfaces. Confirm enabled interfaces still begin at the contact.
- [x] Reproduce the direction reversal in a stem whose original below-midpoint merge was removed. Identify the degree-two path between its nearest surviving upper and lower junctions.
- [x] Keep real junctions, contacts, layers, radii and topology fixed. Allow the artificial midpoint XY to move onto the endpoint chord when radius-aware collision and per-layer movement checks permit it. Reuse the existing branch extrusion.
- [x] Rebuild affected layers from all branch contributions, including junction caps. Validate both endpoint joins, midpoint continuity, permitted feet and distinct-foot connectivity before atomically accepting the upper and lower changes. Retain the valid original candidate when the chord is blocked.
- [x] Verify a clear bowed path, an obstructed chord, fixed endpoints, actual printed trunk geometry, and unchanged pure/mixed build-plate controls. Complete independent source review.
- Package the committed correction and install with a fresh rollback backup; retain the package and installation receipts under `build/verification/organic-foundations/`.

Correction regressions were demonstrated against earlier builds: the installed build fails the thin-ledge stem path bound with a 5.092892 mm departure, and the accepted-foundation and short-gap fallback cases expose interface rows when top interfaces are zero. The actual saved model also has a separate printed-output acceptance check, so a passing synthetic fixture cannot substitute for correcting the reported middle stem.


### Correction verification

- Organic foundation suite: 26 cases and 7,152 assertions passed. Existing support-base suite: 21 cases and 534 assertions passed.
- The thin-ledge output regression retains one connected printed component through its unbranched span and reduces maximum chord departure from 5.092892 mm to about 0.035 mm.
- On the saved Assembly, the reported middle stem's graph endpoints are at Z 182.8 and 202.4 mm. Its printed-footprint centroid departure from that endpoint chord drops from 4.227694 to 0.115454 mm, within the unchanged 0.2 mm bound. The wider cap-inclusive diagnostic at Z 179.6–205.6 drops from 4.424985 to 0.523680 mm; fork-cap centroids are not the branch axis.
- Across 272 changed body-layer pairs, no new printed footprint falls outside the existing 0.353027 mm movement/growth envelope. All 2,695 audited main support components overlap preceding material; original contact footprints and protected upper overhang coverage are retained.
- Pure build-plate and remote mixed-fixture build-plate geometry fingerprints match the installed baseline exactly. Per-segment extrusion roles, overhang degrees and cooling/customization metadata also match exactly.
- The top-zero interface regression covers accepted foundations and a short-gap original-support fallback; top-enabled positive controls retain contact interfaces.
- Independent source review found no remaining blocking issue. These checks validate generated geometry and toolpaths; physical print stability remains untested.
