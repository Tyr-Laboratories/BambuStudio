# Support Base Painting Implementation Plan

**Goal:** Permit organic trees to root on user-painted model surfaces while preserving build-plate-only behavior elsewhere.

**Architecture:** A separate `ModelVolume::support_base_facets` annotation feeds upward surface projections into organic model-contact routing. The existing painter gains an independent mode. Existing geometry and support interfaces remain responsible for collision, contact gaps, and extrusion.

**Tech stack:** Existing C++, wxWidgets/ImGui, TriangleSelector, Clipper geometry, Catch2, and CMake.

**Spec:** [Support base painting design](../specs/2026-11-07-support-base-painting-design.md).

## Constraints

- Branch from public `master`; do not edit the separate bridge-aware infill checkout.
- Existing project behavior must remain unchanged without effective base paint.
- Keep enforcer/blocker paint independent and retain 3MF compatibility.
- Support-base routing must not allow unpainted model contacts or tunnel through obstacles.
- No live printer actions are needed for validation.

## Tasks

- [x] Add independent model annotation and `paint_support_bases` 3MF triangle attribute. Extend annotation copy/reset/merge/reprojection, undo serialization, geometry-sharing comparisons and Print::apply invalidation. Test persistence and independence in `test_support_base_painting.cpp`.
- [x] Add `PrintObject::has_support_base_painting()` and `project_support_base_facets()` in Print.hpp/PrintObject.cpp. Project upward split facets into object layers using the existing transformed slab slicer and support shrinkage. Test horizontal, downward, mirrored, rotated, and multiple-volume cases.
- [x] Restrict organic placeable areas to painted bases under build-plate-only mode, propagate their reachability through collision-aware movement, prefer valid base routes, and prevent arbitrary-model fallback. Test cached geometry in `test_support_base_routes.cpp` and actual elevated/distant support paths in `test_support_base_slicing.cpp`.
- [x] Add **Support bases** painter mode with independent annotation, color, brush filtering, erase/undo labels and preview timestamps. Keep ordinary support controls unchanged.
- [x] Verify focused suites, native app compilation and offline slicing. Review the complete diff, correct findings, and document user controls and limits.

Integration: commit and push `codex/support-base-painting`, then merge only this feature into public `master` after verification.

## Underhang filter follow-up

- [x] Mirror the existing brush and Fill angle test to accept upward-facing facets for base painting, retaining the default downward test for other painters.
- [x] Add the base-mode checkbox, angle slider, matching highlight, independent channel values, and Undo restoration. Invalidate pending Fill selections when filter controls change.
- [x] Verify brush and Fill results on horizontal, sloped, vertical, downward-facing, transformed, and split facets; rebuild and review the native app before packaging the follow-up release.

## Review focus

Check independent paint survives geometry sharing and mesh changes. Check invalid or undersized paint cannot enable unrestricted model contact. Check merged branches preserve their valid destination. Check raft/contact-gap indexing. Check ordinary routing remains unchanged when the feature is ineffective or unrestricted model contact is already enabled.
