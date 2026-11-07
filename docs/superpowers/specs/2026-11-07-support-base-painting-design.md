# Support base painting

## Intent

Allow a user to paint a surface on a printed part as a permitted base for tree supports. This provides a nearby foundation for elevated overhangs that would otherwise require long towers to the bed when **On build plate only** is enabled.

The paint permits a base; it does not create an overhang or require support by itself. Existing support enforcers and blockers remain independent. Unreachable overhangs continue using the bed, and unpainted model surfaces remain prohibited as bases under build-plate-only mode. Existing bottom contact gaps, interface settings, branch sizes, movement angles, and obstacle avoidance still apply.

## Interaction and compatibility

Add a **Support bases** mode to the existing Supports Painting tool. Use a distinct color, left-click to mark, and right-click or Shift+left-click to erase. Normal support painting keeps its current controls. The overhang-only brush filter does not apply to bases. Erase-all affects the active mode. Base edits participate in undo, project saving, model copying, mesh operations, and support reslicing.

Store base paint in an independent model facet annotation and 3MF extension, so older readers can ignore it without interpreting it as ordinary support paint. Preserve existing projects unchanged when no base paint exists.

## Routing

The implementation targets **Tree Organic**, confirmed by the user. Other styles retain their selected algorithms and do not use support-base paint.

Use only upward-facing painted surfaces in the actual print orientation. Intersect these projections with genuine model surfaces and respect the branch footprint, rather than permitting contact merely because the center lies inside the paint. Routes to these surfaces must pass the same collision and movement constraints as ordinary organic branches. Prefer a reachable permitted base over a bed route. Do not enable arbitrary model contact as a side effect.

No paint, ineffective paint, disabled supports, or unrestricted model bases preserve their existing behavior. A base patch that cannot fit a branch is ineffective. The feature does not force a different support style.

## Acceptance

Automated slicing must show a shortened tree rooted on a painted elevated platform, while a distant unpainted platform keeps its bed route. Verify actual support paths and bottom contact placement, not just routing masks. Cover wrong-facing paint, insufficient patch area, obstacles, transformations, independent support paint, save/reload, erase/reslice, and relevant contact/raft options. Verify existing focused suites and build the native app. Work on a feature branch from current public master and merge only this feature after review.
