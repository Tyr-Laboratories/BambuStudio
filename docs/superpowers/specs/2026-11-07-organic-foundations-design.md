# Branched foundations for organic supports

Organic supports rooted on removable model contacts need several separated feet to resist falling over during printing. For trees that both start and end on the model, generate the lower half with the existing organic generator in reversed Z, converging upward from foundation contacts. Retain the existing upper half. Trees rooted on the build plate retain their existing generation and geometry.

The user requested reuse of the existing branching algorithm and settings, with a midpoint split along Z. No new process setting is introduced. Existing branch angle, diameter, tip diameter, branching distance and growth rules drive both halves. Model contact separation and bottom interface settings retain their physical meanings.

## Implementation decisions

Classify connected trees after normal path generation and smoothing. A tree must have an actual model landing above the raft/build plate to qualify. Its midpoint is the printable layer nearest the halfway Z between its foundation and highest supported tip. A midpoint crossing more than one upper stem requires a foundation for each stem.

Keep the two directed forests separate: the existing graph allows multiple branches above a stem but only one stem below a junction. Inverting graph links would violate that invariant. Reuse its generator on a reversed collision-volume view and combine the resulting geometry per tree instead.

Foundation candidates are existing permitted model contact regions near the original landing, restricted by the reverse generator's reachability from the midpoint. Sampled feet belong to the nearest eligible seam stem sharing that landing height, preventing overlapping copies of a platform's foundations. The reversed view maps collision queries to the real physical layer, preserving asymmetric contact gaps and XY clearances. Its avoidance cache is calculated in reversed order, starting at a constrained midpoint landing. It does not allow routes to terminate on intervening surfaces. Support blockers also constrain the new foundations.

Use the existing overhang sampler and propagation/merge functions, followed by the existing point selection, smoothing and tube extrusion. Disable virtual build-plate foot widening because the midpoint is a join, not a plate contact. Pin each feasible seam position during smoothing. Reflect the generated tube mesh back into physical Z. Share the original upper cross-section exactly at the midpoint, and bound successive lower collar sections by the existing per-layer branch movement and radius-growth envelope. Recheck actual sheath support and physical collisions; never modify a contact tip to make the collar fit. Preserve bottom contact generation and run the existing per-tree geometry cleanup on the combined result. Bottom interfaces have an independent direction counter so adding model feet cannot rotate distant build-plate top interfaces.

Only replace the original lower half after all required foundation branches and their seam connections are valid. Short cavities, narrow/obstructed contact surfaces, or restrictive branch-angle limits may not admit separated feet and a valid join; retain the original support when a valid replacement cannot be generated. This fallback must be visible in test coverage, not treated as successful branching.

## Verification

- A synthetic model-to-model cavity must produce separated foundation feet, converge near the midpoint, and retain upper branching and roof coverage.
- Adjacent extrusion footprints through the seam must overlap; the shoulder check allows normal contour-processing tolerance. Bottom gap and interfaces must remain correct.
- A distant build-plate tree must retain its generated geometry, including its plate foot.
- Reverse-volume tests cover direction, constrained seam reachability, blockers, unequal top/bottom gaps, and cancellation during lazy calculation.
- Existing support-base painting, routing, raft and slicing tests continue to pass.
- Source tests and slicing establish software behavior. Physical print stability requires a later print trial.
