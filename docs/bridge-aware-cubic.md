# Bridge aware cubic

A sparse infill pattern: Cubic's lattice everywhere, plus lines under roofs, ledges and other material that would otherwise span
too far over the sparse void, so that nothing above spans more than the Unsupported gap size (`bridge_aware_cubic_max_gap`,
default 5 mm). It exists for single-wall parts at 3% infill whose first roof layer fails on ordinary Cubic.

**Status (8 November 2026).** Being rewritten from `master` against the
[rewrite architecture](superpowers/specs/2026-11-08-bridge-aware-cubic-rewrite-design.md), which replaces the earlier
constructive branch. Nothing of the pattern is in the slicer yet; this document is filled in stage by stage as section 8 of the
architecture is built. The benchmark tooling (`scripts/bridge_aware_cubic_benchmark/`, [procedure](bridge-aware-cubic-benchmark.md))
and the acceptance scripts (`scripts/bridge_aware_cubic_acceptance/`, see its README) are in place.

What a released build will do: plan once per object before the bridge generator, from the sliced surfaces; print Cubic's lines plus
the plan's strands and courses through the ordinary fill machinery; run no check. What a development build adds: the verifier and
the plan checks of the architecture's section 5, as warnings that stop nothing.

How it is judged: the contract of the architecture's section 1 (K1 roofs, K2 the pattern's own lines, no change to any other
setting), the four fixtures through an observer of exported G-code, all 16 plates of the user's parts through the acceptance
scripts, and the release gate of at most 2.0 times official Cubic in instructions retired.
