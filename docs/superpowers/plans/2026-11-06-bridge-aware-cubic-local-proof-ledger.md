# Bridge aware cubic local proof ledger implementation plan

**Goal:** Reduce repeated slicing work enough to pursue the required end-to-end limit of no more than 2× official Bambu Studio regular Cubic on the qualification fixtures, while retaining the approved geometry and irreducibility requirements.

**Architecture:** Retain the provenance of final emitted strands and the exact inputs of their support checks. Use physically preserved material cycles to bound complete affected voids, and revisit collapse candidates when their actual dependencies change. Existing exhaustive verification remains the fallback and final authority.

**Tech stack:** Existing C++ geometry, Clipper, scaled integer coordinates, exact integer predicates, Catch2, and shared cancellation/work budgets. No new external dependency.

**Spec:** [Approved design](../specs/2026-11-05-bridge-aware-cubic-design.md). The [feature documentation](../../bridge-aware-cubic.md) records current implementation and qualification limits.

**Execution:** Implement one independently testable increment at a time. Start with exact Contact query signatures; final emitted provenance is the first ledger increment after that prerequisite. The signature work does not authorize simultaneous changes to `apply_layer_paths`. Builds, slices, and benchmark runs remain centrally coordinated. This plan does not change the approved specification or claim a performance result.

## Global constraints

- Preserve nominal bulk Cubic density and phase away from actual demand; each recursive child independently requires demand or an identified support dependency.
- Preserve complete partition faces and physically supported births. Neither a demand edge nor a logical cell/modifier boundary is material.
- Check actual emitted bead contact and complete consecutive unsupported path runs, including terminal anchors, at each actual output phase and contact elevation.
- Check complete physical voids meeting demand. Missing walls can join cells; a local computation window must not close that opening artificially.
- Preserve shared owners, region gap policy, exact source paths, solid-cut demand, and separate terminal attachment ownership.
- Coarsening must reach irreducibility for the fixed lattice and available collapse operations. A constructive bound that cannot prove a result means fallback, not a reason to retain an unnecessary split.
- Keep finite work, memory, coordinate, depth, and cancellation limits. Incomplete required verification cannot produce a passing slice or an irreducibility certificate.
- Keep the independent final/export checks and the existing physical-print qualification boundary. Runtime success is not print qualification.

## Why the present work repeats

`changed_plan_layers` expands a changed cell into complete layer ranges and generation successors. `check_changed_layers` regenerates those layers, collects all model targets there, then collects and checks construction targets. The construction collector reconstructs strand classification and ownership from geometric masks despite the generator having known the producers earlier.

`check_targets` prepares current lower material and physical support complements for those layers and phases. Its existing early rejection, signatures of whole layers, exact mask caches, and failure replay reduce constants or repeated failures; they do not isolate a local edit from unrelated obligations. The approved design expressly permits local checks with retained ownership; it does not mandate these whole-layer repeated operations.

The major change is therefore in the unit of work: complete emitted strands, complete contact inputs, and physically enclosed voids, rather than a whole layer for every candidate.

## Proof contract

### Exact identity and provenance

A ledger belongs to one object-generation epoch. An emitted strand identity includes its layer and emitting region, exact complete polyline, extrusion role, width, height, flow, force-no-extrusion state, and any representation state affecting serialization. A geometry hash accelerates lookup but never replaces exact comparison. Reused addresses and vector ordinals are not persistent identities.

Every output phase retains the separate `SerializedXY` shift, nozzle offset, and plate-origin components in their actual operation order. Contact keys include actual lower and upper deposition elevations, upper height, selected contact plane, and tolerance. Variable-height layers and thick bridges must not be reduced to adjacent nominal Z values.

Construction provenance is independent of policy selection. Producer IDs are region scoped; ordinary bulk, added partitions, solid replacements/cuts, closures, and ordered attachment transitions remain distinguishable. Opposite endpoint owners are not flattened into a single interchangeable owner set. Lower eligible domains and their effective gap policies are applied after the surviving final strand is known.

### Complete path facts

A reusable span fact includes the complete serialized upper path, actual contact query signature, exact selection geometry, gap, and relevant target policy. Changed contact at any point can join unsupported intervals across several segments: recheck the whole path, never a clipped piece.

The Contact helper merges touching collinear lower sections before constructing masks and finite line witnesses. Changing one primitive segment can change a much longer canonical axis and its numerical predicates. Signatures and invalidation therefore cover the entire changed canonical merged axis, including ordering and exact section data.

Fresh candidate discovery is necessary even when the previous signature listed no supporting bead: material added in previously empty space must be observed. Invalid or incomplete prepared material cannot match a previous passing fact. Span-only facts may serve construction and bridge-followup checks; they cannot replace direct-roof physical-cell checks.

### Physical separator cycles

Let `S` be the actual physical support footprint at one contact plane and phase. A separator certificate consists of a simple closed polygon `C` whose entire boundary has strictly positive clearance inside `S`, plus the exact material witnesses establishing that coverage. Its boundary must be actual material, not an analytic lattice edge assumed to exist.

The construction classifier's tolerance mask around added axes is not `S`. The strict-containment primitive may be reused only against the actual physical sections, with the separator's own identity, coverage, and material-witness checks.

Every void component meeting the interior of `C` stays inside `C`: a connected void cannot cross a closed curve contained in material. Consequently, if the diameter of the interior of `C` is within the applicable permitted gap, all those complete void components satisfy the diameter bound. This is a sufficient positive proof, not a substitute for exact measurement when the bound is inconclusive.

For a direct-roof target, prove that its current selected unsupported demand is covered by certified interiors (and any actual support excluded by the existing selector). A point-in-cycle test of a centroid or a path bounding box is insufficient. Demand migration after a support change must repeat this coverage check.

A candidate that breaks a child cycle can be evaluated inside an enclosing preserved parent cycle. The complete computation boundary lies inside unchanged physical material; thus the local complement cannot invent a closing wall. If no such boundary can be certified, expand to a larger preserved separator or use the full-plane oracle. A local result reaching the computation boundary is inconclusive and falls back.

Do not assume exact byte equivalence between independently clipped and batched open subjects. Keep the current generation/clipping behavior in the first increments. Any later canonical clipping change needs its own output and geometry qualification; provenance alone does not justify it.

### Transaction and irreducibility

Each candidate transaction records all changed emitted strands and canonical lower sections, updated ownership/dependency roots, complete affected upper paths, and affected separator witnesses. A physical support delta is computed from actual post-serialization sections; a nominal parent-cell box is only a discovery accelerator.

A failed-collapse certificate is valid only while its target survives identically and all inputs to the hypothetical collapse and failure witness remain identical. Its read set includes spatial absence queries, canonical merged axes, phase, selection/gap, source-generation inputs, and ownership reachability. No general material-subset monotonicity is assumed: removing solid fallback may restore bulk, and removing an attachment may restore a previous endpoint.

Accepted mutations invalidate overlapping read sets and enqueue affected candidates. Removing a construction target also invalidates failures that relied on that target. When the queue is empty, every surviving collapse operation has a current failed certificate and every actual obligation has a current passing fact. This establishes the same fixed-lattice irreducibility contract. Retain a fresh exhaustive final check of emitted material.

## Component boundaries

The following names define intended responsibilities; the Contact signature declaration should match the implementation already being developed rather than introduce a second signature type.

| Component | Inputs | Output / contract |
| --- | --- | --- |
| `Contact::QuerySignature` in `BridgeAwareCubicContact.hpp/.cpp` | Immutable prepared lower material, complete upper extrusion, exact plane/tolerance, shared budget | Exact ordered canonical candidate data and status; fresh bounded discovery; no support verdict |
| `SpanProofLedger` | Query signature, exact target/phase/selection/gap | A previous completed span result only when all determining inputs match |
| `EmittedConstructionLedger` | Final surviving strands and carried producer/attachment/solid-cut tags | Immutable layer-generation records; no policy-derived demand or support verdict |
| `collect_from_emitted_ledger(...)` | Current layer ledger, current eligible domains and gap policy | Same complete construction targets as the reference collector, in deterministic order |
| `EmissionDelta` | Before/after final records and canonical contact sections | Exact added/removed/changed records, with shared-owner and generation changes |
| `SeparatorLedger` | Actual physical sections, phase/plane, candidate simple cycles, shared budget | Certified material boundaries and enclosed diameter bounds; unavailable when proof is incomplete |
| `LocalProofLedger` | Complete path facts, separator facts, dependency/version read sets | Transactional invalidation, commit, rollback; no mutation of frozen model paths |
| `CollapseWorkQueue` | Current candidates, exact failed certificates and accepted deltas | Deterministic retries until every remaining candidate has a current failure or verification stops incomplete |

New components should live under `src/libslic3r/Fill/BridgeAwareCubic*.hpp/.cpp`; orchestration remains in `src/libslic3r/BridgeAwareCubicSupport.cpp`. Avoid adding a generic mutable global geometry service.

The ledger interfaces should expose these operations, with immutable values and explicit status rather than sentinel passing results:

- `SpanProofLedger::find(key, signature, budget)` returns an optional completed span fact; `publish(key, signature, result, budget)` accepts completed passing results only.
- `EmittedConstructionLedger::seal(final_paths, carried_tags, generation, budget)` returns an immutable complete ledger or unavailable. `records()` exposes the final records in emission order.
- `compare_emissions(before, after, budget)` returns `EmissionDelta`, including geometry, metadata, ownership, and generation changes.
- `SeparatorLedger::certify(cycle, physical_sections, plane_phase, budget)` returns a certificate or unavailable. `covers(selection, gap, budget)` proves complete selected-demand coverage or requests the oracle.
- `LocalProofLedger::begin(delta, budget)` returns a transaction with explicit `commit()` and `rollback()`; `invalidated_candidates()` supplies the deterministic queue updates.
- `CollapseWorkQueue::next()` supplies an unevaluated/invalidated candidate; `record_failure(candidate, exact_read_set)` retains only a completed current failure. These read sets are a later increment, not an extension of the current failure-replay hint into a certificate.

The exact field types for keys, records, and read sets follow the identity contract above. The existing anonymous `Target` type need not become a public ledger type: policy application and conversion to `Target` can remain inside the support collector.

## Implementation increments

### 1. Reuse exact passed span facts after fresh Contact discovery

**Files:** `BridgeAwareCubicContact.hpp/.cpp`, new `BridgeAwareCubicSpanProof.hpp`, `BridgeAwareCubicSupport.cpp`, `test_bridge_aware_cubic_contact.cpp`, new `test_bridge_aware_cubic_span_proof.cpp`, relevant CMake lists.

- [ ] Add failing signature tests for unchanged geometry, added support entering an empty query, deleted support, reordered/merged canonical axes, changed width/height/Z, and exact output-phase components.
- [ ] Expose the bounded signature query using the same candidate discovery and canonical data as actual contact. Compare exact owned data; hashes alone cannot prove equality. All input/status checks remain required.
- [ ] Add span-fact tests where a formerly supported path becomes floating, a former failure becomes supported, and unsupported runs migrate across intermediate joints.
- [ ] Reuse only completed passing span-only results with identical selection and gap. Continue full cell checks for direct roofs. Shared work and resident-allocation bounds cover discovery, equality, retained facts, and fallback.
- [ ] Run focused `[SpanProof]` and Contact tests, then the existing exhaustive-versus-optimized qualification fixture. Report actual hit rate and time removed; no new geometry generation behavior is part of this increment.

### 2. Preserve final emitted construction provenance — minimum viable ledger

**Files:** `BridgeAwareCubic.hpp/.cpp`, `FillBridgeAwareCubic.hpp`, `Fill.cpp`, `BridgeAwareCubicSupport.hpp/.cpp`, new `BridgeAwareCubicEmissionLedger.hpp/.cpp`, `Print.hpp` or the existing object-owned storage boundary, new `test_bridge_aware_cubic_emission_ledger.cpp`, interface/ownership tests and CMake lists.

- [ ] Add differential tests comparing ledger-derived targets with the current collector: complete path/flow identity, emitter, region-scoped producers, ordinary-bulk exclusion, solid-cut selection, gap assignment, and target order.
- [ ] Carry source tags through the actual clipping operations that produce the paths. Existing `OpenPathIndex` can carry attribution within the same operation; it is not permission to change its batching. Retain all owners of shared partitions and the ordinary-bulk evidence that excludes an inherited strand from added-construction classification.
- [ ] Carry tags through solid-family generation, final whole-bead material clipping, closure deduplication, and each ordered attachment transition. Preserve separate start/end transitions and their source/target paths.
- [ ] Seal `EmittedConstructionLedger` at the end of `apply_layer_paths`, after both attachment replays, closure clipping/deduplication, and stationary-construction removal. Only surviving complete emitted paths appear. An early Plan-only sidecar is not authoritative.
- [ ] Replace only the collector's repeated geometric classification/owner reconstruction for a completely admitted, current ledger. Continue the current target-policy projection and full support validation unchanged. An absent/stale/partial ledger uses the original collector for the entire affected layer/region, not a mixture that can omit strands.
- [ ] Keep ambiguous mixtures of ordinary and added source intervals on the reference classification path until equivalence is established. A changed label is a behavior change to investigate, not an acceptable consequence of skipping the old matcher.
- [ ] Verify two owners sharing a strand, removal of either owner, a shared strand that remains ordinary bulk, cross-region closure emission, both-end attachment lineage, saturated solid cuts, reslice generation changes, and pointer/address reuse.
- [ ] Run `[EmissionLedger]`, ownership/interfaces, ribs/modifiers, and exact-target differential tests. Measure classification calls eliminated and total collection time. Do not create the ledger by running the old geometric classifier after every regeneration; that merely relocates the cost.

### 3. Publish exact emission deltas and update complete path facts

**Files:** `BridgeAwareCubicEmissionLedger.hpp/.cpp`, `BridgeAwareCubicSpanProof.hpp`, `BridgeAwareCubicContact.hpp/.cpp`, `BridgeAwareCubicSupport.cpp`, new `test_bridge_aware_cubic_emission_delta.cpp`.

- [ ] Add transaction tests for addition, deletion, changed flow, restored bulk after solid removal, restored attachment endpoints, and owner-only changes without geometric movement.
- [ ] Derive `EmissionDelta` from exact final records for the current phase and elevations. Initially retain current whole-layer regeneration; compare its output and update only affected ledger records/facts.
- [ ] Use conservative spatial indexes to discover affected complete upper paths. Include the whole changed canonical lower axis and all possible contact extents. Ambiguity expands the scope; it does not remove a path from checking.
- [ ] Register absence regions or rerun exact signature discovery so a newly added lower strand invalidates a fact even when no old source ID changed.
- [ ] Recheck surviving changed construction paths with current policy and provenance. Remove obligations only when the exact source and its final emitted path no longer require them.
- [ ] Run `[EmissionDelta]` and a shadow comparison against complete recollection/revalidation after every mutation. Include a tiny primitive edit that changes a long canonical merged axis and a contact change that joins several previously short air intervals.

### 4. Certify material separator cycles and local complete-cell checks

**Files:** new `BridgeAwareCubicSeparators.hpp/.cpp`, `BridgeAwareCubicValidation.hpp/.cpp`, `BridgeAwareCubicSupport.cpp`, new `test_bridge_aware_cubic_separators.cpp`, existing cavity/modifier/variable-height tests.

- [ ] Add failing cycle tests: genuine finite-width material enclosure, logical-only closure, tangency/zero-area contact, cavity opening, clipped demand, and a cycle jointly supported by several owners.
- [ ] Discover candidate cycles from inherited cell faces or measured physical components. Prove simple closed geometry and strict material coverage of every boundary segment at the exact phase/contact plane. Record the exact supporting sources; do not use ordinary extrusion width at an inapplicable height.
- [ ] Add the positive complete-cell shortcut only when current selected unsupported demand is fully covered by certified cycle interiors whose diameter bounds satisfy the existing permitted gap. Near-threshold or uncovered cases retain exact validation.
- [ ] For changed cells, identify a preserved enclosing cycle and run the existing physical-cell measurement inside it with actual material. Do not clip the demand to make coverage succeed. Expand to a larger preserved cycle or use full-plane validation whenever enclosure, topology, or local/global predicate equivalence is not established.
- [ ] Differentially compare local and full-plane results after every mutation. A discovered Boolean-rounding disagreement disables the local result and uses the oracle; derive any safe admission margin without relaxing the requested gap. Do not count fallback-derived equality as a local certificate.
- [ ] Test two individually harmless removals that jointly break a cycle, a missing partition joining several nominal cells, an added support strand splitting a void, a newly created void inside formerly solid material, migrated unsupported demand, concave openings, and a physical component extending outside the changed cell box.
- [ ] Run `[Separators]` and the existing direct-roof/cavity/modifier/phase fixtures. Report local cycle successes, enclosing-cycle expansions, full-plane fallbacks, and component work; final full geometry remains required.

### 5. Replace complete coarsening sweeps with dependency-driven transactions

**Files:** new `BridgeAwareCubicLocalProof.hpp/.cpp`, `BridgeAwareCubicDependencies.hpp/.cpp`, `BridgeAwareCubicSupport.cpp`, existing `BridgeAwareCubicFailureReplay.hpp`, new `test_bridge_aware_cubic_local_proof.cpp`, trial-rejection tests.

- [ ] Add queue tests for exact unchanged read sets, newly added support in an empty query, target removal, shared-owner removal, invalidated parent cycles, and plan-generation changes.
- [ ] Store successful obligation facts and failed candidate certificates with explicit dependency/version read sets. A previous failure by itself remains only a replay hint.
- [ ] Process candidates deterministically. Apply the trial, obtain actual emission deltas, recheck complete affected paths and complete enclosed cells, then atomically commit or roll back geometry, ownership, and proof records.
- [ ] Enqueue all candidates whose hypothetical output or proof inputs changed. Release unsupported dependency cycles only through the existing rooted ownership rule; a surviving owner is not automatically interchangeable with a removed opposite-terminal owner.
- [ ] End only when every current obligation passes and every remaining collapse has a current failure. Keep the existing full-sweep implementation as a diagnostic reference and required fallback when an affected/read-set boundary cannot be proved.
- [ ] Run `[LocalProof]` and compare final refinement IDs, material, paths, ownership, and complete validation results with the reference on small deterministic cases. Include joint cycle break, migrating unsupported demand, and canonical-axis changes in the same multi-mutation sequence.

### 6. Reduce generation scope and qualify end-to-end behavior

**Files:** `BridgeAwareCubic.hpp/.cpp`, `FillBridgeAwareCubic.hpp`, `Fill.cpp`, `BridgeAwareCubicSupport.cpp`, final/export certificate code, existing qualification fixtures and benchmark tooling.

- [ ] Measure the remaining time after the preceding increments. If whole-layer generation dominates, introduce generation buckets with explicit producer, material-domain, flow, and predecessor-preview dependencies; do not infer a local boundary from a cell box alone.
- [ ] Preserve current clipping batches where required for exact output. A bucket can be regenerated independently only when its output equivalence is proved. Otherwise keep generation broad while proof work remains local. Any deliberate canonical clipping redesign is a separate behavioral increment with new emitted-geometry qualification.
- [ ] Capture and verify an unchanged exhaustive final/export certificate after the event queue completes. A downstream coordinate, flow, path, order, or phase change invalidates that certificate.
- [ ] Run the full default feature suite, the existing small/large qualification cases, rib and modifier fixtures, reslice tests, exported-coordinate checks, and existing legacy suites. Execute the paired official-stock benchmark with matching persistent settings.
- [ ] Report process/export times, total moving material, split counts, peak memory, local/fallback work, and any admission failures. The release condition remains end-to-end slicing at most 2× the official baseline on every required fixture.

## Bounded work and oracle fallback

All new acceleration metadata is optional and object-generation scoped. Start with an aggregate 64 MiB resident metadata cap, checked before allocation, in addition to the existing 200,000-vertex and 50,000,000-work query limits and existing 10,000 coarsening-trial limit. A retained record's arrays and referenced immutable storage count toward the aggregate cap. Eviction drops acceleration, not required material or obligations. Avoid hidden unbounded per-phase copies.

Charge index construction, canonicalization, candidate discovery, exact equality, Boolean inputs, invalidation, queue operations, and retries. Probe/local work and subsequent oracle fallback share the verification allowance; no refund on failed admission. Check cancellation at bounded intervals in all traversals and immediately before expensive calls. Optional allocation failure falls back; required geometry failure preserves its existing incomplete/error outcome.

Use the exhaustive path/plane/collector implementation for missing or stale provenance, unsupported serialization/representation, unsafe coordinates, unresolved shared ownership, absent material separators, insufficient local predicate guarantees, boundary-touching local voids, or metadata admission failure. Passing a local path check never suppresses required cell checks. Passing one candidate never suppresses validation of other obligations.

The expected work changes from repeated whole-layer collection/proof toward one initial material/proof construction plus actual changed strands, complete affected paths, and affected enclosed components. The worst case remains broad: a cavity-connected opening, dense saturation, a long merged axis, or every candidate changing the same component can invalidate most facts. Caps must report inability to complete rather than imply a universal fast path.

## Review focus and acceptance

For each increment, add its new test file to `bridge_aware_cubic_tests`, run its named Catch2 tag after the coordinated build, and require all assertions to pass. Use the existing exhaustive reference switch (`BAC_PROFILE_FULL_TRIAL_CHECKS=1` with profiling enabled) for differential slice runs. Never run a second shared build or fixture concurrently merely to satisfy this plan; verification ownership remains unchanged.

| Failure mode | Required observable behavior | Owning increment |
| --- | --- | --- |
| Two changes jointly remove a physical cycle | Second transaction invalidates the cycle and measures the complete merged opening | 4–5 |
| Unsupported demand moves after contact changes | New demand coverage is checked; an old covered patch is not reused | 1, 3–5 |
| Tiny lower edit alters a long merged axis | All affected canonical contact inputs and whole upper paths are reconsidered | 1, 3 |
| Shared geometry survives with different terminal lineage | Geometric presence and valid ordered ownership are checked separately | 2, 5 |
| Same net transform or reused memory hides different output state | Exact phase components and generation-qualified records prevent stale proof reuse | 1–3, 6 |

The proof ledger is successful only if final geometric and irreducibility results remain valid and the paired performance gate passes. Removing repeated work does not establish a runtime result in advance. The unavoidable work includes generating the additional material and checking actual required paths/phases; extremely small gap settings can produce much more output than regular Cubic, so no universal 2× claim across all possible settings is made.

Final verification pending.
