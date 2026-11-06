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

- [x] Add failing signature tests for unchanged geometry, added support entering an empty query, deleted support, reordered/merged canonical axes, changed width/height/Z, and exact output-phase components.
- [x] Expose the bounded signature query using the same candidate discovery and canonical data as actual contact. Compare exact owned data; hashes alone cannot prove equality. All input/status checks remain required.
- [x] Add span-fact tests where a formerly supported path becomes floating, a former failure becomes supported, and unsupported runs migrate across intermediate joints.
- [x] Reuse only completed passing span-only results with identical selection and gap. Continue full cell checks for direct roofs. Shared work and resident-allocation bounds cover discovery, equality, retained facts, and fallback.
- [x] Run focused `[SpanProof]` and Contact tests, then the existing exhaustive-versus-optimized qualification fixture. Report actual hit rate and time removed; no new geometry generation behavior is part of this increment.

Implemented in `af5fbf6a211f1f582278fb71b0b37a8adb3ec6cd`, including the exhaustive-output parity qualification. The speed gate remains open.

### 2. Preserve final emitted construction provenance — minimum viable ledger

**Files:** `BridgeAwareCubic.hpp/.cpp`, `FillBridgeAwareCubic.hpp`, `Fill.cpp`, `BridgeAwareCubicSupport.hpp/.cpp`, new `BridgeAwareCubicEmissionLedger.hpp/.cpp`, `Print.hpp` or the existing object-owned storage boundary, new `test_bridge_aware_cubic_emission_ledger.cpp`, interface/ownership tests and CMake lists.

- [x] Add differential tests comparing ledger-derived targets with the current collector: complete path/flow identity, emitter, region-scoped producers, ordinary-bulk exclusion, solid-cut selection, gap assignment, and target order.
- [x] Carry source tags through the actual clipping operations that produce the paths. Existing `OpenPathIndex` can carry attribution within the same operation; it is not permission to change its batching. Retain all owners of shared partitions and the ordinary-bulk evidence that excludes an inherited strand from added-construction classification.
- [ ] Carry tags through solid-family generation, final whole-bead material clipping, closure deduplication, and each ordered attachment transition. Preserve separate start/end transitions and their source/target paths.
- [x] Seal `EmittedConstructionLedger` at the end of `apply_layer_paths`, after both attachment replays, closure clipping/deduplication, and stationary-construction removal. Only surviving complete emitted paths appear. An early Plan-only sidecar is not authoritative.
- [x] Replace only the collector's repeated geometric classification/owner reconstruction for a completely admitted, current ledger. Continue the current target-policy projection and full support validation unchanged. An absent/stale/partial ledger uses the original collector for the entire affected layer/region, not a mixture that can omit strands.
- [x] Keep ambiguous mixtures of ordinary and added source intervals on the reference classification path until equivalence is established. A changed label is a behavior change to investigate, not an acceptable consequence of skipping the old matcher.
- [x] Verify two owners sharing a strand, removal of either owner, a shared strand that remains ordinary bulk, cross-region closure emission, both-end attachment lineage, saturated solid cuts, reslice generation changes, and pointer/address reuse.
- [x] Run `[EmissionLedger]`, ownership/interfaces, ribs/modifiers, and exact-target differential tests. Measure classification calls eliminated and total collection time. Do not create the ledger by running the old geometric classifier after every regeneration; that merely relocates the cost.

The first admitted scope excludes solid/closure layers and ambiguous classifications; their existing complete reference paths pass the full regression suite. Extending tags into those scopes remains unchecked above and is not required for the next transaction increment. Checkpoint `51f78851a` passes 354 default cases, all three hidden geometry qualifications and the full application/legacy/tooling checks. Exact broad-reference attribution and integer endpoint bounds produce 878 emitter hits and 44,501 reused construction paths in the small diagnostic, but total preparation remains approximately 13 seconds. A separate official-app development pair measures 13,300/278 ms (47.842×), so the release gate remains unmet.

### 3. Publish exact emission deltas and update complete path facts

**Files:** `BridgeAwareCubicEmissionLedger.hpp/.cpp`, `BridgeAwareCubicSpanProofCache.hpp`, `BridgeAwareCubicContact.hpp/.cpp`, `BridgeAwareCubicSupport.cpp`, `test_bridge_aware_cubic_emission_snapshot.cpp` and new canonical-transition tests.

Implement this increment in two independently verified parts. First separate final survival capture from classification and make trial/rollback ownership explicit; then use complete canonical material changes to invalidate whole-path facts. This order avoids paying classification for model-rejected trials and provides an authoritative transition boundary for later reuse.

- [x] Split immutable `EmissionSnapshot` (surviving paths, source tags, active attachment table and ordered transitions) from immutable `EmissionProjection` (construction flags and independent owners). Seal survival only at the current end of `apply_layer_paths`. Generate exact broad references and classify only when construction collection requests a projection after the model gate.
- [x] Keep projection publication in a coarsening-local transaction context. Parallel final generation must not mutate a shared const snapshot. An external collection audit may build a temporary projection; final/export reference collection stays unchanged.
- [x] Retain base snapshot/projection handles across a trial within the same emitter quota. On rejection, regenerate actual fills with optional capture disabled, then reattach retained handles only after exact live path, flow, representation, traversal-order, Plan-binding, Z and reference-input comparison. Failed equality leaves the emitter unavailable. Account base, candidate, reference, projection and simultaneous scratch together; shared handles do not create another allowance.
- [x] Test that model-rejected trials perform no reference generation or classification; restored identical paths at new addresses reuse snapshots; changed flow/order/Plan/lineage, quota exhaustion and cancellation preserve complete fallback without partial publication.
- [ ] Add transaction tests for addition, deletion, changed flow, restored bulk after solid removal, restored attachment endpoints, and owner-only changes without geometric movement.
- [ ] Derive `EmissionDelta` from exact final records for the current phase and elevations. Initially retain current whole-layer regeneration; compare its output and update only affected ledger records/facts.
- [ ] Capture complete lower deposition, including perimeters, ordinary infill and fallback emitters, before using an emission delta as physical evidence. Sparse construction provenance alone cannot identify the complete lower material. Distinguish physical changes from owner-only changes, preserve duplicate emissions, and use exact comparison after any hash lookup.
- [x] Prepare each changed lower plane once and compare complete canonical Contact axes, including zero-width anchors and preparation status. A tiny primitive edit can change an entire merged axis. Physical-cell identity additionally requires the complete ordered positive-area section inputs and unchanged complete topology domain.
- [ ] Use conservative spatial indexes to discover affected complete upper paths. Include the whole changed canonical lower axis and all possible contact extents. Ambiguity expands the scope; it does not remove a path from checking.
- [x] Register absence regions or rerun exact signature discovery so a newly added lower strand invalidates a fact even when no old source ID changed.
- [x] Reuse unaffected facts by exact immutable target/version handles after validated transitions, bypassing repeated signature discovery. Preserve the full target view and topology domain while checking a dirty subset. A nonempty physical-material delta invalidates direct-roof cell proof even when local span contact is unchanged; defer that shortcut to certified separators in increment 4.
- [ ] Recheck surviving changed construction paths with current policy and provenance. Remove obligations only when the exact source and its final emitted path no longer require them.
- [ ] Run `[EmissionDelta]` and a shadow comparison against complete recollection/revalidation after every mutation. Include a tiny primitive edit that changes a long canonical merged axis and a contact change that joins several previously short air intervals.

The first part (3A) is implemented and verified: raw survival snapshots, lazy derived-only projection, bounded base/candidate retention, capture-disabled physical rollback with exact restoration, and an owned exact source-frame delta. Fifteen pure snapshot/projection/delta cases include duplicate owner swaps, hash collisions resolved by exact metadata, reference lifetime, quotas and cancellation. Opt-in per-trial audits compare both initial and repeated optimized collections against every ordered reference target field, while the exhaustive trial fixture preserves final Plans and extrusion trees. Source-frame deltas are not yet consumed as complete physical/phase transitions, and solid-removal correspondence remains outside the current capture scope; those lines stay unchecked above.

The second part (3B) is implemented and verified. Contact-owned canonical lower snapshots and whole-section added/removed deltas include every deposition role, valid empty states, zero-width anchors, exact phase/plane identity and preparation status. Complete-path span facts advance only after their original before-snapshot association and conservative checks against every added/removed whole canonical section. This detects newly entering support as well as removed support. The complete upper signature, selection and gap are freshly checked. Dirty or unavailable transitions retain exact raw signature discovery with the same remaining work allowance. The current delta scan is bounded and linear; a spatial invalidation index remains unchecked above.

A coarsening-local context observes actual current material after accepted and rejected trials. Each exact plane/phase slot keeps its last observed canonical snapshot; at most one previous snapshot/delta is active. Retiring a snapshot atomically demotes cached associations while preserving raw facts. The span cache admits bounded canonical snapshots, shares each raw signature across at most two associations and stages mutations before publication. All canonical/context storage shares the existing 24 MiB metadata allowance with emission metadata (2 MiB contact, 22 MiB emission). Source-frame emission deltas are still not consumed as complete physical/phase authority, and solid-removal lineage extension remains out of scope. Complete direct-roof cell checks and the exhaustive final oracle remain unchanged.

Thirty new Contact/Span transition cases cover whole-axis changes, additions into an empty query, exact phase components, current prepared-source association, source retirement, atomic cancellation/admission, repeated transitions, raw fallback and audit failure. The exhaustive-output qualification preserves final Plans and extrusion trees and audits every optimized transition hit against fresh complete signature discovery. Focused verification passes 116 cases and 42,497 assertions. The complete application build, 17 legacy CTest targets, 44 packaging tests, 70 benchmark-tool tests and 404 feature/qualification executions (378,755 assertions) pass. A separate fixture writer passes 244 assertions.

The small diagnostic restores 2,574 snapshots with zero mismatches and skips projection on all 112 model-rejected trials. Its 878 projections are each requested once in normal slicing; positive projection cache hits come from deliberate audit rereads. Contact reuse captures 885 snapshots, compares 800 transitions and advances 13,669 span facts with no admission fallbacks; peak canonical/context retention is 627,400 bytes across 85 slots. Total preparation remains 10.575 seconds. The isolated official-app development pair measures 10,690/290 ms (36.862×), leaving the 2× release gate unmet. Material and final Plans remain unchanged. Profile the remaining work before selecting the next implementation; positive reuse counters alone do not establish a speed improvement.

A raw-section separator proposal remains unimplemented. Strict cycle coverage in the continuous union of input sections does not yet prove a surviving material collar after the oracle's rounded closed-polygon union/difference. A closed-operation inclusion proof or a witness checked against the actual current oracle union is required before such a shortcut may accept a trial. The earlier open-path rounding bound is not sufficient.

### 3C. Skip selected geometry after a complete-path positive span proof

The post-3B main-thread sample attributes 754 of 3,445 samples to unsupported-selection clipping/offset/union/intersection, versus 166 to complete physical preparation. This is a partial diagnostic sample, not a complete wall-time forecast. It justified a smaller span-only increment before separator work.

- [x] Add an explicit whole-path mode to the existing contact verifier, retaining the same input guards, exact support intervals, arclength, anchoring, uncertainty, gap reserve and closed seam.
- [x] Restrict the positive shortcut to eligible coarsening construction/follow-up span checks. Validate bounded current policy; a complete passing check of every actual run implies acceptance of its selected subset. Any other result retains the original selected check and consumed work. Physical-cell, repair, exhaustive, final and export checks remain unchanged.
- [x] Audit every positive shortcut against fresh original selected validation, treating an incomplete reference as audit failure. Preserve exact final Plans and emitted output in the existing parity qualification.
- [x] Add regression tests for anchored bridges, floating/one-sided runs, joints, repeats, reversal, seams, mask holes/boundaries, uncertainty, invalid input and resource bounds. Demonstrate whole-path failure while a selected subset passes, and verify policy admission leaves cached facts unchanged.
- [x] Run the complete build, relevant focused tests, full feature/qualification suite, legacy/tooling tests and isolated official development pair.

RED verification fails all 17 new verifier cases and the missing positive-probe parity assertion. GREEN focused verification passes 147 cases / 47,991 assertions. Full verification passes 417 unique defaults / 419 default executions plus three hidden qualifications: 422 executions / 379,504 assertions. The complete application build, 17 legacy targets, 44 packaging tests, 70 benchmark-tool tests and separate 244-assertion fixture writer pass. Runtime/test source is tied to test executable SHA-256 `35c28c52ec9dbfaaffe9e0238cea3451c4a91d852dd48a5d04e405690baefab9`.

The small diagnostic records 51,399 complete-path probes and 51,399 positives, retaining 159 trials, 47 acceptances and 112 rejections. Preparation is 10.011 seconds, with model/construction checks at 4.067/1.873 seconds. The isolated official development pair measures 10,251/292 ms (35.106×). Material and final Plans remain unchanged. The 2× gate remains unmet; this is an incremental improvement, not release qualification.

The shortcut proves physical span acceptance using valid bounded input policy. It does not assert that the skipped selected-polygon operation would have produced valid within-cap output: that geometry is not constructed. Every performed operation retains its existing bounds, and an encountered incomplete result cannot publish a fact.


### 3D. Reuse exact direct-roof computational artifacts

The next measured target is repeated contact-mask and unsupported-selection construction for fixed model paths. Reuse only the exact computational output, with fresh complete physical-cell checks; this is not a separator or cell-proof shortcut. Start with fresh raw query-signature matching on every lookup, without adding another epoch-retirement participant.

- [x] Add RED tests for an immutable `ContactsOnly` contact plus nonempty selected geometry. Changes to actual canonical candidates, phase, upper geometry/flow, demand or selection width prevent reuse; distant changes may preserve the artifact but must still expose any changed complete cell.
- [x] Extend Contact's exact signature matching with a narrow retained-output admission guard. Count actual retained segment, line-event and mask-point units inside the same current prepared/upper/candidate/cache allowance before matching; ordinary matching reserves zero extra units. Validate status, plane, tolerance, original positive segment correspondence and the absence of physical-footprint fields. The caller preserves actual artifact/signature provenance; this helper does not reconstruct the retained output.
- [x] Add a bounded immutable artifact cache with exact owned policy, raw signature and output. Keep at most two versions per exact key for rollback. Charge nested vector capacities, intervals, strings, signature storage, metadata and staging. Reallocate the existing 24 MiB allowance to 18 MiB emission, 2 MiB contact epochs and 4 MiB artifacts.
- [x] Enable only non-probe, non-reference coarsening direct-roof checks with the validated frozen model view. A hit reuses contact and nonempty selection but always reaches the current physical-sections/topology-domain/cell-verification branch. Missing identity, unsupported inputs or quota admission retain the unchanged computation and consumed work. Repair, final/export and exhaustive behavior remain unchanged.
- [x] On every diagnostic hit, recompute uncached contact and the original selected geometry with the same remaining work budget. Require valid complete reference results and exact ordered equality of every status, plane, uncertainty, segment, polygon/hole point and rational interval. Area equivalence is insufficient.
- [x] Test cancellation, near-limit active-query admission, bounded staging, rollback, exact policy comparisons and optional publication failure. Then require full emitted-output parity, existing feature/qualification and legacy suites, and an isolated official timing comparison before reporting a performance result.

No cached record contains physical material, topology-domain geometry, a complete cell, a cell verdict or a span verdict. Publication requires actual completed computational output, but a current cell failure does not invalidate that output. This increment cannot omit a physical-cell check or turn a logical/demand boundary into material. The 2× release gate remains unchanged.


RED checkpoint: all 20 new pure cases failed at the unavailable helper/cache operations (124 prerequisite assertions passed), and the hidden output-parity case stopped at the expected zero-artifact-hit assertion after 13,529 passing assertions. Eight runtime/test files were frozen for that build; all root test processes completed before GREEN edits began. Independent review covered the completed Contact helper, root integration and cache. A third-version eviction iterator defect found in review was corrected before GREEN verification.

GREEN verification passes all 21 new cases / 2,767 assertions and the expanded 177-case contact/validation group / 50,961 assertions. Every artifact hit is audited against uncached ContactsOnly and original selection, then receives current complete-cell validation. The full application build, 17 legacy targets, 44 packaging tests, 70 benchmark-tool tests and 244-assertion fixture writer pass. Exact manifest verification covers 438 unique defaults / 440 default executions plus three hidden qualifications: 443 executions / 382,296 assertions. The test executable SHA-256 is `e334e7c6f3a6c09e7b62179e1f78f481622d1992cf5c30104f329ea958c7ecec`; app SHA-256 is `725ecb4599fa723fb5ded7dcf3b0683e31718feb01b4757a8c9af1beafb10844`.

**Performance rejection:** the isolated official development pair measures 10,726/288 ms (37.243×), with whole-process times 11.509/1.846 seconds. A separate diagnostic reports 10.486 seconds preparation versus 3C's 10.011 seconds. Only 18 of 2,605 lookups hit (0.69%); 142 publication attempts produce 81 publications, 79 same-policy version evictions and two remaining entries. Lookup/publication cost 0.150 seconds and 72.8 million charged work units. Peak cache/context storage is 1,029,965 bytes, so a larger cap is not justified by these observations. The 61 declined publications lack reason-specific counters; their exact causes are not established.

The quota carve-out also displaces useful emission records: hits fall from 878 to 784, reference fallbacks rise from 154 to 248, and reused construction paths fall from 44,501 to 39,118. Construction collection increases from 1.324 to 1.538 seconds. Trials remain 159 / 47 accepted / 112 rejected; span and epoch counters remain unchanged. Correctness passes do not justify keeping this measured regression enabled.

The complete experiment is retained on local branch `codex/bridge-aware-cubic-contact-artifact-experiment`. Its production wiring and quota carve-out are not retained on the feature branch; the feature continues from the complete-path span checkpoint, with the 22 MiB emission / 2 MiB epoch split. The 2× gate remains unmet, and no merge, installation or release is authorized by this experiment's results.

### 3E. Reduce repeated target compilation — model identity implemented

The next bounded candidate is exact reuse of source-frame target compilation, before physical verification. Model collection takes 0.946 seconds and construction demand assembly 0.507 seconds in the 3C diagnostic; their combined 1.452-second ceiling is not a forecast and cannot alone meet the 2× gate.

A model-collector identity must include the complete eligible upper path stream and entity/free-ended structure, layer/region order and Z, participation/density/gap policy, lower sparse surfaces, all consulted backward bridge ancestry and its follow-up flags, and inherited material slices. With `unsupported_only=false`, lower deposited support is not part of target geometry; it remains an input to subsequent physical checks. Reuse may borrow frozen targets only after exact equality. A mismatch must run the original collector and target comparison rather than reject the trial: an unrelated path change may leave required targets unchanged. Construction demand reuse must additionally retain exact optional attachment selection and eligible-domain context; producers and endpoint lineage remain freshly assembled.

- [x] Add falsifiers with unchanged immediate upper/lower inputs but changed upstream bridge ancestry, policy or material, plus path metadata/free-ended changes, new eligibility and formerly unselected paths. Include empty target layers, harmless internal/ironing regeneration, derived/fitted fallback, cancellation, consumed work and temporary membership growth.
- [x] Capture explicit scalar/point values into a bounded owned stream with a range for every upper layer. Rewalk all mutable context and requested-layer trees on each lookup; preserve the collector's actual free-set membership. Unknown/derived representations decline reuse. A hit borrows only frozen Targets; all physical/probe/final checks remain unchanged.
- [x] Share 512 KiB, 200,000 vertices and 20,000 items between actual prepared-model capacities, the identity and temporary membership. Pre-admit fixed and variable allocations, distinguish resident/capture-peak storage and retain consumed work on original-collector fallback. The 22 MiB emission / 2 MiB epoch allocation is unchanged.
- [x] Audit every diagnostic hit with original collection, complete ordered Target equality and every copied path metadata field. Charge the full comparison and cancellation checks. Verify exact final output and all existing feature/legacy/tooling suites.

RED: the unavailable helper fails all 12 initial cases (24 prerequisite assertions pass, 13 fail). GREEN: 14 focused cases / 226 assertions pass, including the review additions. Exact exhaustive output parity passes 13,627 assertions. Full application build, 17 legacy targets, 44 packaging tests, 70 benchmark-tool tests and the 244-assertion fixture writer pass. The complete manifest covers 431 unique defaults / 433 default executions plus three hidden qualifications: 436 executions / 379,742 assertions. Test SHA-256 is `ae6f666052ac3e51e3c9b376334c8f991b7d680ca69f4eff7333544a2b4f5b7a`; application SHA-256 is `32cc340a98899aa89a5fc7013f640bfd04368dcf520e578399897ac3cfa6bfa0`.

The isolated small diagnostic admits one 46,082-byte identity, reuses it on all 159 trials, and records no input misses or unavailable lookups. Combined resident storage is 181,186 bytes; capture peak is 182,074 bytes. The original model collector runs five times instead of 164: 0.394 seconds versus 3C's 0.946 seconds. Capture/matching take 0.000227/0.011053 seconds. Preparation is 9.521 seconds versus the earlier 10.011 seconds. The same 159 trials / 47 acceptances / 112 rejections, 878 emission hits / 154 fallbacks and 13,671 span hits remain. The comparison to 3C is diagnostic, not a controlled same-session baseline.

**Release gate remains failed:** one warm-up plus three measured official small-fixture pairs produce feature times 9,830/9,867/9,925 ms and official times 264/265/256 ms. Median ratio is 37.375×; worst pair is 38.770×. Both executables finish successfully and source/executable hashes remain unchanged; the harness correctly rejects qualification. Construction target assembly reuse is still unimplemented. No merge, installed-app replacement or release handoff follows this result.

### 3F. Normalize exact contact-mask key direction — implemented

The existing mask generator includes both `corner + transverse` and `corner - transverse`, then rounds and sorts their convex hull. Negating the already-computed finite transverse vector swaps these inputs, including signed zeros that round to the same integer. Canonicalizing only its sign in the cache key can therefore reuse the identical mask for opposite upper traversal. Keep mask generation, raw reserve identity, line witnesses, output copies and allocation admission unchanged. Do not approximate angles or replace Clipper's offset algorithm.

- [x] Require warmed opposite-direction cache reuse and complete output equality against an uncached query in the current direction. Cover zero/positive lower radius, signed-zero components, diagonal/near-parallel inputs, half-grid rounding, coordinate bounds and memory-pressure ownership transfer.
- [x] Apply key-only sign normalization after finite guards. Keep exact reserve changes as misses and every line-contact witness freshly computed.
- [x] Run focused Contact/admission tests, complete output parity, the regression suite and isolated timing.

Before the change, the four new cases failed only at the reuse assertions (160 failures, 2,275 prerequisites passed). After the change, the 75-case focused group passes 43,184 assertions. Exhaustive output parity passes 13,627 assertions. The full application build, 17 legacy targets, 44 packaging tests, 70 benchmark-tool tests and the 244-assertion fixture writer pass. The complete geometry suite passes 440 executions and 382,177 assertions.

With one warm-up and three measured small-cube pairs, the feature median is 9,780 ms (range 9,754–9,865 ms) versus 270 ms for official Cubic: 36.222× median and 36.629× worst pair. The preceding feature median was 9,867 ms. These overlapping diagnostic samples do not establish a general speedup. The isolated profile reports 9.459 seconds preparation and 6.599 seconds coarsening, with unchanged decisions. The 2× release condition remains unmet.

### 3G. Test known collapse failures before generating unrelated layers — implemented

The small profile repeats 56 rejected candidates. Its remembered model path reproduces the failure in all 56 cases, but the current loop regenerates all changed layers first. Investigate a rejection-only probe of the actual trial deposition immediately below that path. A candidate that changes required model targets is already inadmissible; a candidate preserving them cannot pass a complete, current physical counterexample for one of those targets. The original whole-layer path remains necessary for every pass, unavailable proof or unsupported generation dependency.

Partial generation is limited to a single correctly bound BAC region and pure internal sparse predecessor, with narrow-solid detection, mutable surface behavior, free perimeter paths and unsupported retained trees excluded. The ordinary generator runs on the real layer with speculative emission capture disabled. A scoped swap restores the original owned tree and handles on success, fallback, cancellation or allocation failure. Only a current, valid, completed physical counterexample rejects; first visits and all inconclusive probes retain the full route and consumed work. Diagnostic challenges match one exact failed run or complete cell against the full candidate, then require ordinary rejection. A failed diagnostic goes through ordinary rollback before reporting; rollback itself retains normal cancellation behavior.

The new path was exercised in three modes: exhaustive checks, per-hit auditing, and actual production skips. All final layers, plans and trial decisions match. The complete geometry suite passes 442 executions / 389,095 assertions (437 unique defaults; three hidden qualifications). The three-mode comparison passes 20,523 assertions. All 17 legacy targets, 44 packaging tests, 70 benchmark-tool tests and the 244-assertion fixture writer pass. Test SHA-256: `7595ae70a63f8544e38a1ce3c22e0168a37af346abf52ecba481d1cc8ca0537d`. Application SHA-256: `0ab904d68a4bc224b55288832aff0f5732711e0be86afd85d2840ab2bbbcae55`.

The final small diagnostic has 34 attempts and 34 fresh rejections; 22 remaining repeated hints take the original route. It skips 1,406 generation and 1,440 rollback layer visits. Preparation takes 9.024 seconds; coarsening takes 6.192 seconds. The small-cube official comparison (one warm-up, three measured pairs) is 9,483/255 ms at the median, 37.188×; the worst pair is 37.335×. The large fixture's first completed pair is 151,256/280 ms, 540.2×. Its 149,072 ms preparation and 1,867 ms later toolpath stage each exceed the entire 560 ms allowance. The subsequent rib benchmark rejected an unnormalized startup script in that fixture; it must pass the existing strict audit before reporting a paired result.

Next measurements should distinguish actual empty roof selections and complete paths proved by line anchors, by repair, first sweep, later sweeps and final verification. Investigate the late proof miss by reporting the first differing typed input; do not assume filament-map values have identical output semantics. Retain just-verified target footprints for reservations where no geometry changes between proof and use. For large parts, consider a canonical per-plane refinement program: stable producer endpoints, owner alternatives and stitched continuous extrusion runs can make an edit proportional to changed faces and runs. Qualify incremental and fresh canonical output exactly, including all output phases; any change to legacy refinement coordinates still requires real physical contact/cell qualification. An atlas is not itself a support or irreducibility certificate.

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
