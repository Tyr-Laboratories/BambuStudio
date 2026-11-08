# Parallel whole-project slicing

The user requests that Preview's All plates action slice independent plates in
parallel, alongside the Organic foundation performance fix in the next installed
and published Tyr release. The feature must produce the same per-plate G-code and
preview as slicing those plates individually. Single-plate slicing and projects
containing only imported G-code retain their existing behavior.

## Design decisions

Reuse `BackgroundSlicingProcess` for each admitted plate. Each worker receives the
plate's existing independent `Print`, `GCodeProcessorResult`, output path, and
configuration. Prepare/apply/validate on the UI thread before starting workers;
do not start simultaneous calls against one Print. Reuse normal processing,
filament mapping, G-code export, thumbnails, post-processing, and errors.

A small UI-owned admission queue tracks plate ids, a batch generation, queued /
running / finished states, and aggregate progress. Admit at most four plates,
further limited by CPU and RAM: one slot per two hardware threads and one slot
per 3 GiB after reserving 2 GiB. Clamp to at least one and to pending plate count.
These automatic bounds are an implementation choice, not a new process preset.
The existing TBB pool shares available compute across admitted plates.

Every worker status/completion event carries its plate identity and batch
generation. Ignore events from canceled or replaced batches. Commit each result
only to its own plate. Empty, invalid, and already valid cached plates do not
consume worker slots. An error identifies its plate and does not suppress the
other valid jobs. Whole-project publishing/export cannot treat a batch with
failed plates as successful.

Keep OpenGL thumbnail rendering on the UI thread using the existing worker UI
task dispatch. All workers must stop before model/plate/config mutation or
project destruction. Cancellation cancels queued admission and all running
workers, retains completed valid plates, and leaves interrupted plates invalid.
Remove the legacy timeout path that detaches a thread and immediately exposes
its Print for mutation, including for the preceding single-plate worker. The original selected plate and
the All plates preview selection remain coherent after completion/cancellation.

## Shared state to correct

`PrintStateBase::g_last_timestamp` and G-code result ids are currently plain
process-wide counters; make them atomic. G-code flush restoration stores its
previous extrusion role in a function static; move it to the processor instance.
The Bambu-printer flag used by G-code processing/time formatting must also belong
to the processor/context, not a mutable process-wide flag. External post-process
scripts receive a plate-local child environment, leaving the application
environment unchanged while slicing and G-code generation overlap. No new shell
commands, printer actions, or networking are added to slicing.

Bambu shared-nozzle extrusion and retraction axes belong to one writer. Its
filaments share physical-nozzle state with each other, while other plates and
temporary flow-calculation extruders remain independent. Cover both absolute
and relative extrusion modes and use the Bambu shared-nozzle setting in actual
serial/concurrent export comparisons.

Snapshot printer-vendor and filament-mapping mode on the UI thread, allocate all
temporary plate paths before launch, and commit mapping changes on completion.
Lightning child traversal must use geometry-local deterministic choices instead
of process-global rand(), preserving varied traversal without scheduling effects.

## Acceptance

Tests must establish overlapping independent plate execution, bounded admission,
out-of-order completion, exact plate/generation routing, cancellation with queued
and active work, cached/empty/invalid skips, and no stale success publication.
Interleaved processor tests must preserve each plate's extrusion roles and
printer-specific time handling. Actual serial and parallel native slicing must
produce equivalent G-code/toolpaths for different fixtures, including Organic
supports. Exercise the Preview action in the built app, including cancellation
and a repeat run using caches. Measure whole-project wall time as well as each
plate; parallel completion alone is not a speedup claim.

Retain the measured Organic cache optimization and its unchanged output checks.
After all checks, update the Mac install, merge the complete feature into the
repository's default `master` branch, and publish the next unused Tyr release.
