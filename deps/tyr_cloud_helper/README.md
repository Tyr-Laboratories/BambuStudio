# Tyr cloud helper

This standalone Linux executable hosts the pinned Bambu networking plugin behind
private process pipes. The macOS application keeps its normal print interface;
the helper has no graphical interface and does not start another slicer.

The vendored source comes from FULU Foundation's OrcaSlicer-bambulab revision
`1b556d73f80e69f512881eab3331e6e9f70e5bf5`. [upstream.json](upstream.json)
records the original file hashes. [LICENSE.upstream](LICENSE.upstream) retains
the upstream AGPL-3.0 license. Certificate files are copied from that same
revision. The proprietary plugin is a separate, checksum-pinned runtime input;
it is not implemented by this source and is not part of the offline tests.

Build on Linux with CMake, a C++17 compiler, OpenSSL, Boost headers and
`nlohmann_json`:

```sh
cmake -S deps/tyr_cloud_helper -B build/tyr-cloud-helper -DBUILD_TESTING=OFF
cmake --build build/tyr-cloud-helper --target tyr_bambu_cloud_host
```

The output is `tyr_bambu_cloud_host`, using the Linux libstdc++ C++11 ABI (`1`).
The runtime supplies `PJARCZAK_BAMBU_PLUGIN_DIR`, `PJARCZAK_BAMBU_NETWORK_SO`,
`PJARCZAK_BAMBU_SOURCE_SO` and `PJARCZAK_EXPECTED_BAMBU_NETWORK_VERSION`.
The required networking ABI is `02.05.02.58`. Paths for session data, sliced
files, and certificate files are supplied by the application/runtime. Each
uploader must have its own private writable session directory.

## RPC contract

Frames have a 16-byte little-endian header: magic `0x52424a50`, frame type,
request ID, and payload length, each an unsigned 32-bit integer. Requests have
type `1`, JSON replies type `2`, and binary continuations type `3`. Payloads are
bounded to 16 MiB. A request is `{ "method": "net.start_print", "payload": {} }`.
Replies retain their request ID. `ok` reports RPC dispatch success; `value`
contains the plugin result, where zero means success.

`bridge.handshake` advertises `protocol_version: 1`, the plugin ABI,
`supported_print_params`, and capabilities `tyr_cloud_helper_version: 1`,
`tyr_redaction_version: 1`, and `tyr_wait_reply_version: 1`. The client must check
these before submitting. The pinned ABI lacks `task_timelapse_use_internal`,
`svc_context`, `slicer_uid`, and `queue_plate_id` from newer Studio versions;
the client must not silently promise features that require those fields.

While `net.start_print` is pending, the client continues calling
`bridge.poll_events` on the same framed connection. Event payloads identify
`job_id` so that progress and replies cannot be applied to another upload.

| Event or request | Meaning |
| --- | --- |
| `job.update_status` | SDK `status`, `code`, and `msg` callback. |
| `job.wait` | SDK wait callback, including `status`, `job_info`, and a unique `request_id`. |
| `bridge.job_wait_reply` | Supply `job_id`, `request_id`, and the exact boolean returned by the application's wait callback. |
| `bridge.job_cancel` | Set `cancel: true` for the owned `job_id`; wakes a pending wait and reaches the SDK cancellation callback. |

A wait callback does not imply acceptance. It blocks for its matching reply,
up to 120 seconds; cancellation or timeout returns false. Stale or duplicate
replies do not release a later callback, and replies must contain an explicit
Boolean. A cancellation that arrives before registration is retained, preventing
the SDK call from starting. Each helper process admits only one cloud upload;
duplicate requests and reuse after completion are rejected. The client drains final events before
releasing job state, and confirms process/container termination before deleting
session data. A lost response does not authorize resubmitting the print.

## Local changes to upstream

- Wait callbacks now round-trip the application's response instead of always
  returning true while uncancelled. Cancellation wakes waits promptly.
- Upload admission preserves early cancellation, rejects duplicate identities,
  and prevents reusing an uploader process for a second submission.
- Login diagnostics and HTTP header echoes no longer expose account material.
  The farm's callback-lifetime and binary codec-header fixes are retained.
- Plugin loading is serialized and already-loaded modules are reused; concurrent
  RPC requests do not repeat integrity scans or race loader state.
- Framing is bounded. Malformed request fields return a generic error without
  exposing input values or killing the process.
- The capability handshake explicitly identifies these semantics and the exact
  available print parameter names.

## Offline verification

The test shared library implements only deterministic SDK substitutes. It has
no socket operations and receives no real account, project or printer data.
Tests run the actual helper process and exercise parameter marshaling, framed
RPC, progress, true/false wait responses, stale replies, cancellation, and
credential-diagnostic redaction. A separate test checks frame size bounds.

```sh
docker build -f deps/tyr_cloud_helper/tests/Dockerfile \
  -t tyr-cloud-helper-offline-tests deps/tyr_cloud_helper
docker run --rm --network none tyr-cloud-helper-offline-tests
```

These tests do not establish cloud acceptance or physical printing. Those are
separate checks of the packaged application and its selected runtime.
