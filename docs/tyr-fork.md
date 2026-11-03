# Tyr Laboratories Bambu Studio fork

This fork adds surface painting for wall counts to Bambu Studio. It uses the native Classic and Arachne wall generators so a painted hole, edge, or other surface can have a different wall count from the rest of the model.

The source is based on upstream `master` commit `79df7ae309be398ed30c9f397c13c9911c2edc57`. The wall-painting implementation is commit `eeae2b3e3`; the upstream version in [version.inc](../version.inc) is `02.08.04.57`. This is a Tyr Laboratories fork, not an official Bambu Lab release.

## Paint a wall count

1. Open a model and select one object in Prepare view.
2. In the global process settings, open **Strength > Walls**. Set **Wall loops** for ordinary surfaces and **Painted wall loops** for painted surfaces. For example, normal `1` and painted `3` requests one wall elsewhere and three total walls at the painted surface.
3. Open **Wall Painting** from the object toolbar, or press **W** with the canvas focused. If the panel is clipped on macOS, **Window > Zoom** can expose it.
4. Choose Circle, Sphere, Triangle, Fill, or Gap Fill. **Fill** is also available with **F** while the painter is open. Use the section view to reach interior surfaces such as holes.
5. Paint the intended surface, then slice and inspect the affected layers in Preview. Save the project as a **3MF** to retain the painting and its settings.

| Control | Effect |
| --- | --- |
| Left mouse button | Mark the surface to use the painted wall count. |
| Right mouse button | Erase the surface annotation, returning it to ordinary unpainted behavior. |
| Shift + left mouse button | Erase the surface annotation, returning it to ordinary unpainted behavior. |
| Fill all round internal holes | Find circular bore walls on the selected object and paint them in one undoable action. |
| Erase all painting | Clear this object's wall-painting annotations. |
| Undo | Undo painting edits. |
| Section view | Clip the view to access interior surfaces without cutting the model. |

Right-click and Shift + left-click both erase wall painting and restore the applicable normal wall count. Right-drag erases along the brush stroke; Fill uses the same erase behavior with the right mouse button.

**Fill all round internal holes** searches the selected object's model parts, including surfaces hidden by Section view. It adds the same surface mask as manual painting and uses the existing **Painted wall loops** setting. Existing painting elsewhere is preserved; you can erase individual results or undo the whole action. Canceling detection leaves painting unchanged. Save the project as 3MF to retain the result.

The detector looks for inward-facing cylindrical walls that surround a complete circular bore, in any orientation and at the selected instance's scale. It includes the cylindrical sides of through and blind holes. Exterior cylinders, coarse polygonal or elliptical openings, open channels, tapered surfaces, and flat blind-hole bottoms are not automatic targets. Stepped holes are considered as separate cylindrical sections. Very coarse or damaged meshes, interrupted bores, and very short bores blended smoothly into surrounding curved surfaces may need manual painting. Review the highlighted surfaces and sliced Preview before printing.

Older projects may contain red normal-wall annotations from earlier fork versions. The painter now loads those as unpainted surfaces, preserving their normal wall count and the remaining painted mask.

## Counts and inheritance

**Painted wall loops** (`painted_wall_loops`) is the **total** requested wall count on painted surfaces, not an extra number added to **Wall loops** (`wall_loops`). Both settings accept integers from **0 to 1000** and have a definition default of **2**; a selected preset can supply another value. A painted count can be higher or lower than the normal count.

Set wall counts through the standard settings panels. The setting follows the existing process, object, and part hierarchy:

- **Global process:** set the inherited painted count in **Strength > Walls**.
- **Object:** select the object and set **Painted wall loops** in its normal object settings to override the global value. Remove the override there to inherit the global value again.
- **Part:** select the part and set **Painted wall loops** in its normal part settings to override the object's value. Remove the override there to inherit the object/global value again.

The painting panel contains surface-painting tools only. Numeric wall counts and their inheritance controls remain in the standard global, object, and part settings.

Painting records a surface mask separately from the numeric count. Changing the applicable count updates surfaces already painted when you slice again; repainting is unnecessary. A part override takes precedence over the object or global value, so changing the global value will not change a part with its own override.

## Slicing behavior and limits

Both Classic and Arachne apply the count at painted boundary segments, including holes and partial surface patches. Added walls end within the painted extent; Arachne uses its native transition behavior. Infill follows the walls that are actually generated.

The count is still subject to the slicer's other wall rules and available space. For example, first-layer or top-surface single-wall settings and alternate extra walls can change the emitted count, and narrow regions may not fit every requested wall. A zero count is valid, but it does not disable all other wall or shell rules. Inspect Preview for the actual result.

Wall Painting requires one selected object and is unavailable in spiral vase mode. Painting is stored on the model's surfaces, so inspect it after operations that change the mesh. Supported copy, split, merge, transform, reload, and replacement paths have regression coverage; that coverage is not a guarantee for every possible mesh edit.

## Projects, profiles, and accounts

This fork saves the wall mask and numeric settings in Bambu 3MF projects. Application save/reopen checks confirmed that edited painting remained present and affected the sliced preview. Exported G-code contains the resulting toolpaths; use the 3MF project to retain editable painting.

Official Bambu Studio versions that do not implement this extension ignore the wall-painting data when slicing. Saving a project through such a version may discard the extension. Keep a fork-saved 3MF copy and use this fork to preserve the intended wall counts.

The normal application build retains the Bambu Studio application identity and its usual profile/account data location. Existing printer, filament, and process profiles are intended to remain available without a separate transfer. It shares that storage with the standard application, so keep a backup of your profiles before changing versions. The original feature-development GUI checks used an isolated data directory; see each release's notes for subsequent installation and profile-reuse checks.

This fork also suppresses the **Customized Preset** dialog when its only cause is that a project's printer or filament preset name is absent from the installed library. The warning for detected modified G-code remains in place. A same-project load without the missing-preset dialog was verified.

## Sending prints

Use the same Tyr **Print plate** dialog to choose the printer, print options, and submit the sliced plate. Account login, printer discovery, and status remain in the native macOS application. Cloud submissions run through a private Linux helper in a local Docker container created for each job. The helper sends the already-sliced plate, retaining the generated wall toolpaths.

**Docker Desktop must be installed and running for cloud submissions and Device-tab cloud video.** On first use, the application imports the helper image from the runtime bundled with the release; later jobs reuse that image. Release users do not need Homebrew, a source checkout, or a separate slicer. The helper uses the pinned Bambu Linux networking SDK **02.05.02.58** selected for Tyr Print Farm; it runs locally and does not submit through a Print Farm service.

The helper receives a private copy of the native application's opaque saved account session. It verifies that the copied session identifies the same account, and uses isolated temporary data for each job. Its verified networking libraries and certificates are copied into that same private job directory and mounted read-only, so Docker does not need access to the application's installation folder. The original session and profiles stay in place; no profile migration or second application login is required. Neither official Bambu Studio nor Bambu Connect is bundled or opened to send a job.

The Device tab’s ordinary live-camera session requests also use the private helper, including session refreshes. The existing macOS player receives the authorized cloud connection and continues to decode and display the video in the same camera pane. Camera requests prepare the selected printer but never upload a plate or send printer-control commands. Signing out, changing accounts, or closing Studio cancels pending helper requests. Camera failures do not log out the independent native status connection.

Before uploading, that same helper connects to the cloud, verifies the selected printer is present and online, subscribes to its status, and initializes its device certificate. It requests current status and waits for a valid report from that exact printer. Empty heartbeats and messages from other printers do not satisfy this check. Preparation is cancellable; missing status, expired authentication, or a failed subscription stops the operation before submission. Status requests have bounded retries during preparation; the print submission is attempted once.

For send-delay diagnosis, the application keeps only the latest attempt in `cloud-send-latest-timing.json` in its normal data directory. It records elapsed times for preparation, SDK stage changes, completion waiting, and cleanup, plus numeric result codes. It excludes account, printer and job identifiers, file names, SDK messages, and print contents. The report is restricted to the current macOS user; a failure to write it does not affect sending. Read-only account and printer checks do not replace the latest real-send report.

The pinned helper supports ordinary cloud print submissions. Newer Studio queue jobs and timelapse recorded to internal printer storage are rejected before submission because the pinned SDK does not expose those options. Disable internal-storage timelapse for a supported ordinary print. LAN-only transfers retain their existing native networking path.

If submission is interrupted after it may have reached the cloud, the result can be **unknown**. The application does not automatically resubmit the job or try another transport after an uncertain helper submission. Check the printer's current job and cloud history before deciding whether to submit again; loss of the helper connection does not prove that the print was rejected or canceled.

Before another submission, the application checks for interrupted helper processes, stops only helpers it owns, and removes their copied session and print files. Active submissions are preserved. If an earlier job may have reached a printer, the next attempt stops with an instruction to check that printer. If cleanup cannot be confirmed, recovery data is retained and new submissions remain blocked until cleanup succeeds.

A user confirmed successful sending through the normal Print plate dialog in **tyr.9** on the development Mac, using the existing account and profiles. That test also exposed slower sending and a subsequent Device-page hang. The Device-page fix removes a synchronous JavaScript wait during macOS browser setup; the print completion check now retains the returned job ID so it can recognize a matching printer report. The installed tyr.10 Device page was subsequently verified with current printer progress, temperature and AMS data. Real-send timing still requires the next user-performed print. Successful sending does not establish physical print quality or completion. Keep the editable 3MF project and inspect the sliced Preview before submitting.

## Recorded validation

The following results cover the wall-painting implementation and its macOS application build:

- **Focused automated tests:** 39 cases and 19,524 assertions passed. Coverage includes both generators, partial and curved painted surfaces, holes, zero/high counts, narrow regions, transitions, infill clearance, transforms, variable layer heights, numeric modifiers, per-part settings, annotation lifecycle, and 3MF persistence. Automatic-hole cases cover circular bores, arbitrary orientation and mirroring, circular and elliptical scaling, dense tessellation, chamfers, oblique rims, incomplete channels, spherical cavities, cancellation, and preservation of manual painting through 3MF serialization. See [wall-painting tests](../tests/fff_print/test_wall_painting.cpp) and [round-hole tests](../tests/fff_print/test_round_hole_detection.cpp).
- **Automatic-hole project checks:** all 54 independently identified complete reference bores were detected across 18 aircraft parts containing 8.84 million triangles. The pass found 262 cylindrical sections, including surfaces missed by partial manual selections; independent radius and inward-normal checks found no false positives in those results. The largest 1.93-million-triangle part took about 4.5 seconds to scan, with about 265 MiB peak memory for the standalone validation process including the mesh. These samples do not establish exhaustive recognition of every possible hole.
- **Application build:** the macOS application compiled and linked successfully.
- **Print/send error dispatch:** 11 GUI regression cases and 111 assertions passed. They cover the previously reproduced worker-thread UI failure and exercise real worker threads and the wx event queue, error payload and ordering, late failures after 100% progress, job destruction, single finalization, and native-dialog resizing on the main thread. See [GUI job tests](../tests/gui_jobs/test_job_dispatch.cpp). The suite also checks string and numeric completion-job IDs, malformed responses, and macOS WebKit message registration without a blocking JavaScript wait. Real local-page [WebKit tests](../tests/gui_jobs/test_webview_mac.cpp) cover both newly loaded and already-loaded documents. These tests run locally without sending anything to a printer.
- **Linux cloud helper:** 59 native adapter/recovery/camera cases and 529 assertions passed, including actual subprocess communication, cancellation, interrupted jobs, active parallel jobs, cleanup failures, restricted Docker mounts, staged-file integrity, and target-printer preparation. Timing-report cases check ordered phases, bounded stage records, private file permissions, omitted sensitive data, nonfatal write failures, and preservation across read-only probes. Preparation tests cover delayed status, transient request rejection, empty heartbeats, unrelated or malformed reports, callback storms, authentication/subscription failures, and timeout without submission. The Linux host passed 13 offline protocol cases and a C++ framing test against a fake SDK; 16 packaging tests passed. The bundled image imported successfully into Docker Desktop, and the actual pinned Linux SDK loaded with networking disabled. A private copy of the native Studio session identified the same account in both SDKs. The integrated Mac adapter passed read-only account and selected-printer preparation checks using resources from the installed Applications bundle and the normal user-data location. The latter executed the same preparation used by a send, including status requests and the target-report gate, then exited before submission. Temporary session copies and helper processes were removed. No printer-control or print-submission calls were made during these checks. A user subsequently confirmed integrated dialog sending in tyr.9; the Device-page and performance follow-up remains under validation.
- **Cloud camera follow-up:** the native Mac session request returned `[1:403]` before playback. The same saved account obtained a TUTK camera session through the production helper API; the installed Mac video-source library opened it and received three encoded samples. This verifies authorization and native stream receipt, not sustained GUI playback. Camera-specific tests cover read-only preparation, unchanged send timing, refusal of local/arbitrary transports, cancellation during authorization, and private cleanup. Worker tests cover asynchronous delivery, serialized account transitions and identity snapshots, deferred UI results after cancellation, exception redaction and shutdown. A deterministic print-cancellation regression also verifies that completed jobs receive no late completion reply. Camera connection URLs are omitted from player logs in every build configuration. The first installed playback attempt exposed a stack overflow in file integrity checking: a 1 MiB local buffer exceeded the macOS camera worker’s stack. That buffer now uses heap memory. A child-process regression runs the real camera API on the production worker against a file of 2 MiB plus 17 bytes; the old code crashes and the corrected code passes. A subsequent real cloud/native-stream check through the production worker (536,576-byte stack) obtained a session and received three encoded video samples. Installed GUI playback remains a separate check. See [camera worker tests](../tests/gui_jobs/test_camera_requests.cpp).
- **Manual interaction:** user checks of the original implementation confirmed painting, Shift erase, undo, and section clipping. See release notes for subsequent interaction changes and their verification.
- **Project and output checks:** the GUI loaded, saved, and reopened painted 3MF projects. Preview showed one exterior wall and an additional wall at the painted center hole. An edited project exported G-code successfully and retained its painting after reopening.
- **Broader regression coverage:** a selected existing regression target ran 18 cases, with 12 passing and 6 failing. The failures involved older arrangement fixtures, a removed setting, an exact floating-point comparison, and stale hull expectations. The broader existing FFF test target also encountered removed-API compilation failures. No clean upstream-baseline comparison established that all of these failures predate this fork, so this is not a claim that the full upstream suite passes.

These results establish source, application, and slicing checks. Physical printing, mechanical strength, and production qualification have not been verified.

## Distribution and license

Download the application from the private [Tyr Laboratories releases page](https://github.com/Tyr-Laboratories/BambuStudio/releases). Organization repository access is required. The first release is `v2.8.4.57-tyr.1`.

- **Architecture:** Apple silicon (`arm64`); Intel Macs are not included.
- **Deployment target:** macOS 14.0 or newer. The macOS executable and bundled macOS libraries target 14.0; runtime verification was performed on macOS 26.6.2, not on every supported OS version.
- **Package:** a ZIP containing `BambuStudio.app`, with its resources, required zstd/Imath libraries, and the Linux cloud-helper runtime included. Homebrew and a source checkout are not required to run it. Docker Desktop must be installed and running to submit cloud print jobs; the bundled image is imported on first use.
- **Signing:** ad-hoc signed, without Apple Developer ID signing or notarization. The release includes a SHA-256 checksum. macOS may block first launch; follow [Apple's instructions for opening an app from an unidentified developer](https://support.apple.com/102445), using the per-app **Privacy & Security > Open Anyway** approval if appropriate. Organization device policies may prevent this.

To install, quit Bambu Studio, extract the ZIP, and replace `BambuStudio.app` in **Applications**. Keep a copy of the previous app and back up `~/Library/Application Support/BambuStudio` first. Launch the copy in Applications. Existing profile/account storage is shared; no profile export/import is required. Do not run the official app and this fork against that same data folder simultaneously. Installing a later official Bambu Studio update can replace this fork and remove its painting behavior.

No account data, printer credentials, or profiles from a developer's machine are included in the release. The cloud-helper runtime includes the pinned Linux networking libraries, certificates, image, and notices required by the helper. The optional native macOS Bambu networking plugin retains its normal download and user-data storage workflow. No official Bambu application is bundled.

Installing or updating the native macOS networking plugin requires quitting and reopening Bambu Studio before the new plugin is activated. The installation dialog will explain this. Save any open project before quitting. The fork avoids loading a replacement plugin into the same process: macOS can retain the previous plugin after it is unloaded, and the upgrade path produced a guarded-file-descriptor crash during discovery. An ordinary startup with an already-compatible plugin does not require an extra restart.

Print/send error notifications and their diagnostic details are delivered on the UI thread. This fixes the macOS main-thread assertion triggered when a failed print or file transfer caused the send dialog to resize. The normal dialog retains the transfer error for inspection; successful delivery still depends on resolving that underlying error.

The fork retains the upstream [GNU Affero General Public License, version 3](../LICENSE), credits, and notices. See the [upstream README content](../README.md#license) for attribution and the separate status of the optional networking plugin.

The helper vendors the PJarczak Linux plugin-host and bridge sources from FULU Foundation's OrcaSlicer-bambulab at revision `1b556d73f80e69f512881eab3331e6e9f70e5bf5`, with Tyr integration changes. Its [source provenance](../deps/tyr_cloud_helper/upstream.json) and [upstream AGPL-3.0 license](../deps/tyr_cloud_helper/LICENSE.upstream) are retained in the repository. Bambu's Linux networking libraries remain separate proprietary components; their download archive, SDK version, and checksums are recorded in the [runtime pin](../scripts/tyr_cloud_helper/pin.json).

## Maintaining the fork and release

The private [Tyr repository](https://github.com/Tyr-Laboratories/BambuStudio) uses **`master`**, matching upstream's default branch. It preserves upstream history and records the upstream base above. GitHub public-repository forks remain public, so this private copy is an independent repository rather than a member of GitHub's public fork network. Use `origin` for the Tyr repository and `upstream` for `https://github.com/bambulab/BambuStudio.git`.

Keep upstream merges on `master`, review conflicts in painting and perimeter code, and rerun the focused geometry/persistence tests before tagging a Tyr release. Tags use `v<upstream-version>-tyr.<revision>`. The upstream Homebrew and WinGet publishing jobs are restricted to the upstream repository so private Tyr releases are not submitted to public package indexes.

Build the application with the upstream [macOS build script](../BuildMac.sh), using Apple silicon and a deployment target of 14.0, with public-release configuration (`BBL_RELEASE_TO_PUBLIC=1`, `BBL_INTERNAL_TESTING=0`). Enable `SLIC3R_BUILD_TESTS` to build and run `wall_painting_tests`. An application build directory alone is not a distributable app: its resource link and machine-local library dependencies must be packaged.

With `SLIC3R_GUI` also enabled, build and run `gui_job_tests` to check print/send error dispatch. This target uses wxWidgets and requires a graphical session, but no account, networking plugin, or printer. On macOS, `cloud_send_helper_tests` checks the real native adapter against an isolated fake Docker process, including parameter transfer, cancellation, errors, and recovery. Run `python3 -m unittest discover -s tests/packaging -v` for runtime inventory and release packaging checks.

The release tooling performs that separate step:

```sh
# Requires Xcode command-line tools, CMake, Ninja, Python 3, curl, and running Docker Desktop.
bash scripts/build_tyr_macos_runtime.sh
python3 scripts/tyr_cloud_helper/build_runtime.py \
  --output build/tyr-cloud-helper/runtime

# Commit the release's source/documentation changes first. Use a fresh output directory.
python3 scripts/package_tyr_macos.py \
  --app build/arm64/src/BambuStudio.app \
  --output build/releases/2.8.4.57-tyr.1 \
  --release 2.8.4.57-tyr.1 \
  --cloud-helper-runtime build/tyr-cloud-helper/runtime \
  --zstd build/tyr-runtime/lib/libzstd.1.dylib \
  --imath build/tyr-runtime/lib/libImath-3_2.30.dylib \
  --zstd-license build/tyr-runtime/licenses/zstd/LICENSE \
  --imath-license build/tyr-runtime/licenses/Imath/LICENSE.md
```

The [macOS runtime build script](../scripts/build_tyr_macos_runtime.sh) pins the dependency source URLs and checksums. The [cloud-helper builder](../scripts/tyr_cloud_helper/build_runtime.py) builds the vendored host, downloads the pinned Linux SDK, and produces an image archive and checksum manifest in a new output directory. It records the Docker image identity and verifies the runtime inventory; use `--verify build/tyr-cloud-helper/runtime` to check an existing artifact without starting it. The helper image is Linux ARM64 and includes explicit x86-64 emulation for the Linux SDK and host.

The [packager](../scripts/package_tyr_macos.py) copies the resources, includes the two macOS runtime libraries and notices, replaces external library paths, checks architecture and deployment targets, seals the bundle with an ad-hoc signature, and writes the ZIP, checksum, and source-revision manifest. Supply `--cloud-helper-runtime` for a cloud-enabled release. The packager validates that runtime before copying it and checks it again after signing and ZIP extraction. Set `--app` to the actual build bundle path and replace the example release number with the next unused Tyr version. Verify the extracted archive, launch the packaged app, and repeat the painted-project save/reopen, slicing, and cloud-helper checks before uploading a release. Record software checks, authenticated cloud acceptance, and printer outcomes separately. Keep build logs and private installation backups outside tracked source.
