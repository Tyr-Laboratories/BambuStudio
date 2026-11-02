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
| Right mouse button | Mark the surface to use the normal wall count. |
| Shift + left mouse button | Erase the surface annotation, returning it to ordinary unpainted behavior. |
| Erase all painting | Clear this object's wall-painting annotations. |
| Undo | Undo painting edits. |
| Section view | Clip the view to access interior surfaces without cutting the model. |

Normal and erased surfaces both use the applicable normal wall count. The distinction is that right-click leaves an explicit normal annotation, while erase removes the annotation.

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

## Recorded validation

The following results cover the wall-painting implementation and its macOS application build:

- **Focused automated tests:** 26 cases and 1,366 assertions passed. Coverage includes both generators, partial and curved painted surfaces, holes, zero/high counts, narrow regions, transitions, infill clearance, transforms, variable layer heights, numeric modifiers, per-part settings, annotation lifecycle, and 3MF persistence. See [the focused test source](../tests/fff_print/test_wall_painting.cpp).
- **Application build:** the macOS application compiled and linked successfully.
- **Manual interaction:** user checks confirmed painting, right-click normal, Shift erase, undo, and section clipping.
- **Project and output checks:** the GUI loaded, saved, and reopened painted 3MF projects. Preview showed one exterior wall and an additional wall at the painted center hole. An edited project exported G-code successfully and retained its painting after reopening.
- **Broader regression coverage:** a selected existing regression target ran 18 cases, with 12 passing and 6 failing. The failures involved older arrangement fixtures, a removed setting, an exact floating-point comparison, and stale hull expectations. The broader existing FFF test target also encountered removed-API compilation failures. No clean upstream-baseline comparison established that all of these failures predate this fork, so this is not a claim that the full upstream suite passes.

These results establish source, application, and slicing checks. Physical printing, mechanical strength, and production qualification have not been verified.

## Distribution and license

Download the application from the private [Tyr Laboratories releases page](https://github.com/Tyr-Laboratories/BambuStudio/releases). Organization repository access is required. The first release is `v2.8.4.57-tyr.1`.

- **Architecture:** Apple silicon (`arm64`); Intel Macs are not included.
- **Deployment target:** macOS 14.0 or newer. The executable and bundled libraries target 14.0; runtime verification was performed on macOS 26.6.2, not on every supported OS version.
- **Package:** a ZIP containing `BambuStudio.app`, with its resources and required zstd/Imath libraries included. Homebrew and a source checkout are not required to run it.
- **Signing:** ad-hoc signed, without Apple Developer ID signing or notarization. The release includes a SHA-256 checksum. macOS may block first launch; follow [Apple's instructions for opening an app from an unidentified developer](https://support.apple.com/102445), using the per-app **Privacy & Security > Open Anyway** approval if appropriate. Organization device policies may prevent this.

To install, quit Bambu Studio, extract the ZIP, and replace `BambuStudio.app` in **Applications**. Keep a copy of the previous app and back up `~/Library/Application Support/BambuStudio` first. Launch the copy in Applications. Existing profile/account storage is shared; no profile export/import is required. Do not run the official app and this fork against that same data folder simultaneously. Installing a later official Bambu Studio update can replace this fork and remove its painting behavior.

No account data, printer credentials, profiles from a developer's machine, or optional networking plugin binaries are included in the release. The optional Bambu networking plugin retains its normal download and user-data storage workflow.

On macOS, installing or updating the networking plugin requires quitting and reopening Bambu Studio before the new plugin is activated. The installation dialog will explain this. Save any open project before quitting. The fork avoids loading a replacement plugin into the same process: macOS can retain the previous plugin after it is unloaded, and the upgrade path produced a guarded-file-descriptor crash during discovery. An ordinary startup with an already-compatible plugin does not require an extra restart.

The fork retains the upstream [GNU Affero General Public License, version 3](../LICENSE), credits, and notices. See the [upstream README content](../README.md#license) for attribution and the separate status of the optional networking plugin.

## Maintaining the fork and release

The private [Tyr repository](https://github.com/Tyr-Laboratories/BambuStudio) uses **`master`**, matching upstream's default branch. It preserves upstream history and records the upstream base above. GitHub public-repository forks remain public, so this private copy is an independent repository rather than a member of GitHub's public fork network. Use `origin` for the Tyr repository and `upstream` for `https://github.com/bambulab/BambuStudio.git`.

Keep upstream merges on `master`, review conflicts in painting and perimeter code, and rerun the focused geometry/persistence tests before tagging a Tyr release. Tags use `v<upstream-version>-tyr.<revision>`. The upstream Homebrew and WinGet publishing jobs are restricted to the upstream repository so private Tyr releases are not submitted to public package indexes.

Build the application with the upstream [macOS build script](../BuildMac.sh), using Apple silicon and a deployment target of 14.0, with public-release configuration (`BBL_RELEASE_TO_PUBLIC=1`, `BBL_INTERNAL_TESTING=0`). Enable `SLIC3R_BUILD_TESTS` to build and run `wall_painting_tests`. An application build directory alone is not a distributable app: its resource link and machine-local library dependencies must be packaged.

The release tooling performs that separate step:

```sh
# Requires Xcode command-line tools, CMake, Ninja, Python 3, and curl.
bash scripts/build_tyr_macos_runtime.sh

# Commit the release's source/documentation changes first. Use a fresh output directory.
python3 scripts/package_tyr_macos.py \
  --app build/arm64/src/BambuStudio.app \
  --output build/releases/2.8.4.57-tyr.1 \
  --release 2.8.4.57-tyr.1 \
  --zstd build/tyr-runtime/lib/libzstd.1.dylib \
  --imath build/tyr-runtime/lib/libImath-3_2.30.dylib \
  --zstd-license build/tyr-runtime/licenses/zstd/LICENSE \
  --imath-license build/tyr-runtime/licenses/Imath/LICENSE.md
```

The [runtime build script](../scripts/build_tyr_macos_runtime.sh) pins the dependency source URLs and checksums. The [packager](../scripts/package_tyr_macos.py) copies the resources, includes the two runtime libraries and notices, replaces external library paths, checks architecture and deployment targets, seals the bundle with an ad-hoc signature, and writes the ZIP, checksum, and source-revision manifest. Set `--app` to the actual build bundle path. Verify the extracted archive, launch the packaged app, and repeat the painted-project save/reopen and slicing checks before uploading a release. Keep build logs and private installation backups outside tracked source.
