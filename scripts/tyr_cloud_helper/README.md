# Cloud upload runtime

This directory builds the runtime bundled with the Tyr Bambu Studio macOS app. Docker Desktop must be installed and running on an Apple silicon Mac. The native application starts a private helper container for each upload, using the existing Studio account and print dialog. Coworkers do not build an image, install a Python package, or copy account data into the application bundle.

The runtime uses an ARM64 Debian container and an explicitly invoked x86-64 emulator for Bambu's Linux networking library. It does not register a machine-wide emulator or use the Tyr Print Farm installation. The image contains the host and its operating-system dependencies; the pinned plugin and public certificate files are separate read-only bundle resources.

Build an artifact from the repository root:

```sh
python3 scripts/tyr_cloud_helper/build_runtime.py --output build/cloud-helper-runtime-v1
```

`pin.json` fixes the upstream host revision, plugin download/checksums, Debian image digest, and dated Debian package snapshot. The host source is vendored and patched in `deps/tyr_cloud_helper`. Only that source and this Dockerfile enter the Docker build context. The build downloads public dependencies, creates a new `tyr-bambu-studio-cloud-helper` image, and saves its immutable image ID. It never authenticates or sends a print. Exact byte-for-byte reproducibility across different Docker/compiler versions is not asserted; the completed artifact is verified by its recorded checksums and immutable Docker image identity.

Package the application with:

```sh
python3 scripts/package_tyr_macos.py --cloud-helper-runtime build/cloud-helper-runtime-v1 <other release arguments>
```

To verify an existing artifact without starting it:

```sh
python3 scripts/tyr_cloud_helper/build_runtime.py --verify build/cloud-helper-runtime-v1
python3 -m unittest discover -s tests/packaging -v
```

## Bundle contract

The app stores the artifact at `Contents/Resources/cloud-helper/`:

- `manifest.json`: schema, protocol and host-patch version 1; immutable `image_id`; `image_file` (`image.tar`); `platform` (`linux/arm64`); `executable` (`/runtime/tyr_bambu_cloud_host`); `network_abi`; and a `hashes` map covering every other artifact file.
- `image.tar`: a single Docker image. Its configuration hash must equal `image_id`, and its platform must be Linux ARM64.
- `pin.json`: public dependency identities and checksums.
- `plugin/`: exactly the pinned networking, BambuSource and live555 libraries plus their generated payload manifest.
- `certs/`: public certificates from the pinned upstream source.
- `licenses/`: host license and plugin download provenance.

Symlinks, any unexpected file, missing files, checksum differences, and a different image identity fail validation before packaging. The three certificate filenames and the host license must match the exact checksums recorded in the vendored upstream metadata; plugin provenance accepts only its four public dependency fields. Packaging also compares the checked-in dependency pin and the current host/Dockerfile build-context digest with the artifact, so a changed helper requires rebuilding the runtime. The packager verifies the copied runtime and the extracted final release ZIP. Account tokens, profiles, printer access codes, job files, and session directories are never runtime build or packaging inputs.

The image entry point is `/usr/bin/qemu-x86_64-static` with `/runtime/tyr_bambu_cloud_host` as its command. The native caller loads the image if necessary and runs its immutable ID. It mounts `plugin` at `/plugin` and `certs` at `/certs` read-only, a private job payload at `/job` read-only, and that attempt's private session at `/session` writable. `HOME` is `/session`. RPC uses framed stdin/stdout without a terminal; container lifecycle and any uncertain upload outcome belong to the native caller.
