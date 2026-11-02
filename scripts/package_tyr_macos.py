#!/usr/bin/env python3
"""Stage a relocatable Apple silicon app, audit it, and create a release ZIP.

Requires macOS command-line tools. Does not change the input app, install the
result, include user data, or notarize the package. See docs/tyr-fork.md.
"""
import argparse
import hashlib
import json
from pathlib import Path
import plistlib
import re
import shutil
import stat
import subprocess
import tempfile


OFFICIAL_TEAM_ID = "T3UBR9Y3B2"
OFFICIAL_BUNDLE_ID = "com.bambulab.bambu-studio"
OFFICIAL_RELATIVE_PATH = Path("Contents/Helpers/BambuStudio.app")


def run(*args):
    return subprocess.check_output([str(arg) for arg in args], text=True).strip()


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def official_bundle_digest(app):
    """Hash contents, modes, and symlinks without following links outside the app."""
    digest = hashlib.sha256()
    for path in sorted(app.rglob("*")):
        mode = path.lstat().st_mode
        name = path.relative_to(app).as_posix()
        if path.is_symlink():
            try:
                resolved = path.resolve(strict=True)
            except (OSError, RuntimeError) as error:
                raise RuntimeError(f"Invalid official app symlink: {name}") from error
            if app.resolve() not in resolved.parents:
                raise RuntimeError(f"Official app symlink escapes its bundle: {name}")
            entry = [name, mode, "link", str(path.readlink())]
        elif stat.S_ISDIR(mode):
            entry = [name, mode, "directory"]
        elif stat.S_ISREG(mode):
            entry = [name, mode, "file", sha256_file(path)]
        else:
            raise RuntimeError(f"Unsupported file in official app bundle: {name}")
        digest.update((json.dumps(entry, ensure_ascii=True) + "\n").encode("utf-8"))
    return digest.hexdigest()


def verify_vendor_signature(app):
    """Require the genuine Bambu publisher, not merely any valid local signature."""
    requirement = (f'=anchor apple generic and certificate leaf[subject.OU] = "{OFFICIAL_TEAM_ID}" '
                   f'and identifier "{OFFICIAL_BUNDLE_ID}"')
    try:
        subprocess.check_output([
            "codesign", "--verify", "--deep", "--strict", "--verbose=2",
            "--test-requirement", requirement, str(app),
        ], text=True, stderr=subprocess.STDOUT)
        detail = subprocess.check_output([
            "codesign", "--display", "--verbose=4", str(app),
        ], text=True, stderr=subprocess.STDOUT)
    except subprocess.CalledProcessError as error:
        raise RuntimeError(f"Official app signature verification failed: {error.output.strip()}") from error
    team = re.search(r"^TeamIdentifier=(.+)$", detail, re.MULTILINE)
    publisher = re.search(r"^Authority=(.+)$", detail, re.MULTILINE)
    if not team or team[1] != OFFICIAL_TEAM_ID or not publisher:
        raise RuntimeError("Official app is not signed by the expected Bambu publisher")
    return {"team_identifier": team[1], "publisher": publisher[1]}


def official_app_metadata(app):
    if app.is_symlink():
        raise RuntimeError("Official app bundle must not be a symlink")
    if not app.is_dir():
        raise RuntimeError(f"Missing official app bundle: {app}")
    # Inspect all links before opening any bundle files so malformed inputs cannot
    # cause reads of account data or other files outside this supplied release.
    bundle_digest = official_bundle_digest(app)
    info_path = app / "Contents/Info.plist"
    if not info_path.is_file() or info_path.is_symlink():
        raise RuntimeError("Official bundle is missing a regular Contents/Info.plist")
    try:
        info = plistlib.loads(info_path.read_bytes())
    except (plistlib.InvalidFileException, ValueError) as error:
        raise RuntimeError("Invalid official app bundle Info.plist") from error
    if info.get("CFBundleIdentifier") != OFFICIAL_BUNDLE_ID:
        raise RuntimeError("Unexpected official app bundle identifier")
    if info.get("CFBundleExecutable") != "BambuStudio":
        raise RuntimeError("Unexpected official app executable")
    executable = app / "Contents/MacOS/BambuStudio"
    if not executable.is_file() or executable.is_symlink():
        raise RuntimeError("Official bundle is missing its regular BambuStudio executable")
    version = info.get("CFBundleShortVersionString")
    build = info.get("CFBundleVersion", "")
    if not isinstance(version, str) or not version or not isinstance(build, str):
        raise RuntimeError("Official app bundle has invalid version metadata")
    return {
        "bundle_path": OFFICIAL_RELATIVE_PATH.as_posix(),
        "bundle_identifier": info["CFBundleIdentifier"],
        "version": version,
        "build": build,
        "executable_sha256": sha256_file(executable),
        "bundle_sha256": bundle_digest,
        **verify_vendor_signature(app),
    }


def verify_official_app(app, expected):
    if official_app_metadata(app) != expected:
        raise RuntimeError(f"Official app changed; bundle must remain identical to vendor input: {app}")


def stage_official_app(source, target):
    source_resolved, target_resolved = source.resolve(), target.resolve()
    if (target.exists() or target.is_symlink() or source_resolved == target_resolved
            or source_resolved in target_resolved.parents or target_resolved in source_resolved.parents):
        raise RuntimeError("Official app target exists or overlaps the source bundle")
    metadata = official_app_metadata(source)
    target.parent.mkdir(parents=True, exist_ok=True)
    # ditto preserves bundle metadata, signatures, resource forks and symlinks.
    # Do not use the enclosing fork's signature-stripping copy for the sender.
    run("ditto", source, target)
    verify_official_app(target, metadata)
    return metadata


def dependencies(binary):
    return [line.strip().split(" (compatibility version", 1)[0]
            for line in run("otool", "-L", binary).splitlines()[1:]]


def rpaths(binary):
    return re.findall(r"cmd LC_RPATH\s+cmdsize \d+\s+path (.*?) \(offset",
                      run("otool", "-l", binary))


def os_version(value):
    parts = tuple(int(part) for part in value.split("."))
    return parts + (0,) * (3 - len(parts))


def audit(binary, frameworks, minimum):
    arch = run("lipo", "-archs", binary)
    if arch != "arm64":
        raise RuntimeError(f"Expected arm64, found {arch}: {binary}")
    commands = run("otool", "-l", binary)
    versions = re.findall(r"\bminos (\d+(?:\.\d+)+)", commands)
    if not versions or any(os_version(v) > os_version(minimum) for v in versions):
        raise RuntimeError(f"Deployment target exceeds {minimum}: {binary}: {versions}")
    for dependency in dependencies(binary):
        if dependency.startswith(("/usr/lib/", "/System/Library/")):
            continue
        if dependency.startswith("@rpath/") and (frameworks / dependency[7:]).is_file():
            continue
        raise RuntimeError(f"Unbundled dependency in {binary}: {dependency}")
    for path in rpaths(binary):
        if path not in ("@executable_path/../Frameworks", "@loader_path"):
            raise RuntimeError(f"Nonportable runtime search path: {path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--app", required=True, type=Path)
    parser.add_argument("--official-app", type=Path,
                        help="Untouched vendor-signed Bambu Studio app to bundle as the print sender")
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--zstd", required=True, type=Path)
    parser.add_argument("--imath", required=True, type=Path)
    parser.add_argument("--zstd-license", required=True, type=Path)
    parser.add_argument("--imath-license", required=True, type=Path)
    parser.add_argument("--release", required=True)
    parser.add_argument("--minimum-macos", default="14.0")
    args = parser.parse_args()
    if not re.fullmatch(r"[A-Za-z0-9._-]+", args.release):
        parser.error("release must contain only letters, numbers, dots, underscores or hyphens")
    root = Path(__file__).resolve().parent.parent
    revision = run("git", "-C", root, "rev-parse", "HEAD")
    if run("git", "-C", root, "status", "--porcelain", "--untracked-files=no"):
        parser.error("commit tracked changes before packaging so the source revision is exact")
    if (args.app / OFFICIAL_RELATIVE_PATH).exists():
        parser.error("input fork already contains a sender; use the build app and --official-app separately")
    for source in (args.app / "Contents/MacOS/BambuStudio", args.zstd, args.imath,
                   args.zstd_license, args.imath_license):
        if not source.is_file():
            parser.error(f"missing input: {source}")
    args.output.mkdir(parents=True, exist_ok=False)
    app = args.output / "BambuStudio.app"
    shutil.copytree(args.app, app, symlinks=False,
                    ignore=shutil.ignore_patterns(".DS_Store", "_CodeSignature"))
    contents = app / "Contents"
    frameworks = contents / "Frameworks"
    frameworks.mkdir(exist_ok=True)
    libraries = {
        "libzstd.1.dylib": args.zstd,
        "libImath-3_2.30.dylib": args.imath,
    }
    for name, source in libraries.items():
        target = frameworks / name
        shutil.copy2(source, target)
        run("install_name_tool", "-id", "@rpath/" + name, target)
    executable = contents / "MacOS/BambuStudio"
    binaries = [executable] + [frameworks / name for name in libraries]
    for binary in binaries:
        for dependency in dependencies(binary):
            name = Path(dependency).name
            if name in libraries and dependency != "@rpath/" + name:
                run("install_name_tool", "-change", dependency, "@rpath/" + name, binary)
        for path in rpaths(binary):
            if path not in ("@loader_path", "@executable_path/../Frameworks"):
                run("install_name_tool", "-delete_rpath", path, binary)
    if "@executable_path/../Frameworks" not in rpaths(executable):
        run("install_name_tool", "-add_rpath", "@executable_path/../Frameworks", executable)
    plist_path = contents / "Info.plist"
    info = plistlib.loads(plist_path.read_bytes())
    info["LSMinimumSystemVersion"] = args.minimum_macos
    info["CFBundleVersion"] = info["CFBundleShortVersionString"]
    info["CFBundleGetInfoString"] = "Tyr Laboratories Wall Painting " + args.release
    plist_path.write_bytes(plistlib.dumps(info))
    resources = contents / "Resources"
    notices = resources / "TyrFork"
    notices.mkdir()
    for source, name in ((root / "LICENSE", "BambuStudio-AGPL-3.0.txt"),
                         (args.zstd_license, "zstd-LICENSE.txt"),
                         (args.imath_license, "Imath-LICENSE.md"),
                         (root / "docs/tyr-fork.md", "tyr-fork.md")):
        shutil.copy2(source, notices / name)
    manifest = {
        "release": args.release,
        "source_revision": revision,
        "repository": "https://github.com/Tyr-Laboratories/BambuStudio",
        "upstream_revision": "79df7ae309be398ed30c9f397c13c9911c2edc57",
        "architecture": "arm64",
        "minimum_macos": args.minimum_macos,
        "signing": "ad-hoc; not Developer ID signed or notarized",
        "runtime_libraries": {
            "zstd": {"version": "1.5.7", "source": "https://github.com/facebook/zstd/releases/tag/v1.5.7"},
            "Imath": {"version": "3.2.3", "source": "https://github.com/AcademySoftwareFoundation/Imath/releases/tag/v3.2.3"},
        },
    }
    if args.official_app:
        manifest["official_sender"] = stage_official_app(args.official_app, app / OFFICIAL_RELATIVE_PATH)
    (notices / "release.json").write_text(json.dumps(manifest, indent=2) + "\n")
    for binary in binaries:
        audit(binary, frameworks, args.minimum_macos)
    # Seal nested libraries before the enclosing bundle. Ad-hoc signatures do
    # not establish publisher identity and do not imply Apple notarization.
    for binary in binaries[1:]:
        run("codesign", "--force", "--sign", "-", "--timestamp=none", binary)
    run("codesign", "--force", "--sign", "-", "--timestamp=none", app)
    run("codesign", "--verify", "--deep", "--strict", "--verbose=2", app)
    if args.official_app:
        # Signing the outer app must never re-sign or otherwise modify the inner
        # vendor app. In particular, do not add --deep to the signing command.
        verify_official_app(app / OFFICIAL_RELATIVE_PATH, manifest["official_sender"])
        verify_official_app(args.official_app, manifest["official_sender"])
    archive = args.output / f"BambuStudio-Tyr-{args.release}-macOS-arm64.zip"
    run("ditto", "-c", "-k", "--sequesterRsrc", "--keepParent", app, archive)
    if args.official_app:
        with tempfile.TemporaryDirectory(prefix="verify-sender-", dir=args.output) as unpacked:
            run("ditto", "-x", "-k", archive, unpacked)
            extracted = Path(unpacked) / app.name
            run("codesign", "--verify", "--deep", "--strict", "--verbose=2", extracted)
            verify_official_app(extracted / OFFICIAL_RELATIVE_PATH, manifest["official_sender"])
    digest = sha256_file(archive)
    (args.output / "SHA256SUMS.txt").write_text(f"{digest}  {archive.name}\n")
    (args.output / "release-manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"Validated application: {app}")
    print(f"Release archive: {archive}")
    print(f"SHA256: {digest}")


if __name__ == "__main__":
    main()
