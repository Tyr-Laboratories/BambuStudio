#!/usr/bin/env python3
"""Build a redistributable Docker runtime for the cloud upload helper.

Only pinned open source code and public certificates are build inputs. Vendor
plugin libraries are downloaded separately by the installed app and never
included in this artifact. This command does not launch
Bambu Studio, authenticate, contact a printer, or modify Tyr Print Farm.
"""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import shutil
import stat
import subprocess
import tarfile
import tempfile

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
PLUGIN_FILES = ('libbambu_networking.so', 'libBambuSource.so', 'liblive555.so')
PUBLIC_CERT_FILES = ('slicer_base64.cer', 'ca-certificates.crt', 'printer.cer')
REQUIRED_FILES = {'image.tar', 'pin.json',
                  *('certs/' + name for name in PUBLIC_CERT_FILES),
                  'licenses/host-AGPL-3.0.txt', 'licenses/plugin-source.json',
                  'licenses/runtime-dependencies.txt'}
HEX256 = re.compile(r'[0-9a-f]{64}')


def sha256_file(path):
    result = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for body in iter(lambda: stream.read(1024 * 1024), b''):
            result.update(body)
    return result.hexdigest()


def file_inventory(root):
    if root.is_symlink() or not root.is_dir():
        raise RuntimeError('Runtime must be a real directory, not a symlink')
    result = {}
    for path in sorted(root.rglob('*')):
        if path.is_symlink():
            raise RuntimeError(f'Runtime symlink is not allowed: {path.relative_to(root)}')
        if path.is_dir():
            continue
        if not stat.S_ISREG(path.stat().st_mode):
            raise RuntimeError('Runtime inventory contains a non-regular file')
        result[path.relative_to(root).as_posix()] = path
    return result


def context_digest(files):
    result = hashlib.sha256()
    for name, path in sorted(files.items()):
        result.update((name + '\0' + sha256_file(path) + '\n').encode())
    return result.hexdigest()


def build_context_files():
    """Use the same source inventory for building and checking a release artifact."""
    host = ROOT / 'deps/tyr_cloud_helper'
    files = {'host/' + name: path for name, path in file_inventory(host).items()
             if '__pycache__' not in PurePosixPath(name).parts and not name.endswith('.pyc')}
    files['Dockerfile'] = HERE / 'Dockerfile'
    return files


def public_asset_hashes():
    upstream = json.loads((ROOT / 'deps/tyr_cloud_helper/upstream.json').read_text())
    original = upstream['redistributed_asset_sha256']
    hashes = {'certs/' + name: original['resources/cert/' + name] for name in PUBLIC_CERT_FILES}
    hashes['licenses/host-AGPL-3.0.txt'] = original['LICENSE.upstream']
    if any(not isinstance(value, str) or not HEX256.fullmatch(value) for value in hashes.values()):
        raise RuntimeError('Invalid pinned public asset checksums')
    return hashes


def plugin_provenance(pin):
    return {key: pin[key] for key in ('plugin_url', 'plugin_sha256', 'network_abi', 'network_sha256',
                                     'payload_sha256', 'plugin_entries')}


def validate_plugin_pin(pin):
    if (set(pin['payload_sha256']) != set(PLUGIN_FILES)
            or set(pin['plugin_entries']) != set(PLUGIN_FILES)
            or pin['payload_sha256']['libbambu_networking.so'] != pin['network_sha256']
            or not HEX256.fullmatch(pin['plugin_sha256'])
            or not pin['plugin_url'].startswith('https://')):
        raise RuntimeError('Invalid pinned vendor plugin metadata')
    for name in PLUGIN_FILES:
        entry = pin['plugin_entries'][name]
        if (not isinstance(entry, str) or not entry or entry.startswith('/')
                or '..' in PurePosixPath(entry).parts or '\\' in entry
                or PurePosixPath(entry).name != name
                or not isinstance(pin['payload_sha256'][name], str)
                or not HEX256.fullmatch(pin['payload_sha256'][name])):
            raise RuntimeError('Invalid pinned vendor plugin entry or checksum')


def runtime_dependency_notice(pin):
    return f"""Tyr Bambu Studio cloud helper runtime

This artifact contains the open-source Tyr Linux cloud host and its Debian
execution environment. It contains no Bambu Lab networking or camera plugin
binaries. The installed application obtains the separately licensed plugin
from Bambu Lab using the exact URL, archive checksum, member names and payload
checksums recorded in pin.json and plugin-source.json.

Host source: {pin['source']}
Host upstream revision: {pin['revision']}
Tyr modifications and build recipes: https://github.com/Tyr-Laboratories/BambuStudio
Host license: AGPL-3.0; see host-AGPL-3.0.txt.

Debian base: {pin['debian_base']}
Debian snapshot: {pin['debian_snapshot']}
The image retains Debian package copyright files in /usr/share/doc and common
license texts in /usr/share/common-licenses. Its package database identifies
the exact binary and source package versions. Dependencies include QEMU user
emulation (GPL-2.0 with separately licensed components), glibc (LGPL-2.1 and
other licenses), GCC runtime libraries (GPL-3.0 with GCC Runtime Library
Exception), OpenSSL (Apache-2.0), zlib (zlib license), and CA certificates.
Consult each package's copyright file for authoritative component terms.

Debian corresponding source packages are available at the pinned Debian
snapshot https://snapshot.debian.org/archive/debian/{pin['debian_snapshot']}/
and security snapshot https://snapshot.debian.org/archive/debian-security/{pin['debian_snapshot']}/.
Source-delivery obligations must be satisfied when publishing the binary
release; a link to a dependency or this notice alone is not a source offer.
"""


def forbidden_plugin_path(name):
    basename = PurePosixPath(name).name.lower()
    return (basename.startswith(('libbambu_networking.', 'libbambusource.',
                                 'liblive555.', 'libagora'))
            or basename in ('bambu_networking.dll', 'bambusource.dll'))


def verify_image_archive(path, expected_id):
    """Verify image identity and audit every shipped layer, including deleted files."""
    try:
        known_payload_hashes = set(json.loads((HERE / 'pin.json').read_text())['payload_sha256'].values())
        with tarfile.open(path, 'r:*') as archive:
            entries = archive.getmembers()
            names = [member.name for member in entries]
            if len(names) != len(set(names)):
                raise RuntimeError('Duplicate Docker image archive entry')
            for member in entries:
                if (member.name.startswith('/') or '..' in PurePosixPath(member.name).parts
                        or not (member.isfile() or member.isdir())):
                    raise RuntimeError('Invalid Docker image archive path or entry')
            def read_json(name):
                member = archive.getmember(name)
                if not member.isfile() or member.size > 1024 * 1024:
                    raise RuntimeError('Invalid Docker image manifest entry')
                return archive.extractfile(member).read()
            saved = json.loads(read_json('manifest.json'))
            if not isinstance(saved, list) or len(saved) != 1:
                raise RuntimeError('Runtime archive must contain exactly one image')
            config_name = saved[0]['Config']
            config = read_json(config_name)
            if 'sha256:' + hashlib.sha256(config).hexdigest() != expected_id:
                raise RuntimeError('Docker image identity does not match runtime manifest')
            identity = json.loads(config)
            if identity.get('architecture') != 'arm64' or identity.get('os') != 'linux':
                raise RuntimeError('Runtime Docker image must be linux/arm64')
            layers = saved[0]['Layers']
            if not isinstance(layers, list) or len(set(layers)) != len(layers):
                raise RuntimeError('Invalid Docker image layer inventory')
            diff_ids = identity.get('rootfs', {}).get('diff_ids', [])
            if len(diff_ids) != len(layers):
                raise RuntimeError('Docker image layer identity inventory differs')
            for name, expected_layer in zip(layers, diff_ids):
                member = archive.getmember(name)
                if not member.isfile():
                    raise RuntimeError('Invalid Docker image layer entry')
                # Docker save uses uncompressed layers; compare their immutable
                # identities before inspecting names in all layers, including
                # files hidden by a later whiteout.
                digest = hashlib.sha256()
                with archive.extractfile(member) as stream:
                    for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                        digest.update(chunk)
                if 'sha256:' + digest.hexdigest() != expected_layer:
                    raise RuntimeError('Docker image layer identity mismatch')
                with archive.extractfile(member) as stream, tarfile.open(fileobj=stream, mode='r|') as layer:
                    for item in layer:
                        parts = PurePosixPath(item.name).parts
                        if item.name.startswith('/') or '..' in parts:
                            raise RuntimeError('Invalid Docker layer path')
                        if (forbidden_plugin_path(item.name)
                                or (parts and parts[0] == 'plugin' and (len(parts) > 1 or not item.isdir()))
                                or ((item.issym() or item.islnk()) and
                                    (forbidden_plugin_path(item.linkname)
                                     or 'plugin' in PurePosixPath(item.linkname).parts))):
                            raise RuntimeError('Proprietary plugin payload is forbidden in public Docker layers')
                        if item.isfile():
                            payload_hash = hashlib.sha256()
                            with layer.extractfile(item) as payload:
                                for chunk in iter(lambda: payload.read(1024 * 1024), b''):
                                    payload_hash.update(chunk)
                            if payload_hash.hexdigest() in known_payload_hashes:
                                raise RuntimeError('Renamed proprietary plugin payload is forbidden in public Docker layers')
            for member in entries:
                if not member.isfile() or member.name in {config_name, *layers}:
                    continue
                # OCI image/index metadata are JSON, not additional layers or
                # arbitrary unreferenced blobs that could conceal payloads.
                if member.name not in ('manifest.json', 'repositories', 'index.json', 'oci-layout') and not member.name.startswith('blobs/sha256/'):
                    raise RuntimeError('Unexpected Docker image archive entry')
                json.loads(read_json(member.name))
    except (tarfile.TarError, KeyError, ValueError, TypeError, OSError) as error:
        raise RuntimeError('Invalid runtime Docker image archive') from error


def validate_runtime(root):
    root = Path(root)
    files = file_inventory(root)
    if (root / 'plugin').exists():
        raise RuntimeError('Public runtime must not contain a plugin directory')
    try:
        manifest = json.loads(files['manifest.json'].read_text())
        hashes = manifest['hashes']
        if (manifest['schema_version'] != 2 or manifest.get('plugin_delivery') != 'vendor-download'
                or manifest['protocol_version'] != 1
                or manifest['host_patch_version'] != 1
                or manifest['image_file'] != 'image.tar'
                or manifest['platform'] != 'linux/arm64'
                or manifest['executable'] != '/runtime/tyr_bambu_cloud_host'
                or not re.fullmatch(r'sha256:[0-9a-f]{64}', manifest['image_id'])
                or not isinstance(hashes, dict)):
            raise RuntimeError('Unsupported cloud helper runtime manifest')
        if (set(files) != REQUIRED_FILES | {'manifest.json'} or set(hashes) != REQUIRED_FILES):
            raise RuntimeError('Runtime file inventory is incomplete or has unexpected files')
        for name, expected in hashes.items():
            if not isinstance(expected, str) or not HEX256.fullmatch(expected) or sha256_file(files[name]) != expected:
                raise RuntimeError(f'Runtime checksum mismatch: {name}')
        pin = json.loads(files['pin.json'].read_text())
        if pin != json.loads((HERE / 'pin.json').read_text()):
            raise RuntimeError('Runtime dependency pin differs from the checked-in pin; rebuild the runtime')
        if manifest.get('upstream_revision') != pin['revision']:
            raise RuntimeError('Runtime upstream revision differs from its dependency pin')
        if pin['network_abi'] != manifest['network_abi']:
            raise RuntimeError('Runtime plugin ABI does not match its pin')
        validate_plugin_pin(pin)
        for name, expected in public_asset_hashes().items():
            if hashes[name] != expected:
                raise RuntimeError(f'Runtime public asset differs from its pinned upstream content: {name}')
        if json.loads(files['licenses/plugin-source.json'].read_text()) != plugin_provenance(pin):
            raise RuntimeError('Runtime plugin provenance contains unexpected content')
        if files['licenses/runtime-dependencies.txt'].read_text() != runtime_dependency_notice(pin):
            raise RuntimeError('Runtime dependency notice contains unexpected content')
        if manifest.get('host_source_sha256') != context_digest(build_context_files()):
            raise RuntimeError('Runtime host source or Dockerfile changed; rebuild the runtime before packaging')
        verify_image_archive(files['image.tar'], manifest['image_id'])
        return manifest
    except (KeyError, ValueError, TypeError) as error:
        raise RuntimeError('Invalid cloud helper runtime manifest or pin') from error


def stage_runtime(source, target):
    source, target = Path(source), Path(target)
    resolved_source, resolved_target = source.resolve(), target.resolve()
    if (target.exists() or target.is_symlink() or resolved_source == resolved_target
            or resolved_source in resolved_target.parents or resolved_target in resolved_source.parents):
        raise RuntimeError('Runtime target exists or overlaps its source')
    manifest = validate_runtime(source)
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(source, target)
    if validate_runtime(target) != manifest:
        raise RuntimeError('Runtime changed while being copied')
    return manifest


def build(output):
    output = Path(output).absolute()
    if output.exists():
        raise RuntimeError('Choose a new output directory; existing artifacts will not be overwritten')
    pin = json.loads((HERE / 'pin.json').read_text())
    host_source = ROOT / 'deps/tyr_cloud_helper'
    if not (host_source / 'CMakeLists.txt').is_file():
        raise RuntimeError('Vendored cloud helper source is missing')
    output.mkdir(parents=True)
    validate_plugin_pin(pin)
    shutil.copy2(HERE / 'pin.json', output / 'pin.json')
    public_sources = {'certs/' + name: host_source / 'resources/cert' / name for name in PUBLIC_CERT_FILES}
    public_sources['licenses/host-AGPL-3.0.txt'] = host_source / 'LICENSE.upstream'
    for name, expected in public_asset_hashes().items():
        if sha256_file(public_sources[name]) != expected:
            raise RuntimeError(f'Vendored public asset differs from pinned upstream content: {name}')
        target = output / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(public_sources[name], target)
    (output / 'licenses/plugin-source.json').write_text(json.dumps(plugin_provenance(pin), indent=2) + '\n')
    (output / 'licenses/runtime-dependencies.txt').write_text(runtime_dependency_notice(pin))
    with tempfile.TemporaryDirectory(prefix='tyr-studio-helper-build-') as temporary:
        context = Path(temporary)
        # Minimal build context: no repository .git, user profile, session, or credentials.
        for name, source in build_context_files().items():
            target = context / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
        source_digest = context_digest(file_inventory(context))
        tag = 'tyr-bambu-studio-cloud-helper:' + source_digest[:16]
        subprocess.run(['docker', 'build', '--platform', 'linux/arm64', '--tag', tag,
                        '--build-arg', 'DEBIAN_BASE=' + pin['debian_base'],
                        '--build-arg', 'DEBIAN_SNAPSHOT=' + pin['debian_snapshot'], str(context)], check=True)
    image_id = subprocess.check_output(['docker', 'image', 'inspect', tag, '--format', '{{.Id}}'], text=True).strip()
    subprocess.run(['docker', 'image', 'save', '--output', str(output / 'image.tar'), image_id], check=True)
    manifest = {
        'schema_version': 2, 'plugin_delivery': 'vendor-download', 'protocol_version': 1, 'host_patch_version': 1,
        'image_id': image_id, 'image_file': 'image.tar', 'platform': 'linux/arm64',
        'executable': '/runtime/tyr_bambu_cloud_host', 'network_abi': pin['network_abi'],
        'host_source_sha256': source_digest, 'upstream_revision': pin['revision'],
        'hashes': {name: sha256_file(path) for name, path in file_inventory(output).items()},
    }
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    validate_runtime(output)
    print(f'Validated cloud helper runtime: {output}')
    print(f'Immutable Docker image: {image_id}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--verify', type=Path, help='Verify an existing artifact without building or starting it')
    args = parser.parse_args()
    if bool(args.output) == bool(args.verify):
        parser.error('choose exactly one of --output or --verify')
    if args.verify:
        print(json.dumps(validate_runtime(args.verify), indent=2))
    else:
        build(args.output)


if __name__ == '__main__':
    main()
