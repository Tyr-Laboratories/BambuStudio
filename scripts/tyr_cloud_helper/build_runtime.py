#!/usr/bin/env python3
"""Build an immutable Docker runtime for the app's private cloud upload helper.

Only public pinned source/plugin data are inputs. This command does not launch
Bambu Studio, authenticate, contact a printer, or modify Tyr Print Farm.
"""
import argparse
import hashlib
import io
import json
from pathlib import Path, PurePosixPath
import re
import shutil
import stat
import subprocess
import tarfile
import tempfile
import urllib.request
import zipfile

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
PLUGIN_FILES = ('libbambu_networking.so', 'libBambuSource.so', 'liblive555.so')
PUBLIC_CERT_FILES = ('slicer_base64.cer', 'ca-certificates.crt', 'printer.cer')
REQUIRED_FILES = {'image.tar', 'pin.json', 'plugin/linux_payload_manifest.json',
                  *('plugin/' + name for name in PLUGIN_FILES),
                  *('certs/' + name for name in PUBLIC_CERT_FILES),
                  'licenses/host-AGPL-3.0.txt', 'licenses/plugin-source.json'}
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
    return {key: pin[key] for key in ('plugin_url', 'plugin_sha256', 'network_abi', 'network_sha256')}


def plugin_payload_manifest(pin, hashes):
    return {'files': [{'name': name, 'sha256': hashes[name], 'abi_version': pin['network_abi']}
                      for name in PLUGIN_FILES]}


def verify_image_archive(path, expected_id):
    """Check Docker's immutable image config identity without extracting layers."""
    try:
        with tarfile.open(path, 'r:*') as archive:
            def read_json(name):
                if name.startswith('/') or '..' in PurePosixPath(name).parts:
                    raise RuntimeError('Invalid Docker image manifest path')
                member = archive.getmember(name)
                if not member.isfile() or member.size > 1024 * 1024:
                    raise RuntimeError('Invalid Docker image manifest entry')
                return archive.extractfile(member).read()
            saved = json.loads(read_json('manifest.json'))
            if not isinstance(saved, list) or len(saved) != 1:
                raise RuntimeError('Runtime archive must contain exactly one image')
            config = read_json(saved[0]['Config'])
            if 'sha256:' + hashlib.sha256(config).hexdigest() != expected_id:
                raise RuntimeError('Docker image identity does not match runtime manifest')
            identity = json.loads(config)
            if identity.get('architecture') != 'arm64' or identity.get('os') != 'linux':
                raise RuntimeError('Runtime Docker image must be linux/arm64')
    except (tarfile.TarError, KeyError, ValueError, TypeError, OSError) as error:
        raise RuntimeError('Invalid runtime Docker image archive') from error


def validate_runtime(root):
    root = Path(root)
    files = file_inventory(root)
    try:
        manifest = json.loads(files['manifest.json'].read_text())
        hashes = manifest['hashes']
        if (manifest['schema_version'] != 1 or manifest['protocol_version'] != 1
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
        if hashes['plugin/libbambu_networking.so'] != pin['network_sha256']:
            raise RuntimeError('Runtime networking library does not match its pinned checksum')
        library_hashes = {name: hashes['plugin/' + name] for name in PLUGIN_FILES}
        if json.loads(files['plugin/linux_payload_manifest.json'].read_text()) != plugin_payload_manifest(pin, library_hashes):
            raise RuntimeError('Runtime plugin payload manifest contains unexpected content')
        for name, expected in public_asset_hashes().items():
            if hashes[name] != expected:
                raise RuntimeError(f'Runtime public asset differs from its pinned upstream content: {name}')
        if json.loads(files['licenses/plugin-source.json'].read_text()) != plugin_provenance(pin):
            raise RuntimeError('Runtime plugin provenance contains unexpected content')
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


def install_plugin_archive(body, pin, target):
    if hashlib.sha256(body).hexdigest() != pin['plugin_sha256']:
        raise RuntimeError('Pinned plugin archive checksum mismatch')
    payloads = {}
    try:
        with zipfile.ZipFile(io.BytesIO(body)) as archive:
            for name in PLUGIN_FILES:
                candidates = [entry for entry in archive.infolist() if PurePosixPath(entry.filename).name == name]
                if len(candidates) != 1 or candidates[0].file_size > 100 * 1024 * 1024:
                    raise RuntimeError('Unexpected plugin archive layout')
                payloads[name] = archive.read(candidates[0])
    except zipfile.BadZipFile as error:
        raise RuntimeError('Invalid plugin archive') from error
    hashes = {name: hashlib.sha256(data).hexdigest() for name, data in payloads.items()}
    if hashes['libbambu_networking.so'] != pin['network_sha256']:
        raise RuntimeError('Pinned networking library checksum mismatch')
    target = Path(target)
    target.mkdir(parents=True, exist_ok=False)
    for name, payload in payloads.items():
        (target / name).write_bytes(payload)
    (target / 'linux_payload_manifest.json').write_text(json.dumps(plugin_payload_manifest(pin, hashes), indent=2) + '\n')


def build(output):
    output = Path(output).absolute()
    if output.exists():
        raise RuntimeError('Choose a new output directory; existing artifacts will not be overwritten')
    pin = json.loads((HERE / 'pin.json').read_text())
    host_source = ROOT / 'deps/tyr_cloud_helper'
    if not (host_source / 'CMakeLists.txt').is_file():
        raise RuntimeError('Vendored cloud helper source is missing')
    output.mkdir(parents=True)
    with urllib.request.urlopen(pin['plugin_url'], timeout=120) as response:
        payload = response.read(50 * 1024 * 1024 + 1)
    if len(payload) > 50 * 1024 * 1024:
        raise RuntimeError('Pinned plugin archive exceeds the download limit')
    install_plugin_archive(payload, pin, output / 'plugin')
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
        'schema_version': 1, 'protocol_version': 1, 'host_patch_version': 1,
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
