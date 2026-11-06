#!/usr/bin/env python3
"""Collect exact corresponding sources for a verified public helper runtime.

Downloads Debian source packages, including QEMU's static Built-Using sources,
from official mirrors. Never downloads vendor plugins or accesses an account.
The resulting source archive is published beside the binary release.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import gzip
import hashlib
import json
import lzma
from pathlib import Path, PurePosixPath
import re
import shutil
import subprocess
import tarfile
import tempfile
import urllib.error
import urllib.request

import build_runtime as runtime

PACKAGE_FORMAT = '${binary:Package}\t${Version}\t${source:Package}\t${source:Version}\t${Built-Using}\n'
PACKAGE = re.compile(r'[a-z0-9][a-z0-9+.-]*')
VERSION = re.compile(r'[A-Za-z0-9.+:~_-]+')


def parse_packages(text):
    result = []
    for line in text.splitlines():
        binary, version, source, source_version, built_using = line.split('\t')
        static = []
        for dependency in filter(None, (item.strip() for item in built_using.split(','))):
            match = re.fullmatch(r'([a-z0-9+.-]+) \(= ([^)]+)\)', dependency)
            if not match:
                raise RuntimeError('Unexpected Debian Built-Using dependency')
            static.append({'package': match[1], 'version': match[2]})
        if not PACKAGE.fullmatch(source) or not VERSION.fullmatch(source_version):
            raise RuntimeError('Invalid Debian source identity')
        result.append({'binary': binary, 'version': version, 'source': source,
                       'source_version': source_version, 'built_using': static})
    return result


def required_sources(packages):
    result = {(package['source'], package['source_version']) for package in packages}
    result.update((dependency['package'], dependency['version'])
                  for package in packages for dependency in package['built_using'])
    return result


def incorporated_host_packages(packages):
    # Include CRT startup objects and C/kernel headers as well as explicit C++
    # libraries. Cross-libc packages can embed an older glibc/linux snapshot
    # than the runtime's dynamically loaded libc; keep both exact identities.
    return [item for item in packages if item['binary'].startswith(
        ('libboost', 'nlohmann-json3-dev', 'libstdc++', 'libgcc', 'libssl-dev',
         'libc6', 'libc-dev-bin', 'linux-libc-dev'))]


def parse_source_index(body):
    result = {}
    for paragraph in body.split('\n\n'):
        fields = {}
        key = None
        for line in paragraph.splitlines():
            if line.startswith((' ', '\t')) and key:
                fields[key] += '\n' + line[1:]
            elif ':' in line:
                key, value = line.split(':', 1)
                fields[key] = value.strip()
        if 'Package' not in fields:
            continue
        directory = PurePosixPath(fields['Directory'])
        if directory.is_absolute() or '..' in directory.parts or directory.parts[0] != 'pool':
            raise RuntimeError('Invalid Debian source directory')
        files = []
        for row in fields['Checksums-Sha256'].splitlines():
            if not row.strip():
                continue
            checksum, size, name = row.split()
            if (not runtime.HEX256.fullmatch(checksum) or not size.isdecimal()
                    or int(size) > 8 * 1024 * 1024 * 1024 or '/' in name or '\\' in name
                    or name in ('.', '..')):
                raise RuntimeError('Invalid Debian source file record')
            files.append({'name': name, 'size': int(size), 'sha256': checksum})
        if not files or len({item['name'] for item in files}) != len(files):
            raise RuntimeError('Invalid Debian source file inventory')
        result[fields['Package'], fields['Version']] = {
            'package': fields['Package'], 'version': fields['Version'],
            'directory': str(directory), 'files': files}
    return result


def download_verified(url, target, expected_size, expected_sha256):
    """Write only a bounded, complete, checksum-verified source package."""
    target = Path(target)
    if target.is_symlink():
        raise RuntimeError('Source cache target must not be a symlink')
    if target.exists():
        if target.stat().st_size == expected_size and runtime.sha256_file(target) == expected_sha256:
            return
        raise RuntimeError('Existing source cache checksum differs')
    target.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=target.parent, delete=False) as stream:
        temporary = Path(stream.name)
        try:
            digest = hashlib.sha256()
            received = 0
            with urllib.request.urlopen(url, timeout=60) as response:
                while chunk := response.read(1024 * 1024):
                    received += len(chunk)
                    if received > expected_size:
                        raise RuntimeError('Source download exceeds expected size')
                    digest.update(chunk)
                    stream.write(chunk)
            if received != expected_size or digest.hexdigest() != expected_sha256:
                raise RuntimeError('Source download checksum or size mismatch')
            stream.close()
            temporary.rename(target)
        finally:
            temporary.unlink(missing_ok=True)


def package_inventory(image):
    text = subprocess.check_output(['docker', 'run', '--rm', '--network', 'none',
                                    '--entrypoint', '/usr/bin/dpkg-query', image,
                                    '-W', '-f=' + PACKAGE_FORMAT], text=True)
    return parse_packages(text)


def host_build_inventory(pin):
    with tempfile.TemporaryDirectory(prefix='tyr-helper-source-context-') as temporary:
        context = Path(temporary)
        for name, source in runtime.build_context_files().items():
            target = context / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
        tag = 'tyr-helper-source-inventory:' + runtime.context_digest(runtime.build_context_files())[:16]
        subprocess.run(['docker', 'build', '--platform', 'linux/arm64', '--target', 'build',
                        '--tag', tag, '--build-arg', 'DEBIAN_BASE=' + pin['debian_base'],
                        '--build-arg', 'DEBIAN_SNAPSHOT=' + pin['debian_snapshot'], str(context)], check=True)
    return package_inventory(tag)


def source_control_record(name, version, directory, archive, pin, cache):
    # Cross-toolchain packages retain CRT/header inputs older than the current
    # suite indexes. Debian keeps these exact source controls in the pool.
    # Never substitute the currently indexed source version.
    filename = name + '_' + version.split(':')[-1] + '.dsc'
    suffix = directory + '/' + filename
    urls = [f'https://deb.debian.org/{archive}/{suffix}',
            f'https://snapshot.debian.org/archive/{archive}/{pin["debian_snapshot"]}/{suffix}']
    control = cache / 'source-controls' / archive / suffix
    origin = control.with_suffix('.origin.json')
    if control.exists() and origin.exists():
        body = control.read_bytes()
        provenance = json.loads(origin.read_text())
        if hashlib.sha256(body).hexdigest() != provenance['sha256']:
            raise RuntimeError('Cached source control checksum mismatch')
    else:
        for url in urls:
            try:
                with urllib.request.urlopen(url, timeout=20) as response:
                    body = response.read(2 * 1024 * 1024 + 1)
                if len(body) > 2 * 1024 * 1024:
                    raise RuntimeError('Debian source control exceeds size limit')
                provenance = {'url': url, 'sha256': hashlib.sha256(body).hexdigest(),
                              'kind': 'exact-version-debian-source-control'}
                control.parent.mkdir(parents=True, exist_ok=True)
                control.write_bytes(body)
                origin.write_text(json.dumps(provenance, indent=2) + '\n')
                break
            except (urllib.error.URLError, TimeoutError):
                if url == urls[-1]:
                    raise RuntimeError(f'Exact Debian source version unavailable: {name} {version}')
    text = body.decode().split('-----BEGIN PGP SIGNATURE-----', 1)[0]
    if text.startswith('-----BEGIN PGP SIGNED MESSAGE-----'):
        text = text.split('\n\n', 1)[1]
    if not re.search(r'^Source: ' + re.escape(name) + r'$', text, re.MULTILINE):
        raise RuntimeError('Debian source control package identity differs')
    records = parse_source_index(f'Package: {name}\nDirectory: {directory}\n' + text)
    if (name, version) not in records or len(records) != 1:
        raise RuntimeError('Debian source control version identity differs')
    record = records[name, version]
    record['files'].append({'name': filename, 'size': len(body), 'sha256': provenance['sha256']})
    record['archive'] = archive
    record['source_index'] = provenance
    return record


def source_plan(required, pin, cache):
    indices = {}
    provenance = []
    for archive, suite in (('debian', 'bookworm'), ('debian-security', 'bookworm-security')):
        # Prefer the pinned snapshot; exact versions from the official current
        # mirror are equivalent source inputs and avoid snapshot outages.
        suffix = f'dists/{suite}/main/source/Sources.xz'
        urls = [f'https://snapshot.debian.org/archive/{archive}/{pin["debian_snapshot"]}/{suffix}',
                f'https://deb.debian.org/{archive}/{suffix}']
        cached = cache / (archive + '-Sources.xz')
        metadata = cache / (archive + '-index.json')
        if cached.exists() and metadata.exists():
            original = json.loads(metadata.read_text())
            raw = cached.read_bytes()
            if hashlib.sha256(raw).hexdigest() != original['sha256']:
                raise RuntimeError('Source index cache checksum mismatch')
        else:
            for url in urls:
                try:
                    with urllib.request.urlopen(url, timeout=20) as response:
                        raw = response.read(32 * 1024 * 1024 + 1)
                    if len(raw) > 32 * 1024 * 1024:
                        raise RuntimeError('Source index exceeds size limit')
                    original = {'url': url, 'sha256': hashlib.sha256(raw).hexdigest()}
                    cached.write_bytes(raw)
                    metadata.write_text(json.dumps(original, indent=2) + '\n')
                    break
                except (urllib.error.URLError, TimeoutError):
                    if url == urls[-1]:
                        raise
        parsed = parse_source_index(lzma.decompress(raw).decode())
        for identity, record in parsed.items():
            record['archive'] = archive
            record['source_index'] = original
            indices[identity] = record
        provenance.append(original)
    missing = sorted(required - indices.keys())
    for name, version in missing:
        existing = [item for identity, item in indices.items() if identity[0] == name]
        if not existing:
            raise RuntimeError('Exact Debian source versions unavailable: ' + repr((name, version)))
        indexed = existing[0]
        record = source_control_record(name, version, indexed['directory'], indexed['archive'], pin, cache)
        indices[name, version] = record
        provenance.append(record['source_index'])
    return [indices[identity] for identity in sorted(required)], provenance


def write_source_archive(output):
    archive_path = output.parent / (output.name + '.tar.gz')
    if archive_path.exists():
        raise RuntimeError('Source archive already exists')
    with archive_path.open('wb') as raw, gzip.GzipFile(fileobj=raw, mode='wb', filename='', mtime=0) as compressed:
        with tarfile.open(fileobj=compressed, mode='w|') as archive:
            for path in [output, *sorted(output.rglob('*'))]:
                if path.is_symlink() or not (path.is_file() or path.is_dir()):
                    raise RuntimeError('Source archive contains unsupported file')
                info = archive.gettarinfo(str(path), arcname=(Path(output.name) / path.relative_to(output)).as_posix())
                info.uid = info.gid = info.mtime = 0
                info.uname = info.gname = ''
                if path.is_file():
                    with path.open('rb') as stream:
                        archive.addfile(info, stream)
                else:
                    archive.addfile(info)
    return archive_path


def build_sources(runtime_dir, output, cache):
    manifest = runtime.validate_runtime(runtime_dir)
    pin = json.loads((Path(runtime_dir) / 'pin.json').read_text())
    output, cache = Path(output).absolute(), Path(cache).absolute()
    if output.exists() or (output.parent / (output.name + '.tar.gz')).exists():
        raise RuntimeError('Choose a new source output directory')
    if output == cache or output in cache.parents or cache in output.parents:
        raise RuntimeError('Source cache and output must not overlap')
    cache.mkdir(parents=True, exist_ok=True)
    subprocess.run(['docker', 'image', 'load', '--input', str(Path(runtime_dir) / 'image.tar')], check=True)
    packages = package_inventory(manifest['image_id'])
    build_packages = host_build_inventory(pin)
    # These development packages contribute headers or static runtime code to
    # the host. General build tools themselves need not be redistributed here.
    incorporated = incorporated_host_packages(build_packages)
    required = required_sources(packages + incorporated)
    plan, index_provenance = source_plan(required, pin, cache)
    print(f'Collecting {len(plan)} exact Debian source packages; '
          f'{sum(item["size"] for package in plan for item in package["files"])} bytes', flush=True)
    tasks = []
    for package in plan:
        for item in package['files']:
            if item['size'] > 512 * 1024 * 1024:
                raise RuntimeError('Selected source package exceeds download limit')
            item['url'] = f'https://deb.debian.org/{package["archive"]}/{package["directory"]}/{item["name"]}'
            tasks.append((package, item))
    def fetch(task):
        package, item = task
        target = cache / 'files' / package['archive'] / package['directory'] / item['name']
        try:
            download_verified(item['url'], target, item['size'], item['sha256'])
        except urllib.error.HTTPError as error:
            if error.code != 404:
                raise
            item['url'] = f'https://snapshot.debian.org/archive/{package["archive"]}/{pin["debian_snapshot"]}/{package["directory"]}/{item["name"]}'
            download_verified(item['url'], target, item['size'], item['sha256'])
        return target
    with ThreadPoolExecutor(max_workers=4) as pool:
        downloaded = list(pool.map(fetch, tasks))
    output.mkdir(parents=True)
    for (package, item), source in zip(tasks, downloaded):
        target = output / 'debian-sources' / package['archive'] / package['directory'] / item['name']
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)
    for name, source in runtime.build_context_files().items():
        target = output / 'host-build-context' / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)
    recipes = output / 'recipes'
    recipes.mkdir()
    for name in ('build_runtime.py', 'build_sources.py', 'pin.json'):
        shutil.copyfile(runtime.HERE / name, recipes / name)
    (output / 'runtime-manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    (output / 'README.txt').write_text(f'''Corresponding source for the Tyr Bambu Studio cloud helper runtime

Runtime image: {manifest['image_id']}
Host source context SHA256: {manifest['host_source_sha256']}

This archive contains the complete vendored/patched helper host source, its
AGPL license, build recipe, and exact Debian source packages for all installed
runtime packages plus statically incorporated dependencies declared through
Built-Using and the host's development/header libraries. No proprietary Bambu
plugin binaries or sources are included. The application's source is published
in the same release's BambuStudio repository source archive. This archive is
limited to the Linux helper runtime; it does not claim to contain every native
macOS slicer dependency. Native dependency definitions/build recipes are in
the repository's deps/CMakeLists.txt, deps subdirectories, and
scripts/build_tyr_macos_runtime.sh (including the pinned zstd/Imath sources).

Each Debian .dsc and its associated orig/debian tarballs reside together under
debian-sources. On Debian, unpack using dpkg-source -x <package>.dsc. Debian
copyright/license texts and build rules are present in each source package.
The source-manifest.json records binary/source versions, Built-Using metadata,
source archive URLs, SHA256 digests and the complete output file inventory.

To build the helper runtime image, from host-build-context run:
  docker build --platform linux/arm64 \\
    --build-arg DEBIAN_BASE='{pin['debian_base']}' \\
    --build-arg DEBIAN_SNAPSHOT='{pin['debian_snapshot']}' .

The Dockerfile uses the pinned Debian snapshot. The corresponding source files
included here were verified against SHA256 hashes from official Debian source
indexes and exact-version source controls, and match the incorporated binary
packages' source version identities (including older cross-toolchain CRT and
header inputs).
''')
    source_manifest = {'schema_version': 1, 'runtime_image': manifest['image_id'],
                       'host_source_sha256': manifest['host_source_sha256'],
                       'runtime_packages': packages, 'host_build_packages': build_packages,
                       'host_incorporated_packages': incorporated, 'source_indices': index_provenance,
                       'source_packages': plan,
                       'hashes': {name: runtime.sha256_file(path) for name, path in runtime.file_inventory(output).items()}}
    (output / 'source-manifest.json').write_text(json.dumps(source_manifest, indent=2) + '\n')
    archive = write_source_archive(output)
    checksum = runtime.sha256_file(archive)
    (archive.parent / (archive.name + '.sha256')).write_text(f'{checksum}  {archive.name}\n')
    print(f'Corresponding source archive: {archive}\nSHA256: {checksum}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runtime', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--cache', type=Path, required=True)
    args = parser.parse_args()
    build_sources(args.runtime, args.output, args.cache)


if __name__ == '__main__':
    main()
