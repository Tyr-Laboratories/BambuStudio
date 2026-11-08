#!/usr/bin/env python3
"""Package exact native macOS dependency sources without building or running apps.

Read the app's existing Ninja metadata, verified ExternalProject downloads and
clean wxWidgets Git checkouts. Download only explicitly pinned public supplemental
sources/notices. The application source is supplied by the matching public release
tag, separately from this archive. No workspace, account data or binaries are
recursively copied. See the generated README for scope and reconstruction steps.
"""
import argparse
import gzip
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import shlex
import shutil
import subprocess
import tarfile
import urllib.parse
import urllib.request
import zipfile


REPOSITORY = 'https://github.com/Tyr-Laboratories/BambuStudio'
MINIZIP_SHA256 = 'bb329a0a2cd0274d05519d61c667c062e06990d72e125ee2dfa8de64f0119d16'
MINIZIP_URL = 'https://zlib.net/fossils/zlib-1.3.2.tar.gz'
LINKED = {'Assimp', 'Blosc', 'Boost', 'CURL', 'FREETYPE', 'GLFW', 'GMP',
          'JPEG', 'MPFR', 'NLopt', 'OCCT', 'OpenCV', 'OpenMeshCraft',
          'OpenSSL', 'OpenVDB', 'PNG', 'TBB', 'TIFF', 'libnoise'}
DOWNLOADED = LINKED | {'CGAL', 'Cereal', 'FFMPEG', 'OpenCSG', 'OpenEXR', 'Qhull'}
PUBLIC_SOURCE_HOSTS = {'github.com', 'raw.githubusercontent.com', 'codeload.github.com',
                       'gitlab.com', 'www.mpfr.org', 'download.osgeo.org', 'zlib.net'}
STATIC_SOURCES = {
    'GLEW': 'GLEW', 'OpenMeshCraft': 'OpenMeshCraft', 'OpenMeshCraftBoolean': 'OpenMeshCraft',
    'shewchuk_predicates': 'OpenMeshCraft', 'assimp': 'Assimp', 'blosc': 'Blosc',
    'crypto': 'OpenSSL', 'ssl': 'OpenSSL', 'curl': 'CURL', 'expat': 'EXPAT',
    'freetype': 'FREETYPE', 'glfw3': 'GLFW', 'gmp': 'GMP', 'gmpxx': 'GMP',
    'hpdf': 'libharu', 'jpeg': 'JPEG', 'libnoise_static': 'libnoise', 'mpfr': 'MPFR',
    'nlopt': 'NLopt', 'opencv_world': 'OpenCV', 'tegra_hal': 'OpenCV',
    'libpng': 'OpenCV', 'libtiff': 'OpenCV', 'openvdb': 'OpenVDB',
    'png': 'PNG', 'tbb': 'TBB', 'tbbmalloc': 'TBB', 'tiff': 'TIFF',
}
LICENSE_NAME = re.compile(r'(?i)(licen[cs]e|licendoc|copying|copyright|notice|'
                          r'authors|contributors|gpl|lgpl|agpl|ftl)(?:[._-].*)?$')
EXTRA_NOTICE_NAMES = {'README.ijg', 'OCCT_LGPL_EXCEPTION.txt',
                      'NLopt_License_and_Copyright.md'}
SUPPLEMENTAL_NOTICES = [
    ('Blosc-zstd-1.4.1', 'LICENSE', 'https://raw.githubusercontent.com/facebook/zstd/v1.4.1/LICENSE',
     '2c1a7fa704df8f3a606f6fc010b8b5aaebf403f3aeec339a12048f1ba7331a0b'),
    ('Blosc-zstd-1.4.1', 'COPYING', 'https://raw.githubusercontent.com/facebook/zstd/v1.4.1/COPYING',
     'f9c375a1be4a41f7b70301dd83c91cb89e41567478859b77eef375a52d782505'),
    ('HIDAPI-0.9.0', 'LICENSE.txt',
     'https://raw.githubusercontent.com/libusb/hidapi/7da5cc91fc0d2dbe4df4f08cd31f6ca1a262418f/LICENSE.txt',
     '7d3b087c34f35d4d538e3bcddd1ff8f66e92f9ef336881999482800ddf840913'),
    ('HIDAPI-0.9.0', 'LICENSE-gpl3.txt',
     'https://raw.githubusercontent.com/libusb/hidapi/7da5cc91fc0d2dbe4df4f08cd31f6ca1a262418f/LICENSE-gpl3.txt',
     '8ceb4b9ee5adedde47b31e975c1d90c73ad27b6b165a1dcd80c7c545eb65b903'),
    ('HIDAPI-0.9.0', 'LICENSE-bsd.txt',
     'https://raw.githubusercontent.com/libusb/hidapi/7da5cc91fc0d2dbe4df4f08cd31f6ca1a262418f/LICENSE-bsd.txt',
     '30eb1bef29b46f8ba7ab8b416035dbd93cb034a45481dd97815b944284582cd2'),
    ('HIDAPI-0.9.0', 'LICENSE-orig.txt',
     'https://raw.githubusercontent.com/libusb/hidapi/7da5cc91fc0d2dbe4df4f08cd31f6ca1a262418f/LICENSE-orig.txt',
     'fb5436aa63d1b71a8dfbf74ecaf1a5b4e1ec4df7f80074d11fec99284f69ca5f'),
    ('Lucene-4.5.1-release', 'LICENSE.txt',
     'https://raw.githubusercontent.com/apache/lucene-solr/62ffee2bba5d2b46ff619d385eb67a8b7910c7fd/lucene/LICENSE.txt',
     'aeed74bce08f5c6219a22997e6d75945e4eb81b313502dd690f0fcb775c1dc8b'),
    ('Lucene-4.5.1-release', 'NOTICE.txt',
     'https://raw.githubusercontent.com/apache/lucene-solr/62ffee2bba5d2b46ff619d385eb67a8b7910c7fd/lucene/NOTICE.txt',
     'c7a2d579420a6479f6ab6553581dafd735da908f50da71b432aaa674965a852c'),
    ('semver-0.2.0', 'LICENSE',
     'https://raw.githubusercontent.com/h2non/semver.c/89e66f36544e0250def32640b84b7e15c8585da4/LICENSE',
     '85ffa04d14965497779ceb017229dae1448f470c62db422c97a14a4ce66c95b7'),
    ('shared_ptr-2019-notice', 'LICENSE.txt',
     'https://raw.githubusercontent.com/SRombauts/shared_ptr/6aa0b4498e35afe849d7a2a135fa203b28f39c89/LICENSE.txt',
     '36d9aaf88cad805aeebcebe54e4482822ef9ff968f5299d90b26fae4853a364b'),
]


def run(*args):
    return subprocess.check_output([str(arg) for arg in args], text=True).strip()


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as source:
        for block in iter(lambda: source.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def public_url(value):
    url = urllib.parse.urlparse(value)
    if (url.scheme != 'https' or url.hostname not in PUBLIC_SOURCE_HOSTS
            or url.username or url.password or url.port or url.query or url.fragment):
        raise RuntimeError('Source URL must use an approved public HTTPS source host without credentials')
    return value


def parse_urlinfo(text):
    values = dict(line.split('=', 1) for line in text.splitlines() if '=' in line)
    checksum = values.get('hash', '')
    if not re.fullmatch(r'SHA256=[0-9a-fA-F]{64}', checksum):
        raise RuntimeError('Dependency source must have a SHA256 pin')
    # MPFR metadata records an HTTP mirror before its authoritative HTTPS URL.
    urls = [url for url in values.get('url(s)', '').split(';') if url.startswith('https:')]
    if not urls:
        raise RuntimeError('Dependency source has no public HTTPS URL')
    return [public_url(url) for url in urls], checksum.split('=')[1].lower()


def copy_verified(source, target, expected):
    if sha256_file(source) != expected.lower():
        raise RuntimeError(f'Source checksum mismatch: {Path(source).name}')
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source, target)


def validate_recipe_pin(urls, checksum, recipe):
    if checksum.lower() not in recipe.lower() or any(url not in recipe for url in urls):
        raise RuntimeError('Cached source metadata does not match the committed public recipe')


def download_verified(url, target, expected):
    public_url(url)
    if target.exists():
        if sha256_file(target) != expected:
            raise RuntimeError(f'Cached source checksum mismatch: {target.name}')
        return
    target.parent.mkdir(parents=True, exist_ok=True)
    partial = target.with_name(target.name + '.partial')
    try:
        with urllib.request.urlopen(url, timeout=60) as source, partial.open('wb') as out:
            shutil.copyfileobj(source, out)
        if sha256_file(partial) != expected:
            raise RuntimeError(f'Downloaded source checksum mismatch: {target.name}')
        partial.replace(target)
    finally:
        partial.unlink(missing_ok=True)


def archive_git_source(repo, target, prefix):
    if run('git', '-C', repo, 'status', '--porcelain', '--untracked-files=no'):
        raise RuntimeError(f'Tracked source edits prevent exact snapshot: {Path(repo).name}')
    revision = run('git', '-C', repo, 'rev-parse', 'HEAD')
    target.parent.mkdir(parents=True, exist_ok=True)
    # Git's archive allowlist excludes all untracked files and .git metadata.
    with target.open('wb') as raw, gzip.GzipFile(filename='', fileobj=raw, mode='wb', mtime=0) as out:
        process = subprocess.Popen(['git', '-C', str(repo), 'archive', '--format=tar',
                                    '--prefix=' + prefix, revision], stdout=subprocess.PIPE)
        shutil.copyfileobj(process.stdout, out)
        process.stdout.close()
        if process.wait():
            raise RuntimeError('Git source archive failed')
    return revision


def write_archive(stage):
    paths = [stage] + sorted(stage.rglob('*'))
    if any(path.is_symlink() or not (path.is_file() or path.is_dir()) for path in paths):
        raise RuntimeError('Source staging must contain only regular files and directories')
    archive = stage.with_name(stage.name + '.tar.gz')
    temporary = archive.with_name(archive.name + '.partial')
    with temporary.open('wb') as raw, gzip.GzipFile(filename='', fileobj=raw, mode='wb',
                                                mtime=0, compresslevel=6) as compressed:
        with tarfile.open(fileobj=compressed, mode='w') as tar:
            for path in paths:
                info = tar.gettarinfo(path, arcname=path.relative_to(stage.parent).as_posix())
                info.uid = info.gid = info.mtime = 0
                info.uname = info.gname = ''
                info.mode = 0o755 if path.is_dir() else 0o644
                info.pax_headers = {}
                if path.is_file():
                    with path.open('rb') as source:
                        tar.addfile(info, source)
                else:
                    tar.addfile(info)
    temporary.replace(archive)
    return archive


def notice_path(name):
    path = PurePosixPath(name)
    return (bool(LICENSE_NAME.fullmatch(path.name)) or path.name in EXTRA_NOTICE_NAMES
            or 'LICENSES' in path.parts
            or (path.name == 'README' and any(part in ('bdf', 'pcf', 'algs') for part in path.parts))
            or name.endswith('/zstd-1.4.1/common/xxhash.h') or name.endswith('/src/gzip/zlib.h'))


def collect_archive_notices(archive, output):
    """Copy notice members directly; never extract archive paths or follow links."""
    def save(name, data):
        path = PurePosixPath(name)
        if path.is_absolute() or '..' in path.parts or '\\' in name:
            raise RuntimeError('Unsafe source archive member path')
        if not notice_path(name) or len(data) > 2 * 1024 * 1024:
            return
        target = output / path
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
    if zipfile.is_zipfile(archive):
        with zipfile.ZipFile(archive) as source:
            for member in source.infolist():
                if not member.is_dir() and member.file_size <= 2 * 1024 * 1024 and notice_path(member.filename):
                    save(member.filename, source.read(member))
    else:
        with tarfile.open(archive) as source:
            for member in source:
                if member.isfile() and member.size <= 2 * 1024 * 1024 and notice_path(member.name):
                    save(member.name, source.extractfile(member).read())


def link_inventory(build, repo):
    blocks = (build / 'build.ninja').read_text().split('\n\n')
    for block in blocks:
        lines = block.splitlines()
        # CMake can emit either a bundle or a raw executable which the public
        # app packager subsequently stages into its bundle.
        if (lines and lines[0].startswith(('build src/BambuStudio:',
                                          'build src/BambuStudio.app/Contents/MacOS/BambuStudio:'))
                and 'CXX_EXECUTABLE_LINKER' in lines[0]):
            libraries = next(line.split('=', 1)[1] for line in lines if 'LINK_LIBRARIES =' in line)
            break
    else:
        raise RuntimeError('Native application link target missing from build metadata')
    result = {'dependency_prefix_static_libraries': [], 'repository_static_libraries': [],
              'external_static_libraries': [], 'runtime_libraries_replaced_by_packager': [],
              'system_libraries': [], 'system_frameworks': [], 'linked_source_groups': []}
    groups = set()
    tokens = shlex.split(libraries)
    result['system_frameworks'] = sorted({tokens[i + 1] for i, item in enumerate(tokens[:-1])
                                          if item in ('-framework', '-weak_framework')})
    for item in sorted(set(tokens)):
        if item.endswith('.a'):
            if '/BambuStudio_deps/usr/local/' in item:
                result['dependency_prefix_static_libraries'].append(item.split('/usr/local/', 1)[1])
                name = Path(item).name.removeprefix('lib').removesuffix('.a')
                provider = ('Boost' if name.startswith('boost_') else
                            'OCCT' if name.startswith('TK') else
                            'wxWidgets' if name.startswith('wx') else STATIC_SOURCES.get(name))
                if not provider:
                    raise RuntimeError('No source mapping for linked dependency: ' + name)
                groups.add(provider)
            elif item.startswith('src/'):
                result['repository_static_libraries'].append(item)
            elif '/minizip/1.3.2_1/' in item:
                result['external_static_libraries'].append('Homebrew minizip 1.3.2 revision 1: libminizip.a')
                groups.add('minizip')
            else:
                raise RuntimeError('Unrecognized external static dependency: ' + Path(item).name)
        elif item.endswith('.dylib'):
            if Path(item).name not in ('libzstd.1.5.7.dylib', 'libImath-3_2.30.3.2.3.dylib'):
                raise RuntimeError('Unrecognized external runtime dependency: ' + Path(item).name)
            result['runtime_libraries_replaced_by_packager'].append(Path(item).name)
        elif item.startswith('-lwx'):
            result['dependency_prefix_static_libraries'].append('lib/lib' + item[2:] + '.a')
            groups.add('wxWidgets')
        elif item.endswith('.tbd') or item.startswith('-l'):
            result['system_libraries'].append(Path(item).name)
    result['linked_source_groups'] = sorted(groups)
    return result


def source_record(stage, name, target, urls, role, **extra):
    return {'name': name, 'file': target.relative_to(stage).as_posix(),
            'sha256': sha256_file(target), 'size': target.stat().st_size,
            'urls': urls, 'role': role, **extra}


def boost_patch_observation(deps):
    """Verify the known difference between the declared patch and built headers."""
    headers = ('posix/executor.hpp', 'posix/pipe_out.hpp', 'used_handles.hpp', 'windows/handles.hpp')
    source_base = deps / 'dep_Boost-prefix/src/dep_Boost/libs/process/include/boost/process/detail'
    installed_base = deps / 'BambuStudio_deps/usr/local/include/boost/process/detail'
    receipt = {}
    with tarfile.open(deps / 'Boost/boost-1.84.0.tar.gz') as archive:
        for relative in headers:
            original = archive.extractfile('boost-1.84.0/libs/process/include/boost/process/detail/' + relative).read()
            if (source_base / relative).read_bytes() != original or (installed_base / relative).read_bytes() != original:
                raise RuntimeError('Boost patch-state observation changed; review the native source reconstruction note')
            receipt[relative] = hashlib.sha256(original).hexdigest()
    return {'dependency': 'Boost', 'state': 'unpatched upstream headers observed in extracted and installed trees',
            'declared_but_unapplied_patch': 'deps/Boost/0001-FIX-OBS-cannot-start-streaming-on-MAC.patch',
            'reconstruction': 'Omit this Boost PATCH_COMMAND to match the observed native build headers',
            'header_sha256': receipt}


def package(args):
    repo = Path(__file__).resolve().parents[1]
    deps = args.deps.resolve()
    stage = args.output.resolve()
    if stage.exists():
        raise RuntimeError('Output directory already exists; choose a new source artifact directory')
    stage.mkdir(parents=True)
    application_revision = run('git', '-C', repo, 'rev-parse', 'HEAD')
    records = []
    for info in sorted(deps.glob('dep_*-prefix/src/*-stamp/*-urlinfo.txt')):
        name = info.parent.name.removeprefix('dep_').removesuffix('-stamp')
        if name not in DOWNLOADED:
            raise RuntimeError('Unexpected cached dependency outside the public source allowlist: ' + name)
        urls, expected = parse_urlinfo(info.read_text())
        committed_recipe = subprocess.check_output(
            ['git', '-C', str(repo), 'show', application_revision + f':deps/{name}/{name}.cmake'], text=True)
        validate_recipe_pin(urls, expected, committed_recipe)
        candidates = [path for path in (deps / name).iterdir() if path.is_file()]
        if len(candidates) != 1:
            raise RuntimeError('Expected one source archive for ' + name)
        target = stage / 'sources' / name / candidates[0].name
        copy_verified(candidates[0], target, expected)
        role = ('linked static library source' if name in LINKED else
                'header dependency' if name in ('CGAL', 'Cereal') else
                'dependency build input; no distinct library in app link metadata')
        records.append(source_record(stage, name, target, urls, role,
                                     recipe=f'recipes/deps/{name}/{name}.cmake'))
    missing = LINKED.union({'CGAL', 'Cereal'}) - {record['name'] for record in records}
    if missing:
        raise RuntimeError('Missing native dependency sources: ' + ', '.join(sorted(missing)))

    omc = deps / 'dep_OpenMeshCraft-prefix/src/dep_OpenMeshCraft'
    recipe = subprocess.check_output(
        ['git', '-C', str(repo), 'show', application_revision + ':deps/OpenMeshCraft/OpenMeshCraft.cmake'], text=True)
    for url, checksum, folder in re.findall(
            r'omc_fetch_zip\(\s*"([^"]+)"\s*"([0-9A-Fa-f]{64})"\s*"[^"\n]+"\s*"([^"]+)"', recipe):
        target = stage / 'sources/OpenMeshCraft-nested' / (folder + '.zip')
        copy_verified(omc / '.omc_fetch' / target.name, target, checksum)
        records.append(source_record(stage, 'OpenMeshCraft/' + folder, target,
                                     [public_url(url)], 'nested dependency source',
                                     recipe='recipes/deps/OpenMeshCraft/OpenMeshCraft.cmake'))

    wx = deps / 'dep_wxWidgets-prefix/src/dep_wxWidgets'
    target = stage / 'sources/wxWidgets/wxWidgets.tar.gz'
    revision = archive_git_source(wx, target, 'wxWidgets/')
    wx_commit = revision
    records.append(source_record(stage, 'wxWidgets', target,
                                 ['https://github.com/bambulab/wxWidgets'], 'linked static library source',
                                 commit=revision, restore_path='.',
                                 recipe='recipes/deps/wxWidgets/wxWidgets.cmake'))
    submodules = run('git', '-C', wx, 'submodule', 'status', '--recursive').splitlines()
    for line in submodules:
        fields = line.strip().split()
        revision, relative = fields[:2]
        if line[:1] in ('+', '-', 'U'):
            raise RuntimeError('wxWidgets submodule checkout does not match its pin')
        sub = wx / relative
        url = public_url(run('git', '-C', sub, 'remote', 'get-url', 'origin'))
        target = stage / 'sources/wxWidgets' / (relative.replace('/', '-') + '.tar.gz')
        actual = archive_git_source(sub, target, relative + '/')
        if actual != revision:
            raise RuntimeError('wxWidgets submodule source revision mismatch')
        records.append(source_record(stage, 'wxWidgets/' + relative, target, [url],
                                     'wxWidgets pinned submodule; system codecs selected in this build',
                                     commit=revision, restore_path=relative))

    for name, filename, url, checksum in (
            ('zstd', 'zstd-1.5.7.tar.gz', 'https://github.com/facebook/zstd/releases/download/v1.5.7/zstd-1.5.7.tar.gz',
             'eb33e51f49a15e023950cd7825ca74a4a2b43db8354825ac24fc1b7ee09e6fa3'),
            ('Imath', 'Imath-v3.2.3.tar.gz', 'https://codeload.github.com/AcademySoftwareFoundation/Imath/tar.gz/refs/tags/v3.2.3',
             'e10c12b3f21f45bf08e09d4215d9c7691368d747beebd840de0b6fefed2df9f8')):
        target = stage / 'sources' / name / filename
        copy_verified(args.runtime / 'downloads' / filename, target, checksum)
        records.append(source_record(stage, name, target, [url], 'packaged runtime dylib source',
                                     recipe='recipes/scripts/build_tyr_macos_runtime.sh'))

    target = stage / 'sources/minizip/zlib-1.3.2.tar.gz'
    download_verified(MINIZIP_URL, target, MINIZIP_SHA256)
    formula = args.minizip_formula.read_text()
    if MINIZIP_SHA256 not in formula or 'revision 1' not in formula:
        raise RuntimeError('Installed minizip recipe does not match the linked source version')
    recipe_target = stage / 'recipes/Homebrew/minizip.rb'
    recipe_target.parent.mkdir(parents=True)
    recipe_target.write_text(formula)
    records.append(source_record(stage, 'minizip 1.3.2 revision 1', target, [MINIZIP_URL],
                                 'linked static library source (contrib/minizip)',
                                 recipe='recipes/Homebrew/minizip.rb'))

    # Snapshot committed, public native dependency source/recipes only. Main app
    # sources and the independently packaged Linux helper are supplied separately.
    tracked = run('git', '-C', repo, 'ls-files', '-z', 'deps', 'cmake', 'CMakeLists.txt',
                  'LICENSE', 'BuildMac.sh', 'CMakePresets.json', 'version.inc',
                  'scripts/build_tyr_macos_runtime.sh').split('\0')
    skipped = []
    for relative in tracked:
        if not relative or relative.startswith('deps/tyr_cloud_helper/'):
            continue
        if Path(relative).suffix.lower() in ('.a', '.lib', '.dll', '.dylib', '.so', '.exe', '.o', '.obj', '.res', '.pdb'):
            skipped.append(relative)
            continue
        target = stage / 'recipes' / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(subprocess.check_output(['git', '-C', str(repo), 'show', application_revision + ':' + relative]))
    shutil.copyfile(__file__, stage / 'recipes/package_tyr_native_sources.py')
    (stage / 'LICENSES').mkdir(exist_ok=True)
    shutil.copyfile(stage / 'recipes/LICENSE', stage / 'LICENSES/BambuStudio-AGPL-3.0.txt')

    for record in records:
        collect_archive_notices(stage / record['file'], stage / 'LICENSES' / record['name'].replace('/', '__'))
    for path in (stage / 'recipes/deps').rglob('*'):
        if path.is_file() and notice_path(path.as_posix()):
            target = stage / 'LICENSES/repository-deps' / path.relative_to(stage / 'recipes/deps')
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(path, target)
    # Complete notices referenced by upstream files but omitted by vendor copies.
    for group, name, url, checksum in SUPPLEMENTAL_NOTICES:
        target = stage / 'LICENSES' / group / name
        download_verified(url, target, checksum)
        records.append(source_record(stage, group + '/' + name, target, [url],
                                     'supplemental upstream notice; release-wide text may mention unused components'))
    inline_notice_sources = {
        'src/ankerl/unordered_dense.h', 'src/fast_float/fast_float.h',
        'src/glu-libtess/src/geom.h', 'src/Shiny/ShinyManager.c',
        'src/slic3r/GUI/fts_fuzzy_match.h', 'src/libslic3r/FilamentMixerModel.hpp',
        'src/hidapi/mac/hid.c', 'src/semver/semver.h', 'src/libslic3r/clonable_ptr.hpp',
        'src/slic3r/Utils/ASCIIFolding.cpp', 'src/imguizmo/ImGuizmo.h',
        'src/tinybvh/tiny_bvh.h',
    }
    for relative in run('git', '-C', repo, 'ls-files', '-z', 'src').split('\0'):
        if relative and (notice_path(relative) or relative in inline_notice_sources
                         or ('nanosvg' in relative.lower() and relative.endswith('.h'))
                         or ('nlohmann' in relative and relative.endswith('.hpp'))
                         or (Path(relative).name.startswith(('stb_', 'imstb_')) and relative.endswith('.h'))):
            target = stage / 'LICENSES/repository-src' / relative.removeprefix('src/')
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(subprocess.check_output(['git', '-C', str(repo), 'show', application_revision + ':' + relative]))
    inventory = {'schema': 1, 'release': args.release, 'application_source_repository': REPOSITORY,
                 'application_source_release': REPOSITORY + '/releases/tag/' + args.release,
                 'recipe_snapshot_revision': application_revision, 'architecture': 'arm64', 'minimum_macos': '14.0',
                 'source_archives_and_supplemental_notices': records,
                 'application_link_inventory': link_inventory(args.build, repo),
                 'observed_recipe_exceptions': [boost_patch_observation(deps)],
                 'repository_dependency_sources': ['deps/EXPAT/expat', 'deps/GLEW/glew', 'deps/libharu/libharu'],
                 'omitted_tracked_non_macos_binary_files': skipped}
    (stage / 'source-inventory.json').write_text(json.dumps(inventory, indent=2) + '\n')
    (stage / 'README.txt').write_text(f'''Tyr Bambu Studio native macOS dependency sources - {args.release}

Publish this archive next to the arm64 application and matching application
source tag at {REPOSITORY}/releases/tag/{args.release}.
The application and its in-tree third-party source are in that public tag.
The Linux cloud-helper runtime has its own corresponding-source archive.

CONTENTS AND EVIDENCE
sources/ contains the exact checksum-verified cached dependency archives, clean
Git source snapshots for wxWidgets and its pinned submodules, the Homebrew
minizip source, and the two locally built runtime dylib sources. source-inventory.json
records public URLs, SHA256, archive sizes, source role and the app link inventory.
Recipes include the public repository dependency patches/overlays and build
instructions at commit {application_revision}, plus the installed minizip formula. No
application binaries, local build trees, account data, profiles or projects are
copied. Original upstream source archives remain byte-for-byte unchanged and can
themselves contain upstream test fixtures or platform artifacts.

LICENSES/ copies upstream notices for convenient access; their original copies
and inline copyright notices remain in the source archives. Blosc embeds zstd
1.4.1, separate from the packaged zstd 1.5.7 dylib. Its omitted license texts are
supplied from the exact upstream tag. Presence of an auxiliary source archive is
not evidence that its library was linked. In particular, FFMPEG, OpenCSG,
OpenEXR and the downloaded Qhull are recorded as build inputs; the app links the
repository's own Qhull library. CGAL and Cereal are header dependencies. EXPAT,
GLEW and libharu are tracked directly in recipes/deps and in the public tag.
HIDAPI notices match the bundled 0.9.0 commit. Lucene 4.5.1 LICENSE/NOTICE are
verbatim release documents for the ASCII-folding port and mention other upstream
components; their inclusion does not claim all those components are incorporated.
The semver 0.2.0 and shared_ptr 2019 supplemental MIT notices retain upstream
terms, while the repository's actual source and copyright headers remain the
authority for the bundled derivatives. Their exact historical copied commits
have not been established; the supplement does not assert such a match.

RECONSTRUCTING THE BUILD
Start from the matching public application tag and use its macOS build instructions.
Use the dependency recipes/overlays in recipes/deps, with the observed patch-state
exceptions recorded below; the archived upstream sources plus these recipes make
the source choices reviewable. Applying every declared patch without checking the
recorded state is not a claim to reproduce the existing build.
The declared Boost OBS process patch was not reflected in any of its four
affected extracted or installed headers: both trees exactly match the unpatched
Boost 1.84.0 source archive. Omit that Boost PATCH_COMMAND to match this observed
build. source-inventory.json records the four header checksums. The GMP and CGAL
patches were observed applied in the existing source trees. This package does
not change the installed dependencies or rebuild the application.
Copy each top-level dependency archive into deps/build/arm64/<dependency>/<filename>
before configuring deps with arm64 and CMAKE_OSX_DEPLOYMENT_TARGET=14.0. Preserve
the SHA256 checks from the recipes. The cached dependencies used Release; the app
used RelWithDebInfo, SLIC3R_STATIC=ON, SLIC3R_GUI=ON,
SLIC3R_ENABLE_OPENMESHCRAFT_BOOLEAN=ON and SLIC3R_ALLOW_MCUT_BOOLEAN=OFF.

The original wxWidgets recipe names a moving master branch. Set its GIT_TAG to
{wx_commit} before configuring. The wxWidgets archive contains that exact tree;
extract all six submodule archives from the wxWidgets root: each archive already
contains its restore_path prefix. To consume these archives without cloning, replace GIT_REPOSITORY
and GIT_TAG in the wxWidgets ExternalProject recipe with SOURCE_DIR pointing to
the restored tree, DOWNLOAD_COMMAND "" and UPDATE_COMMAND "". Setting SOURCE_DIR
alone while retaining the Git download method can delete and reclone that tree.
OpenMeshCraft's four nested archives belong in its extracted .omc_fetch directory;
the pinned prepare_source.cmake overlay describes their destination directories.

Build minizip from sources/minizip/zlib-1.3.2.tar.gz, contrib/minizip, using the
included formula recipe; the app originally linked Homebrew 1.3.2 revision 1.
The source archive was retrieved from zlib's fossils mirror with the identical
SHA256 recorded in that installed formula. System macOS libraries and Xcode are
build/runtime prerequisites, not bundled source. The runtime dylibs are built by
recipes/scripts/build_tyr_macos_runtime.sh from the included zstd/Imath archives.

This inventory describes source access and recorded build inputs. It does not
claim a clean-machine rebuild, bit-identical output or comprehensive legal review.
Keep the main application source, this archive, and Linux runtime sources
available alongside each release; do not substitute a later branch for its tag.
''')
    files = sorted(path for path in stage.rglob('*') if path.is_file())
    (stage / 'SHA256SUMS.txt').write_text(''.join(
        f'{sha256_file(path)}  {path.relative_to(stage).as_posix()}\n' for path in files))
    archive = write_archive(stage)
    digest = sha256_file(archive)
    archive.with_name(archive.name + '.sha256').write_text(f'{digest}  {archive.name}\n')
    print(json.dumps({'archive': str(archive), 'size': archive.stat().st_size,
                      'sha256': digest, 'source_records': len(records),
                      'license_files': sum(path.is_file() for path in (stage / 'LICENSES').rglob('*'))}, indent=2))


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--release', required=True)
    parser.add_argument('--deps', type=Path, default=root / 'deps/build/arm64')
    parser.add_argument('--build', type=Path, default=root / 'build/wall-painting')
    parser.add_argument('--runtime', type=Path, default=root / 'build/tyr-runtime')
    parser.add_argument('--minizip-formula', type=Path,
                        default=Path('/opt/homebrew/Cellar/minizip/1.3.2_1/.brew/minizip.rb'))
    args = parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9._-]+', args.release):
        parser.error('release must be a safe public tag name')
    package(args)


if __name__ == '__main__':
    main()
