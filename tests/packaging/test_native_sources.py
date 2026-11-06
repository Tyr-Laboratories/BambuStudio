"""Native release sources must preserve pins and exclude workstation material."""
import hashlib
import importlib.util
import io
from pathlib import Path
import subprocess
import tarfile
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().parents[2] / 'scripts/package_tyr_native_sources.py'


class NativeSources(unittest.TestCase):
    def setUp(self):
        self.assertTrue(SCRIPT.is_file(), 'native source packager must exist')
        spec = importlib.util.spec_from_file_location('native_sources', SCRIPT)
        self.sources = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.sources)
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def test_urlinfo_requires_pinned_public_https_sources(self):
        text = 'url(s)=https://github.com/public/source.tar.gz\nhash=SHA256=' + 'a' * 64 + '\n'
        self.assertEqual(self.sources.parse_urlinfo(text),
                         (['https://github.com/public/source.tar.gz'], 'a' * 64))
        for changed in (text.replace('SHA256', 'MD5'), text.replace('https:', 'file:'),
                        text.replace('github.com', 'name:secret@github.com'),
                        text.replace('github.com', '127.0.0.1'),
                        text.replace('github.com', 'private.example'),
                        text.replace('.tar.gz', '.tar.gz?token=secret')):
            with self.subTest(changed=changed), self.assertRaises(RuntimeError):
                self.sources.parse_urlinfo(changed)

    def test_cached_source_metadata_must_match_committed_recipe_pins(self):
        url = 'https://github.com/public/source.tar.gz'
        recipe = 'URL ' + url + '\nURL_HASH SHA256=' + 'a' * 64 + '\n'
        self.sources.validate_recipe_pin([url], 'a' * 64, recipe)
        for changed in (recipe.replace('public', 'other'), recipe.replace('a' * 64, 'b' * 64)):
            with self.subTest(changed=changed), self.assertRaises(RuntimeError):
                self.sources.validate_recipe_pin([url], 'a' * 64, changed)

    def test_source_checksum_mismatch_never_copies_the_input(self):
        source = self.root / 'source.tar.gz'
        source.write_bytes(b'wrong source')
        destination = self.root / 'output' / source.name
        with self.assertRaises(RuntimeError):
            self.sources.copy_verified(source, destination, '0' * 64)
        self.assertFalse(destination.exists())
        digest = hashlib.sha256(source.read_bytes()).hexdigest()
        self.sources.copy_verified(source, destination, digest)
        self.assertEqual(destination.read_bytes(), source.read_bytes())

    def test_link_inventory_maps_static_inputs_and_rejects_unknown_libraries(self):
        build = self.root / 'build'
        build.mkdir()
        metadata = ('build src/BambuStudio.app/Contents/MacOS/BambuStudio: CXX_EXECUTABLE_LINKER__Release\n'
                    '  LINK_LIBRARIES = /local/BambuStudio_deps/usr/local/lib/libboost_thread.a '
                    '-lwxregexu-3.1 -framework Cocoa /SDK/usr/lib/libz.tbd src/miniz/libminiz_static.a\n')
        (build / 'build.ninja').write_text(metadata)
        result = self.sources.link_inventory(build, self.root)
        self.assertEqual(result['linked_source_groups'], ['Boost', 'wxWidgets'])
        self.assertEqual(result['system_frameworks'], ['Cocoa'])
        self.assertNotIn('-lwxregexu-3.1', result['system_libraries'])
        (build / 'build.ninja').write_text(metadata.replace('libboost_thread.a', 'libunknown.a'))
        with self.assertRaises(RuntimeError):
            self.sources.link_inventory(build, self.root)

    def test_git_snapshot_excludes_untracked_data_and_refuses_tracked_edits(self):
        repo = self.root / 'repo'
        repo.mkdir()
        subprocess.run(['git', 'init', '-q', repo], check=True)
        (repo / 'LICENSE').write_text('public license\n')
        subprocess.run(['git', '-C', repo, 'add', 'LICENSE'], check=True)
        subprocess.run(['git', '-C', repo, '-c', 'user.name=Test', '-c',
                        'user.email=test@example.org', 'commit', '-qm', 'fixture'], check=True)
        (repo / 'private-session.json').write_text('do not publish')
        target = self.root / 'source.tar.gz'
        revision = self.sources.archive_git_source(repo, target, 'fixture/')
        self.assertEqual(len(revision), 40)
        with tarfile.open(target) as archive:
            self.assertEqual(archive.getnames(), ['fixture', 'fixture/LICENSE'])
            self.assertEqual(archive.extractfile('fixture/LICENSE').read(), b'public license\n')
        (repo / 'LICENSE').write_text('uncommitted edit')
        with self.assertRaises(RuntimeError):
            self.sources.archive_git_source(repo, self.root / 'second.tar.gz', 'fixture/')

    def test_final_archive_normalizes_metadata_and_rejects_symlinks(self):
        stage = self.root / 'native-source'
        stage.mkdir()
        (stage / 'source.txt').write_bytes(b'exact source')
        (self.root / 'private-session.json').write_text('not in archive')
        result = self.sources.write_archive(stage)
        with tarfile.open(result) as archive:
            self.assertEqual(archive.getnames(), ['native-source', 'native-source/source.txt'])
            self.assertTrue(all(m.uid == 0 and m.gid == 0 and m.mtime == 0
                                and not m.uname and not m.gname for m in archive.getmembers()))
        (stage / 'unexpected').symlink_to(self.root / 'private-session.json')
        with self.assertRaises(RuntimeError):
            self.sources.write_archive(stage)


if __name__ == '__main__':
    unittest.main()
