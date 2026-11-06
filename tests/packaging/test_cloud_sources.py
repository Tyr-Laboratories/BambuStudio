"""Corresponding-source collection is exact-version and never consumes user data."""
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import tarfile
import tempfile
import unittest
from unittest.mock import patch

SCRIPTS = Path(__file__).resolve().parents[2] / 'scripts/tyr_cloud_helper'
sys.path.insert(0, str(SCRIPTS))
import build_sources as sources
sys.path.pop(0)


class SourceCollection(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)

    def test_exact_source_versions_include_static_built_using(self):
        packages = sources.parse_packages(
            'qemu-user-static\t1:7.2-1+b3\tqemu\t1:7.2-1\tglibc (= 2.36-9), glib2.0 (= 2.74-1)\n'
            'libc6:arm64\t2.36-9\tglibc\t2.36-9\t\n')
        self.assertEqual(sources.required_sources(packages),
                         {('qemu', '1:7.2-1'), ('glibc', '2.36-9'), ('glib2.0', '2.74-1')})
        self.assertEqual(packages[0]['version'], '1:7.2-1+b3')

    def test_cross_libc_crt_and_headers_keep_their_actual_built_using_sources(self):
        packages = sources.parse_packages(
            'libc6-dev-amd64-cross\t2.36-8cross1\tcross-toolchain-base\t66\tbinutils (= 2.40-2), linux (= 6.1.4-1), gcc-12 (= 12.2.0-14), glibc (= 2.36-8)\n'
            'libc6-amd64-cross\t2.36-8cross1\tcross-toolchain-base\t66\tglibc (= 2.36-8)\n'
            'linux-libc-dev-amd64-cross\t6.1.4-1cross1\tcross-toolchain-base\t66\tlinux (= 6.1.4-1)\n'
            'libc6-dev:arm64\t2.36-9+deb12u14\tglibc\t2.36-9+deb12u14\t\n'
            'libc-dev-bin\t2.36-9+deb12u14\tglibc\t2.36-9+deb12u14\t\n'
            'cmake\t3.25.1-1\tcmake\t3.25.1-1\t\n')
        incorporated = sources.incorporated_host_packages(packages)
        self.assertEqual(len(incorporated), 5)
        self.assertEqual(sources.required_sources(incorporated), {
            ('cross-toolchain-base', '66'), ('binutils', '2.40-2'), ('linux', '6.1.4-1'),
            ('gcc-12', '12.2.0-14'), ('glibc', '2.36-8'), ('glibc', '2.36-9+deb12u14')})

    def test_nonexact_built_using_is_rejected(self):
        with self.assertRaises(RuntimeError):
            sources.parse_packages('qemu\t1\tqemu\t1\tglibc (>= 2.36)\n')

    def test_debian_multiline_checksums_parse_exact_files(self):
        text = 'Package: qemu\nVersion: 1:7.2-1\nDirectory: pool/main/q/qemu\nChecksums-Sha256:\n ' + 'a' * 64 + ' 123 qemu.orig.tar.xz\n'
        record = sources.parse_source_index(text)['qemu', '1:7.2-1']
        self.assertEqual(record['files'], [{'name': 'qemu.orig.tar.xz', 'size': 123, 'sha256': 'a' * 64}])
        for malicious in ('../qemu.tar.xz', '/qemu.tar.xz', 'nested/qemu.tar.xz', 'nested\\qemu.tar.xz'):
            with self.subTest(malicious=malicious), self.assertRaises(RuntimeError):
                sources.parse_source_index(text.replace('qemu.orig.tar.xz', malicious))

    def test_download_requires_full_checksum_and_size_and_cleans_temporary_file(self):
        payload = b'complete Debian source'
        target = self.root / 'source.tar'
        expected = hashlib.sha256(payload).hexdigest()
        for size, checksum, response in ((len(payload), '0' * 64, payload),
                                          (len(payload) - 1, expected, payload),
                                          (len(payload), expected, payload[:-1])):
            with self.subTest(size=size, checksum=checksum), \
                 patch.object(sources.urllib.request, 'urlopen', return_value=io.BytesIO(response)), \
                 self.assertRaises(RuntimeError):
                sources.download_verified('https://deb.debian.org/source', target, size, checksum)
            self.assertFalse(target.exists())
            self.assertEqual(list(self.root.iterdir()), [])
        with patch.object(sources.urllib.request, 'urlopen', return_value=io.BytesIO(payload)):
            sources.download_verified('https://deb.debian.org/source', target, len(payload), expected)
        self.assertEqual(target.read_bytes(), payload)
        with patch.object(sources.urllib.request, 'urlopen', side_effect=AssertionError('cache must avoid request')):
            sources.download_verified('https://deb.debian.org/source', target, len(payload), expected)

    def test_archive_includes_only_source_tree_and_preserves_file_bytes(self):
        output = self.root / 'helper-runtime-corresponding-source-tyr.17'
        output.mkdir()
        (output / 'source.txt').write_bytes(b'exact source')
        (self.root / 'private-session.json').write_bytes(b'must not be packaged')
        archive = sources.write_source_archive(output)
        self.assertEqual(archive.name, output.name + '.tar.gz')
        with tarfile.open(archive) as tar:
            self.assertEqual(tar.getnames(), [output.name, output.name + '/source.txt'])
            self.assertEqual(tar.extractfile(output.name + '/source.txt').read(), b'exact source')
            self.assertTrue(all(item.uid == 0 and item.mtime == 0 for item in tar.getmembers()))

    def test_old_cross_libc_uses_exact_source_control_and_member_checksums(self):
        body = (b'-----BEGIN PGP SIGNED MESSAGE-----\nHash: SHA512\n\n'
                b'Source: glibc\nVersion: 2.36-8\nChecksums-Sha256:\n ' + b'a' * 64 +
                b' 123 glibc_2.36.orig.tar.xz\n-----BEGIN PGP SIGNATURE-----\nexample signature\n')
        with patch.object(sources.urllib.request, 'urlopen', return_value=io.BytesIO(body)):
            record = sources.source_control_record('glibc', '2.36-8', 'pool/main/g/glibc',
                'debian', {'debian_snapshot': '20261101T000000Z'}, self.root)
        self.assertEqual(record['version'], '2.36-8')
        self.assertEqual(record['files'], [
            {'name': 'glibc_2.36.orig.tar.xz', 'size': 123, 'sha256': 'a' * 64},
            {'name': 'glibc_2.36-8.dsc', 'size': len(body), 'sha256': hashlib.sha256(body).hexdigest()}])
        with patch.object(sources.urllib.request, 'urlopen', return_value=io.BytesIO(body)), \
             self.assertRaisesRegex(RuntimeError, 'version identity'):
            sources.source_control_record('glibc', '2.36-9', 'pool/main/g/glibc',
                'debian', {'debian_snapshot': '20261101T000000Z'}, self.root)

    def test_source_index_does_not_substitute_newer_package_version(self):
        pin = {'debian_snapshot': '20261101T000000Z'}
        import lzma
        raw = lzma.compress(b'Package: package\nVersion: 2\nDirectory: pool/main/p/package\nChecksums-Sha256:\n ' + b'a' * 64 + b' 1 package.tar.xz\n')
        with patch.object(sources.urllib.request, 'urlopen', side_effect=lambda *args, **kwargs: io.BytesIO(raw)):
            with self.assertRaisesRegex((RuntimeError, UnicodeError), 'source|decode'):
                sources.source_plan({('package', '1')}, pin, self.root)


if __name__ == '__main__':
    unittest.main()
