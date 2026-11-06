"""Native dependency notices must survive staging and release extraction intact."""
import hashlib
import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
import zipfile


SCRIPT = Path(__file__).resolve().parents[2] / 'scripts/package_tyr_macos.py'
sys.path.insert(0, str(SCRIPT.parent))
try:
    spec = importlib.util.spec_from_file_location('tyr_native_notice_packager', SCRIPT)
    packager = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(packager)
finally:
    sys.path.pop(0)


class NativeNotices(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.source = self.root / 'LICENSES'
        self.source.mkdir()
        self.files = {'library A/LICENSE.txt': b'Copyright library A\n',
                      'library B/nested/COPYING': b'License library B\n'}
        for name, body in self.files.items():
            path = self.source / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(body)
        self.target = self.root / 'app/Contents/Resources/TyrFork/native-licenses'

    def stage(self):
        self.assertTrue(callable(getattr(packager, 'stage_native_licenses', None)),
                        'native notices are not staged into the public app')
        return packager.stage_native_licenses(self.source, self.target)

    def test_required_cli_input(self):
        result = subprocess.run([sys.executable, str(SCRIPT)], capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('--native-licenses', result.stderr)

    def test_all_nested_notices_and_hashes_survive_archive_round_trip(self):
        inventory = self.stage()
        self.assertEqual(inventory, {name: hashlib.sha256(body).hexdigest()
                                     for name, body in self.files.items()})
        archive = self.root / 'notices.zip'
        with zipfile.ZipFile(archive, 'w') as output:
            for name in inventory:
                output.write(self.target / name, 'native-licenses/' + name)
        extracted = self.root / 'extracted'
        with zipfile.ZipFile(archive) as output:
            output.extractall(extracted)
        packager.verify_native_licenses(extracted / 'native-licenses', inventory)
        self.assertEqual({name: (extracted / 'native-licenses' / name).read_bytes()
                          for name in inventory}, self.files)

    def test_symlink_file_directory_broken_link_and_root_are_rejected_before_copy(self):
        outside = self.root / 'private'
        outside.mkdir()
        (outside / 'secret').write_bytes(b'must not be copied')
        for linked in (outside / 'secret', outside, self.root / 'missing'):
            with self.subTest(linked=linked):
                link = self.source / 'linked'
                link.symlink_to(linked)
                with self.assertRaisesRegex(RuntimeError, 'symlink|regular'):
                    self.stage()
                self.assertFalse(self.target.exists())
                link.unlink()
        alias = self.root / 'alias'
        alias.symlink_to(self.source, target_is_directory=True)
        self.source = alias
        with self.assertRaisesRegex(RuntimeError, 'symlink|directory'):
            self.stage()
        self.assertFalse(self.target.exists())

    def test_empty_and_special_file_sources_are_rejected_before_copy(self):
        shutil.rmtree(self.source)
        self.source.mkdir()
        with self.assertRaisesRegex(RuntimeError, 'empty|notice'):
            self.stage()
        os.mkfifo(self.source / 'pipe')
        with self.assertRaisesRegex(RuntimeError, 'regular'):
            self.stage()
        self.assertFalse(self.target.exists())

    def test_missing_changed_and_additional_notice_fail_verification(self):
        inventory = self.stage()
        notice = self.target / 'library A/LICENSE.txt'
        for mutation in ('missing', 'changed', 'additional'):
            with self.subTest(mutation=mutation):
                if mutation == 'missing':
                    notice.unlink()
                elif mutation == 'changed':
                    notice.write_bytes(b'changed')
                else:
                    (self.target / 'unexpected').write_bytes(b'extra')
                with self.assertRaisesRegex(RuntimeError, 'changed|inventory|checksum'):
                    packager.verify_native_licenses(self.target, inventory)
                notice.write_bytes(self.files['library A/LICENSE.txt'])

    def test_existing_destination_and_overlap_preserve_inputs(self):
        self.target.mkdir(parents=True)
        (self.target / 'keep').write_bytes(b'keep')
        with self.assertRaisesRegex(RuntimeError, 'exists'):
            self.stage()
        self.assertEqual((self.target / 'keep').read_bytes(), b'keep')
        self.target = self.source / 'child'
        with self.assertRaisesRegex(RuntimeError, 'overlap'):
            self.stage()
        self.assertFalse(self.target.exists())


if __name__ == '__main__':
    unittest.main()
