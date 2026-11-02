"""Exercise packaging's real file handling; signing tests require macOS.

Run: python3 -m unittest discover -s tests/packaging -v
Set TYR_OFFICIAL_APP to exercise the real vendor app's signature and archive.
"""
import hashlib
import importlib.util
import os
from pathlib import Path
import plistlib
import shutil
import stat
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

SCRIPT = Path(__file__).resolve().parents[2] / "scripts/package_tyr_macos.py"
spec = importlib.util.spec_from_file_location("package_tyr_macos", SCRIPT)
package = importlib.util.module_from_spec(spec)
spec.loader.exec_module(package)


class OfficialSenderFiles(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.app = self.root / "Vendor.app"
        contents = self.app / "Contents"
        (contents / "MacOS").mkdir(parents=True)
        (contents / "Resources").mkdir()
        (contents / "_CodeSignature").mkdir()
        (contents / "Info.plist").write_bytes(plistlib.dumps({
            "CFBundleIdentifier": "com.bambulab.bambu-studio",
            "CFBundleExecutable": "BambuStudio",
            "CFBundleShortVersionString": "02.08.02.61",
            "CFBundleVersion": "",
        }))
        (contents / "MacOS/BambuStudio").write_bytes(b"signed executable fixture\0\xff")
        (contents / "MacOS/BambuStudio").chmod(0o755)
        (contents / "Resources/preset.json").write_bytes(b'{"system":"preset"}')
        (contents / "_CodeSignature/CodeResources").write_bytes(b"sealed vendor resources")
        self.identity = {"team_identifier": "T3UBR9Y3B2",
                         "publisher": "Developer ID Application: Shanghai Lunkuo Technology Co., Ltd (T3UBR9Y3B2)"}

    def require_feature(self):
        # A failing assertion, rather than an import error, identifies the missing feature.
        self.assertTrue(hasattr(package, "stage_official_app"), "official sender staging is not implemented")

    @unittest.skipUnless(sys.platform == "darwin", "ditto is a macOS packaging dependency")
    def test_copy_keeps_sealed_files_symlinks_and_exact_manifest_identity(self):
        self.require_feature()
        (self.app / "Contents/Resources/alias.json").symlink_to("preset.json")
        target = self.root / "Fork.app/Contents/Helpers/BambuStudio.app"
        with patch.object(package, "verify_vendor_signature", return_value=self.identity):
            metadata = package.stage_official_app(self.app, target)
        self.assertEqual((target / "Contents/_CodeSignature/CodeResources").read_bytes(), b"sealed vendor resources")
        self.assertEqual((target / "Contents/MacOS/BambuStudio").read_bytes(), b"signed executable fixture\0\xff")
        self.assertEqual(stat.S_IMODE((target / "Contents/MacOS/BambuStudio").stat().st_mode), 0o755)
        self.assertTrue((target / "Contents/Resources/alias.json").is_symlink())
        self.assertEqual(os.readlink(target / "Contents/Resources/alias.json"), "preset.json")
        self.assertEqual(metadata["version"], "02.08.02.61")
        self.assertEqual(metadata["build"], "")
        self.assertEqual(metadata["bundle_identifier"], "com.bambulab.bambu-studio")
        self.assertEqual(metadata["executable_sha256"], hashlib.sha256(b"signed executable fixture\0\xff").hexdigest())
        self.assertEqual(metadata["publisher"], self.identity["publisher"])
        self.assertEqual(metadata["team_identifier"], "T3UBR9Y3B2")
        self.assertNotIn(str(self.root), str(metadata))

    def test_rejects_symlink_bundle_and_external_or_dangling_internal_links(self):
        self.require_feature()
        alias = self.root / "Alias.app"
        alias.symlink_to(self.app)
        with self.assertRaisesRegex(RuntimeError, "symlink"):
            package.stage_official_app(alias, self.root / "Target.app")
        outside = self.root / "private-account.json"
        outside.write_bytes(b"must not be packaged")
        link = self.app / "Contents/Resources/private"
        for dest in (outside, self.root / "missing"):
            with self.subTest(destination=str(dest)):
                link.symlink_to(dest)
                with self.assertRaisesRegex(RuntimeError, "symlink"):
                    package.stage_official_app(self.app, self.root / "Target.app")
                link.unlink()
        self.assertFalse((self.root / "Target.app").exists())

    def test_rejects_invalid_executable_and_bundle_identity(self):
        self.require_feature()
        plist = self.app / "Contents/Info.plist"
        baseline = plistlib.loads(plist.read_bytes())
        for key, value in (("CFBundleExecutable", "../../private"),
                           ("CFBundleIdentifier", "com.example.imposter")):
            with self.subTest(key=key):
                info = dict(baseline)
                info[key] = value
                plist.write_bytes(plistlib.dumps(info))
                with self.assertRaisesRegex(RuntimeError, "official|bundle|executable"):
                    package.stage_official_app(self.app, self.root / "Target.app")
        self.assertFalse((self.root / "Target.app").exists())

    def test_rejects_missing_bundle_and_existing_or_overlapping_target(self):
        self.require_feature()
        with self.assertRaisesRegex(RuntimeError, "missing|bundle"):
            package.stage_official_app(self.root / "missing.app", self.root / "Target.app")
        for target in (self.app, self.app / "Nested.app", self.root):
            with self.subTest(target=str(target)):
                with self.assertRaisesRegex(RuntimeError, "target|overlap|exists"):
                    package.stage_official_app(self.app, target)
        self.assertEqual((self.app / "Contents/MacOS/BambuStudio").read_bytes(), b"signed executable fixture\0\xff")

    @unittest.skipUnless(sys.platform == "darwin", "codesign is a macOS dependency")
    def test_rejects_unsigned_bundle_without_creating_target(self):
        self.require_feature()
        with self.assertRaisesRegex(RuntimeError, "signature|signed"):
            package.stage_official_app(self.app, self.root / "Target.app")
        self.assertFalse((self.root / "Target.app").exists())

    @unittest.skipUnless(sys.platform == "darwin", "ditto is a macOS dependency")
    def test_detects_resource_and_executable_changes_after_staging(self):
        self.require_feature()
        target = self.root / "Target.app"
        with patch.object(package, "verify_vendor_signature", return_value=self.identity):
            metadata = package.stage_official_app(self.app, target)
            (target / "Contents/Resources/preset.json").write_bytes(b"changed")
            with self.assertRaisesRegex(RuntimeError, "changed|identical|digest"):
                package.verify_official_app(target, metadata)
            (target / "Contents/Resources/preset.json").write_bytes(b'{"system":"preset"}')
            (target / "Contents/MacOS/BambuStudio").write_bytes(b"changed")
            with self.assertRaisesRegex(RuntimeError, "changed|identical|digest"):
                package.verify_official_app(target, metadata)


@unittest.skipUnless(sys.platform == "darwin" and os.environ.get("TYR_OFFICIAL_APP"),
                     "set TYR_OFFICIAL_APP to a real vendor-signed release")
class OfficialSenderSignatureIntegration(unittest.TestCase):
    def test_nested_app_keeps_vendor_identity_through_outer_signing_and_zip(self):
        self.assertTrue(hasattr(package, "stage_official_app"), "official sender staging is not implemented")
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            outer = root / "Fork.app"
            (outer / "Contents/MacOS").mkdir(parents=True)
            # A real native executable gives codesign a valid enclosing app to seal.
            shutil.copyfile("/usr/bin/true", outer / "Contents/MacOS/Fork")
            (outer / "Contents/MacOS/Fork").chmod(0o755)
            (outer / "Contents/Info.plist").write_bytes(plistlib.dumps({
                "CFBundleIdentifier": "com.tyrlaboratories.packaging-test",
                "CFBundleExecutable": "Fork", "CFBundlePackageType": "APPL",
                "CFBundleVersion": "1", "CFBundleShortVersionString": "1",
            }))
            target = outer / "Contents/Helpers/BambuStudio.app"
            metadata = package.stage_official_app(Path(os.environ["TYR_OFFICIAL_APP"]), target)
            subprocess.run(["codesign", "--force", "--sign", "-", "--timestamp=none", str(outer)], check=True)
            subprocess.run(["codesign", "--verify", "--deep", "--strict", str(outer)], check=True)
            package.verify_official_app(target, metadata)
            archive = root / "release.zip"
            subprocess.run(["ditto", "-c", "-k", "--sequesterRsrc", "--keepParent", str(outer), str(archive)], check=True)
            subprocess.run(["ditto", "-x", "-k", str(archive), str(root / "unpacked")], check=True)
            package.verify_official_app(root / "unpacked/Fork.app/Contents/Helpers/BambuStudio.app", metadata)
            self.assertEqual(metadata["team_identifier"], "T3UBR9Y3B2")
            self.assertEqual(metadata["bundle_identifier"], "com.bambulab.bambu-studio")


if __name__ == "__main__":
    unittest.main()
