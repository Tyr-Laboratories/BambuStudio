"""Runtime artifacts are immutable data; these tests never contact a printer."""
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import tarfile
import tempfile
import unittest
from unittest.mock import patch
import zipfile

SCRIPT = Path(__file__).resolve().parents[2] / 'scripts/tyr_cloud_helper/build_runtime.py'
if SCRIPT.exists():
    spec = importlib.util.spec_from_file_location('cloud_runtime', SCRIPT)
    runtime = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runtime)
else:
    runtime = None


def digest(body):
    return hashlib.sha256(body).hexdigest()


class RuntimeArtifacts(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(runtime, 'cloud runtime artifact validation is not implemented')
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.bundle = self.root / 'runtime'
        self.bundle.mkdir()
        self.pin = {'source': 'https://example.invalid/source', 'revision': '1' * 40,
                    'network_abi': '02.05.02.58', 'plugin_url': 'https://example.invalid/plugin.zip',
                    'plugin_sha256': '2' * 64, 'network_sha256': digest(b'network')}
        self.checkout = self.root / 'checkout'
        self.helper_source = self.checkout / 'deps/tyr_cloud_helper'
        self.script_source = self.checkout / 'scripts/tyr_cloud_helper'
        (self.helper_source / 'vendor').mkdir(parents=True)
        self.script_source.mkdir(parents=True)
        self.source_files = {
            'host/CMakeLists.txt': b'add_executable(cloud_host vendor/host.cpp)',
            'host/vendor/host.cpp': b'int main() { return 0; }',
            'Dockerfile': b'FROM pinned-base\nCOPY host /source\n',
        }
        public_assets = {'LICENSE.upstream': b'license',
                         'resources/cert/slicer_base64.cer': b'public slicer certificate',
                         'resources/cert/ca-certificates.crt': b'public CA certificates',
                         'resources/cert/printer.cer': b'public printer certificate'}
        self.source_files.update({'host/' + name: body for name, body in public_assets.items()})
        self.source_files['host/upstream.json'] = json.dumps({
            'redistributed_asset_sha256': {name: digest(body) for name, body in public_assets.items()}
        }).encode()
        for name, body in self.source_files.items():
            path = self.script_source / name if name == 'Dockerfile' else self.helper_source / name[5:]
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(body)
        (self.script_source / 'pin.json').write_text(json.dumps(self.pin))
        root_patch = patch.object(runtime, 'ROOT', self.checkout)
        here_patch = patch.object(runtime, 'HERE', self.script_source)
        root_patch.start(); here_patch.start()
        self.addCleanup(root_patch.stop); self.addCleanup(here_patch.stop)
        context_bytes = ''.join(name + '\0' + digest(body) + '\n'
                                for name, body in sorted(self.source_files.items())).encode()
        self.source_digest = digest(context_bytes)
        config = json.dumps({'architecture': 'arm64', 'os': 'linux'}).encode()
        image_id = digest(config)
        with tarfile.open(self.bundle / 'image.tar', 'w') as tar:
            for name, body in ((image_id + '.json', config), ('manifest.json', json.dumps([{'Config': image_id + '.json', 'RepoTags': [], 'Layers': []}]).encode())):
                item = tarfile.TarInfo(name)
                item.size = len(body)
                tar.addfile(item, io.BytesIO(body))
        files = {'pin.json': json.dumps(self.pin).encode(),
                 'plugin/libbambu_networking.so': b'network', 'plugin/libBambuSource.so': b'source',
                 'plugin/liblive555.so': b'video', 'plugin/linux_payload_manifest.json': json.dumps({'files': [
                     {'name': name, 'sha256': digest(body), 'abi_version': self.pin['network_abi']}
                     for name, body in (('libbambu_networking.so', b'network'), ('libBambuSource.so', b'source'), ('liblive555.so', b'video'))
                 ]}).encode(),
                 'certs/slicer_base64.cer': b'public slicer certificate',
                 'certs/ca-certificates.crt': b'public CA certificates',
                 'certs/printer.cer': b'public printer certificate',
                 'licenses/host-AGPL-3.0.txt': b'license',
                 'licenses/plugin-source.json': json.dumps({key: self.pin[key] for key in
                     ('plugin_url', 'plugin_sha256', 'network_abi', 'network_sha256')}).encode()}
        for name, body in files.items():
            path = self.bundle / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(body)
        self.manifest = {'schema_version': 1, 'protocol_version': 1, 'host_patch_version': 1,
                         'image_id': 'sha256:' + image_id, 'image_file': 'image.tar',
                         'platform': 'linux/arm64', 'executable': '/runtime/tyr_bambu_cloud_host',
                         'network_abi': '02.05.02.58', 'host_source_sha256': self.source_digest,
                         'upstream_revision': self.pin['revision'],
                         'hashes': {str(p.relative_to(self.bundle)): digest(p.read_bytes()) for p in self.bundle.rglob('*') if p.is_file()}}
        self.write_manifest()

    def write_manifest(self):
        (self.bundle / 'manifest.json').write_text(json.dumps(self.manifest))

    def test_stage_copies_verified_bytes_and_exact_manifest(self):
        result = runtime.stage_runtime(self.bundle, self.root / 'app/cloud-helper')
        self.assertEqual(result, self.manifest)
        self.assertEqual((self.root / 'app/cloud-helper/plugin/libbambu_networking.so').read_bytes(), b'network')
        self.assertEqual(json.loads((self.root / 'app/cloud-helper/manifest.json').read_text()), self.manifest)

    def test_changed_image_and_missing_plugin_fail_before_copy(self):
        (self.bundle / 'image.tar').write_bytes(b'corrupt')
        with self.assertRaisesRegex(RuntimeError, 'checksum'):
            runtime.stage_runtime(self.bundle, self.root / 'target')
        self.assertFalse((self.root / 'target').exists())
        (self.bundle / 'plugin/liblive555.so').unlink()
        with self.assertRaisesRegex(RuntimeError, 'inventory|missing'):
            runtime.validate_runtime(self.bundle)

    def test_different_image_identity_is_rejected_even_when_file_hashes_match(self):
        self.manifest['image_id'] = 'sha256:' + '0' * 64
        self.write_manifest()
        with self.assertRaisesRegex(RuntimeError, 'image.*identity|image.*ID'):
            runtime.validate_runtime(self.bundle)

    def test_unlisted_and_even_listed_credentials_are_not_packaged(self):
        for listed in (False, True):
            with self.subTest(listed=listed):
                (self.bundle / 'account.json').write_bytes(b'private session')
                if listed:
                    self.manifest['hashes']['account.json'] = digest(b'private session')
                    self.write_manifest()
                with self.assertRaisesRegex(RuntimeError, 'inventory|unexpected|allowed'):
                    runtime.stage_runtime(self.bundle, self.root / 'target')
        self.assertFalse((self.root / 'target').exists())

    def test_symlink_escape_and_path_traversal_are_rejected(self):
        outside = self.root / 'private'
        outside.write_bytes(b'private session')
        (self.bundle / 'certs/private').symlink_to(outside)
        with self.assertRaisesRegex(RuntimeError, 'symlink'):
            runtime.validate_runtime(self.bundle)
        (self.bundle / 'certs/private').unlink()
        self.manifest['hashes']['../private'] = digest(b'private session')
        self.write_manifest()
        with self.assertRaisesRegex(RuntimeError, 'path|inventory|allowed'):
            runtime.validate_runtime(self.bundle)

    def test_existing_target_and_overlapping_source_are_unchanged(self):
        target = self.root / 'existing'
        target.mkdir()
        (target / 'keep').write_bytes(b'keep')
        for path in (target, self.bundle / 'child'):
            with self.assertRaisesRegex(RuntimeError, 'exists|overlap'):
                runtime.stage_runtime(self.bundle, path)
        self.assertEqual((target / 'keep').read_bytes(), b'keep')

    def test_pin_plugin_mismatch_is_not_accepted_as_new_manifest(self):
        (self.bundle / 'plugin/libbambu_networking.so').write_bytes(b'different network')
        self.manifest['hashes']['plugin/libbambu_networking.so'] = digest(b'different network')
        self.write_manifest()
        with self.assertRaisesRegex(RuntimeError, 'pinned|network'):
            runtime.validate_runtime(self.bundle)


    def test_changed_host_source_requires_rebuild_even_when_runtime_is_internally_valid(self):
        (self.helper_source / 'vendor/host.cpp').write_bytes(b'int main() { return 1; }')
        with self.assertRaisesRegex(RuntimeError, 'source|rebuild'):
            runtime.stage_runtime(self.bundle, self.root / 'target')
        self.assertFalse((self.root / 'target').exists())

    def test_changed_dockerfile_requires_rebuild(self):
        (self.script_source / 'Dockerfile').write_bytes(b'FROM different-pinned-base')
        with self.assertRaisesRegex(RuntimeError, 'source|rebuild'):
            runtime.validate_runtime(self.bundle)

    def test_stale_pin_is_rejected_even_when_payload_still_matches_it(self):
        updated = dict(self.pin, plugin_url='https://example.invalid/updated.zip')
        (self.script_source / 'pin.json').write_text(json.dumps(updated))
        with self.assertRaisesRegex(RuntimeError, 'pin|rebuild'):
            runtime.validate_runtime(self.bundle)

    def test_private_files_under_public_directories_are_rejected(self):
        for name in ('certs/account.json', 'licenses/session.json'):
            with self.subTest(name=name):
                (self.bundle / name).write_bytes(b'private session')
                self.manifest['hashes'][name] = digest(b'private session')
                self.write_manifest()
                with self.assertRaisesRegex(RuntimeError, 'inventory|unexpected|allowed'):
                    runtime.validate_runtime(self.bundle)
                (self.bundle / name).unlink()
                del self.manifest['hashes'][name]

    def test_changed_public_assets_are_rejected_even_with_updated_manifest_hash(self):
        for name in ('certs/printer.cer', 'licenses/host-AGPL-3.0.txt'):
            with self.subTest(name=name):
                original = (self.bundle / name).read_bytes()
                (self.bundle / name).write_bytes(b'private content in public filename')
                self.manifest['hashes'][name] = digest(b'private content in public filename')
                self.write_manifest()
                with self.assertRaisesRegex(RuntimeError, 'public|upstream|asset'):
                    runtime.validate_runtime(self.bundle)
                (self.bundle / name).write_bytes(original)
                self.manifest['hashes'][name] = digest(original)

    def test_plugin_payload_manifest_cannot_carry_private_or_unknown_fields(self):
        name = 'plugin/linux_payload_manifest.json'
        payload = json.loads((self.bundle / name).read_text())
        payload['session'] = {'token': 'must never be packaged'}
        (self.bundle / name).write_text(json.dumps(payload))
        self.manifest['hashes'][name] = digest((self.bundle / name).read_bytes())
        self.write_manifest()
        with self.assertRaisesRegex(RuntimeError, 'payload manifest'):
            runtime.validate_runtime(self.bundle)

    def test_plugin_provenance_cannot_carry_extra_account_fields(self):
        name = 'licenses/plugin-source.json'
        provenance = json.loads((self.bundle / name).read_text())
        provenance['account_token'] = 'must never be packaged'
        (self.bundle / name).write_text(json.dumps(provenance))
        self.manifest['hashes'][name] = digest((self.bundle / name).read_bytes())
        self.write_manifest()
        with self.assertRaisesRegex(RuntimeError, 'provenance'):
            runtime.validate_runtime(self.bundle)


class PluginArchive(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(runtime, 'cloud runtime artifact validation is not implemented')
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def archive(self, duplicate=False):
        data = io.BytesIO()
        with zipfile.ZipFile(data, 'w') as z:
            for name, body in (('libbambu_networking.so', b'network'), ('libBambuSource.so', b'source'), ('liblive555.so', b'video')):
                z.writestr('payload/' + name, body)
            z.writestr('../../account.json', b'must not extract')
            if duplicate:
                z.writestr('duplicate/libbambu_networking.so', b'network')
        return data.getvalue()

    def test_extracts_only_verified_library_payloads_and_generated_manifest(self):
        body = self.archive()
        pin = {'plugin_sha256': digest(body), 'network_sha256': digest(b'network'), 'network_abi': '02.05.02.58'}
        runtime.install_plugin_archive(body, pin, self.root / 'plugin')
        self.assertEqual(sorted(p.name for p in (self.root / 'plugin').iterdir()), ['libBambuSource.so', 'libbambu_networking.so', 'liblive555.so', 'linux_payload_manifest.json'])
        self.assertFalse((self.root / 'account.json').exists())
        self.assertEqual((self.root / 'plugin/libbambu_networking.so').read_bytes(), b'network')

    def test_bad_checksum_and_duplicate_library_never_create_destination(self):
        body = self.archive()
        for data, checksum in ((body, '0' * 64), (self.archive(True), digest(self.archive(True)))):
            with self.subTest(checksum=checksum):
                with self.assertRaisesRegex(RuntimeError, 'checksum|layout'):
                    runtime.install_plugin_archive(data, {'plugin_sha256': checksum, 'network_sha256': digest(b'network'), 'network_abi': '02.05.02.58'}, self.root / 'plugin')
                self.assertFalse((self.root / 'plugin').exists())


if __name__ == '__main__':
    unittest.main()
