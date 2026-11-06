"""Runtime artifacts are immutable data; these tests never contact a printer."""
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import tarfile
import sys
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
                    'plugin_sha256': '2' * 64, 'network_sha256': digest(b'network'),
                    'payload_sha256': {name: digest(body) for name, body in
                        (('libbambu_networking.so', b'network'), ('libBambuSource.so', b'source'), ('liblive555.so', b'video'))},
                    'plugin_entries': {name: name for name in runtime.PLUGIN_FILES},
                    'debian_base': 'debian:bookworm@sha256:' + '3' * 64,
                    'debian_snapshot': '20261101T000000Z'}
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
                 'certs/slicer_base64.cer': b'public slicer certificate',
                 'certs/ca-certificates.crt': b'public CA certificates',
                 'certs/printer.cer': b'public printer certificate',
                 'licenses/host-AGPL-3.0.txt': b'license',
                 'licenses/plugin-source.json': json.dumps({key: self.pin[key] for key in
                     ('plugin_url', 'plugin_sha256', 'network_abi', 'network_sha256', 'payload_sha256', 'plugin_entries')}).encode(),
                 'licenses/runtime-dependencies.txt': runtime.runtime_dependency_notice(self.pin).encode()}
        for name, body in files.items():
            path = self.bundle / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(body)
        self.manifest = {'schema_version': 2, 'plugin_delivery': 'vendor-download', 'protocol_version': 1, 'host_patch_version': 1,
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
        self.assertFalse((self.root / 'app/cloud-helper/plugin').exists())
        self.assertEqual(json.loads((self.root / 'app/cloud-helper/manifest.json').read_text()), self.manifest)

    def test_changed_image_and_missing_certificate_fail_before_copy(self):
        (self.bundle / 'image.tar').write_bytes(b'corrupt')
        with self.assertRaisesRegex(RuntimeError, 'checksum'):
            runtime.stage_runtime(self.bundle, self.root / 'target')
        self.assertFalse((self.root / 'target').exists())
        (self.bundle / 'certs/printer.cer').unlink()
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

    def test_legacy_runtime_is_never_publicly_packaged(self):
        self.manifest['schema_version'] = 1
        self.write_manifest()
        with self.assertRaisesRegex(RuntimeError, 'Unsupported|public'):
            runtime.stage_runtime(self.bundle, self.root / 'target')
        self.assertFalse((self.root / 'target').exists())

    def test_plugin_directory_is_rejected_even_empty(self):
        (self.bundle / 'plugin').mkdir()
        with self.assertRaisesRegex(RuntimeError, 'plugin|proprietary'):
            runtime.validate_runtime(self.bundle)

    def test_plugin_payload_is_rejected_even_when_listed(self):
        (self.bundle / 'plugin').mkdir()
        (self.bundle / 'plugin/libbambu_networking.so').write_bytes(b'network')
        self.manifest['hashes']['plugin/libbambu_networking.so'] = digest(b'network')
        self.write_manifest()
        with self.assertRaisesRegex(RuntimeError, 'plugin|inventory|proprietary'):
            runtime.stage_runtime(self.bundle, self.root / 'target')
        self.assertFalse((self.root / 'target').exists())

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

    def test_vendor_pin_requires_all_payload_checksums_and_safe_exact_entries(self):
        for key, change in (('payload_sha256', {'libbambu_networking.so': digest(b'network')}),
                            ('plugin_entries', dict(self.pin['plugin_entries'], **{'libBambuSource.so': '../libBambuSource.so'})),
                            ('payload_sha256', dict(self.pin['payload_sha256'], **{'liblive555.so': 'not a checksum'}))):
            with self.subTest(key=key, change=change):
                altered = dict(self.pin, **{key: change})
                with self.assertRaisesRegex(RuntimeError, 'pin|entry|checksum'):
                    runtime.validate_plugin_pin(altered)

    def test_runtime_build_never_downloads_or_stages_vendor_payloads(self):
        output = self.root / 'built-runtime'
        def docker_run(command, **kwargs):
            if command[:3] == ['docker', 'image', 'save']:
                import shutil
                shutil.copy2(self.bundle / 'image.tar', command[4])
        with patch.object(runtime.subprocess, 'run', side_effect=docker_run) as run, \
             patch.object(runtime.subprocess, 'check_output', return_value=self.manifest['image_id']):
            runtime.build(output)
        self.assertEqual({p.relative_to(output).as_posix() for p in output.rglob('*') if p.is_file()},
                         runtime.REQUIRED_FILES | {'manifest.json'})
        self.assertFalse((output / 'plugin').exists())
        self.assertEqual([call.args[0][:3] for call in run.call_args_list],
                         [['docker', 'build', '--platform'], ['docker', 'image', 'save']])
        self.assertEqual(runtime.validate_runtime(output)['plugin_delivery'], 'vendor-download')

    def test_plugin_provenance_cannot_carry_extra_account_fields(self):
        name = 'licenses/plugin-source.json'
        provenance = json.loads((self.bundle / name).read_text())
        provenance['account_token'] = 'must never be packaged'
        (self.bundle / name).write_text(json.dumps(provenance))
        self.manifest['hashes'][name] = digest((self.bundle / name).read_bytes())
        self.write_manifest()
        with self.assertRaisesRegex(RuntimeError, 'provenance'):
            runtime.validate_runtime(self.bundle)


class PublicImageArchive(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def archive(self, files, extra_outer=None):
        layer_bytes = io.BytesIO()
        with tarfile.open(fileobj=layer_bytes, mode='w') as layer:
            for name, body in files:
                member = tarfile.TarInfo(name)
                member.size = len(body)
                layer.addfile(member, io.BytesIO(body))
        layer_bytes = layer_bytes.getvalue()
        config = json.dumps({'architecture': 'arm64', 'os': 'linux',
                             'rootfs': {'type': 'layers', 'diff_ids': ['sha256:' + digest(layer_bytes)]}}).encode()
        image_id = 'sha256:' + digest(config)
        manifest = json.dumps([{'Config': 'config.json', 'Layers': ['layer.tar']}]).encode()
        image = self.root / 'image.tar'
        with tarfile.open(image, 'w') as tar:
            for name, body in [('config.json', config), ('manifest.json', manifest), ('layer.tar', layer_bytes)] + (extra_outer or []):
                member = tarfile.TarInfo(name)
                member.size = len(body)
                tar.addfile(member, io.BytesIO(body))
        return image, image_id

    def test_open_runtime_layer_is_accepted(self):
        image, identity = self.archive([('runtime/tyr_bambu_cloud_host', b'open source host'),
                                        ('usr/share/doc/qemu-user-static/copyright', b'GPL license')])
        runtime.verify_image_archive(image, identity)

    def test_proprietary_libraries_in_any_layer_are_rejected(self):
        for name in (*runtime.PLUGIN_FILES, 'libagora_rtc_sdk.so', 'libBambuSource.dylib'):
            with self.subTest(name=name):
                image, identity = self.archive([('usr/lib/' + name, b'proprietary')])
                with self.assertRaisesRegex(RuntimeError, 'proprietary|plugin'):
                    runtime.verify_image_archive(image, identity)

    def test_plugin_payload_cannot_be_renamed_or_whiteouted(self):
        for name in ('plugin/renamed.bin', 'plugin/.wh.libbambu_networking.so'):
            with self.subTest(name=name):
                image, identity = self.archive([(name, b'payload')])
                with self.assertRaisesRegex(RuntimeError, 'proprietary|plugin'):
                    runtime.verify_image_archive(image, identity)

    def test_renamed_vendor_bytes_are_rejected_outside_plugin_directory(self):
        image, identity = self.archive([('opt/renamed.dat', b'vendor bytes')])
        pin_dir = self.root / 'pins'
        pin_dir.mkdir()
        (pin_dir / 'pin.json').write_text(json.dumps({'payload_sha256': {'vendor': digest(b'vendor bytes')}}))
        with patch.object(runtime, 'HERE', pin_dir), self.assertRaisesRegex(RuntimeError, 'Renamed proprietary'):
            runtime.verify_image_archive(image, identity)

    def test_unreferenced_layer_cannot_hide_a_plugin(self):
        hidden = io.BytesIO()
        with tarfile.open(fileobj=hidden, mode='w') as tar:
            member = tarfile.TarInfo('secret/libbambu_networking.so')
            member.size = 6
            tar.addfile(member, io.BytesIO(b'secret'))
        image, identity = self.archive([('runtime/host', b'host')], [('hidden.tar', hidden.getvalue())])
        with self.assertRaisesRegex(RuntimeError, 'proprietary|plugin|[Uu]nexpected'):
            runtime.verify_image_archive(image, identity)

    def test_duplicate_or_traversal_image_entries_are_rejected(self):
        for name in ('../secret', 'layer.tar'):
            with self.subTest(name=name):
                image, identity = self.archive([('runtime/host', b'host')], [(name, b'secret')])
                with self.assertRaisesRegex(RuntimeError, 'path|duplicate|entry|unexpected'):
                    runtime.verify_image_archive(image, identity)


class PublicAppPayloads(unittest.TestCase):
    def test_materialized_resource_symlink_cannot_smuggle_vendor_libraries_or_nested_app(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            sys.path.insert(0, str(SCRIPT.parents[1]))
            try:
                spec = importlib.util.spec_from_file_location('tyr_packager', SCRIPT.parents[1] / 'package_tyr_macos.py')
                packager = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(packager)
            finally:
                sys.path.pop(0)
            source = root / 'input.app'
            (source / 'Contents').mkdir(parents=True)
            resources = root / 'linked-resources'
            resources.mkdir()
            (source / 'Contents/Resources').symlink_to(resources, target_is_directory=True)
            for name, directory in (('libBambuSource.dylib', False), ('OfficialStudio.app', True)):
                with self.subTest(name=name):
                    forbidden = resources / name
                    if directory:
                        forbidden.mkdir()
                    else:
                        forbidden.write_bytes(b'vendor library')
                    with self.assertRaisesRegex(RuntimeError, 'vendor plugin|nested application'):
                        packager.stage_public_app(source, root / ('staged-' + name))
                    if directory:
                        forbidden.rmdir()
                    else:
                        forbidden.unlink()


if __name__ == '__main__':
    unittest.main()
