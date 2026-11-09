"""Archive parity regressions using checked-in parser reference projects."""
import importlib.util
import json
import re
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest import mock
import zipfile

REPO_ROOT = Path(__file__).resolve().parents[2]
TOOL_ROOT = REPO_ROOT / 'scripts' / 'bridge_aware_cubic_benchmark'
PROJECTS = REPO_ROOT / 'tests' / 'data' / 'bridge_aware_cubic_benchmark' / 'projects'
spec = importlib.util.spec_from_file_location('fixture_audit_under_test', TOOL_ROOT / 'family_fixture_audit.py')
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


class FixtureAuditTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='bac-fixture-audit-')
        self.root = Path(self.temporary.name) / 'projects'
        shutil.copytree(PROJECTS, self.root)
        self.block_launch = mock.patch('subprocess.Popen', side_effect=AssertionError('Subprocess launches forbidden in pure tests'))
        self.block_launch.start()

    def tearDown(self):
        self.block_launch.stop()
        self.temporary.cleanup()

    def rewrite(self, size, side, entry, mutate):
        path = self.root / f'{size}-{side}.3mf'
        with zipfile.ZipFile(path) as archive:
            parts = {name: archive.read(name) for name in archive.namelist()}
        original = parts[entry]
        parts[entry] = mutate(original)
        self.assertNotEqual(parts[entry], original, 'Mutation must change the archive payload')
        with zipfile.ZipFile(path, 'w') as archive:
            for name, raw in parts.items():
                archive.writestr(name, raw)

    def pair(self, size='modifier'):
        return audit.validate_fixture_pair(self.root / f'{size}-cubic.3mf', self.root / f'{size}-bridgeawarecubic.3mf')

    def reject(self, size='modifier'):
        with self.assertRaises(audit.InvalidFixture):
            self.pair(size)

    def test_all_native_parser_reference_pairs(self):
        for size in ('small', 'large', 'rib', 'modifier'):
            with self.subTest(size=size):
                self.assertFalse(self.pair(size)['legacy_cube_manifest'])

    def test_local_density_mutation(self):
        self.rewrite('modifier', 'cubic', 'Metadata/model_settings.config',
                     lambda data: data.replace(b'value="12%"', b'value="13%"'))
        self.reject()

    def test_local_gap_missing(self):
        self.rewrite('modifier', 'bridgeawarecubic', 'Metadata/model_settings.config',
                     lambda data: data.replace(b'<metadata key="bridge_aware_cubic_max_gap" value="3"/>', b''))
        self.reject()

    def test_local_cubic_gap_forbidden(self):
        self.rewrite('modifier', 'cubic', 'Metadata/model_settings.config',
                     lambda data: data.replace(b'<metadata key="infill_direction"',
                         b'<metadata key="bridge_aware_cubic_max_gap" value="3"/><metadata key="infill_direction"'))
        self.reject()

    def test_object_override_rejected(self):
        self.rewrite('modifier', 'cubic', 'Metadata/model_settings.config',
                     lambda data: data.replace(b'<metadata face_count=',
                         b'<metadata key="sparse_infill_density" value="99%"/><metadata face_count='))
        self.reject()

    def test_plate_override_rejected(self):
        self.rewrite('modifier', 'cubic', 'Metadata/model_settings.config',
                     lambda data: data.replace(b'<plate>', b'<plate><metadata key="layer_height" value="1"/>'))
        self.reject()

    def test_geometry_pair_mutation_rejected(self):
        self.rewrite('modifier', 'cubic', '3D/3dmodel.model',
                     lambda data: data.replace(b'x="24" y="14"', b'x="23" y="14"', 1))
        self.reject()

    def test_transform_mismatch_rejected(self):
        self.rewrite('modifier', 'cubic', '3D/3dmodel.model',
                     lambda data: data.replace(b'1 36 14 9', b'1 35 14 9'))
        self.reject()

    def test_modifier_model_type_rejected(self):
        self.rewrite('modifier', 'cubic', '3D/3dmodel.model',
                     lambda data: data.replace(b'id="2" type="other"', b'id="2" type="model"'))
        self.reject()

    def test_local_arbitrary_metadata_rejected(self):
        self.rewrite('modifier', 'cubic', 'Metadata/model_settings.config',
                     lambda data: data.replace(b'<mesh_stat', b'<metadata key="foo" value="bar"/><mesh_stat', 1))
        self.reject()

    def test_nonfunctional_date_only_allowed(self):
        self.rewrite('modifier', 'cubic', '3D/3dmodel.model',
                     lambda data: re.sub(rb'(<metadata name="(?:CreationDate|ModificationDate)">)\d{4}-\d{2}-\d{2}',
                                         rb'\g<1>2000-01-01', data))
        self.pair()

    def test_arbitrary_metadata_pair_change_rejected(self):
        self.rewrite('modifier', 'cubic', '3D/3dmodel.model',
                     lambda data: data.replace(b'<metadata name="Title"></metadata>', b'<metadata name="Title">Different</metadata>'))
        self.reject()

    def test_common_json_density_mismatch_rejected(self):
        for side in ('cubic', 'bridgeawarecubic'):
            path = self.root / f'modifier-{side}.config.json'
            settings = json.loads(path.read_text())
            settings['sparse_infill_density'] = '99%'
            path.write_text(json.dumps(settings))
            self.rewrite('modifier', side, 'Metadata/project_settings.config', lambda data: json.dumps(settings).encode())
        self.reject()

    def test_other_global_pair_difference_rejected(self):
        path = self.root / 'modifier-cubic.config.json'
        settings = json.loads(path.read_text())
        settings['wall_loops'] = '3'
        path.write_text(json.dumps(settings))
        self.rewrite('modifier', 'cubic', 'Metadata/project_settings.config', lambda data: json.dumps(settings).encode())
        self.reject()

    def test_manifest_local_config_tamper_rejected(self):
        path = self.root / 'manifest.json'
        manifest = json.loads(path.read_text())
        record = next(record for record in manifest['fixtures'] if record['name'] == 'modifier-bridgeawarecubic')
        record['volumes'][1]['serialized_config']['sparse_infill_density'] = '99%'
        path.write_text(json.dumps(manifest))
        self.reject()

    def test_nonnil_expected_map(self):
        path = self.root / 'manifest.json'
        manifest = json.loads(path.read_text())
        record = next(record for record in manifest['fixtures'] if record['name'] == 'modifier-cubic')
        record['nil_config_keys'] = ['filename_format']
        self.assertIn('filename_format', record['serialized_config'])
        path.write_text(json.dumps(manifest))
        expected = audit.load_expected(self.root, 'modifier-cubic.3mf')
        self.assertNotIn('filename_format', expected)
        self.assertFalse(set(expected) & audit.BANNED_CONFIG_KEYS)

    def test_ini_and_manifest_mismatch_rejected(self):
        path = self.root / 'modifier-cubic.config.ini'
        path.write_text(path.read_text().replace('wall_loops = 2', 'wall_loops = 3'))
        self.reject()

    def test_duplicate_expected_json_key_rejected(self):
        path = self.root / 'modifier-cubic.config.json'
        path.write_text(path.read_text().replace('{', '{"wall_loops":"2",', 1))
        self.reject()


if __name__ == '__main__':
    unittest.main(verbosity=2)
