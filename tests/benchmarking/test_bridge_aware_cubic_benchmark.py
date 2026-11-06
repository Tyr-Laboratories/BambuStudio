"""Pure Python benchmark guard tests. Never launches a slicer or build."""
import copy
import importlib.util
import json
import shutil
from pathlib import Path
import tempfile
import unittest
from unittest import mock

REPO_ROOT = Path(__file__).resolve().parents[2]
TOOL_ROOT = REPO_ROOT / 'scripts' / 'bridge_aware_cubic_benchmark'
DATA_ROOT = REPO_ROOT / 'tests' / 'data' / 'bridge_aware_cubic_benchmark'
spec = importlib.util.spec_from_file_location('benchmark_under_test', TOOL_ROOT / 'run_benchmark.py')
r = importlib.util.module_from_spec(spec)
spec.loader.exec_module(r)
FIXTURES = DATA_ROOT / 'projects'
RESULT_SCHEMA = DATA_ROOT / 'official-result-schema.json'

class BenchmarkV2Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixtures = {size: r.validate_fixture_pair(FIXTURES / f'{size}-cubic.3mf', FIXTURES / f'{size}-bridgeawarecubic.3mf') for size in r.REQUIRED_CASES}
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='bac-v2-unit-')
        self.path = Path(self.tmp.name)
        self.block_launch = mock.patch.object(r.subprocess, 'Popen', side_effect=AssertionError('Subprocess launches forbidden in pure tests'))
        self.block_launch.start()
    def tearDown(self):
        self.block_launch.stop()
        self.tmp.cleanup()
    def result(self, size='small'):
        f = self.fixtures[size]
        result = json.loads(RESULT_SCHEMA.read_text())
        result['sparse_infill_density'] = f['density_percent']
        p = result['sliced_plates'][0]
        p['triangle_count'] = f['model_triangle_count']
        o = p['objects'][0]
        o['name'] = 'BAC benchmark ' + size
        o['triangle_count'] = f['model_triangle_count']
        o['bbox'] = dict(zip(('x','y','z','width','depth','height'), f['bounds_min_mm'] + f['dimensions_mm']))
        return result
    def validate(self, result, size='small'):
        path = self.path / 'result.json'; path.write_text(json.dumps(result))
        return r.validate_result(path, self.fixtures[size])
    def gcode(self, count=90, alter=None):
        lines = ['; CONFIG_BLOCK_START'] + [f'; key_{i} = value' for i in range(60)] + ['; CONFIG_BLOCK_END', f'; total layer number: {count}', 'G90','G21','M83','G1 X0 Y0 Z0']
        for layer in range(1,count+1):
            lines += ['; CHANGE_LAYER',f'; Z_HEIGHT: {layer*.2:.1f}',f'G1 Z{layer*.2:.1f}',f'G1 X{layer} Y0 E1']
        text='\n'.join(lines)+'\n'
        path=self.path/'test.gcode';path.write_text(alter(text) if alter else text);return path
    def report(self, sizes=None, development=False):
        sizes = list(r.REQUIRED_CASES) if sizes is None else sizes
        report = {'sizes': sizes, 'fixtures': self.fixtures, 'runs': [], 'pairs': [], 'development_mode': development, 'official_only': False}
        for size in sizes:
            for warmup,count in ((True,1),(False,5)):
                for i in range(1,count+1):
                    for side,ms in (('official',100),('fork',150)):
                        report['runs'].append({'size':size,'side':side,'warmup':warmup,'iteration':i,'status':'passed_individual_checks',
                            'intended_config_check':{'rejected_differences':{}},'slice_ms':ms,'whole_process_wall_s':99})
                    report['pairs'].append({'size':size,'warmup':warmup,'iteration':i,'slice_ratio':1.5,
                        'order':['official','fork'] if i%2 else ['fork','official'],'parity':{'rejected_differences':{}}})
        return report
    def test_reviewed_expected_guards_all_fixtures(self):
        for fixture in self.fixtures.values():
            for side in ('cubic','bac'):
                record=fixture[side];r.validate_reviewed_guards(record['expected_serialized_config'],local_configs=record['local_configs'])
    def test_reviewed_actual_guards_recorded_official_config(self):
        actual=json.loads((DATA_ROOT/'official-emitted-config.json').read_text())
        r.validate_reviewed_guards(self.fixtures['small']['cubic']['expected_serialized_config'],actual,[{}])
    def test_reviewed_guard_rejects_active_feature(self):
        expected=copy.deepcopy(self.fixtures['small']['cubic']['expected_serialized_config']);expected['enable_support']='1'
        with self.assertRaises(r.InvalidBenchmark):r.validate_reviewed_guards(expected,local_configs=[{}])
    def test_reviewed_guard_rejects_local_override(self):
        expected=self.fixtures['modifier']['cubic']['expected_serialized_config']
        with self.assertRaises(r.InvalidBenchmark):r.validate_reviewed_guards(expected,local_configs=[{}, {'enable_prime_tower':'1'}])
    def test_full_preflight_launches_nothing(self):
        binary=self.path/'dummy-executable';binary.write_bytes(b'not launched')
        output=self.path/'preflight';snapshot={'head':'a'*40,'snapshot_sha256':'f'*64}
        with mock.patch.object(r,'source_snapshot',return_value=snapshot):
            code=r.main(['--official',str(binary),'--fork',str(binary),'--fixtures',str(FIXTURES),
                         '--output',str(output),'--repo',str(self.path),'--check-inputs-only'])
        self.assertEqual(code,0)
        report=json.loads((output/'benchmark.json').read_text())
        self.assertEqual(report['required_release_cases'],list(r.REQUIRED_CASES))
        self.assertEqual(report['reviewed_exception_guards']['sha256'],r.GUARDS_LOADED_SHA256)
        self.assertEqual(report['fixture_audit_helper']['sha256'],r.HELPER_LOADED_SHA256)
        self.assertFalse(report['gate_passed']);self.assertEqual(report['runs'],[])
        for fixture in report['fixtures'].values():
            for side in ('cubic','bac'):self.assertEqual(fixture[side]['reviewed_expected_guards'],'passed')
    def test_native_manifest_all_families(self):
        self.assertEqual({x['family'] for x in self.fixtures.values()}, {'cube','rib','modifier'})
        self.assertEqual(self.fixtures['modifier']['bac']['local_configs'][1][r.GAP], '3')
    def test_recorded_official_result_schema(self):
        result,ms=self.validate(json.loads(RESULT_SCHEMA.read_text()));self.assertIsInstance(ms,int);self.assertGreater(ms,0)
    def test_all_family_result_schemas(self):
        for size in r.REQUIRED_CASES:
            with self.subTest(size=size):self.validate(self.result(size),size)
    def test_modifier_total_triangles_not_runtime_model_count(self):
        for where in ('plate','object'):
            result=self.result('modifier');plate=result['sliced_plates'][0]
            (plate if where=='plate' else plate['objects'][0])['triangle_count']=24
            with self.subTest(where=where), self.assertRaises(r.InvalidBenchmark):self.validate(result,'modifier')
    def test_rib_cannot_report_cube_triangles(self):
        result=self.result('rib');result['sliced_plates'][0]['triangle_count']=12
        with self.assertRaises(r.InvalidBenchmark):self.validate(result,'rib')
    def test_count_ids_require_integers(self):
        for key in ('id','obj_cached_cnt','triangle_count'):
            result=self.result();result['sliced_plates'][0][key]=True
            with self.subTest(key=key),self.assertRaises(r.InvalidBenchmark):self.validate(result)
    def test_wrong_bbox_rejected(self):
        for value in (81.,float('nan'),float('inf'),'80'):
            result=self.result();result['sliced_plates'][0]['objects'][0]['bbox']['x']=value
            with self.subTest(value=value),self.assertRaises(r.InvalidBenchmark):self.validate(result)
    def test_runtime_object_ids_may_differ(self):
        result=self.result();result['sliced_plates'][0]['objects'][0]['id']=98765;self.validate(result)
    def test_timing_requires_positive_integer(self):
        for value in (0,-1,True,1.5,float('inf'),'100'):
            result=self.result();result['sliced_plates'][0]['times']['sliced_time']=value
            with self.subTest(value=value),self.assertRaises(r.InvalidBenchmark):self.validate(result)
    def test_modifier_global_density_not_local(self):
        result=self.result('modifier');result['sparse_infill_density']=12
        with self.assertRaises(r.InvalidBenchmark):self.validate(result,'modifier')
    def test_extra_objects_and_plates_rejected(self):
        for location in ('objects','plates'):
            result=self.result();items=result['sliced_plates'][0]['objects'] if location=='objects' else result['sliced_plates'];items.append(copy.deepcopy(items[0]))
            with self.subTest(location=location),self.assertRaises(r.InvalidBenchmark):self.validate(result)
    def test_dynamic_actual_layer_counts(self):
        for count in (75,90):
            config,stats=r.parse_gcode(self.gcode(count),count);self.assertEqual(stats['layers'],count);self.assertEqual(stats['moving_extrusion_segments'],count)
    def test_wrong_rib_layer_count_rejected(self):
        with self.assertRaises(r.InvalidBenchmark):r.parse_gcode(self.gcode(90),75)
    def test_fake_z_comments_rejected(self):
        with self.assertRaises(r.InvalidBenchmark):r.parse_gcode(self.gcode(75,lambda text:r.re.sub(r'G1 Z[0-9.]+','G1 Z0',text)),75)
    def test_missing_actual_layer_rejected(self):
        with self.assertRaises(r.InvalidBenchmark):r.parse_gcode(self.gcode(75,lambda text:text.replace('G1 X75 Y0 E1','G1 X75 Y0 E0')),75)
    def test_xyz_reset_rejected(self):
        with self.assertRaises(r.InvalidBenchmark):r.parse_gcode(self.gcode(75,lambda text:text.replace('G1 Z','G92 Z')),75)
    def test_nonfinite_motion_rejected(self):
        with self.assertRaises(r.InvalidBenchmark):r.parse_gcode(self.gcode(75,lambda text:text.replace('G1 X1 ','G1 Xnan ',1)),75)
    def test_exact_family_gap_parity(self):
        cubic={r.PATTERN:'cubic'};bac={r.PATTERN:'bridgeawarecubic',r.GAP:'8'}
        self.assertFalse(r.compare_emitted(cubic,bac,{},8)['rejected_differences'])
        self.assertIn(r.GAP,r.compare_emitted(cubic,bac,{},5)['rejected_differences'])
    def test_no_blanket_config_exception(self):
        self.assertIn('wrong',r.compare_intended({'wrong':'expected'},{'wrong':'actual'}, {})['rejected_differences'])
        self.assertIn('wrong',r.compare_emitted({'wrong':'expected'},{'wrong':'actual'}, {},5)['rejected_differences'])
    def test_all4_complete_can_pass(self):
        report=self.report();r.summarize(report,5,1);self.assertTrue(report['gate_passed']);self.assertEqual(report['release_cases_pending'],[])
    def test_cube_only_cannot_pass_release(self):
        report=self.report(['small','large']);r.summarize(report,5,1);self.assertTrue(report['scope_gate_passed']);self.assertFalse(report['gate_passed']);self.assertEqual(report['release_coverage_pending'],['rib','modifier'])
    def test_all_families_without_large_cannot_pass(self):
        report=self.report(['small','rib','modifier']);r.summarize(report,5,1);self.assertFalse(report['gate_passed']);self.assertEqual(report['release_cases_pending'],['large'])
    def test_mixed_legacy_manifest_either_side_cannot_pass(self):
        copied=self.path/'fixtures';shutil.copytree(FIXTURES,copied)
        original=json.loads((copied/'manifest.json').read_text())
        for pattern in ('cubic','bridgeawarecubic'):
            manifest=copy.deepcopy(original)
            record=next(record for record in manifest['fixtures'] if record['name']=='small-'+pattern)
            for key in ('family','model_triangle_count','volume_count','volumes','filament_count'):record.pop(key)
            (copied/'manifest.json').write_text(json.dumps(manifest))
            pair=r.validate_fixture_pair(copied/'small-cubic.3mf',copied/'small-bridgeawarecubic.3mf')
            self.assertTrue(pair['legacy_cube_manifest'])
            self.assertTrue(pair['cubic' if pattern=='cubic' else 'bac']['legacy_cube_manifest'])
            report=copy.deepcopy(self.report());report['fixtures']['small']=pair
            r.summarize(report,5,1);self.assertFalse(report['gate_passed'])
    def test_legacy_derivative_cannot_pass_release(self):
        report=copy.deepcopy(self.report());report['fixtures']['small']['legacy_cube_manifest']=True
        r.summarize(report,5,1);self.assertFalse(report['gate_passed'])
    def test_expected_fixed_settings_all_families(self):
        for fixture in self.fixtures.values():
            for side,key in (('official','cubic'),('fork','bac')):
                r.check_emitted_config(fixture[key]['expected_serialized_config'],side,fixture)
    def test_development_cannot_pass(self):
        report=self.report(development=True);r.summarize(report,5,1);self.assertFalse(report['gate_passed'])
    def test_worst_sample_and_median_both_gate(self):
        report=self.report();pair=next(p for p in report['pairs'] if p['size']=='rib' and not p['warmup']);pair['slice_ratio']=2.01
        run=next(p for p in report['runs'] if p['size']=='rib' and not p['warmup'] and p['side']=='fork' and p['iteration']==pair['iteration']);run['slice_ms']=201
        r.summarize(report,5,1);self.assertFalse(report['gate_passed']);self.assertEqual(report['summary']['rib']['median_slice_ratio'],1.5)
    def test_missing_warmup_rejected(self):
        report=self.report();report['runs']=[run for run in report['runs'] if not(run['size']=='rib' and run['warmup'])]
        with self.assertRaises(r.InvalidBenchmark):r.summarize(report,5,1)
    def test_duplicate_pair_rejected(self):
        report=self.report();report['pairs'].append(copy.deepcopy(report['pairs'][0]))
        with self.assertRaises(r.InvalidBenchmark):r.summarize(report,5,1)
    def test_config_failure_rejected(self):
        report=self.report();report['runs'][0]['intended_config_check']['rejected_differences']={'density':{}}
        with self.assertRaises(r.InvalidBenchmark):r.summarize(report,5,1)
    def test_non_alternating_order_rejected(self):
        report=self.report();report['pairs'][0]['order']=['fork','official']
        with self.assertRaises(r.InvalidBenchmark):r.summarize(report,5,1)
    def test_fabricated_ratio_rejected(self):
        report=self.report();report['pairs'][0]['slice_ratio']=1.0
        with self.assertRaises(r.InvalidBenchmark):r.summarize(report,5,1)
    def test_source_snapshot_includes_clean_head(self):
        def snapshot(head):
            with mock.patch.object(r.subprocess,'check_output',side_effect=[head.encode()+b'\n',b'',b'',b'']):return r.source_snapshot(self.path)
        self.assertNotEqual(snapshot('a'*40)['snapshot_sha256'],snapshot('b'*40)['snapshot_sha256'])
    def test_cocoa_profile_denies_live_state_and_network(self):
        path=self.path/'test.sb';r.sandbox_profile(path,self.path/'unavailable-live-home');text=path.read_text()
        self.assertIn('(deny network*)',text);self.assertIn('com.bambulab.bambu-studio.savedState',text)
        self.assertIn('user-preference-read user-preference-write',text);self.assertIn('Application Support/BambuStudio',text)

if __name__ == '__main__':unittest.main(verbosity=2)
