"""Pure Python benchmark guard tests. Never launches a slicer or build."""
import contextlib
import copy
import importlib.util
import io
import json
import re
import shutil
from pathlib import Path
import tempfile
import types
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
BASELINES = TOOL_ROOT / 'baselines' / 'official-2.9.1.70'

# What the real commands leave behind, captured from the matrix.sh runs of M-1's instruction measurement (official 2.9.1.70
# and the fork's release app, small fixtures, iteration 1). `/usr/bin/time -l` of the official app's full slice (--slice 1):
REAL_TIME_REPORT = '''\
        0.49 real         0.37 user         0.12 sys
           126156800  maximum resident set size
                   0  average shared memory size
                   0  average unshared data size
                   0  average unshared stack size
               13116  page reclaims
                  12  page faults
                   0  swaps
                   0  block input operations
                   0  block output operations
                   1  messages sent
                   0  messages received
                   0  signals received
                 106  voluntary context switches
                5118  involuntary context switches
          3968020713  instructions retired
          1332196160  cycles elapsed
           308773992  peak memory footprint
'''
# The startup-and-load process (--slice 99) of the official app and of the fork: stdout, exit status 254, and result.json.
LOAD_STDOUT = {
    'official': ('[2026-11-08 03:24:32.241672] [0x00000001ee616180] [error]   [BambuStudio.cpp:5718 run] invalid plate id 99, total 1\n'
                 'run found error, exit\n'),
    'fork': ('[2026-11-08 03:24:32.391489] [0x00000001ee616180] [trace]   Initializing StaticPrintConfigs\n'
             '[2026-11-08 03:24:32.402126] [0x00000001ee616180] [error]   invalid plate id 99, total 1\n'
             'run found error, exit\n')}
LOAD_RESULT = {'error_string': 'Invalid parameters to the slicer.', 'layer_height': 0.0, 'peak_rss_mb': 47.78125, 'plate_index': 0,
               'return_code': -2, 'sparse_infill_density': 0.0,
               'times': {'export_3mf_time': 0, 'export_slicedata_time': 0, 'export_time': 4294902280, 'import_time': 3,
                         'load_slicedata_time': 0, 'prepare_time': 4269197492224},
               'wall_loops': 0}
LOAD_EXIT_STATUS = 254

def time_report(instructions):
    """The captured `/usr/bin/time -l` report with another instructions retired figure."""
    return REAL_TIME_REPORT.replace('3968020713', str(instructions))

def gcode_with_config(config, layers):
    """Minimal valid G-code that carries the given emitted configuration block."""
    lines = ['; CONFIG_BLOCK_START'] + [f'; {key} = {value}' for key, value in config.items()] + [
        '; CONFIG_BLOCK_END', f'; total layer number: {layers}', 'G90', 'G21', 'M83', 'G1 X0 Y0 Z0']
    for layer in range(1, layers + 1):
        lines += ['; CHANGE_LAYER', f'; Z_HEIGHT: {layer * .2:.1f}', f'G1 Z{layer * .2:.1f}', f'G1 X{layer} Y0 E1']
    return '\n'.join(lines) + '\n'

class FakeSlicer:
    """Stands in for subprocess.Popen in the tests that run run_one and main without a slicer.

    A launch of `/usr/bin/time -l -o FILE ... APP ... --slice N FIXTURE` leaves what the real command leaves: time's report in
    FILE, the two logs, result.json and, for a full slice, G-code carrying the app's recorded small-fixture configuration.
    Process instruction totals and sliced_time are per arm (official, fork on BAC, fork on Cubic); tests may replace them."""
    def __init__(self, apps):
        self.apps = apps
        self.launches = []
        official = json.loads((DATA_ROOT / 'official-emitted-config.json').read_text())
        fork = json.loads((DATA_ROOT / 'fork-emitted-config.json').read_text())
        self.configs = {'official': official, 'fork': fork, 'fork_cubic': {**fork, r.PATTERN: 'cubic'}}
        self.result = json.loads(RESULT_SCHEMA.read_text())
        # (full slice, load) instructions retired by one run's two processes; slice instructions 3.2 G, 4.8 G and 3.12 G.
        self.totals = {'official': (4_000_000_000, 800_000_000), 'fork': (5_600_000_000, 800_000_000),
                       'fork_cubic': (3_904_000_000, 784_000_000)}
        self.sliced_ms = {'official': 300, 'fork': 450, 'fork_cubic': 310}
        self.omit_instructions = set()   # modes ('slice', 'load') whose time report lacks the instructions line
        self.slice_status = 0
        self.load_status = LOAD_EXIT_STATUS
        self.load_stdout = None          # replaces the app's recorded stdout of the load process
        self.load_result = None          # replaces result.json of the load process
        self.load_extra_files = {}       # files the load process leaves in its output directory
    def __call__(self, command, cwd=None, env=None, stdout=None, stderr=None, start_new_session=False):
        executable = command[command.index('--datadir') - 1]
        launch = {'command': list(command), 'cwd': cwd, 'env': dict(env), 'start_new_session': start_new_session,
                  'executable': executable, 'app': self.apps[executable], 'plate': int(command[command.index('--slice') + 1]),
                  'fixture': command[-1], 'output': Path(command[command.index('--outputdir') + 1]),
                  'time_file': Path(command[command.index('-o') + 1])}
        launch['pattern'] = 'bac' if 'bridgeawarecubic' in launch['fixture'] else 'cubic'
        launch['arm'] = 'official' if launch['app'] == 'official' else 'fork' if launch['pattern'] == 'bac' else 'fork_cubic'
        launch['mode'] = 'slice' if launch['plate'] == 1 else 'load'
        self.launches.append(launch)
        self.perform(launch, stdout)
        return types.SimpleNamespace(pid=-1, wait=lambda timeout=None: launch['returncode'])
    def perform(self, launch, stdout):
        arm, mode = launch['arm'], launch['mode']
        report = time_report(self.totals[arm][0 if mode == 'slice' else 1])
        if mode in self.omit_instructions:
            report = re.sub(r'.*instructions retired\n', '', report)
        launch['time_file'].write_text(report)
        output = launch['output']
        if mode == 'slice':
            launch['returncode'] = self.slice_status
            stdout.write(b'slice log\n')
            result = copy.deepcopy(self.result)
            result['sliced_plates'][0]['times']['sliced_time'] = self.sliced_ms[arm]
            (output / 'result.json').write_text(json.dumps(result))
            (output / 'plate_1.gcode').write_text(gcode_with_config(self.configs[arm], 90))
        else:
            launch['returncode'] = self.load_status
            text = LOAD_STDOUT['official' if launch['app'] == 'official' else 'fork'] if self.load_stdout is None else self.load_stdout
            stdout.write(text.encode())
            (output / 'result.json').write_text(json.dumps(LOAD_RESULT if self.load_result is None else self.load_result))
            for name, content in self.load_extra_files.items():
                (output / name).write_text(content)

class BenchmarkV2Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixtures = {size: r.validate_fixture_pair(FIXTURES / f'{size}-cubic.3mf', FIXTURES / f'{size}-bridgeawarecubic.3mf') for size in r.REQUIRED_CASES}
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='bac-v2-unit-')
        self.path = Path(self.tmp.name)
        self.block_launch = mock.patch.object(r.subprocess, 'Popen', side_effect=AssertionError('Subprocess launches forbidden in pure tests'))
        self.block_launch.start()
        # A fake process has no real process group; nothing may ever be killed by a test.
        self.block_kill = mock.patch.object(r.os, 'killpg', side_effect=AssertionError('Process-group kills forbidden in pure tests'))
        self.block_kill.start()
    def tearDown(self):
        self.block_kill.stop()
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
    # Synthetic report builder. Every run is two processes of one app on one fixture: the full slice (--slice 1) and the
    # startup-and-load process (--slice 99), with raw instructions retired for both. The default counts give slice instructions
    # of 3.2 G (official Cubic), 4.8 G (fork BAC) and 3.12 G (fork Cubic): gate ratio 1.5, build parity 0.975. The wall-clock
    # sliced_time figures (100, 150 and 105 ms) are deliberately unrelated to them.
    OFFICIAL_APP, FORK_APP = '/apps/official/BambuStudio', '/apps/fork/BambuStudio'
    DEFAULT_COUNTS = {'official': (4_000_000_000, 800_000_000, 100), 'fork': (5_600_000_000, 800_000_000, 150),
                      'fork_cubic': (3_904_000_000, 784_000_000, 105)}
    def measured_run(self, size, side, warmup, iteration, full, load, ms):
        app = self.OFFICIAL_APP if side == 'official' else self.FORK_APP
        fixture = self.fixtures[size]['bac' if side == 'fork' else 'cubic']['path']
        return {'size': size, 'side': side, 'warmup': warmup, 'iteration': iteration, 'status': 'passed_individual_checks',
                'intended_config_check': {'rejected_differences': {}}, 'slice_ms': ms, 'whole_process_wall_s': 99,
                'slice_process': {'plate_arg': 1, 'executable': app, 'fixture': fixture, 'returncode': 0, 'instructions_retired': full},
                'load_process': {'plate_arg': 99, 'executable': app, 'fixture': fixture, 'returncode': 254,
                                 'instructions_retired': load, 'output_check': 'passed'},
                'slice_instructions': full - load}
    def pair_run(self, report, pair, side):
        return next(run for run in report['runs'] if run['size'] == pair['size'] and run['warmup'] is pair['warmup']
                    and run['iteration'] == pair['iteration'] and run['side'] == side)
    def refresh_pairs(self, report):
        """Recompute every pair's reported ratios from its runs' figures, as the tool does."""
        for pair in report['pairs']:
            official, fork, cubic = (self.pair_run(report, pair, side) for side in ('official', 'fork', 'fork_cubic'))
            pair['slice_ratio'] = fork['slice_instructions'] / official['slice_instructions']
            pair['build_parity_ratio'] = cubic['slice_instructions'] / official['slice_instructions']
            pair['sliced_time_ratio_information_only'] = fork['slice_ms'] / official['slice_ms']
    def set_counts(self, report, size, iteration, side, full=None, load=None, ms=None, warmup=False):
        """Change one run's raw counts or sliced_time and keep its derived figures and its pair's ratios consistent."""
        run = next(run for run in report['runs'] if (run['size'], run['side'], run['iteration'], run['warmup']) == (size, side, iteration, warmup))
        if full is not None:
            run['slice_process']['instructions_retired'] = full
        if load is not None:
            run['load_process']['instructions_retired'] = load
        if ms is not None:
            run['slice_ms'] = ms
        run['slice_instructions'] = run['slice_process']['instructions_retired'] - run['load_process']['instructions_retired']
        self.refresh_pairs(report)
        return run
    def report(self, sizes=None, development=False):
        sizes = list(r.REQUIRED_CASES) if sizes is None else sizes
        report = {'sizes': sizes, 'fixtures': self.fixtures, 'runs': [], 'pairs': [], 'development_mode': development, 'official_only': False}
        for size in sizes:
            for warmup,count in ((True,1),(False,5)):
                for i in range(1,count+1):
                    for side, (full, load, ms) in self.DEFAULT_COUNTS.items():
                        report['runs'].append(self.measured_run(size, side, warmup, i, full, load, ms))
                    report['pairs'].append({'size':size,'warmup':warmup,'iteration':i,
                        'order':['official','fork'] if i%2 else ['fork','official'],
                        'execution_order':['official','fork','fork_cubic'] if i%2 else ['fork_cubic','fork','official'],
                        'parity':{'rejected_differences':{}},'build_parity_config':{'rejected_differences':{}}})
        self.refresh_pairs(report)
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
        with mock.patch.object(r,'source_snapshot',return_value=snapshot), contextlib.redirect_stdout(io.StringIO()):
            code=r.main(['--official',str(binary),'--fork',str(binary),'--fixtures',str(FIXTURES),
                         '--output',str(output),'--repo',str(self.path),'--check-inputs-only'])
        self.assertEqual(code,0)
        report=json.loads((output/'benchmark.json').read_text())
        self.assertEqual(report['schema_version'],3)
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
        report=self.report();self.set_counts(report,'rib',3,'fork',full=7_232_000_000)   # slice instructions 6.432 G over 3.2 G: pair ratio 2.01
        r.summarize(report,5,1);self.assertFalse(report['gate_passed']);self.assertEqual(report['summary']['rib']['median_slice_ratio'],1.5)
        self.assertEqual(report['summary']['rib']['worst_observed_pair_ratio'],2.01)
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

    # ---- Instructions retired: parsing /usr/bin/time -l, the subtraction and the load process ----
    def test_time_output_instructions_retired_parsed(self):
        self.assertEqual(r.parse_time_output(REAL_TIME_REPORT), 3968020713)
        self.assertEqual(r.parse_time_output(time_report(765098538)), 765098538)
        self.assertEqual(r.parse_time_output('  12  instructions retired'), 12)
    def test_time_output_without_instructions_line_fails_clearly(self):
        gnu = ('\tCommand being timed: "slicer"\n\tUser time (seconds): 0.37\n\tSystem time (seconds): 0.12\n'
               '\tMaximum resident set size (kbytes): 123200\n\tExit status: 0\n')
        usage = "time: invalid option -- 'l'\nusage: time [-apvV] [-f format] [-o file] command [argument ...]\n"
        cases = {'empty output': '', 'only the elapsed-time line': REAL_TIME_REPORT.splitlines()[0],
                 'macOS report without the line': REAL_TIME_REPORT.replace('          3968020713  instructions retired\n', ''),
                 'not a count': REAL_TIME_REPORT.replace('3968020713', 'n/a'), 'GNU time': gnu, 'time rejecting -l': usage}
        for name, text in cases.items():
            with self.subTest(name):
                with self.assertRaises(r.InvalidBenchmark) as caught:
                    r.parse_time_output(text)
                for phrase in ('instructions retired', '/usr/bin/time -l', 'macOS'):
                    self.assertIn(phrase, str(caught.exception))
    def test_time_output_unusable_instruction_figures_rejected(self):
        for name, text in (('zero', time_report(0)), ('reported twice', REAL_TIME_REPORT + '           5  instructions retired\n')):
            with self.subTest(name), self.assertRaises(r.InvalidBenchmark):
                r.parse_time_output(text)
    def test_slice_instructions_is_full_slice_minus_load(self):
        # The official small Cubic run of M-1's matrix: full slice and startup-and-load process.
        self.assertEqual(r.slice_instructions(3968020713, 765098538), 3202922175)
        self.assertEqual(r.slice_instructions(765098539, 765098538), 1)
    def test_slice_instructions_rejects_unusable_counts(self):
        for full, load in ((765098538, 765098538), (700_000_000, 765098538), (0, 5), (5, 0), (-5, 3), (True, 1), (10.0, 5), ('10', 5), (None, 5)):
            with self.subTest(full=full, load=load), self.assertRaises(r.InvalidBenchmark):
                r.slice_instructions(full, load)
    def load_process_directory(self, name, stdout=LOAD_STDOUT['official'], result=LOAD_RESULT, extra=None):
        """A load process directory as the app leaves it (stdout.log and output/), for check_load_process."""
        directory = self.path / name
        (directory / 'output').mkdir(parents=True)
        (directory / 'stdout.log').write_text(stdout)
        if result is not None:
            (directory / 'output' / 'result.json').write_text(result if isinstance(result, str) else json.dumps(result))
        for filename, content in (extra or {}).items():
            (directory / 'output' / filename).write_text(content)
        return directory
    def test_load_process_accepts_exactly_what_slice_99_gives(self):
        for app in ('official', 'fork'):
            with self.subTest(app=app):
                r.check_load_process(LOAD_EXIT_STATUS, self.load_process_directory('load-' + app, LOAD_STDOUT[app]), 1)
    def test_load_process_rejects_anything_else(self):
        accepted = LOAD_STDOUT['official']
        cases = {'exit status 0': {'returncode': 0}, 'exit status 1': {'returncode': 1}, 'exit status 2': {'returncode': 2},
                 'exit status 139': {'returncode': 139}, 'killed by a signal': {'returncode': -11},
                 'plate message missing': {'stdout': 'run found error, exit\n'},
                 'a different plate id': {'stdout': accepted.replace('id 99', 'id 98')},
                 'a different plate total': {'stdout': accepted.replace('total 1', 'total 2')},
                 'no exit line': {'stdout': accepted.replace('run found error, exit\n', '')},
                 'result.json missing': {'result': None},
                 'result.json not JSON': {'result': 'not json'},
                 'result.json says success': {'result': {**LOAD_RESULT, 'return_code': 0, 'error_string': 'Success.'}},
                 'result.json other failure code': {'result': {**LOAD_RESULT, 'return_code': -1}},
                 'result.json other message': {'result': {**LOAD_RESULT, 'error_string': 'Failed to load the project.'}},
                 'result.json a plate was requested': {'result': {**LOAD_RESULT, 'plate_index': 1}},
                 'result.json lists sliced plates': {'result': {**LOAD_RESULT, 'sliced_plates': []}},
                 'G-code left behind': {'extra': {'plate_1.gcode': '; sliced'}},
                 'another file left behind': {'extra': {'plate_1.3mf': 'archive'}}}
        for name, case in cases.items():
            directory = self.load_process_directory(re.sub(r'\W+', '-', name), case.get('stdout', accepted), case.get('result', LOAD_RESULT), case.get('extra'))
            returncode = case.get('returncode', LOAD_EXIT_STATUS)
            with self.subTest(name), self.assertRaises(r.InvalidBenchmark):
                r.check_load_process(returncode, directory, 1)

    # ---- run_one and main with a fake slicer: two processes per run, same app, same fixture ----
    def run_setup(self, name='out'):
        # Resolved, as main() resolves its executable arguments: the fake recognises an app by the path in the command.
        official, fork = (self.path / (name + '-official-exe')).resolve(), (self.path / (name + '-fork-exe')).resolve()
        official.write_bytes(b'official app')
        fork.write_bytes(b'fork app')
        args = types.SimpleNamespace(output=self.path / name, official=official, fork=fork, fixtures=FIXTURES, timeout=3600,
                                     intended_exceptions=r.load_intended_exceptions(BASELINES / 'intended_exceptions.json'))
        args.output.mkdir()
        return args, {'runs': []}, FakeSlicer({str(official): 'official', str(fork): 'fork'})
    def run_one(self, args, report, slicer, side, iteration=1, warmup=False, size='small'):
        with mock.patch.object(r.subprocess, 'Popen', slicer), mock.patch.object(r, 'verify_frozen_inputs'), \
                contextlib.redirect_stdout(io.StringIO()) as printed:
            entry, config = r.run_one(args, report, size, side, iteration, warmup, self.fixtures)
        return entry, config, printed.getvalue()
    def test_each_run_is_two_processes_of_the_same_app_on_the_same_fixture(self):
        args, report, slicer = self.run_setup()
        entry, config, printed = self.run_one(args, report, slicer, 'official', iteration=2)
        slice_launch, load_launch = slicer.launches
        self.assertEqual((slice_launch['plate'], load_launch['plate']), (1, 99))
        self.assertEqual((slice_launch['mode'], load_launch['mode']), ('slice', 'load'))
        profile = str(args.output / 'small-sample-02-official' / 'sandbox.sb')
        def stripped(launch):
            """The command with its own directory, the shared sandbox profile and the plate id blanked out."""
            command = ['<profile>' if part == profile else part.replace(str(launch['output'].parent), '<directory>')
                       for part in launch['command']]
            command[command.index('--slice') + 1] = '<plate>'
            return command
        self.assertEqual(stripped(slice_launch), stripped(load_launch))
        for launch in slicer.launches:
            command = launch['command']
            self.assertEqual(command[:3], ['/usr/bin/time', '-l', '-o'])
            self.assertEqual(command[command.index('--datadir') - 1], str(args.official))
            self.assertEqual(launch['fixture'], self.fixtures['small']['cubic']['path'])
            self.assertTrue(launch['start_new_session'])
            directory = launch['output'].parent
            self.assertEqual(launch['cwd'], directory)
            self.assertEqual(launch['time_file'], directory / 'time.txt')
            self.assertEqual(launch['env']['TMPDIR'], str(directory / 'tmp') + '/')
            self.assertEqual(launch['env']['CFFIXED_USER_HOME'], str(directory / 'cocoa-home'))
            self.assertEqual(launch['env']['LC_ALL'], 'C')
        # Fresh, separate state for the two processes; the slice process keeps the layout of earlier reports.
        self.assertEqual(slice_launch['output'], args.output / 'small-sample-02-official' / 'output')
        self.assertEqual(load_launch['output'], args.output / 'small-sample-02-official' / 'load' / 'output')
        self.assertNotEqual(slice_launch['command'][slice_launch['command'].index('--datadir') + 1],
                            load_launch['command'][load_launch['command'].index('--datadir') + 1])
        self.assertEqual(entry['slice_process']['instructions_retired'], 4_000_000_000)
        self.assertEqual(entry['load_process']['instructions_retired'], 800_000_000)
        self.assertEqual(entry['slice_instructions'], 3_200_000_000)
        self.assertEqual((entry['slice_process']['returncode'], entry['load_process']['returncode']), (0, LOAD_EXIT_STATUS))
        self.assertEqual(entry['load_process']['output_check'], 'passed')
        self.assertEqual(entry['slice_process']['executable'], entry['load_process']['executable'])
        self.assertEqual(entry['slice_process']['fixture'], entry['load_process']['fixture'])
        self.assertEqual(report['runs'], [entry])
        self.assertEqual(entry['status'], 'passed_individual_checks')
        for key in ('size', 'family', 'side', 'iteration', 'warmup', 'directory', 'command', 'cocoa_home', 'HOME_preserved',
                    'whole_process_wall_s', 'returncode', 'reviewed_exception_guards', 'intended_config_check', 'slice_ms',
                    'slice_s', 'task_times', 'plate_times', 'peak_rss_mb', 'gcode_sha256', 'emitted_config_path', 'layers',
                    'moving_extrusion_segments'):
            self.assertIn(key, entry)
        self.assertEqual(entry['slice_ms'], 300)    # sliced_time is still reported, as information
        self.assertIn('START small-sample-02-official', printed)
        done = next(line for line in printed.splitlines() if line.startswith('DONE'))
        self.assertIn('slice_instructions=3200000000', done)
        self.assertRegex(done, r'information only: sliced_time=0\.300000s process_wall=')
    def test_fork_cubic_arm_runs_the_fork_on_the_cubic_fixture(self):
        args, report, slicer = self.run_setup()
        entry, config, printed = self.run_one(args, report, slicer, 'fork_cubic')
        for launch in slicer.launches:
            self.assertEqual(launch['executable'], str(args.fork))
            self.assertEqual(launch['fixture'], self.fixtures['small']['cubic']['path'])
        self.assertEqual(entry['side'], 'fork_cubic')
        self.assertEqual(entry['slice_instructions'], 3_120_000_000)
        self.assertEqual(config[r.PATTERN], 'cubic')
        self.assertEqual(config[r.GAP], '5')   # the fork writes its own default gap although the Cubic input omits it
    def test_fork_cubic_arm_rejects_other_emitted_pattern_or_gap(self):
        for name, key, value in (('pattern', r.PATTERN, 'bridgeawarecubic'), ('gap', r.GAP, '8')):
            args, report, slicer = self.run_setup('fork-cubic-' + name)
            slicer.configs['fork_cubic'][key] = value
            with self.subTest(name), self.assertRaises(r.InvalidBenchmark):
                self.run_one(args, report, slicer, 'fork_cubic')
    def test_emitted_config_check_per_side(self):
        small = self.fixtures['small']
        official = json.loads((DATA_ROOT / 'official-emitted-config.json').read_text())
        fork = json.loads((DATA_ROOT / 'fork-emitted-config.json').read_text())
        fork_cubic = {**fork, r.PATTERN: 'cubic'}
        r.check_emitted_config(official, 'official', small)
        r.check_emitted_config(fork, 'fork', small)
        r.check_emitted_config(fork_cubic, 'fork_cubic', small)   # the Cubic input omits the gap; the fork writes its default, 5
        rejected = {('official', 'the BAC pattern'): {**official, r.PATTERN: 'bridgeawarecubic'},
                    ('official', 'a gap'): {**official, r.GAP: '5'},
                    ('fork', 'the Cubic pattern'): fork_cubic,
                    ('fork', 'another gap'): {**fork, r.GAP: '8'},
                    ('fork_cubic', 'the BAC pattern'): fork,
                    ('fork_cubic', 'another gap'): {**fork_cubic, r.GAP: '8'},
                    ('fork_cubic', 'no gap'): {key: value for key, value in fork_cubic.items() if key != r.GAP},
                    ('fork_cubic', 'another density'): {**fork_cubic, 'sparse_infill_density': '6%'},
                    ('unknown', 'an otherwise acceptable official configuration'): official}
        for (side, name), config in rejected.items():
            with self.subTest(side=side, config=name), self.assertRaises(r.InvalidBenchmark):
                r.check_emitted_config(config, side, small)
    def test_missing_instructions_line_fails_the_run_clearly(self):
        for mode in ('slice', 'load'):
            args, report, slicer = self.run_setup('missing-' + mode)
            slicer.omit_instructions = {mode}
            with self.subTest(mode=mode):
                with self.assertRaises(r.InvalidBenchmark) as caught:
                    self.run_one(args, report, slicer, 'official')
                for phrase in ('instructions retired', '/usr/bin/time -l', 'macOS', 'small-sample-01-official'):
                    self.assertIn(phrase, str(caught.exception))
    def test_load_process_not_ending_like_slice_99_fails_the_run(self):
        for name, change in (('exit status', {'load_status': 0}), ('plate message', {'load_stdout': 'run found error, exit\n'}),
                             ('result', {'load_result': {**LOAD_RESULT, 'return_code': 0}}),
                             ('output', {'load_extra_files': {'plate_1.gcode': '; sliced'}})):
            args, report, slicer = self.run_setup('load-' + name.replace(' ', '-'))
            for attribute, value in change.items():
                setattr(slicer, attribute, value)
            with self.subTest(name), self.assertRaises(r.InvalidBenchmark):
                self.run_one(args, report, slicer, 'official')
    def test_failed_slice_process_fails_the_run(self):
        args, report, slicer = self.run_setup()
        slicer.slice_status = 1
        with self.assertRaisesRegex(r.InvalidBenchmark, 'process exit 1'):
            self.run_one(args, report, slicer, 'official')
    def test_slice_process_not_above_its_load_process_fails_the_run(self):
        args, report, slicer = self.run_setup()
        slicer.totals['official'] = (700_000_000, 800_000_000)
        with self.assertRaises(r.InvalidBenchmark):
            self.run_one(args, report, slicer, 'official')
    def test_timeout_reaps_the_process_group_and_is_reported(self):
        args, report, slicer = self.run_setup()
        class Hung:
            pid = 4242
            calls = 0
            def wait(self, timeout=None):
                Hung.calls += 1
                if Hung.calls == 1:
                    raise r.subprocess.TimeoutExpired('slicer', timeout)
                return -15
        with mock.patch.object(r.subprocess, 'Popen', return_value=Hung()), mock.patch.object(r.os, 'killpg') as killpg, \
                mock.patch.object(r, 'verify_frozen_inputs'), contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(r.InvalidBenchmark, 'exceeded real process timeout'):
                r.run_one(args, report, 'small', 'official', 1, False, self.fixtures)
        killpg.assert_called_once_with(4242, r.signal.SIGTERM)
        self.assertEqual(report['runs'][0]['slice_process']['status'], 'timeout')
        self.assertEqual(report['runs'][0]['status'], 'running')   # main() marks it failed_individual_checks
    def run_main(self, slicer, args, name='main', arguments=()):
        """main() on the small fixture with a fake slicer; returns (exit code, report, printed output)."""
        output = self.path / name
        snapshot = {'head': 'a' * 40, 'snapshot_sha256': 'f' * 64}
        with mock.patch.object(r.subprocess, 'Popen', slicer), mock.patch.object(r, 'source_snapshot', return_value=snapshot), \
                contextlib.redirect_stdout(io.StringIO()) as printed:
            try:
                code = r.main(['--official', str(args.official), '--fork', str(args.fork), '--fixtures', str(FIXTURES),
                               '--output', str(output), '--repo', str(self.path), '--development', '--sizes', 'small',
                               '--intended-exceptions', str(BASELINES / 'intended_exceptions.json'),
                               '--parity-exceptions', str(BASELINES / 'parity_exceptions.json'), *arguments])
            finally:
                report = json.loads((output / 'benchmark.json').read_text())
        return code, report, printed.getvalue()
    def test_run_pairs_slice_and_load_processes_in_alternating_interleaved_order(self):
        args, report, slicer = self.run_setup()
        code, report, printed = self.run_main(slicer, args, arguments=('--samples', '2'))
        self.assertEqual(code, 1)   # development mode can never pass the release gate
        self.assertEqual([(launch['arm'], launch['mode']) for launch in slicer.launches], [
            ('official', 'slice'), ('official', 'load'), ('fork', 'slice'), ('fork', 'load'), ('fork_cubic', 'slice'), ('fork_cubic', 'load'),
            ('fork_cubic', 'slice'), ('fork_cubic', 'load'), ('fork', 'slice'), ('fork', 'load'), ('official', 'slice'), ('official', 'load')])
        self.assertEqual([(run['side'], run['iteration']) for run in report['runs']],
                         [('official', 1), ('fork', 1), ('fork_cubic', 1), ('fork_cubic', 2), ('fork', 2), ('official', 2)])
        for run in report['runs']:
            self.assertEqual(run['slice_instructions'], run['slice_process']['instructions_retired'] - run['load_process']['instructions_retired'])
        self.assertEqual([(pair['iteration'], pair['order'], pair['execution_order']) for pair in report['pairs']],
                         [(1, ['official', 'fork'], ['official', 'fork', 'fork_cubic']), (2, ['fork', 'official'], ['fork_cubic', 'fork', 'official'])])
    def test_development_run_gates_on_slice_instructions_and_reports_build_parity(self):
        args, report, slicer = self.run_setup()
        code, report, printed = self.run_main(slicer, args, arguments=('--samples', '2'))
        self.assertEqual(report['schema_version'], 3)
        self.assertEqual([pair['slice_ratio'] for pair in report['pairs']], [1.5, 1.5])
        self.assertEqual([pair['build_parity_ratio'] for pair in report['pairs']], [0.975, 0.975])
        summary = report['summary']['small']
        self.assertEqual((summary['median_slice_ratio'], summary['worst_observed_pair_ratio'], summary['within_2x']), (1.5, 1.5, True))
        self.assertEqual((summary['median_build_parity_ratio'], summary['worst_build_parity_ratio']), (0.975, 0.975))
        self.assertEqual(summary['median_slice_instructions'], {'official': 3_200_000_000, 'fork': 4_800_000_000, 'fork_cubic': 3_120_000_000})
        self.assertFalse(report['gate_passed'])
        self.assertEqual(report['status'], 'failed_or_diagnostic_only')
        # The printed summary and the report label the metric and call sliced_time and wall times information.
        self.assertIn('instructions retired', report['metric'])
        self.assertIn('information only', report['information_only_metrics']['sliced_time'])
        self.assertIn('information only', report['information_only_metrics']['whole_process_wall_s'])
        self.assertEqual(json.loads(printed[printed.index('{'):])['summary']['small']['median_slice_ratio'], 1.5)
    def test_sliced_time_and_wall_time_do_not_gate(self):
        # A fork ten times slower on the clock with the same instructions passes ...
        args, report, slicer = self.run_setup('slow-clock')
        slicer.sliced_ms['fork'] = 3000
        code, report, printed = self.run_main(slicer, args, 'slow-clock-main')
        summary = report['summary']['small']
        self.assertTrue(summary['within_2x'])
        self.assertEqual(summary['median_slice_ms']['fork'], 3000)
        self.assertEqual(report['pairs'][0]['sliced_time_ratio_information_only'], 10.0)
        # ... and a fork with equal sliced_time but 2.5 times the instructions fails.
        args, report, slicer = self.run_setup('equal-clock')
        slicer.totals['fork'] = (8_800_000_000, 800_000_000)
        slicer.sliced_ms['fork'] = slicer.sliced_ms['official']
        code, report, printed = self.run_main(slicer, args, 'equal-clock-main')
        summary = report['summary']['small']
        self.assertFalse(summary['within_2x'])
        self.assertEqual(summary['median_slice_ratio'], 2.5)
        self.assertEqual(report['pairs'][0]['sliced_time_ratio_information_only'], 1.0)
    def test_official_only_preflight_still_runs_two_processes_per_run(self):
        args, report, slicer = self.run_setup()
        code, report, printed = self.run_main(slicer, args, arguments=('--official-only',))
        self.assertEqual(code, 0)
        self.assertEqual(report['status'], 'official_preflight_passed_diagnostic_only')
        self.assertEqual([(launch['arm'], launch['mode']) for launch in slicer.launches], [('official', 'slice'), ('official', 'load')])
        self.assertEqual(report['pairs'], [])
    def test_main_records_the_missing_instructions_line_in_the_report(self):
        args, report, slicer = self.run_setup()
        slicer.omit_instructions = {'slice'}
        with self.assertRaises(r.InvalidBenchmark):
            self.run_main(slicer, args)
        report = json.loads((self.path / 'main' / 'benchmark.json').read_text())
        self.assertEqual(report['status'], 'invalid_or_incomplete')
        self.assertFalse(report['gate_passed'])
        self.assertIn('instructions retired', report['error'])
        self.assertEqual(report['runs'][-1]['status'], 'failed_individual_checks')

    # ---- The gate: slice instructions, per interleaved pair, median and worst pair ----
    def test_gate_ratio_is_fork_bac_over_official_cubic_slice_instructions_per_pair(self):
        report = self.report(['small'])
        self.set_counts(report, 'small', 2, 'fork', full=4_400_000_000)   # slice 3.6 G over 3.2 G
        self.set_counts(report, 'small', 1, 'official', full=1_000_000_000, warmup=True)   # warm-up pair ratio 24: not in the statistics
        r.summarize(report, 5, 1)
        summary = report['summary']['small']
        self.assertEqual(summary['all_pair_ratios'], [1.5, 3.6e9 / 3.2e9, 1.5, 1.5, 1.5])
        self.assertEqual(summary['median_slice_ratio'], 1.5)
        self.assertEqual(summary['worst_observed_pair_ratio'], 1.5)
        self.assertEqual(summary['ratio_limit'], 2.0)
        self.assertTrue(summary['within_2x'])
        self.assertEqual(summary['all_slice_instructions']['official'], [3_200_000_000] * 5)
        self.assertEqual(summary['all_slice_instructions']['fork'], [4_800_000_000, 3_600_000_000] + [4_800_000_000] * 3)
    def test_gate_median_is_over_pair_ratios_not_a_ratio_of_medians(self):
        report = self.report(['small'])
        for iteration, (official, fork) in enumerate(((100, 300), (200, 200), (300, 330), (400, 400), (500, 500)), 1):
            self.set_counts(report, 'small', iteration, 'official', full=800_000_000 + official * 10_000_000)
            self.set_counts(report, 'small', iteration, 'fork', full=800_000_000 + fork * 10_000_000)
        r.summarize(report, 5, 1)
        summary = report['summary']['small']
        self.assertEqual(summary['all_pair_ratios'], [3.0, 1.0, 1.1, 1.0, 1.0])
        self.assertEqual(summary['median_slice_ratio'], 1.0)   # the median of the pair ratios; the ratio of medians would be 1.1
        self.assertEqual(summary['worst_observed_pair_ratio'], 3.0)
        self.assertFalse(summary['within_2x'])
    def test_gate_limit_is_2_for_median_and_worst_pair_each(self):
        def gate(*fork_slice_g):
            report = self.report(['small'])
            for iteration, value in enumerate(fork_slice_g, 1):
                self.set_counts(report, 'small', iteration, 'fork', full=800_000_000 + round(value * 1e9))
            r.summarize(report, 5, 1)
            return report['summary']['small']
        self.assertTrue(gate(6.4, 6.4, 6.4, 6.4, 6.4)['within_2x'])        # exactly 2.0 is within the limit
        summary = gate(4.8, 4.8, 4.8, 4.8, 6.432)                           # worst pair 2.01, median 1.5
        self.assertFalse(summary['within_2x'])
        self.assertEqual((summary['median_slice_ratio'], summary['worst_observed_pair_ratio']), (1.5, 2.01))
        summary = gate(4.8, 4.8, 6.592, 6.592, 6.592)                       # median 2.06 and worst 2.06
        self.assertFalse(summary['within_2x'])
        self.assertEqual((summary['median_slice_ratio'], summary['worst_observed_pair_ratio']), (2.06, 2.06))
    def test_sliced_time_no_longer_gates_in_summary(self):
        report = self.report(['small'])
        for iteration in range(1, 6):
            self.set_counts(report, 'small', iteration, 'fork', ms=1500)   # fifteen times the official's sliced_time, same instructions
        r.summarize(report, 5, 1)
        summary = report['summary']['small']
        self.assertTrue(summary['within_2x'])
        self.assertEqual(summary['median_slice_ms'], {'official': 100, 'fork': 1500, 'fork_cubic': 105})
        self.assertEqual(report['pairs'][1]['sliced_time_ratio_information_only'], 15.0)
        report = self.report(['small'])
        for iteration in range(1, 6):
            self.set_counts(report, 'small', iteration, 'fork', full=8_800_000_000, ms=100)   # equal sliced_time, instructions 2.5 times
        r.summarize(report, 5, 1)
        self.assertFalse(report['summary']['small']['within_2x'])
        # Neither can the whole-process wall times: any magnitude leaves the verdict alone.
        report = self.report(['small'])
        for run in report['runs']:
            run['whole_process_wall_s'] = 1e9 if run['side'] == 'fork' else 1e-3
        r.summarize(report, 5, 1)
        self.assertTrue(report['summary']['small']['within_2x'])
    def test_build_parity_is_reported_and_never_gates(self):
        report = self.report(['small'])
        r.summarize(report, 5, 1)
        summary = report['summary']['small']
        self.assertEqual(summary['all_build_parity_ratios'], [0.975] * 5)
        self.assertEqual((summary['median_build_parity_ratio'], summary['worst_build_parity_ratio']), (0.975, 0.975))
        report = self.report(['small'])
        for iteration in range(1, 6):
            self.set_counts(report, 'small', iteration, 'fork_cubic', full=16_784_000_000)   # slice 16 G over 3.2 G: five times official Cubic
        r.summarize(report, 5, 1)
        summary = report['summary']['small']
        self.assertEqual((summary['median_build_parity_ratio'], summary['worst_build_parity_ratio']), (5.0, 5.0))
        self.assertTrue(summary['within_2x'])
        # The whole release gate, over all four cases, is unaffected as well.
        report = self.report()
        for run in [run for run in report['runs'] if run['side'] == 'fork_cubic']:
            self.set_counts(report, run['size'], run['iteration'], 'fork_cubic', full=16_784_000_000, warmup=run['warmup'])
        r.summarize(report, 5, 1)
        self.assertTrue(report['gate_passed'])
        self.assertEqual(report['status'], 'release_speed_gate_passed')
    def test_summary_keeps_old_fields_and_adds_instruction_fields(self):
        report = self.report(['small'])
        r.summarize(report, 5, 1)
        summary = report['summary']['small']
        for key in ('family', 'all_slice_ms', 'all_pair_ratios', 'worst_observed_pair_ratio', 'median_slice_ms', 'median_slice_ratio',
                    'ratio_limit', 'within_2x', 'timing_precision', 'official_minimum_slice_ms', 'short_baseline_precision_caution',
                    'whole_process_wall_s', 'all_slice_instructions', 'median_slice_instructions', 'all_build_parity_ratios',
                    'median_build_parity_ratio', 'worst_build_parity_ratio'):
            self.assertIn(key, summary)
        self.assertEqual(summary['all_slice_ms'], {'official': [100] * 5, 'fork': [150] * 5, 'fork_cubic': [105] * 5})
        self.assertEqual(summary['official_minimum_slice_ms'], 100)
        self.assertEqual(report['status'], 'scope_gate_passed_release_coverage_pending')   # only 'small' was measured
        self.assertTrue(report['scope_gate_passed'])
        self.assertFalse(report['gate_passed'])
    def test_summary_rechecks_the_subtraction_from_raw_counts(self):
        def tampered(change):
            report = self.report(['small'])
            change(next(run for run in report['runs'] if run['side'] == 'fork' and run['iteration'] == 3 and not run['warmup']))
            return report
        changes = {'derived figure off by one': lambda run: run.update(slice_instructions=run['slice_instructions'] + 1),
                   'load count changed alone': lambda run: run['load_process'].update(instructions_retired=run['load_process']['instructions_retired'] - 1),
                   'slice count changed alone': lambda run: run['slice_process'].update(instructions_retired=run['slice_process']['instructions_retired'] + 1),
                   'load count above the slice count': lambda run: run.update(slice_instructions=-1, load_process={**run['load_process'], 'instructions_retired': 6_000_000_000}),
                   'count is a float': lambda run: run['slice_process'].update(instructions_retired=5.6e9),
                   'count missing': lambda run: run['load_process'].pop('instructions_retired')}
        for name, change in changes.items():
            report = tampered(change)
            with self.subTest(name), self.assertRaises(r.InvalidBenchmark):
                r.summarize(report, 5, 1)
    def test_summary_rejects_fabricated_ratios(self):
        for key in ('slice_ratio', 'build_parity_ratio'):
            report = self.report(['small'])
            report['pairs'][3][key] = 1.0
            with self.subTest(key=key), self.assertRaises(r.InvalidBenchmark):
                r.summarize(report, 5, 1)
    def test_summary_requires_each_load_process_to_pair_with_its_slice_process(self):
        changes = {'other app': lambda run: run['load_process'].update(executable='/apps/other/BambuStudio'),
                   'other fixture': lambda run: run['load_process'].update(fixture=self.fixtures['small']['bac']['path']),
                   'load process missing': lambda run: run.pop('load_process'),
                   'slice process missing': lambda run: run.pop('slice_process'),
                   'load process is a second full slice': lambda run: run['load_process'].update(plate_arg=1),
                   'slice process is a load': lambda run: run['slice_process'].update(plate_arg=99),
                   'load process exit status 0': lambda run: run['load_process'].update(returncode=0),
                   'load process output not checked': lambda run: run['load_process'].update(output_check='skipped'),
                   'slice process failed': lambda run: run['slice_process'].update(returncode=1)}
        for name, change in changes.items():
            report = self.report(['small'])
            change(next(run for run in report['runs'] if run['side'] == 'official' and run['iteration'] == 2 and not run['warmup']))
            with self.subTest(name), self.assertRaises(r.InvalidBenchmark):
                r.summarize(report, 5, 1)
    def test_summary_requires_all_three_sides_for_every_pair(self):
        report = self.report(['small'])
        report['runs'] = [run for run in report['runs'] if not (run['side'] == 'fork_cubic' and run['iteration'] == 2 and not run['warmup'])]
        with self.assertRaises(r.InvalidBenchmark):
            r.summarize(report, 5, 1)
        report = self.report(['small'])
        report['runs'].append(copy.deepcopy(next(run for run in report['runs'] if run['side'] == 'fork_cubic')))
        with self.assertRaises(r.InvalidBenchmark):
            r.summarize(report, 5, 1)
    def test_summary_rejects_build_parity_configuration_failure_and_wrong_execution_order(self):
        report = self.report(['small'])
        report['pairs'][0]['build_parity_config']['rejected_differences'] = {'layer_height': {}}
        with self.assertRaises(r.InvalidBenchmark):
            r.summarize(report, 5, 1)
        report = self.report(['small'])
        report['pairs'][2]['execution_order'] = ['official', 'fork', 'fork_cubic']   # sample iteration 2 must run in reverse
        with self.assertRaises(r.InvalidBenchmark):
            r.summarize(report, 5, 1)

if __name__ == '__main__':unittest.main(verbosity=2)
