#!/usr/bin/env python3
"""Strict, isolated official Cubic vs feature BAC comparison in instructions retired. Never run implicitly."""
from __future__ import annotations
import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import signal
import statistics
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
import zipfile

HELPER_PATH = Path(__file__).resolve().with_name('family_fixture_audit.py')
HELPER_LOADED_SHA256 = hashlib.sha256(HELPER_PATH.read_bytes()).hexdigest()
_helper_spec = importlib.util.spec_from_file_location('bac_family_fixture_audit_v2', HELPER_PATH)
fixture_audit = importlib.util.module_from_spec(_helper_spec)
_helper_spec.loader.exec_module(fixture_audit)
if hashlib.sha256(HELPER_PATH.read_bytes()).hexdigest() != HELPER_LOADED_SHA256:
    raise RuntimeError('Fixture audit helper changed during import')
GUARDS_PATH = Path(__file__).resolve().with_name('reviewed_exception_guards.py')
GUARDS_LOADED_SHA256 = hashlib.sha256(GUARDS_PATH.read_bytes()).hexdigest()
_guards_spec = importlib.util.spec_from_file_location('bac_reviewed_exception_guards_v2', GUARDS_PATH)
reviewed_guards = importlib.util.module_from_spec(_guards_spec)
_guards_spec.loader.exec_module(reviewed_guards)
if hashlib.sha256(GUARDS_PATH.read_bytes()).hexdigest() != GUARDS_LOADED_SHA256:
    raise RuntimeError('Reviewed exception guards changed during import')
PATTERN, GAP = fixture_audit.PATTERN, fixture_audit.GAP
BANNED_CONFIG_KEYS = fixture_audit.BANNED_CONFIG_KEYS
SIZES = {name: tuple(info['dimensions_mm']) for name, info in fixture_audit.FAMILIES.items()}
REQUIRED_CASES = ('small', 'large', 'rib', 'modifier')
REQUIRED_FAMILIES = ('cube', 'rib', 'modifier')
SCHEMA_VERSION = 3
GATE_LIMIT = 2.0

# The metric is instructions retired by a process, all threads, as macOS `/usr/bin/time -l` reports them. Wall and CPU time
# depend on the machine's power state and load; they are reported as information and nothing gates on them. A measured run is
# two processes of one app on one fixture: a full slice (--slice 1) and a startup-and-load process (--slice 99). The latter
# loads the project, rejects the missing plate, writes result.json and exits without slicing or opening a window. The run's
# slice instructions are the first process's instructions retired minus the second's.
TIME_BINARY = '/usr/bin/time'
SLICE_PLATE, LOAD_PLATE = 1, 99
LOAD_EXIT_STATUS = 254  # CLI_INVALID_PARAMS (-2, src/libslic3r/Utils.hpp) as a process exit status
LOAD_RESULT_RETURN_CODE = -2
LOAD_RESULT_ERROR = 'Invalid parameters to the slicer.'
INSTRUCTIONS_RETIRED = re.compile(r'^\s*(\d+)\s+instructions retired\s*$', re.MULTILINE)
# The sides measured in each interleaved iteration. 'official' and 'fork' are the gate's pair; 'fork_cubic' is the fork on the
# same Cubic fixture as 'official' and gives the build parity, as information.
ARMS = {'official': {'executable': 'official', 'input': 'cubic', 'config_side': 'official'},
        'fork': {'executable': 'fork', 'input': 'bac', 'config_side': 'fork'},
        'fork_cubic': {'executable': 'fork', 'input': 'cubic', 'config_side': 'fork'}}
SIDES = tuple(ARMS)
# The fork serializes its own default for the BAC gap (PrintConfig.cpp) into the G-code configuration of a Cubic input too,
# although the Cubic fixtures omit the key.
FORK_DEFAULT_GAP_MM = 5.

# These setting values must never be waived in a parity exception file.
PROTECTED = {PATTERN, GAP, 'nozzle_diameter', 'filament_diameter', 'filament_type',
    'layer_height', 'initial_layer_print_height', 'wall_loops', 'wall_generator',
    'sparse_infill_density', 'top_shell_layers', 'bottom_shell_layers',
    'top_shell_thickness', 'bottom_shell_thickness', 'infill_combination',
    'enable_support', 'use_relative_e_distances', 'infill_direction', 'fill_multiline'}

InvalidBenchmark = fixture_audit.InvalidFixture

def require(condition, message):
    if not condition:
        raise InvalidBenchmark(message)

def sha256(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()

def write_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + '\n')
    temporary.replace(path)

def no_duplicates(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, f'Duplicate JSON key: {key}')
        result[key] = value
    return result

def read_json(path):
    return json.loads(Path(path).read_text(), object_pairs_hook=no_duplicates)

def source_snapshot(repo):
    def git(*args):
        return subprocess.check_output(['git', '-C', str(repo), *args])
    head = git('rev-parse', 'HEAD').decode().strip()
    status = git('status', '--porcelain=v1', '-z', '--untracked-files=all')
    diff = git('diff', '--binary', 'HEAD')
    untracked = git('ls-files', '--others', '--exclude-standard', '-z').split(b'\0')
    files = []
    for encoded in untracked:
        if encoded:
            name = os.fsdecode(encoded)
            path = repo / name
            digest = hashlib.sha256(os.readlink(path).encode()).hexdigest() if path.is_symlink() else sha256(path)
            files.append({'path': name, 'sha256': digest})
    return {'repo': str(repo), 'head': head, 'dirty': bool(status),
        'status_porcelain_sha256': hashlib.sha256(status).hexdigest(),
        'tracked_diff_binary_sha256': hashlib.sha256(diff).hexdigest(),
        'untracked_files': files,
        'snapshot_sha256': hashlib.sha256(head.encode() + b'\0' + status + diff + json.dumps(files, sort_keys=True).encode()).hexdigest(),
        'meaning': 'Source working tree at benchmark invocation; build-provenance must tie it to the executable separately.'}

read_expected = fixture_audit.load_expected
def validate_fixture_pair(cubic, bac, fixtures_dir=None):
    pair = fixture_audit.validate_fixture_pair(cubic, bac, fixtures_dir=fixtures_dir)
    # Legacy mode is per manifest record; a mixed-schema pair must not hide
    # a legacy BAC input behind the native Cubic record's classification.
    for side, path in (('cubic', cubic), ('bac', bac)):
        directory = Path(fixtures_dir) if fixtures_dir is not None else Path(path).parent
        pair[side]['legacy_cube_manifest'] = fixture_audit.fixture_record(directory, path)['legacy_cube_manifest']
    pair['legacy_cube_manifest'] = any(pair[side]['legacy_cube_manifest'] for side in ('cubic', 'bac'))
    return pair

canonical_xml = fixture_audit.canonical_xml

def validate_reviewed_guards(expected, actual=None, local_configs=None):
    try:
        reviewed_guards.validate_reviewed_guards(expected, actual, local_configs)
    except ValueError as error:
        raise InvalidBenchmark(f'Reviewed exception guard failed: {error}') from error

def compare_intended(expected, actual, exceptions):
    differences = {}
    rejected = {}
    for key in sorted(expected.keys() | actual.keys()):
        if key in expected and key in actual and expected[key] == actual[key]:
            continue
        record = {'expected': expected.get(key), 'actual': actual.get(key)}
        differences[key] = record
        approved = exceptions.get(key)
        if not (approved and record == {name: approved[name] for name in ('expected', 'actual')}):
            rejected[key] = record
    return {'all_differences': differences, 'rejected_differences': rejected}

def load_intended_exceptions(path):
    result = {'official': {}, 'fork': {}} if path is None else read_json(path)
    require(set(result) == {'official', 'fork'}, 'Intended exceptions must have official and fork maps')
    for side, records in result.items():
        require(isinstance(records, dict), f'{side} exceptions must be an object')
        for key, record in records.items():
            require(key not in PROTECTED, f'Protected intended setting cannot be excepted: {key}')
            require(isinstance(record, dict) and set(record) == {'expected', 'actual', 'reason'},
                    f'Intended exception {key} needs exact expected, actual, reason')
            require(isinstance(record['reason'], str) and record['reason'].strip(), f'Exception {key} lacks reason')
            for field in ('expected', 'actual'):
                require(record[field] is None or isinstance(record[field], str), f'Exception {key}.{field} must be string/null')
    return result

def finite_number(value, name, positive=False):
    require(isinstance(value, (int, float)) and not isinstance(value, bool), f'{name} is not numeric')
    require(math.isfinite(value), f'{name} is not finite')
    require(value > 0 if positive else value >= 0, f'{name} is outside its accepted range')
    return float(value)

def parse_gcode(path, expected_layer_count, layer_height=.2, initial_layer_height=.2):
    require(type(expected_layer_count) is int and expected_layer_count > 0, 'Invalid expected layer count')
    finite_number(layer_height, 'expected layer_height', positive=True)
    finite_number(initial_layer_height, 'expected initial_layer_height', positive=True)
    config = {}
    inside = False
    blocks = ends = 0
    layers = []
    layer_z = []
    layer_changes = 0
    extruded_layers = set()
    relative_e = None
    exact_e = 0.0
    moving_extrusion_segments = 0
    xy = [None, None]
    exact_z = None
    absolute_xyz = False
    millimeters = False
    for line_number, line in enumerate(Path(path).read_text().splitlines(), 1):
        stripped = line.strip()
        if stripped == '; CONFIG_BLOCK_START':
            require(not inside and blocks == 0, 'Duplicate or nested G-code configuration block')
            inside = True
            blocks += 1
            continue
        if stripped == '; CONFIG_BLOCK_END':
            require(inside, 'Unmatched G-code configuration block end')
            inside = False
            ends += 1
            continue
        if inside:
            match = re.fullmatch(r'; ([A-Za-z0-9_]+) = (.*)', line)
            require(match is not None, f'Malformed config line {line_number}')
            key, value = match.groups()
            require(key not in config, f'Duplicate emitted config key {key}')
            config[key] = value
            continue
        match = re.fullmatch(r';\s*total layer number:\s*(\d+)', stripped)
        if match:
            layers.append(int(match.group(1)))
        if stripped == '; CHANGE_LAYER':
            layer_changes += 1
        z_match = re.fullmatch(r'; Z_HEIGHT:\s*([0-9.eE+-]+)', stripped)
        if z_match:
            layer_z.append(float(z_match.group(1)))
        command = stripped.split(';', 1)[0].strip()
        if not command:
            continue
        words = command.split()
        if words[0] == 'G21':
            millimeters = True
        elif words[0] == 'G20':
            raise InvalidBenchmark('Unexpected inch units in benchmark G-code')
        elif words[0] == 'G90':
            absolute_xyz = True
        elif words[0] == 'G91':
            raise InvalidBenchmark('Unexpected relative XYZ mode in benchmark G-code')
        elif words[0] == 'M83':
            relative_e = True
        elif words[0] == 'M82':
            relative_e = False
        elif words[0] in ('G0', 'G1', 'G2', 'G3', 'G92'):
            axes = {}
            for word in words[1:]:
                if word[:1] in ('X', 'Y', 'Z', 'E'):
                    try:
                        require(word[0] not in axes, f'Duplicate G-code axis on line {line_number}')
                        axes[word[0]] = float(word[1:])
                        require(math.isfinite(axes[word[0]]), f'Nonfinite G-code axis on line {line_number}')
                    except ValueError:
                        raise InvalidBenchmark(f'Unparseable G-code axis on line {line_number}')
            if words[0] == 'G92':
                require(not (set(axes) & {'X', 'Y', 'Z'}), 'Unexpected XYZ coordinate reset in benchmark G-code')
                exact_e = axes.get('E', exact_e)
                exact_z = axes.get('Z', exact_z)
                for index, key in enumerate(('X', 'Y')):
                    xy[index] = axes.get(key, xy[index])
                continue
            require(absolute_xyz and millimeters, 'Motion precedes explicit absolute XYZ/millimeter mode')
            new_xy = [axes.get('X', xy[0]), axes.get('Y', xy[1])]
            exact_z = axes.get('Z', exact_z)
            if 'E' in axes:
                require(relative_e is not None, 'E motion precedes explicit extrusion distance mode')
                delta = axes['E'] if relative_e else axes['E'] - exact_e
                if delta > 0 and None not in xy and new_xy != xy:
                    require(layer_changes > 0 and len(layer_z) == layer_changes, 'Extrusion lacks current declared layer')
                    require(exact_z is not None and math.isclose(exact_z, layer_z[-1], abs_tol=1e-5, rel_tol=0),
                            f'Positive extrusion at wrong actual Z on line {line_number}: {exact_z}, layer {layer_z[-1]}')
                    moving_extrusion_segments += 1
                    extruded_layers.add(layer_changes)
                exact_e = exact_e + axes['E'] if relative_e else axes['E']
            xy = new_xy
    require(blocks == 1 and ends == 1 and not inside and len(config) > 50,
            'Missing or incomplete emitted configuration block')
    require(layers == [expected_layer_count], f'Expected one {expected_layer_count}-layer header, got {layers}')
    require(moving_extrusion_segments > 0, 'No positive moving extrusion in G-code')
    require(layer_changes == expected_layer_count and len(layer_z) == expected_layer_count,
            f'Actual layer-change/Z record count is not {expected_layer_count}')
    require(all(math.isclose(z, initial_layer_height + i * layer_height, abs_tol=1e-5, rel_tol=0) for i, z in enumerate(layer_z)),
            f'Actual layer Z sequence is incorrect: {layer_z}')
    require(extruded_layers == set(range(1, expected_layer_count + 1)),
            f'Actual moving extrusion does not populate all {expected_layer_count} layers')
    return config, {'layers': expected_layer_count, 'moving_extrusion_segments': moving_extrusion_segments}

def check_emitted_config(config, side, fixture):
    require(side in ARMS, f'Unknown benchmark side: {side}')
    expected = {PATTERN: 'bridgeawarecubic' if side == 'fork' else 'cubic',
        'sparse_infill_density': f'{fixture["density_percent"]:g}%', 'layer_height': '0.2',
        'infill_direction': fixture_audit.FAMILIES[fixture['size']]['angle'],
        'initial_layer_print_height': '0.2', 'wall_loops': '2', 'wall_generator': 'classic',
        'top_shell_layers': '3', 'bottom_shell_layers': '3',
        'top_shell_thickness': '0', 'bottom_shell_thickness': '0',
        'use_relative_e_distances': '1'}
    for key, value in expected.items():
        require(config.get(key) == value, f'{side} actual {key}: {config.get(key)!r}; expected {value!r}')
    if side == 'fork':
        require(config.get(GAP) == f'{fixture["bac_gap_mm"]:g}', f'Actual BAC gap is {config.get(GAP)!r}')
    elif side == 'fork_cubic':
        require(config.get(GAP) == f'{FORK_DEFAULT_GAP_MM:g}',
                f'Fork Cubic output must carry the fork default BAC gap {FORK_DEFAULT_GAP_MM:g}, got {config.get(GAP)!r}')
    else:
        require(GAP not in config, 'Official Cubic output unexpectedly contains the BAC gap')
    for key in ('nozzle_diameter', 'filament_diameter', 'filament_type'):
        require(bool(config.get(key)), f'Missing actual {key}')

def validate_result(path, fixture):
    result = read_json(path)
    require(type(result.get('return_code')) is int and result['return_code'] == 0,
            f'CLI result failure: {result.get("return_code")}; {result.get("error_string")}')
    require(type(result.get('plate_index')) is int and result['plate_index'] == 1, 'Unexpected requested plate index')
    require(isinstance(result.get('error_string'), str), 'Missing CLI result status text')
    for key, expected in {'layer_height': .2, 'sparse_infill_density': fixture['density_percent']}.items():
        require(math.isclose(finite_number(result.get(key), key), expected, abs_tol=1e-8, rel_tol=0),
                f'Unexpected result global {key}')
    require(type(result.get('wall_loops')) is int and result['wall_loops'] == 2, 'Unexpected result wall loops')
    plates = result.get('sliced_plates')
    require(isinstance(plates, list) and len(plates) == fixture['plate_count'] == 1 and isinstance(plates[0], dict)
            and type(plates[0].get('id')) is int and plates[0]['id'] == 1,
            'Expected exactly one successful plate with integer id 1')
    plate = plates[0]
    # Both CLIs count is_model_part() facets here. Parameter-modifier triangles
    # remain audited in the archive total but are not included in result counts.
    triangles = fixture['model_triangle_count']
    require(type(plate.get('triangle_count')) is int and plate['triangle_count'] == triangles,
            f'Expected {triangles} model triangles, got {plate.get("triangle_count")}')
    require(type(plate.get('obj_cached_cnt')) is int and plate['obj_cached_cnt'] == 0, 'Unexpected cached object slicing')
    objects = plate.get('objects')
    require(isinstance(objects, list) and len(objects) == fixture['object_count'] == 1 and isinstance(objects[0], dict),
            'Expected exactly one object')
    obj = objects[0]
    require(type(obj.get('id')) is int and obj['id'] > 0, 'Invalid runtime object id')
    require(obj.get('name') == 'BAC benchmark ' + fixture['size'], 'Unexpected runtime object identity')
    require(type(obj.get('triangle_count')) is int and obj['triangle_count'] == triangles, 'Unexpected object model mesh size')
    bbox = obj.get('bbox')
    require(isinstance(bbox, dict), 'Missing runtime object bounds')
    for key, expected in zip(('x', 'y', 'z', 'width', 'depth', 'height'), fixture['bounds_min_mm'] + fixture['dimensions_mm']):
        actual = finite_number(bbox.get(key), f'bbox.{key}')
        require(math.isclose(actual, expected, abs_tol=1e-5, rel_tol=0),
                f'Unexpected bbox.{key}: {actual}, expected {expected}')
    filaments = plate.get('filaments')
    require(isinstance(filaments, list) and len(filaments) == fixture['filament_count'] == 1
            and isinstance(filaments[0], dict), 'Unexpected runtime filament count')
    filament = filaments[0]
    require(type(filament.get('id')) is int and filament['id'] == 1, 'Unexpected runtime filament id')
    # These default fixtures have zero filament_density, so mass may be zero;
    # positive deposited motion is checked independently in the G-code.
    total = finite_number(filament.get('total_used_g'), 'filament.total_used_g')
    main = finite_number(filament.get('main_used_g'), 'filament.main_used_g')
    require(total + 1e-8 >= main, 'Filament total usage is smaller than model usage')
    times = plate.get('times')
    require(isinstance(times, dict), 'Missing per-plate timing')
    sliced_ms = times.get('sliced_time')
    require(type(sliced_ms) is int and sliced_ms > 0, 'sliced_time must be positive integer milliseconds')
    return result, sliced_ms

def load_exceptions(path):
    if path is None:
        return {}
    exceptions = read_json(path)
    require(isinstance(exceptions, dict), 'Exceptions must be a key -> exact values/reason object')
    for key, record in exceptions.items():
        require(key not in PROTECTED, f'Protected setting cannot be excepted: {key}')
        require(isinstance(record, dict) and set(record) == {'official', 'fork', 'reason'},
                f'Exception {key} must have official, fork, reason')
        require(isinstance(record['reason'], str) and record['reason'].strip(), f'Exception {key} lacks reason')
        for side in ('official', 'fork'):
            require(record[side] is None or isinstance(record[side], str), f'Exception {key}.{side} is not a string/null')
    return exceptions

def compare_emitted(official, fork, exceptions, gap_mm):
    all_differences = {}
    rejected = {}
    used = {}
    for key in sorted(official.keys() | fork.keys()):
        left, right = official.get(key), fork.get(key)
        if key in official and key in fork and left == right:
            continue
        record = {'official': left, 'fork': right}
        all_differences[key] = record
        if key == PATTERN and record == {'official': 'cubic', 'fork': 'bridgeawarecubic'}:
            continue
        if key == GAP and key not in official and right == f'{gap_mm:g}':
            continue
        approved = exceptions.get(key)
        if approved and record == {side: approved[side] for side in ('official', 'fork')}:
            used[key] = approved
        else:
            rejected[key] = record
    return {'all_differences': all_differences, 'approved_differences': used, 'rejected_differences': rejected}

def sandbox_profile(path, live_home):
    # Process-local denies preserve the user's installed app and Cocoa state.
    def quoted(value):
        return json.dumps(str(value))
    lines = ['(version 1)', '(allow default)', '(deny network*)']
    for name in ('BambuStudio', 'BambuStudioBeta'):
        profile = live_home / 'Library' / 'Application Support' / name
        lines.append(f'(deny file-read* file-write* (subpath {quoted(profile)}))')
    for bundle_id in ('com.bambulab.bambu-studio', 'com.bambulab.bambu-studio-beta'):
        for relative in (f'Library/Saved Application State/{bundle_id}.savedState',
                         f'Library/Preferences/{bundle_id}.plist',
                         f'Library/Caches/{bundle_id}',
                         f'Library/Containers/{bundle_id}'):
            lines.append(f'(deny file-read* file-write* (subpath {quoted(live_home / relative)}))')
        lines.append(f'(deny user-preference-read user-preference-write (preference-domain {quoted(bundle_id)}))')
    path.write_text('\n'.join(lines) + '\n')

def parse_time_output(text):
    """Instructions retired, as an integer, from the report of `/usr/bin/time -l`."""
    figures = INSTRUCTIONS_RETIRED.findall(text)
    require(figures, f'{TIME_BINARY} -l printed no "instructions retired" line; this benchmark measures instructions retired '
                     'and needs the macOS /usr/bin/time (Apple silicon), whose -l report includes that line')
    require(len(figures) == 1, f'{TIME_BINARY} -l printed more than one "instructions retired" line')
    count = int(figures[0])
    require(count > 0, f'{TIME_BINARY} -l reported 0 instructions retired; the counter is unavailable (virtual machine?)')
    return count

def slice_instructions(full_slice, load):
    """A run's slice instructions: its full slice's instructions retired minus its startup-and-load process's."""
    for name, value in (('Full-slice', full_slice), ('Load', load)):
        require(type(value) is int and value > 0, f'{name} instructions retired must be a positive integer, got {value!r}')
    require(full_slice > load, f'The full slice retired {full_slice} instructions, not more than its startup-and-load process ({load})')
    return full_slice - load

def check_load_process(returncode, directory, plate_count):
    """Accept a startup-and-load process only if it ended exactly as `--slice 99` ends on a project with `plate_count` plates.

    The app loads the project, finds no plate 99 and logs 'invalid plate id 99, total N'. It writes a result.json that reports
    Invalid parameters (return code -2, no plate, no sliced plates), prints 'run found error, exit' and exits with status 254,
    having sliced and exported nothing. Any other end means the process did not do the startup-and-load work that the
    subtraction removes from the full slice."""
    require(returncode == LOAD_EXIT_STATUS, f'exit status {returncode}, expected {LOAD_EXIT_STATUS}')
    stdout = (directory / 'stdout.log').read_text(errors='replace')
    require(re.search(rf'invalid plate id {LOAD_PLATE}, total {plate_count}\s*$', stdout, re.MULTILINE),
            f'stdout lacks "invalid plate id {LOAD_PLATE}, total {plate_count}"')
    require(re.search(r'^run found error, exit\s*$', stdout, re.MULTILINE), 'stdout lacks "run found error, exit"')
    output = directory / 'output'
    left = sorted(path.name for path in output.iterdir()) if output.is_dir() else None
    require(left == ['result.json'], f'output directory holds {left}, expected only result.json')
    try:
        result = read_json(output / 'result.json')
    except ValueError as error:
        raise InvalidBenchmark(f'result.json is not valid JSON: {error}') from error
    require(isinstance(result, dict), 'result.json is not an object')
    require(type(result.get('return_code')) is int and result['return_code'] == LOAD_RESULT_RETURN_CODE and
            result.get('error_string') == LOAD_RESULT_ERROR,
            f'result.json reports {result.get("return_code")!r} {result.get("error_string")!r}, '
            f'expected {LOAD_RESULT_RETURN_CODE} {LOAD_RESULT_ERROR!r}')
    require(type(result.get('plate_index')) is int and result['plate_index'] == 0 and 'sliced_plates' not in result,
            'result.json names a plate or lists sliced plates')

def process_environment(directory):
    # Redirect Cocoa filesystem state separately; preserve HOME and deny live app preferences.
    environment = os.environ.copy()
    environment['TMPDIR'] = str(directory / 'tmp') + os.sep
    environment['LC_ALL'] = 'C'
    environment['CFFIXED_USER_HOME'] = str(directory / 'cocoa-home')
    return environment

def process_command(args, executable, profile, directory, fixture, plate):
    """One measured process: the app under the sandbox and /usr/bin/time -l, which reports instructions retired in time.txt."""
    return [TIME_BINARY, '-l', '-o', str(directory / 'time.txt'),
        '/usr/bin/sandbox-exec', '-f', str(profile), '/usr/bin/arch', '-arm64', str(executable),
        '--datadir', str(directory / 'data'), '--outputdir', str(directory / 'output'),
        '--arrange', '0', '--orient', '0', '--mstpp', str(args.timeout), '--debug', '1',
        '--slice', str(plate), fixture]

def launch_process(args, report, record, directory, label):
    """Run record['command'] in its own session, logging stdout and stderr in `directory`; returns its exit status.

    The whole child group is reaped on timeout or interruption."""
    environment = process_environment(directory)
    record.update(status='running', cocoa_home=environment['CFFIXED_USER_HOME'],
                  HOME_preserved=environment.get('HOME') == os.environ.get('HOME'))
    started = time.perf_counter()
    with (directory / 'stdout.log').open('wb') as stdout, (directory / 'stderr.log').open('wb') as stderr:
        process = subprocess.Popen(record['command'], cwd=directory, env=environment, stdout=stdout,
                                   stderr=stderr, start_new_session=True)
        try:
            returncode = process.wait(timeout=args.timeout + 120)
        except BaseException as error:
            # Reap the entire isolated child group on timeout or interruption.
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait()
            record.update(status='timeout' if isinstance(error, subprocess.TimeoutExpired) else 'interrupted',
                          whole_process_wall_s=time.perf_counter() - started)
            write_json(args.output / 'benchmark.json', report)
            if isinstance(error, subprocess.TimeoutExpired):
                raise InvalidBenchmark(f'{label} exceeded real process timeout') from error
            raise
    record.update(status='finished', whole_process_wall_s=time.perf_counter() - started, returncode=returncode)
    return returncode

def measure_process(args, report, record, directory, label):
    """Run one measured process, record its instructions retired and return its exit status."""
    returncode = launch_process(args, report, record, directory, label)
    time_file = directory / 'time.txt'
    text = time_file.read_text(errors='replace') if time_file.is_file() else ''
    try:
        record['instructions_retired'] = parse_time_output(text)
    except InvalidBenchmark as error:
        raise InvalidBenchmark(f'{label}: {error} (exit status {returncode}; see {time_file} and {directory / "stderr.log"})') from error
    record['time_output_path'] = str(time_file)
    return returncode

def run_one(args, report, size, side, iteration, warmup, fixtures):
    """One measured run: a full slice and a startup-and-load process of the same app on the same fixture.

    Both run under /usr/bin/time -l; the slice's output is validated as before."""
    arm = ARMS[side]
    token = f'{size}-{"warmup" if warmup else "sample"}-{iteration:02d}-{side}'
    directory = args.output / token
    load_directory = directory / 'load'
    directory.mkdir()
    load_directory.mkdir()
    for process_directory in (directory, load_directory):
        for name in ('data', 'output', 'tmp', 'cocoa-home'):
            (process_directory / name).mkdir()
    profile = directory / 'sandbox.sb'
    sandbox_profile(profile, Path.home())
    executable = getattr(args, arm['executable'])
    input_record = fixtures[size][arm['input']]
    fixture = input_record['path']
    verify_frozen_inputs(args, report, fixtures)
    command = process_command(args, executable, profile, directory, fixture, SLICE_PLATE)
    environment = process_environment(directory)
    entry = {'size': size, 'family': fixtures[size]['family'], 'side': side, 'iteration': iteration, 'warmup': warmup,
             'directory': str(directory), 'command': command, 'status': 'running',
             'cocoa_home': environment['CFFIXED_USER_HOME'], 'HOME_preserved': environment.get('HOME') == os.environ.get('HOME'),
             'slice_process': {'plate_arg': SLICE_PLATE, 'executable': str(executable), 'fixture': fixture,
                               'directory': str(directory), 'command': command},
             'load_process': {'plate_arg': LOAD_PLATE, 'executable': str(executable), 'fixture': fixture,
                              'directory': str(load_directory),
                              'command': process_command(args, executable, profile, load_directory, fixture, LOAD_PLATE)}}
    report['runs'].append(entry)
    write_json(args.output / 'benchmark.json', report)
    print(f'START {token}', flush=True)
    returncode = measure_process(args, report, entry['slice_process'], directory, token)
    entry.update(whole_process_wall_s=entry['slice_process']['whole_process_wall_s'], returncode=returncode)
    require(returncode == 0, f'{token}: process exit {returncode}; inspect {directory}/stderr.log')
    load_label = f'{token} (load process)'
    load_returncode = measure_process(args, report, entry['load_process'], load_directory, load_label)
    try:
        check_load_process(load_returncode, load_directory, fixtures[size]['plate_count'])
        entry['slice_instructions'] = slice_instructions(entry['slice_process']['instructions_retired'],
                                                         entry['load_process']['instructions_retired'])
    except InvalidBenchmark as error:
        raise InvalidBenchmark(f'{load_label}: {error}; inspect {load_directory}') from error
    entry['load_process']['output_check'] = 'passed'
    result, sliced_ms = validate_result(directory / 'output' / 'result.json', fixtures[size])
    gcode = directory / 'output' / 'plate_1.gcode'
    require(gcode.is_file(), f'Expected emitted G-code at {gcode}')
    config, geometric_evidence = parse_gcode(gcode, fixtures[size]['expected_layer_count'])
    check_emitted_config(config, side, fixtures[size])
    expected = read_expected(args.fixtures, fixture)
    if side == 'fork_cubic':
        # The Cubic input omits the BAC gap, but the fork serializes its own default for it.
        expected = {**expected, GAP: f'{FORK_DEFAULT_GAP_MM:g}'}
    validate_reviewed_guards(expected, config, input_record['local_configs'])
    entry['reviewed_exception_guards'] = 'passed'
    intended = compare_intended(expected, config, args.intended_exceptions[arm['config_side']])
    write_json(directory / 'intended-config-check.json', intended)
    write_json(directory / 'emitted-config.json', config)
    entry.update(status='passed_individual_checks' if not intended['rejected_differences'] else 'failed_intended_config',
                 intended_config_check=intended, slice_ms=sliced_ms,
                 slice_s=sliced_ms / 1000., task_times=result.get('times'),
                 plate_times=result['sliced_plates'][0]['times'], peak_rss_mb=result.get('peak_rss_mb'),
                 gcode_sha256=sha256(gcode), emitted_config_path=str(directory / 'emitted-config.json'),
                 **geometric_evidence)
    write_json(args.output / 'benchmark.json', report)
    require(not intended['rejected_differences'], f'{token}: emitted settings differ from intended; inspect {directory}/intended-config-check.json')
    print(f'DONE {token} slice_instructions={entry["slice_instructions"]} '
          f'(full slice {entry["slice_process"]["instructions_retired"]}, load {entry["load_process"]["instructions_retired"]}); '
          f'information only: sliced_time={sliced_ms / 1000.:.6f}s process_wall={entry["whole_process_wall_s"]:.6f}s', flush=True)
    return entry, config

def verify_frozen_inputs(args, report, fixtures):
    require(sha256(__file__) == report['harness']['sha256'], 'Benchmark harness changed since preflight')
    require(sha256(HELPER_PATH) == report['fixture_audit_helper']['sha256'] == HELPER_LOADED_SHA256,
            'Fixture audit helper changed since import/preflight')
    require(sha256(GUARDS_PATH) == report['reviewed_exception_guards']['sha256'] == GUARDS_LOADED_SHA256,
            'Reviewed exception guards changed since import/preflight')
    require(sha256(args.fixtures / 'manifest.json') == report['fixture_manifest_sha256'], 'Fixture manifest changed since preflight')
    for fixture in fixtures.values():
        for side in ('cubic', 'bac'):
            record = fixture[side]
            require(sha256(record['path']) == record['sha256'], 'Input archive changed since preflight')
            for sidecar in record['sidecars'].values():
                require(sha256(sidecar['path']) == sidecar['sha256'], 'Input configuration snapshot changed since preflight')
    for side in ('official', 'fork'):
        require(sha256(getattr(args, side)) == report['executables'][side]['sha256'], 'Executable changed since preflight')


def exact_int(value, expected):
    return type(value) is int and value == expected

def verify_run_measurement(run):
    """Re-derive a run's slice instructions from the raw counts of its two processes; return them.

    A run is a slice process (--slice 1) and a load process (--slice 99) of one app on one fixture, which ended as required."""
    label = f'{run["size"]}/{run["side"]}/{"warmup" if run["warmup"] else "sample"} {run["iteration"]}'
    slice_process, load_process = run.get('slice_process'), run.get('load_process')
    require(isinstance(slice_process, dict) and isinstance(load_process, dict), f'{label}: run lacks its slice or load process')
    require(exact_int(slice_process.get('plate_arg'), SLICE_PLATE) and exact_int(load_process.get('plate_arg'), LOAD_PLATE),
            f'{label}: the processes are not a --slice {SLICE_PLATE} and a --slice {LOAD_PLATE} process')
    require(slice_process.get('executable') == load_process.get('executable') and
            slice_process.get('fixture') == load_process.get('fixture'),
            f'{label}: the load process is not the same app on the same fixture as the slice process')
    require(exact_int(slice_process.get('returncode'), 0) and exact_int(load_process.get('returncode'), LOAD_EXIT_STATUS) and
            load_process.get('output_check') == 'passed', f'{label}: the slice or load process did not end as required')
    try:
        derived = slice_instructions(slice_process.get('instructions_retired'), load_process.get('instructions_retired'))
    except InvalidBenchmark as error:
        raise InvalidBenchmark(f'{label}: {error}') from error
    require(run.get('slice_instructions') == derived, f'{label}: reported slice instructions differ from the full slice minus the load process')
    return derived

def summarize(report, samples, warmups):
    result = {}
    require(set(report['sizes']) <= set(REQUIRED_CASES) and len(report['sizes']) == len(set(report['sizes'])),
            'Unknown or duplicate benchmark case')
    for size in report['sizes']:
        fixture = report['fixtures'][size]
        require(fixture['family'] == fixture_audit.FAMILIES[size]['family'], 'Wrong fixture family in report')
        case_runs = [run for run in report['runs'] if run['size'] == size]
        case_pairs = [pair for pair in report['pairs'] if pair['size'] == size]
        for warmup, count in ((True, warmups), (False, samples)):
            expected_run_ids = {(side, i) for side in SIDES for i in range(1, count + 1)}
            selected_runs = [run for run in case_runs if run['warmup'] is warmup]
            require(len(selected_runs) == len(expected_run_ids) and
                    {(run['side'], run['iteration']) for run in selected_runs} == expected_run_ids,
                    f'Incomplete or duplicate {"warmup" if warmup else "sample"} runs for {size}')
            require(all(run['status'] == 'passed_individual_checks' and
                        not run['intended_config_check']['rejected_differences'] for run in selected_runs),
                    f'Unvalidated individual run for {size}')
            selected_pairs = [pair for pair in case_pairs if pair['warmup'] is warmup]
            require(len(selected_pairs) == count and {pair['iteration'] for pair in selected_pairs} == set(range(1, count + 1)),
                    f'Incomplete or duplicate paired samples for {size}')
            for pair in selected_pairs:
                require(not pair['parity']['rejected_differences'], f'Failed configuration parity for {size}')
                require(not pair['build_parity_config']['rejected_differences'], f'Failed build-parity configuration for {size}')
                odd = pair['iteration'] % 2 == 1
                require(pair['order'] == (['official', 'fork'] if odd else ['fork', 'official']),
                        'Paired measurements did not alternate execution order')
                require(pair['execution_order'] == (['official', 'fork', 'fork_cubic'] if odd else ['fork_cubic', 'fork', 'official']),
                        'Measurements did not alternate execution order')
                runs = {run['side']: run for run in selected_runs if run['iteration'] == pair['iteration']}
                for run in runs.values():
                    # Information only: sliced_time must still be well formed, but nothing gates on its value.
                    finite_number(run['slice_ms'], 'slice_ms', positive=True)
                measured = {side: verify_run_measurement(run) for side, run in runs.items()}
                require(pair['slice_ratio'] == measured['fork'] / measured['official'], 'Reported pair ratio differs from actual samples')
                require(pair['build_parity_ratio'] == measured['fork_cubic'] / measured['official'],
                        'Reported build-parity ratio differs from actual samples')
        runs = [run for run in case_runs if not run['warmup']]
        pairs = [pair for pair in case_pairs if not pair['warmup']]
        instructions = {side: [run['slice_instructions'] for run in runs if run['side'] == side] for side in SIDES}
        slice_ms = {side: [run['slice_ms'] for run in runs if run['side'] == side] for side in SIDES}
        # The gate: the median and the worst of the per-pair ratios of the fork's BAC over official Cubic slice instructions.
        pair_ratios = [pair['slice_ratio'] for pair in pairs]
        parity_ratios = [pair['build_parity_ratio'] for pair in pairs]
        median_ratio, worst_ratio = statistics.median(pair_ratios), max(pair_ratios)
        official_ms = {run['iteration']: run['slice_ms'] for run in runs if run['side'] == 'official'}
        fork_ms = {run['iteration']: run['slice_ms'] for run in runs if run['side'] == 'fork'}
        time_ratios = [fork_ms[iteration] / official_ms[iteration] for iteration in sorted(official_ms)]
        result[size] = {'family': fixture['family'],
            'gate_basis': 'slice instructions retired (full slice minus startup-and-load process): fork BAC over official Cubic per interleaved pair',
            'all_slice_instructions': instructions,
            'median_slice_instructions': {side: statistics.median(values) for side, values in instructions.items()},
            'all_pair_ratios': pair_ratios, 'worst_observed_pair_ratio': worst_ratio, 'median_slice_ratio': median_ratio,
            'ratio_limit': GATE_LIMIT, 'within_2x': median_ratio <= GATE_LIMIT and worst_ratio <= GATE_LIMIT,
            'build_parity_basis': 'fork Cubic over official Cubic slice instructions per interleaved iteration; information only, never gated',
            'all_build_parity_ratios': parity_ratios, 'median_build_parity_ratio': statistics.median(parity_ratios),
            'worst_build_parity_ratio': max(parity_ratios),
            'sliced_time_basis': 'sliced_time and wall times: information only, nothing gates on them',
            'all_slice_ms': slice_ms, 'median_slice_ms': {side: statistics.median(values) for side, values in slice_ms.items()},
            'median_sliced_time_ratio_information_only': statistics.median(time_ratios),
            'worst_sliced_time_ratio_information_only': max(time_ratios),
            'timing_precision': 'sliced_time: integer milliseconds, information only; instructions: exact counts, no denominator floor',
            'official_minimum_slice_ms': min(slice_ms['official']),
            'short_baseline_precision_caution': min(slice_ms['official']) < 100,
            'whole_process_wall_s': {side: [run['whole_process_wall_s'] for run in runs if run['side'] == side]
                                     for side in SIDES}}
    covered_families = {fixture_audit.FAMILIES[size]['family'] for size in report['sizes']}
    legacy_cases = [size for size in report['sizes'] if report['fixtures'][size]['legacy_cube_manifest']]
    sufficient = (not report['development_mode'] and not report.get('official_only', False) and
                  samples >= 5 and warmups >= 1 and not legacy_cases)
    report['summary'] = result
    report['qualification_sampling'] = sufficient
    report['legacy_fixture_cases'] = legacy_cases
    report['scope_gate_passed'] = sufficient and all(result[size]['within_2x'] for size in result)
    report['release_cases_pending'] = [size for size in REQUIRED_CASES if size not in report['sizes']]
    report['release_coverage_pending'] = [family for family in REQUIRED_FAMILIES if family not in covered_families]
    report['gate_passed'] = (report['scope_gate_passed'] and not report['release_cases_pending'] and
                             not report['release_coverage_pending'])
    report['status'] = ('release_speed_gate_passed' if report['gate_passed'] else
                        'scope_gate_passed_release_coverage_pending' if report['scope_gate_passed'] else
                        'failed_or_diagnostic_only')

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--official', required=True, type=Path)
    parser.add_argument('--fork', required=True, type=Path)
    parser.add_argument('--fixtures', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path, help='Must not already exist')
    parser.add_argument('--development', action='store_true', help='Diagnostic only: defaults to one pair/no warm-up; can never pass release gate')
    parser.add_argument('--official-only', action='store_true', help='Development preflight: validate only official outputs; no BAC timing or release gate')
    parser.add_argument('--repo', required=True, type=Path)
    parser.add_argument('--build-provenance', type=Path, help='Optional root-produced immutable build input manifest')
    parser.add_argument('--samples', type=int)
    parser.add_argument('--warmups', type=int)
    parser.add_argument('--timeout', type=int, default=3600)
    parser.add_argument('--sizes', nargs='+', choices=tuple(SIZES), default=list(SIZES))
    parser.add_argument('--parity-exceptions', type=Path)
    parser.add_argument('--intended-exceptions', type=Path)
    parser.add_argument('--revision', default='unspecified')
    parser.add_argument('--check-inputs-only', action='store_true', help='No executable is launched')
    args = parser.parse_args(argv)
    args.samples = (1 if args.development else 5) if args.samples is None else args.samples
    args.warmups = (0 if args.development else 1) if args.warmups is None else args.warmups
    require(not args.official_only or args.development, '--official-only requires --development')
    require(args.development or (args.samples >= 5 and args.warmups >= 1), 'Reduced sampling requires explicit --development mode')
    require(args.samples >= 1 and args.warmups >= 0 and args.timeout > 0, 'Invalid sampling/timeout settings')
    require(len(args.sizes) == len(set(args.sizes)), 'Duplicate requested sizes')
    for field in ('official', 'fork', 'fixtures', 'output', 'repo'):
        setattr(args, field, getattr(args, field).resolve())
    require(args.official.is_file() and args.fork.is_file(), 'Executable missing')
    require(not args.output.exists(), 'Output directory already exists; preserve previous evidence')
    args.output.mkdir(parents=True)
    report = {'schema_version': SCHEMA_VERSION, 'status': 'preparing', 'gate_passed': False, 'runs': []}
    write_json(args.output / 'benchmark.json', report)
    try:
        exceptions = load_exceptions(args.parity_exceptions)
        args.intended_exceptions = load_intended_exceptions(args.intended_exceptions)
        fixtures = {size: validate_fixture_pair(args.fixtures / f'{size}-cubic.3mf',
                    args.fixtures / f'{size}-bridgeawarecubic.3mf') for size in args.sizes}
        for size in args.sizes:
            for variant in ('cubic', 'bac'):
                record = fixtures[size][variant]
                expected = read_expected(args.fixtures, record['path'])
                validate_reviewed_guards(expected, local_configs=record['local_configs'])
                record['reviewed_expected_guards'] = 'passed'
                record['expected_emitted_config_keys'] = len(expected)
                record['expected_emitted_config_sha256'] = hashlib.sha256(json.dumps(expected, sort_keys=True).encode()).hexdigest()
                manifest_record = fixture_audit.fixture_record(args.fixtures, record['path'])
                record['sidecars'] = {key: {'path': str(args.fixtures / manifest_record[key]),
                                            'sha256': sha256(args.fixtures / manifest_record[key])}
                                      for key in ('config_ini', 'project_config_json')}
        report = {'schema_version': SCHEMA_VERSION, 'status': 'input_checks_passed', 'gate_passed': False,
            'harness': {'path': str(Path(__file__).resolve()), 'sha256': sha256(__file__)},
            'fixture_audit_helper': {'path': str(HELPER_PATH), 'sha256': HELPER_LOADED_SHA256},
            'reviewed_exception_guards': {'path': str(GUARDS_PATH), 'sha256': GUARDS_LOADED_SHA256},
            'metric': 'slice instructions: instructions retired (all threads, /usr/bin/time -l) by the full slice (--slice 1) '
                      'minus those retired by the same app\'s startup-and-load process (--slice 99) on the same fixture',
            'metric_boundary': 'per-plate setup/apply/validation through process and G-code export and result.json; startup, '
                               'project load and process exit are removed by the subtraction; no 3MF packaging',
            'whole_process_wall_metric': 'outer monotonic duration of each process; information only, never used in a ratio or gate',
            'gate': {'metric': 'slice_instructions', 'ratio': 'fork BAC over official Cubic, per interleaved pair',
                     'statistics': ['median of the pair ratios', 'worst pair ratio'], 'limit': GATE_LIMIT,
                     'build_parity': 'fork Cubic over official Cubic slice instructions; information only, never gated'},
            'measurement': {'tool': f'{TIME_BINARY} -l', 'figure': 'instructions retired', 'processes_per_run': 2,
                            'slice_process': f'--slice {SLICE_PLATE}: a full slice',
                            'load_process': f'--slice {LOAD_PLATE}: loads the project, rejects the missing plate, writes result.json '
                                            f'and exits with status {LOAD_EXIT_STATUS} without slicing or opening a window',
                            'sides': {'official': 'official app on the Cubic fixture', 'fork': 'fork app on the BAC fixture',
                                      'fork_cubic': 'fork app on the Cubic fixture (build parity)'}},
            'information_only_metrics': {
                'sliced_time': 'result.json sliced_plates[0].times.sliced_time, wall-clock milliseconds of the slice process; '
                               'information only, nothing gates on it',
                'whole_process_wall_s': 'outer wall-clock seconds of a process; information only, nothing gates on it',
                'sliced_time_ratio': 'fork BAC over official Cubic sliced_time per pair; information only'},
            'architecture': 'arm64', 'network_policy': 'sandbox deny network*',
            'profile_policy': 'isolated datadir/TMPDIR/CFFIXED_USER_HOME; HOME preserved; live Bambu profiles/Cocoa state/file and app-domain preferences denied',
            'sizes': args.sizes, 'samples': args.samples, 'warmups': args.warmups,
            'fixture_family_scope': sorted({fixtures[size]['family'] for size in args.sizes}),
            'required_release_families': list(REQUIRED_FAMILIES), 'required_release_cases': list(REQUIRED_CASES),
            'revision': args.revision, 'fixtures': fixtures,
            'fixture_manifest_sha256': sha256(args.fixtures / 'manifest.json'), 'approved_parity_exceptions': exceptions,
            'approved_intended_exceptions': args.intended_exceptions,
            'source_snapshot_at_start': source_snapshot(args.repo),
            'build_provenance': read_json(args.build_provenance) if args.build_provenance else None,
            'development_mode': args.development, 'official_only': args.official_only,
            'executables': {side: {'path': str(getattr(args, side)), 'sha256': sha256(getattr(args, side))}
                            for side in ('official', 'fork')}, 'runs': [], 'pairs': []}
        write_json(args.output / 'benchmark.json', report)
    except BaseException as error:
        report.update(status='invalid_input_preparation', gate_passed=False, error=f'{type(error).__name__}: {error}')
        write_json(args.output / 'benchmark.json', report)
        raise
    if args.check_inputs_only:
        print(json.dumps({'status': 'input_checks_passed', 'report': str(args.output / 'benchmark.json')}))
        return 0
    try:
        reference_configs = {}
        for size in args.sizes:
            for warmup, count in ((True, args.warmups), (False, args.samples)):
                for iteration in range(1, count + 1):
                    # Odd iterations run official Cubic, the fork's BAC, then the fork's Cubic; even ones run the reverse, so
                    # the gate's pair alternates and so does the build-parity pair.
                    sides = ('official',) if args.official_only else (SIDES if iteration % 2 else SIDES[::-1])
                    measured = {}
                    configs = {}
                    for side in sides:
                        measured[side], configs[side] = run_one(args, report, size, side, iteration, warmup, fixtures)
                        identity = (size, side)
                        if identity in reference_configs:
                            require(configs[side] == reference_configs[identity],
                                    f'Actual settings changed between runs for {size}/{side}')
                        else:
                            reference_configs[identity] = configs[side]
                    if args.official_only:
                        continue
                    parity = compare_emitted(configs['official'], configs['fork'], exceptions, fixtures[size]['bac_gap_mm'])
                    # The fork's Cubic against official Cubic, on the same input: the fork adds only its default BAC gap.
                    build_parity_config = compare_emitted(configs['official'], configs['fork_cubic'], exceptions, FORK_DEFAULT_GAP_MM)
                    official_slice = measured['official']['slice_instructions']
                    pair = {'size': size, 'family': fixtures[size]['family'], 'warmup': warmup, 'iteration': iteration,
                        'order': [side for side in sides if side != 'fork_cubic'], 'execution_order': list(sides),
                        # The gate's ratio, and the build parity beside it, in slice instructions; the sliced_time ratio is information only.
                        'slice_ratio': measured['fork']['slice_instructions'] / official_slice,
                        'build_parity_ratio': measured['fork_cubic']['slice_instructions'] / official_slice,
                        'sliced_time_ratio_information_only': measured['fork']['slice_ms'] / measured['official']['slice_ms'],
                        'parity': parity, 'build_parity_config': build_parity_config}
                    report['pairs'].append(pair)
                    write_json(args.output / 'benchmark.json', report)
                    require(not parity['rejected_differences'],
                            f'Unapproved emitted-config differences: {parity["rejected_differences"]}')
                    require(not build_parity_config['rejected_differences'],
                            f'Unapproved emitted-config differences between official Cubic and fork Cubic: '
                            f'{build_parity_config["rejected_differences"]}')
        verify_frozen_inputs(args, report, fixtures)
        report['source_snapshot_at_end'] = source_snapshot(args.repo)
        require(report['source_snapshot_at_end']['head'] == report['source_snapshot_at_start']['head'] and
                report['source_snapshot_at_end']['snapshot_sha256'] == report['source_snapshot_at_start']['snapshot_sha256'],
                'Source working tree changed during benchmark')
        report['executables_at_end'] = {side: sha256(getattr(args, side)) for side in ('official', 'fork')}
        require(all(report['executables_at_end'][side] == report['executables'][side]['sha256'] for side in ('official', 'fork')),
                'Executable changed while benchmark ran')
        if args.official_only:
            report.update(status='official_preflight_passed_diagnostic_only', gate_passed=False, summary={})
        else:
            summarize(report, args.samples, args.warmups)
        write_json(args.output / 'benchmark.json', report)
        print(json.dumps({'status': report['status'], 'gate_passed': report['gate_passed'],
                          'summary': report['summary'], 'report': str(args.output / 'benchmark.json')}, indent=2))
        return 0 if args.official_only or report.get('gate_passed') else 1
    except BaseException as error:
        report.update(status='invalid_or_incomplete', gate_passed=False, error=f'{type(error).__name__}: {error}')
        if report['runs'] and report['runs'][-1]['status'] == 'running':
            report['runs'][-1]['status'] = 'failed_individual_checks'
        write_json(args.output / 'benchmark.json', report)
        raise

if __name__ == '__main__':
    try:
        sys.exit(main())
    except (InvalidBenchmark, OSError, ValueError, KeyError, ET.ParseError, zipfile.BadZipFile) as error:
        print(f'INVALID BENCHMARK: {error}', file=sys.stderr)
        sys.exit(2)
