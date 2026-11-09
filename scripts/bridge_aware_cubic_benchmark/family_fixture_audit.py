"""Manifest-driven 3MF fixture auditing; pure Python, never launches a slicer.

Public API: fixture_record(directory, archive), load_expected(directory, archive),
audit_archive(path, record=None, fixtures_dir=None), validate_fixture_pair(a, b).
The last function returns serializable pair metadata, not archive payloads.
Legacy cube-only v1 manifests are accepted with explicit fixed cube expectations.
"""
from __future__ import annotations
import copy
import hashlib
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET
import zipfile

PATTERN = 'sparse_infill_pattern'
GAP = 'bridge_aware_cubic_max_gap'
# These keys are omitted by the G-code configuration serializer, not waived in
# archive parity. They remain present in the archive-side expected snapshot.
BANNED_CONFIG_KEYS = {'compatible_printers', 'compatible_prints', 'print_host', 'print_host_webui',
    'printhost_apikey', 'printhost_cafile', 'printhost_user', 'printhost_password', 'printhost_port'}
FAMILIES = {
    'small': {'family': 'cube', 'dimensions_mm': [32., 28., 18.], 'expected_layer_count': 90,
              'model_triangle_count': 12, 'volume_count': 1, 'triangle_count': 12, 'density_percent': 5., 'gap': 5., 'angle': '45'},
    'large': {'family': 'cube', 'dimensions_mm': [64., 56., 18.], 'expected_layer_count': 90,
              'model_triangle_count': 12, 'volume_count': 1, 'triangle_count': 12, 'density_percent': 5., 'gap': 5., 'angle': '45'},
    'rib': {'family': 'rib', 'dimensions_mm': [32., 26., 15.], 'expected_layer_count': 75,
            'model_triangle_count': 36, 'volume_count': 1, 'triangle_count': 36, 'density_percent': 12., 'gap': 5., 'angle': '17'},
    'modifier': {'family': 'modifier', 'dimensions_mm': [48., 28., 18.], 'expected_layer_count': 90,
                 'model_triangle_count': 12, 'volume_count': 2, 'triangle_count': 24, 'density_percent': 5., 'gap': 8., 'angle': '45'},
}
CORE = '{http://schemas.microsoft.com/3dmanufacturing/core/2015/02}'
REPAIR_KEYS = ('edges_fixed', 'degenerate_facets', 'facets_removed', 'facets_reversed', 'backwards_edges')

class InvalidFixture(RuntimeError):
    pass

# Useful when adopted directly by the timing runner.
InvalidBenchmark = InvalidFixture

def require(condition, message):
    if not condition:
        raise InvalidFixture(message)

def no_duplicates(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, f'Duplicate JSON key: {key}')
        result[key] = value
    return result

def read_json(path):
    return json.loads(Path(path).read_text(), object_pairs_hook=no_duplicates)

def sha256(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()

def _inside(directory, name):
    directory = Path(directory).resolve()
    require(isinstance(name, str) and name and not Path(name).is_absolute(), 'Invalid fixture relative path')
    path = (directory / name).resolve()
    require(path.is_relative_to(directory), f'Fixture sidecar escapes directory: {name}')
    return path

def _strings(value, context):
    require(isinstance(value, dict) and all(isinstance(k, str) and isinstance(v, str) for k, v in value.items()),
            f'{context} must map strings to serialized strings')
    return value

def _nil_keys(record):
    keys = record.get('nil_config_keys')
    require(isinstance(keys, list) and all(isinstance(k, str) for k in keys) and len(keys) == len(set(keys)),
            'Manifest must list unique nil_config_keys, even when empty')
    require(set(keys) <= set(record['serialized_config']), 'Nil key missing from serialized config')
    return set(keys)

def load_manifest(fixtures_dir):
    manifest = read_json(Path(fixtures_dir) / 'manifest.json')
    require(manifest.get('schema_version') == 1, 'Unknown fixture manifest schema')
    require(manifest.get('slicing_performed') is False, 'Fixture generator must not pre-slice input')
    require(manifest.get('allowed_pair_config_differences') == [PATTERN, GAP], 'Unexpected allowed pair differences')
    records = manifest.get('fixtures')
    require(isinstance(records, list) and records, 'Manifest has no fixture records')
    archives = [r.get('archive') for r in records]
    require(all(isinstance(a, str) for a in archives) and len(archives) == len(set(archives)), 'Duplicate/malformed fixture archive')
    return manifest

def _expected_volumes(record):
    family = record['family']
    first = {'index': 0, 'name': 'Benchmark inward ribs' if family == 'rib' else 'Benchmark cube',
             'type': 'normal_part', 'triangle_count': record['model_triangle_count'],
             'bounds_min_mm': [0., 0., 0.], 'bounds_max_mm': record['dimensions_mm'],
             'serialized_config': {}, 'nil_config_keys': []}
    volumes = [first]
    if family == 'modifier':
        local = {'sparse_infill_density': '12%', 'infill_direction': '17'}
        if record['pattern'] == 'bridgeawarecubic':
            local[GAP] = '3'
        volumes.append({'index': 1, 'name': 'Benchmark right half modifier', 'type': 'modifier_part',
                        'triangle_count': 12, 'bounds_min_mm': [24., 0., 0.], 'bounds_max_mm': [48., 28., 18.],
                        'serialized_config': local, 'nil_config_keys': []})
    return volumes

def fixture_record(fixtures_dir, fixture):
    directory = Path(fixtures_dir).resolve()
    records = [r for r in load_manifest(directory)['fixtures'] if r['archive'] == Path(fixture).name]
    require(len(records) == 1, f'No unique fixture manifest entry for {fixture}')
    record = copy.deepcopy(records[0])
    require(_inside(directory, record['archive']) == (directory / Path(fixture).name).resolve(), 'Archive must be in fixture root')
    size = record.get('size')
    require(size in FAMILIES, f'Unknown fixture family/size: {size}')
    expected = FAMILIES[size]
    new_keys = {'family', 'model_triangle_count', 'volume_count', 'volumes', 'filament_count'}
    missing = new_keys - record.keys()
    legacy = bool(missing)
    if legacy:
        require(missing == new_keys and size in ('small', 'large'), 'Incomplete extended fixture manifest')
        record.update({key: expected[key] for key in ('family', 'model_triangle_count', 'volume_count')})
        record['filament_count'] = 1
        record['volumes'] = _expected_volumes(record)
    record['legacy_cube_manifest'] = legacy
    for key in ('family', 'dimensions_mm', 'expected_layer_count', 'model_triangle_count',
                'volume_count', 'triangle_count', 'density_percent'):
        require(record.get(key) == expected[key], f'Unexpected {size} manifest {key}: {record.get(key)}')
    for key in ('object_count', 'instance_count', 'plate_count', 'filament_count'):
        require(record.get(key) == 1, f'Unexpected manifest {key}')
    require(record.get('pattern') in ('cubic', 'bridgeawarecubic'), 'Unexpected fixture pattern')
    require(record.get('name') == f'{size}-{record["pattern"]}' and record['archive'] == record['name'] + '.3mf',
            'Fixture identity disagrees with filename')
    require(record.get('instance_offset_mm') == [80., 80., 0.] and record.get('bounds_min_mm') == [80., 80., 0.],
            'Unexpected benchmark placement')
    maximum = [80. + record['dimensions_mm'][0], 80. + record['dimensions_mm'][1], record['dimensions_mm'][2]]
    require(record.get('bounds_max_mm') == maximum, 'Unexpected benchmark bounds')
    require(record.get('bac_gap_mm') == (expected['gap'] if record['pattern'] == 'bridgeawarecubic' else None),
            'Unexpected manifest BAC gap')
    config = _strings(record.get('serialized_config'), 'serialized_config')
    require(record.get('config_key_count') == len(config), 'Manifest config count mismatch')
    _nil_keys(record)
    required = {PATTERN: record['pattern'], 'sparse_infill_density': f'{expected["density_percent"]:g}%',
                'infill_direction': expected['angle'], 'layer_height': '0.2', 'initial_layer_print_height': '0.2'}
    for key, value in required.items():
        require(config.get(key) == value, f'Manifest {key} differs from fixture requirement')
    if record['pattern'] == 'cubic':
        require(GAP not in config, 'Cubic input must omit unknown BAC setting')
    else:
        require(config.get(GAP) == f'{expected["gap"]:g}', 'Serialized BAC gap disagrees with manifest')
    require(record['volumes'] == _expected_volumes(record), 'Volume manifest differs from approved family geometry or local settings')
    return record

def load_expected(fixtures_dir, fixture):
    """Return exact G-code-serializable, non-nil global configuration snapshot."""
    record = fixture_record(fixtures_dir, fixture)
    ini = _inside(fixtures_dir, record['config_ini'])
    expected = {}
    for line in ini.read_text().splitlines():
        if not line or line.startswith('#'):
            continue
        match = re.fullmatch(r'([A-Za-z0-9_]+) = (.*)', line)
        require(match is not None, f'Malformed expected INI line: {ini}')
        key, value = match.groups()
        require(key not in expected, f'Duplicate expected INI key: {key}')
        expected[key] = value
    require(expected == record['serialized_config'], 'Manifest serialized_config differs from INI snapshot')
    return {key: value for key, value in expected.items() if key not in BANNED_CONFIG_KEYS | _nil_keys(record)}

def canonical_xml(data):
    """Canonical structure: only inter-element formatting is discarded."""
    def significant(value):
        return '' if value is None or not value.strip() else value
    def element(node):
        return (node.tag, tuple(sorted(node.attrib.items())), significant(node.text), significant(node.tail),
                tuple(element(child) for child in node))
    return element(ET.fromstring(data) if isinstance(data, (bytes, str)) else data)

def _xml_hash(node):
    return hashlib.sha256(json.dumps(canonical_xml(node), separators=(',', ':')).encode()).hexdigest()

def _numbers(value, count, context):
    try:
        numbers = [float(v) for v in value.split()]
    except (AttributeError, ValueError) as error:
        raise InvalidFixture(f'Invalid {context}') from error
    require(len(numbers) == count and all(math.isfinite(v) for v in numbers), f'Invalid {context}')
    return numbers

def _same_numbers(actual, expected, context, tolerance=1e-6):
    require(len(actual) == len(expected) and all(abs(a-b) <= tolerance for a, b in zip(actual, expected)),
            f'{context}: expected {expected}, found {actual}')

def _metadata(node, allowed, keyless=None):
    result = {}
    keyless_seen = 0
    for child in node.findall('metadata'):
        require(len(child) == 0 and not (child.text or '').strip(), 'Unexpected metadata body')
        if 'key' not in child.attrib:
            require(keyless is not None and child.attrib == keyless, 'Unexpected keyless local metadata')
            keyless_seen += 1
            continue
        require(set(child.attrib) == {'key', 'value'}, 'Malformed local metadata attributes')
        key = child.get('key')
        require(key in allowed and key not in result, f'Unexpected/repeated local metadata key: {key}')
        result[key] = child.get('value')
    require(keyless_seen == int(keyless is not None), 'Incorrect keyless metadata count')
    return result

def _children(node, allowed):
    require(all(child.tag in allowed for child in node), f'Unexpected child of {node.tag}')
    require(not (node.text or '').strip(), f'Unexpected text in {node.tag}')

def _audit_local(raw, record):
    root = ET.fromstring(raw)
    require(root.tag == 'config' and not root.attrib, 'Unexpected model settings root')
    _children(root, {'object', 'plate', 'assemble'})
    require(len(root.findall('object')) == len(root.findall('plate')) == len(root.findall('assemble')) == 1,
            'Expected one local object, plate and empty assembly')
    assemble = root.find('assemble')
    require(not assemble.attrib and len(assemble) == 0 and not (assemble.text or '').strip(), 'Unexpected assembly override')
    obj = root.find('object')
    require(set(obj.attrib) == {'id'}, 'Unexpected local object attributes')
    _children(obj, {'metadata', 'part'})
    object_metadata = _metadata(obj, {'name'}, {'face_count': str(record['model_triangle_count'])})
    require(object_metadata == {'name': 'BAC benchmark ' + record['size']}, 'Unexpected object name/config')
    parts = obj.findall('part')
    require(len(parts) == record['volume_count'], 'Unexpected local volume count')
    part_records = []
    part_ids = set()
    for part, volume in zip(parts, record['volumes']):
        require(set(part.attrib) in ({'id', 'subtype'}, {'id', 'subtype', 'uuid'}), 'Unexpected part attributes')
        require(part.get('id') not in part_ids, 'Repeated part mesh id')
        part_ids.add(part.get('id'))
        require(part.get('subtype') == volume['type'], 'Part subtype differs from manifest')
        _children(part, {'metadata', 'mesh_stat'})
        expected_config = _strings(volume['serialized_config'], 'volume serialized_config')
        metadata = _metadata(part, {'name', 'matrix'} | set(expected_config))
        require(metadata.get('name') == volume['name'], 'Part name differs from manifest')
        matrix = _numbers(metadata.get('matrix'), 16, 'part matrix')
        _same_numbers(matrix[12:], [0., 0., 0., 1.], 'Part homogeneous row', 0.)
        local_config = {k: v for k, v in metadata.items() if k not in ('name', 'matrix')}
        require(local_config == expected_config, f'Part {volume["index"]} local config differs from manifest')
        stats = part.findall('mesh_stat')
        require(len(stats) == 1 and len(stats[0]) == 0 and stats[0].attrib ==
                {'face_count': str(volume['triangle_count']), **{k: '0' for k in REPAIR_KEYS}}, 'Unexpected repaired mesh')
        part_records.append({'mesh_id': part.get('id'), 'matrix': matrix, 'volume': volume, 'local_config': local_config})
        # Only this fully validated setting may differ between the input pair.
        for child in list(part):
            if child.tag == 'metadata' and child.get('key') == GAP:
                part.remove(child)
    plate = root.find('plate')
    require(not plate.attrib, 'Unexpected plate attributes')
    _children(plate, {'metadata', 'model_instance'})
    require(_metadata(plate, {'plater_id', 'plater_name', 'locked'}) ==
            {'plater_id': '1', 'plater_name': 'Benchmark plate', 'locked': 'false'}, 'Unexpected plate config')
    assigned = plate.findall('model_instance')
    require(len(assigned) == 1 and not assigned[0].attrib, 'Unexpected assigned instance count')
    _children(assigned[0], {'metadata'})
    identity = _metadata(assigned[0], {'object_id', 'instance_id', 'identify_id'})
    require(set(identity) == {'object_id', 'instance_id', 'identify_id'} and identity['object_id'] == obj.get('id')
            and identity['instance_id'] == '0' and re.fullmatch(r'[0-9]+', identity['identify_id']), 'Invalid assigned instance identity')
    return root, obj.get('id'), part_records

def _apply_transform(values, vertex):
    # 3MF 12-number transform stores 3x3 by columns, then translation.
    return [sum(values[c * 3 + r] * vertex[c] for c in range(3)) + values[9+r] for r in range(3)]

def _audit_model(raw, record, object_id, parts):
    model = ET.fromstring(raw)
    require(model.tag == CORE + 'model' and model.get('unit') == 'millimeter', 'Expected millimeter 3MF model')
    require(set(model.attrib) <= {'unit', '{http://www.w3.org/XML/1998/namespace}lang'}, 'Unexpected model attributes/extensions')
    require(all(n.tag in {CORE+'metadata', CORE+'resources', CORE+'build'} for n in model), 'Unexpected model child')
    timestamps = {}
    for metadata in model.findall(CORE+'metadata'):
        require(set(metadata.attrib) == {'name'} and len(metadata) == 0, 'Unexpected global model metadata')
        name = metadata.get('name')
        if name in ('CreationDate', 'ModificationDate'):
            require(name not in timestamps and re.fullmatch(r'\d{4}-\d{2}-\d{2}', metadata.text or ''), 'Unexpected timestamp format')
            timestamps[name] = metadata.text
            metadata.text = '<nonfunctional-date>'
    resources = model.findall(CORE+'resources')
    builds = model.findall(CORE+'build')
    require(len(resources) == len(builds) == 1 and not resources[0].attrib and not builds[0].attrib,
            'Expected one plain resources/build block')
    resources, build = resources[0], builds[0]
    require(all(n.tag == CORE+'object' for n in resources), 'Unexpected material/paint resource')
    objects = {}
    object_types = {object_id: 'model', **{p['mesh_id']: ('other' if p['volume']['type'] == 'modifier_part' else 'model') for p in parts}}
    for node in resources:
        require(set(node.attrib) == {'id', 'type'} and node.get('type') == object_types.get(node.get('id')),
                'Unexpected object type/material/property override')
        require(node.get('id') not in objects, 'Repeated resource object id')
        objects[node.get('id')] = node
    require(set(objects) == {object_id} | {p['mesh_id'] for p in parts} and len(objects) == len(parts)+1,
            'Unexpected model/mesh resource set')
    container = objects[object_id]
    require(len(container) == 1 and container[0].tag == CORE+'components' and not container[0].attrib, 'Expected component model')
    components = list(container[0])
    require(len(components) == len(parts), 'Unexpected component count')
    require(len(build) == 1 and build[0].tag == CORE+'item' and len(build[0]) == 0 and
            set(build[0].attrib) == {'objectid', 'transform', 'printable'} and
            build[0].get('objectid') == object_id and build[0].get('printable') == '1', 'Unexpected build instance')
    build_transform = _numbers(build[0].get('transform'), 12, 'build transform')
    _same_numbers(build_transform, [1.,0.,0.,0.,1.,0.,0.,0.,1.] + record['instance_offset_mm'], 'Build placement', 0.)
    model_points = []
    total_triangles = 0
    for component, part in zip(components, parts):
        volume = part['volume']
        require(component.tag == CORE+'component' and len(component) == 0 and set(component.attrib) == {'objectid', 'transform'}
                and component.get('objectid') == part['mesh_id'], 'Component/part order or identity mismatch')
        transform = _numbers(component.get('transform'), 12, 'component transform')
        matrix = part['matrix']
        _same_numbers(transform, [matrix[r*4+c] for c in range(3) for r in range(3)] + [matrix[r*4+3] for r in range(3)],
                      'Part/component transform disagreement')
        mesh_object = objects[part['mesh_id']]
        require(len(mesh_object) == 1 and mesh_object[0].tag == CORE+'mesh' and not mesh_object[0].attrib, 'Expected plain mesh object')
        mesh = mesh_object[0]
        require([n.tag for n in mesh] == [CORE+'vertices', CORE+'triangles'] and all(not n.attrib for n in mesh), 'Unexpected mesh structure')
        vertices = []
        for vertex in mesh[0]:
            require(vertex.tag == CORE+'vertex' and len(vertex) == 0 and set(vertex.attrib) == {'x','y','z'}, 'Unexpected painted/property vertex')
            point = _numbers(' '.join(vertex.get(k) for k in ('x','y','z')), 3, 'mesh vertex')
            vertices.append(_apply_transform(transform, point))
        require(len(vertices) == (24 if record['family'] == 'rib' else 8), 'Unexpected mesh vertex count')
        triangles = list(mesh[1])
        require(len(triangles) == volume['triangle_count'], 'Manifest mesh triangle count mismatch')
        for triangle in triangles:
            require(triangle.tag == CORE+'triangle' and len(triangle) == 0 and set(triangle.attrib) == {'v1','v2','v3'},
                    'Unexpected painted/property triangle')
            indices = [triangle.get(k) for k in ('v1','v2','v3')]
            require(all(re.fullmatch(r'[0-9]+', i) and int(i) < len(vertices) for i in indices) and len({int(i) for i in indices}) == 3,
                    'Invalid triangle vertex indices')
        for key, actual in [('bounds_min_mm', [min(v[i] for v in vertices) for i in range(3)]),
                            ('bounds_max_mm', [max(v[i] for v in vertices) for i in range(3)])]:
            _same_numbers(actual, volume[key], 'Volume ' + key)
        if volume['type'] == 'normal_part':
            model_points.extend(_apply_transform(build_transform, point) for point in vertices)
        total_triangles += len(triangles)
    require(total_triangles == record['triangle_count'], 'Wrong total triangle count')
    for key, actual in [('bounds_min_mm', [min(v[i] for v in model_points) for i in range(3)]),
                        ('bounds_max_mm', [max(v[i] for v in model_points) for i in range(3)])]:
        _same_numbers(actual, record[key], 'Model ' + key)
    geometry_hash = hashlib.sha256((_xml_hash(resources) + _xml_hash(build)).encode()).hexdigest()
    return model, geometry_hash, timestamps

def audit_archive(path, record=None, fixtures_dir=None):
    path = Path(path).resolve()
    directory = Path(fixtures_dir).resolve() if fixtures_dir is not None else path.parent
    checked = fixture_record(directory, path)
    require(record is None or record == checked, 'Provided fixture record is stale or unvalidated')
    record = checked
    require(path == _inside(directory, record['archive']), 'Archive path differs from manifest')
    expected_serialized = load_expected(directory, path)
    project = read_json(_inside(directory, record['project_config_json']))
    with zipfile.ZipFile(path) as archive:
        names = archive.namelist()
        normalized = [n.replace('\\', '/').lower() for n in names]
        require(len(normalized) == len(set(normalized)), 'Duplicate/colliding ZIP entries')
        require(all(n and n == n.replace('\\', '/') and not n.startswith('/') and '..' not in Path(n).parts for n in names), 'Unsafe ZIP path')
        required = {'[Content_Types].xml', '3D/3dmodel.model', 'Metadata/project_settings.config', 'Metadata/model_settings.config',
                    'Metadata/cut_information.xml', 'Metadata/slice_info.config', 'Metadata/filament_sequence.json', '_rels/.rels'}
        require(set(names) == required, f'Unexpected fixture payloads: {sorted(set(names) ^ required)}')
        actual = json.loads(archive.read('Metadata/project_settings.config'), object_pairs_hook=no_duplicates)
        require(actual == project, 'Archive global config differs from project JSON snapshot')
        for key in (PATTERN, 'sparse_infill_density', 'infill_direction', 'layer_height', 'initial_layer_print_height'):
            require(actual.get(key) == record['serialized_config'][key], f'Archive {key} differs from manifest')
        require((GAP not in actual) if record['pattern'] == 'cubic' else actual.get(GAP) == record['serialized_config'][GAP],
                'Archive BAC gap differs from manifest')
        require(isinstance(actual.get('filament_settings_id'), list) and
                len(actual['filament_settings_id']) == record['filament_count'], 'Archive filament count differs from manifest')
        require(actual.get('inherits') == '' and actual.get('inherits_group') == [''] * (record['filament_count'] + 2),
                'Archive activates inheritance or has invalid inheritance slot count')
        local, object_id, parts = _audit_local(archive.read('Metadata/model_settings.config'), record)
        model, geometry_hash, timestamps = _audit_model(archive.read('3D/3dmodel.model'), record, object_id, parts)
        payloads = {}
        for name in names:
            if name == 'Metadata/project_settings.config':
                continue
            if name == 'Metadata/model_settings.config':
                payloads[name] = canonical_xml(local)
            elif name == '3D/3dmodel.model':
                payloads[name] = canonical_xml(model)
            elif name.endswith(('.xml', '.rels', '.config')):
                payloads[name] = canonical_xml(archive.read(name))
            else:
                payloads[name] = ('sha256', hashlib.sha256(archive.read(name)).hexdigest())
    return {'path': str(path), 'sha256': sha256(path), 'record': record, 'global_config': actual,
            'expected_serialized_config': expected_serialized, 'pair_payloads': payloads,
            'geometry_transform_sha256': geometry_hash, 'nonfunctional_dates': timestamps,
            'local_configs': [p['local_config'] for p in parts], 'mesh_ids_by_volume': [p['mesh_id'] for p in parts]}

def validate_fixture_pair(cubic, bac, fixtures_dir=None):
    a = audit_archive(cubic, fixtures_dir=fixtures_dir)
    b = audit_archive(bac, fixtures_dir=fixtures_dir)
    ar, br = a['record'], b['record']
    require(ar['pattern'] == 'cubic' and br['pattern'] == 'bridgeawarecubic', 'Fixture pair order/pattern mismatch')
    require(ar['size'] == br['size'] and ar['family'] == br['family'], 'Mismatched fixture family')
    for field in ('global_config', 'expected_serialized_config'):
        aa, bb = a[field], b[field]
        differences = {key: {'cubic': aa.get(key), 'bac': bb.get(key)} for key in sorted(aa.keys() | bb.keys())
                       if key not in (PATTERN, GAP) and (key not in aa or key not in bb or aa[key] != bb[key])}
        require(not differences, f'Paired {field} differs: {differences}')
    require(a['geometry_transform_sha256'] == b['geometry_transform_sha256'], 'Paired geometry/transforms differ')
    differences = [key for key in a['pair_payloads'].keys() | b['pair_payloads'].keys()
                   if a['pair_payloads'].get(key) != b['pair_payloads'].get(key)]
    require(not differences, f'Paired model/local configuration/archive payloads differ: {sorted(differences)}')
    for index, (aa, bb) in enumerate(zip(a['local_configs'], b['local_configs'])):
        require({k:v for k,v in aa.items() if k != GAP} == {k:v for k,v in bb.items() if k != GAP},
                f'Paired volume {index} differs beyond BAC gap')
    result = {key: ar[key] for key in ('size', 'family', 'dimensions_mm', 'bounds_min_mm', 'bounds_max_mm',
              'instance_offset_mm', 'expected_layer_count', 'triangle_count', 'model_triangle_count', 'volume_count',
              'object_count', 'instance_count', 'plate_count', 'filament_count', 'density_percent', 'legacy_cube_manifest')}
    result.update({'bac_gap_mm': br['bac_gap_mm'], 'geometry_transform_sha256': a['geometry_transform_sha256'],
                   'common_config_keys': len(set(a['global_config']) - {PATTERN, GAP}),
                   'model_payload_entries': sorted(a['pair_payloads']),
                   'permitted_pair_differences': {'global': [PATTERN, GAP], 'per_volume': [GAP],
                                                'model_metadata': ['CreationDate', 'ModificationDate']}})
    for side, audited in (('cubic', a), ('bac', b)):
        result[side] = {key: audited[key] for key in ('path', 'sha256', 'expected_serialized_config',
                         'local_configs', 'mesh_ids_by_volume', 'nonfunctional_dates')}
    return result
