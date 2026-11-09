"""Pure configuration guards for the exact official-v4 benchmark exceptions.

All maps contain C++ opt_serialize strings, not raw project JSON values.
Call once on the intended snapshot and every audited local config, then again
with the emitted map. None means no local configuration maps supplied; archive
paint absence, exactly one used filament, placement and CLI flags MUST still be
proved by the separate archive/invocation audit. This function never launches a
process or changes files. It returns None or raises ValueError.
"""
from collections.abc import Mapping, Sequence

INPUT_EXACT = {'additional_cooling_fan_speed': '0',
 'auxiliary_fan': '0',
 'bridge_acceleration': '0',
 'bridge_layer_density': '100%',
 'bridge_line_travel_extend': '15%',
 'enable_pre_heating': '0',
 'enable_prime_tower': '0',
 'enable_support': '0',
 'enable_wrapping_detection': '0',
 'enforce_support_layers': '0',
 'extra_wall_slope_angle': '0',
 'extruder_ams_count': '',
 'extruder_max_nozzle_count': '1',
 'extruder_type': 'Direct Drive',
 'filament_map': '1',
 'filament_map_2': '1',
 'filament_map_mode': 'Auto For Flush',
 'filament_nozzle_map': '1',
 'filament_volume_map': '0',
 'has_filament_switcher': '0',
 'infill_wall_overlap': '15%',
 'inherits': '',
 'inherits_group': ';;',
 'internal_bridge_layers': '1',
 'master_extruder_id': '1',
 'nozzle_diameter': '0.4',
 'nozzle_volume_type': 'Standard',
 'painted_wall_loops': '2',
 'print_extruder_id': '1',
 'print_extruder_variant': '"Direct Drive Standard"',
 'print_sequence': 'by layer',
 'raft_layers': '0',
 'single_extruder_multi_material': '0'}

EMITTED_EXACT = {'additional_cooling_fan_speed': '0',
 'auxiliary_fan': '0',
 'enable_pre_heating': '0',
 'enable_prime_tower': '0',
 'enable_support': '0',
 'enable_wrapping_detection': '0',
 'enforce_support_layers': '0',
 'extruder_ams_count': '',
 'extruder_max_nozzle_count': '1',
 'extruder_type': 'Direct Drive',
 'filament_map': '1',
 'filament_map_2': '0',
 'filament_map_mode': 'Auto For Flush',
 'filament_nozzle_map': '0',
 'filament_volume_map': '0',
 'has_filament_switcher': '0',
 'infill_wall_overlap': '15%',
 'inherits': '',
 'master_extruder_id': '1',
 'nozzle_diameter': '0.4',
 'nozzle_volume_type': 'Standard',
 'print_extruder_id': '1',
 'print_extruder_variant': '"Direct Drive Standard"',
 'print_sequence': 'by layer',
 'raft_layers': '0',
 'single_extruder_multi_material': '0'}

NEW_EXACT_IF_PRESENT = {'ams_nozzle_idx': '0',
 'aux_fan_independent_control': '0',
 'aux_fan_standard_side': 'right',
 'close_optional_fan_first_x_layers': '1',
 'default_wipe_tower_pos': '0.644,0.977',
 'different_settings_to_system': ';;',
 'extruder_nozzle_count': '1',
 'extruder_nozzle_diameter': '0.4',
 'extruder_nozzle_volume_type': 'Standard',
 'filament_diameter_map': '',
 'filament_idle_travel_cooling_enabled': '1',
 'filament_pre_heating_enabled': '1',
 'filament_prime_tower_speed_factor_curve': '""',
 'fins_extrude_safe_temp': '0',
 'first_x_layer_optional_fan_speed': '0',
 'handle_hotend_as_extruder': '0',
 'hotend_cooling_rate_on_rack': '2',
 'min_temperature_when_ec': '-1',
 'optional_cooling_fan_speed': '0',
 'optional_fan_full_speed_layer': '0',
 'prime_tower_acceleration': '0',
 'prime_tower_feedrate_factor': '1',
 'prime_tower_ironing_speed': '10',
 'prime_tower_line_ironing': '1',
 'prime_tower_line_width': '0',
 'prime_tower_ramming_line_width': '0',
 'prime_tower_side_scrape': '0',
 'prime_tower_side_scrape_along_length': '2',
 'prime_tower_side_scrape_face': 'front_back',
 'prime_tower_side_scrape_height': '1.5',
 'prime_tower_side_scrape_length': '2',
 'prime_tower_side_scrape_point_spacing': '1',
 'prime_tower_side_scrape_speed': '1400',
 'prime_tower_side_scrape_wall_gap': '0.2',
 'prime_tower_tail_slowdown_volume': '10',
 'prime_tower_tail_speed_factor': '100%',
 'prime_tower_wipe_distance': '0',
 'prime_tower_wipe_speed': '0',
 'print_compatible_printers': '',
 'printer_technology': 'FFF',
 'sync_auxiliary_fan_to_second_fan': '0',
 'tree_support_tip_diameter': '0.8',
 'upward_compatible_machine': ''}

FORK_EXACT_IF_PRESENT = {'bridge_acceleration': '0',
 'bridge_layer_density': '100%',
 'bridge_line_travel_extend': '15%',
 'extra_wall_slope_angle': '0',
 'internal_bridge_layers': '1',
 'painted_wall_loops': '2'}

GOVERNED_KEYS = ('ams_nozzle_idx',
 'aux_fan_independent_control',
 'aux_fan_standard_side',
 'bridge_acceleration',
 'bridge_layer_density',
 'bridge_line_travel_extend',
 'close_optional_fan_first_x_layers',
 'default_wipe_tower_pos',
 'different_settings_to_system',
 'extra_wall_slope_angle',
 'extruder_nozzle_count',
 'extruder_nozzle_diameter',
 'extruder_nozzle_volume_type',
 'filament_diameter_map',
 'filament_idle_travel_cooling_enabled',
 'filament_map_2',
 'filament_nozzle_map',
 'filament_pre_heating_enabled',
 'filament_prime_tower_speed_factor_curve',
 'fins_extrude_safe_temp',
 'first_x_layer_optional_fan_speed',
 'handle_hotend_as_extruder',
 'hotend_cooling_rate_on_rack',
 'internal_bridge_layers',
 'max_additional_fan',
 'min_temperature_when_ec',
 'optional_cooling_fan_speed',
 'optional_fan_full_speed_layer',
 'painted_wall_loops',
 'prime_tower_acceleration',
 'prime_tower_feedrate_factor',
 'prime_tower_ironing_speed',
 'prime_tower_line_ironing',
 'prime_tower_line_width',
 'prime_tower_ramming_line_width',
 'prime_tower_side_scrape',
 'prime_tower_side_scrape_along_length',
 'prime_tower_side_scrape_face',
 'prime_tower_side_scrape_height',
 'prime_tower_side_scrape_length',
 'prime_tower_side_scrape_point_spacing',
 'prime_tower_side_scrape_speed',
 'prime_tower_side_scrape_wall_gap',
 'prime_tower_tail_slowdown_volume',
 'prime_tower_tail_speed_factor',
 'prime_tower_wipe_distance',
 'prime_tower_wipe_speed',
 'print_compatible_printers',
 'printer_technology',
 'sync_auxiliary_fan_to_second_fan',
 'tree_support_tip_diameter',
 'upward_compatible_machine')

def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _config(value, context):
    _require(isinstance(value, Mapping), context + " must be a serialized configuration mapping")
    _require(all(isinstance(k, str) and isinstance(v, str) for k, v in value.items()),
             context + " must map string keys to serialized string values")
    return value


def _exact(config, values, context, optional=False):
    for key, value in values.items():
        if optional and key not in config:
            continue
        _require(config.get(key) == value,
                 f"{context}: {key} must equal {value!r}, got {config.get(key)!r}")


def _scripts(config):
    return {key: value for key, value in config.items() if key.endswith('_gcode')}


def _check_script_references(scripts, context):
    for key, value in scripts.items():
        for governed in GOVERNED_KEYS:
            _require(governed not in value,
                     f"{context}: {key} references excepted inactive/derived setting {governed}")


def validate_reviewed_guards(expected_config, actual_config=None, local_configs=None):
    """Validate fixed global controls and inherited local controls.

    local_configs may be one serialized map or a sequence of such maps. Omitted
    local keys inherit the already checked global value. Other local overrides
    (such as the reviewed density/angle modifier) remain the archive auditor's
    responsibility and are not accepted or altered by this function.
    """
    expected = _config(expected_config, "expected configuration")
    _exact(expected, INPUT_EXACT, "expected configuration")
    _exact(expected, NEW_EXACT_IF_PRESENT, "new input defaults", optional=True)
    _require(expected.get('printer_technology', 'FFF') == 'FFF', 'Input technology must be FFF')
    scripts = _scripts(expected)
    _require('machine_start_gcode' in scripts and 'machine_end_gcode' in scripts,
             'Expected snapshot must contain the complete machine-script settings')
    _check_script_references(scripts, 'expected scripts')
    if local_configs is None:
        locals_checked = []
    elif isinstance(local_configs, Mapping):
        locals_checked = [local_configs]
    else:
        _require(isinstance(local_configs, Sequence) and not isinstance(local_configs, (str, bytes)),
                 'local_configs must be one serialized map or a sequence of maps')
        locals_checked = local_configs
    for index, local in enumerate(locals_checked):
        local = _config(local, f'local configuration {index}')
        _exact(local, INPUT_EXACT, f'local configuration {index}', optional=True)
        _exact(local, NEW_EXACT_IF_PRESENT, f'local new defaults {index}', optional=True)
        local_scripts = _scripts(local)
        _check_script_references(local_scripts, f'local scripts {index}')
        for key, value in local_scripts.items():
            _require(key in scripts and value == scripts[key],
                     f'Local script {index}/{key} differs from the fixed global script')
    if actual_config is None:
        return None
    actual = _config(actual_config, 'actual configuration')
    _exact(actual, EMITTED_EXACT, 'actual effective controls')
    _exact(actual, NEW_EXACT_IF_PRESENT, 'actual new defaults', optional=True)
    _exact(actual, FORK_EXACT_IF_PRESENT, 'actual fork controls', optional=True)
    _require(actual.get('printer_technology', 'FFF') == 'FFF', 'Actual technology must be FFF')
    _require(_scripts(actual) == scripts, 'Actual custom scripts differ from intended snapshot')
    _check_script_references(_scripts(actual), 'actual scripts')
    return None
