"""Pure tests of the reviewed official/fork exception conditions.

These cases encode the reviewed benchmark contract independently of the guard
module's dictionaries. Deleting a guard must not delete its corresponding test.
The fixtures and captured serialized settings are repository data; no slicer,
build, local profile, or external research files are needed.
"""
import importlib.util
import json
from pathlib import Path
import unittest


REPO_ROOT = Path(__file__).resolve().parents[2]
TOOL_ROOT = REPO_ROOT / "scripts" / "bridge_aware_cubic_benchmark"
DATA_ROOT = REPO_ROOT / "tests" / "data" / "bridge_aware_cubic_benchmark"
PROJECTS = DATA_ROOT / "projects"


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


guards = load_module("reviewed_guards_under_test", TOOL_ROOT / "reviewed_exception_guards.py")
audit = load_module("guard_fixture_audit", TOOL_ROOT / "family_fixture_audit.py")

# Realistic alternate values that invalidate the exact reviewed conditions:
# active support/tower/fans, changed bridge controls, different hardware, or
# profile inheritance. Every case is checked globally and on a local override.
INPUT_ALTERNATIVES = {
    "additional_cooling_fan_speed": "30",
    "auxiliary_fan": "1",
    "bridge_acceleration": "500",
    "bridge_layer_density": "50%",
    "bridge_line_travel_extend": "20%",
    "enable_pre_heating": "1",
    "enable_prime_tower": "1",
    "enable_support": "1",
    "enable_wrapping_detection": "1",
    "enforce_support_layers": "2",
    "extra_wall_slope_angle": "30",
    "extruder_ams_count": "1",
    "extruder_max_nozzle_count": "2",
    "extruder_type": "Bowden",
    "filament_map": "2",
    "filament_map_2": "0",
    "filament_map_mode": "Manual",
    "filament_nozzle_map": "0",
    "filament_volume_map": "1",
    "has_filament_switcher": "1",
    "infill_wall_overlap": "20%",
    "inherits": "Unexpected parent",
    "inherits_group": "parent;;",
    "internal_bridge_layers": "2",
    "master_extruder_id": "2",
    "nozzle_diameter": "0.6",
    "nozzle_volume_type": "High Flow",
    "painted_wall_loops": "3",
    "print_extruder_id": "2",
    "print_extruder_variant": '"Direct Drive High Flow"',
    "print_sequence": "by object",
    "raft_layers": "2",
    "single_extruder_multi_material": "1",
}

# These controls must remain fixed after the CLI has resolved the project.
# The two singleton nozzle maps normalize from input 1 to emitted 0; accepting
# input 1 as an emitted map would hide a missing normalization or wrong mapping.
EMITTED_ALTERNATIVES = {
    "additional_cooling_fan_speed": "30",
    "auxiliary_fan": "1",
    "enable_pre_heating": "1",
    "enable_prime_tower": "1",
    "enable_support": "1",
    "enable_wrapping_detection": "1",
    "enforce_support_layers": "2",
    "extruder_ams_count": "1",
    "extruder_max_nozzle_count": "2",
    "extruder_type": "Bowden",
    "filament_map": "2",
    "filament_map_2": "1",
    "filament_map_mode": "Manual",
    "filament_nozzle_map": "1",
    "filament_volume_map": "1",
    "has_filament_switcher": "1",
    "infill_wall_overlap": "20%",
    "inherits": "Unexpected parent",
    "master_extruder_id": "2",
    "nozzle_diameter": "0.6",
    "nozzle_volume_type": "High Flow",
    "print_extruder_id": "2",
    "print_extruder_variant": '"Direct Drive High Flow"',
    "print_sequence": "by object",
    "raft_layers": "2",
    "single_extruder_multi_material": "1",
}

# Additions in the official release are waived only at these reviewed values
# and under the master controls above, never for an arbitrary new option value.
OFFICIAL_ADDITION_ALTERNATIVES = {
    "ams_nozzle_idx": "1",
    "aux_fan_independent_control": "1",
    "aux_fan_standard_side": "left",
    "close_optional_fan_first_x_layers": "2",
    "default_wipe_tower_pos": "0.5,0.5",
    "different_settings_to_system": "wall_loops;;",
    "extruder_nozzle_count": "2",
    "extruder_nozzle_diameter": "0.6",
    "extruder_nozzle_volume_type": "High Flow",
    "filament_diameter_map": "1.75",
    "filament_idle_travel_cooling_enabled": "0",
    "filament_pre_heating_enabled": "0",
    "filament_prime_tower_speed_factor_curve": '"0,50;10,100"',
    "fins_extrude_safe_temp": "180",
    "first_x_layer_optional_fan_speed": "30",
    "handle_hotend_as_extruder": "1",
    "hotend_cooling_rate_on_rack": "3",
    "min_temperature_when_ec": "150",
    "optional_cooling_fan_speed": "30",
    "optional_fan_full_speed_layer": "2",
    "prime_tower_acceleration": "500",
    "prime_tower_feedrate_factor": "0.5",
    "prime_tower_ironing_speed": "20",
    "prime_tower_line_ironing": "0",
    "prime_tower_line_width": "0.4",
    "prime_tower_ramming_line_width": "0.4",
    "prime_tower_side_scrape": "1",
    "prime_tower_side_scrape_along_length": "3",
    "prime_tower_side_scrape_face": "left_right",
    "prime_tower_side_scrape_height": "2",
    "prime_tower_side_scrape_length": "3",
    "prime_tower_side_scrape_point_spacing": "2",
    "prime_tower_side_scrape_speed": "1200",
    "prime_tower_side_scrape_wall_gap": "0.3",
    "prime_tower_tail_slowdown_volume": "20",
    "prime_tower_tail_speed_factor": "80%",
    "prime_tower_wipe_distance": "5",
    "prime_tower_wipe_speed": "30",
    "print_compatible_printers": "Other printer",
    "printer_technology": "SLA",
    "sync_auxiliary_fan_to_second_fan": "1",
    "tree_support_tip_diameter": "1.2",
    "upward_compatible_machine": "Other printer",
}

# These fork-only controls match the reviewed official hardcoded behavior.
FORK_BRIDGE_ALTERNATIVES = {
    "bridge_acceleration": "500",
    "bridge_layer_density": "50%",
    "bridge_line_travel_extend": "20%",
    "extra_wall_slope_angle": "30",
    "internal_bridge_layers": "2",
    "painted_wall_loops": "3",
}


class ReviewedExceptionGuardsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixtures = {}
        for family in ("small", "large", "rib", "modifier"):
            for pattern in ("cubic", "bridgeawarecubic"):
                name = f"{family}-{pattern}"
                cls.fixtures[name] = audit.audit_archive(
                    PROJECTS / f"{name}.3mf", fixtures_dir=PROJECTS
                )
        cls.expected = cls.fixtures["small-cubic"]["expected_serialized_config"]
        cls.bac_expected = cls.fixtures["small-bridgeawarecubic"]["expected_serialized_config"]
        cls.official = json.loads((DATA_ROOT / "official-emitted-config.json").read_text())
        cls.fork = json.loads((DATA_ROOT / "fork-emitted-config.json").read_text())

    def test_all_eight_native_projects_satisfy_guards(self):
        for name, fixture in self.fixtures.items():
            with self.subTest(project=name):
                self.assertIsNone(guards.validate_reviewed_guards(
                    fixture["expected_serialized_config"],
                    local_configs=fixture["local_configs"],
                ))

    def test_captured_official_output_satisfies_guards(self):
        self.assertIsNone(guards.validate_reviewed_guards(self.expected, self.official))

    def test_captured_fork_output_satisfies_guards(self):
        self.assertIsNone(guards.validate_reviewed_guards(self.bac_expected, self.fork))

    def test_unrelated_local_settings_remain_archive_auditor_responsibility(self):
        self.assertIsNone(guards.validate_reviewed_guards(
            self.expected,
            local_configs={"sparse_infill_density": "12%", "infill_direction": "17"},
        ))

    def test_changed_global_review_conditions_are_rejected(self):
        for key, alternative in INPUT_ALTERNATIVES.items():
            with self.subTest(setting=key):
                changed = dict(self.expected, **{key: alternative})
                with self.assertRaisesRegex(ValueError, key):
                    guards.validate_reviewed_guards(changed)

    def test_local_override_cannot_activate_excepted_behavior(self):
        for key, alternative in INPUT_ALTERNATIVES.items():
            with self.subTest(setting=key):
                # Empty preceding maps inherit the globals; a later volume
                # must not bypass checks on a guarded local setting.
                with self.assertRaisesRegex(ValueError, key):
                    guards.validate_reviewed_guards(
                        self.expected, local_configs=[{}, {key: alternative}]
                    )

    def test_changed_emitted_effective_controls_are_rejected(self):
        for key, alternative in EMITTED_ALTERNATIVES.items():
            with self.subTest(setting=key):
                changed = dict(self.official, **{key: alternative})
                with self.assertRaisesRegex(ValueError, key):
                    guards.validate_reviewed_guards(self.expected, changed)

    def test_official_additions_require_exact_reviewed_values(self):
        for key, alternative in OFFICIAL_ADDITION_ALTERNATIVES.items():
            with self.subTest(setting=key):
                changed = dict(self.official, **{key: alternative})
                with self.assertRaisesRegex(ValueError, key):
                    guards.validate_reviewed_guards(self.expected, changed)

    def test_fork_bridge_controls_require_reviewed_official_equivalence(self):
        for key, alternative in FORK_BRIDGE_ALTERNATIVES.items():
            with self.subTest(setting=key):
                changed = dict(self.fork, **{key: alternative})
                with self.assertRaisesRegex(ValueError, key):
                    guards.validate_reviewed_guards(self.bac_expected, changed)

    def test_custom_scripts_cannot_use_excepted_optional_fan_setting(self):
        for key in ("machine_start_gcode", "filament_start_gcode"):
            with self.subTest(script=key):
                changed = dict(self.expected, **{key: "{optional_cooling_fan_speed}"})
                with self.assertRaisesRegex(ValueError, "optional_cooling_fan_speed"):
                    guards.validate_reviewed_guards(changed)

    def test_emitted_custom_scripts_must_match_intended_scripts(self):
        for key in ("machine_start_gcode", "filament_start_gcode"):
            with self.subTest(script=key):
                changed = dict(self.official, **{key: "G1 X999"})
                with self.assertRaisesRegex(ValueError, "custom scripts differ"):
                    guards.validate_reviewed_guards(self.expected, changed)

    def test_local_custom_script_cannot_replace_global_script(self):
        with self.assertRaisesRegex(ValueError, "Local script"):
            guards.validate_reviewed_guards(
                self.expected, local_configs={"machine_start_gcode": "G1 X999"}
            )

    def test_malformed_local_container_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "local_configs"):
            guards.validate_reviewed_guards(self.expected, local_configs="not a map")


if __name__ == "__main__":
    unittest.main()
