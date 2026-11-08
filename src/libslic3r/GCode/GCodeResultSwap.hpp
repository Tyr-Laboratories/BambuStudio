#ifndef slic3r_GCodeResultSwap_hpp_
#define slic3r_GCodeResultSwap_hpp_
#include "GCodeProcessor.hpp"
#include <utility>

namespace Slic3r {
// Publish a completed worker result without copying its potentially large moves
// buffer or changing the address held by the preview. Call only after the worker
// has stopped. Keep this list aligned with GCodeProcessorResult's data members;
// its mutex belongs to the containing object and is deliberately not swapped.
inline void swap_gcode_results(GCodeProcessorResult &a, GCodeProcessorResult &b)
{
    using std::swap;
    swap(a.conflict_result, b.conflict_result);
    swap(a.gcode_check_result, b.gcode_check_result);
    swap(a.filament_printable_reuslt, b.filament_printable_reuslt);
    swap(a.nozzle_group_result, b.nozzle_group_result);
    swap(a.initial_layer_time, b.initial_layer_time);
    swap(a.filename, b.filename);
    swap(a.id, b.id);
    swap(a.moves, b.moves);
    swap(a.lines_ends, b.lines_ends);
    swap(a.printable_area, b.printable_area);
    swap(a.bed_exclude_area, b.bed_exclude_area);
    swap(a.wrapping_exclude_area, b.wrapping_exclude_area);
    swap(a.extruder_areas, b.extruder_areas);
    swap(a.extruder_heights, b.extruder_heights);
    swap(a.toolpath_outside, b.toolpath_outside);
    swap(a.label_object_enabled, b.label_object_enabled);
    swap(a.support_material_on_wipe_tower, b.support_material_on_wipe_tower);
    swap(a.long_retraction_when_cut, b.long_retraction_when_cut);
    swap(a.timelapse_warning_code, b.timelapse_warning_code);
    swap(a.support_traditional_timelapse, b.support_traditional_timelapse);
    swap(a.update_imgui_flag, b.update_imgui_flag);
    swap(a.is_helio_gcode, b.is_helio_gcode);
    swap(a.printable_height, b.printable_height);
    swap(a.settings_ids, b.settings_ids);
    swap(a.filaments_count, b.filaments_count);
    swap(a.extruder_colors, b.extruder_colors);
    swap(a.printer_extruder_variant, b.printer_extruder_variant);
    swap(a.extruder_types, b.extruder_types);
    swap(a.printer_extruder_id, b.printer_extruder_id);
    swap(a.filament_diameters, b.filament_diameters);
    swap(a.required_nozzle_HRC, b.required_nozzle_HRC);
    swap(a.filament_densities, b.filament_densities);
    swap(a.filament_costs, b.filament_costs);
    swap(a.filament_vitrification_temperature, b.filament_vitrification_temperature);
    swap(a.filament_maps, b.filament_maps);
    swap(a.limit_filament_maps, b.limit_filament_maps);
    swap(a.print_statistics, b.print_statistics);
    swap(a.custom_gcode_per_print_z, b.custom_gcode_per_print_z);
    swap(a.spiral_vase_layers, b.spiral_vase_layers);
    swap(a.warnings, b.warnings);
    swap(a.layer_filaments, b.layer_filaments);
    swap(a.nozzle_change_sequence, b.nozzle_change_sequence);
    swap(a.filament_change_sequence, b.filament_change_sequence);
    swap(a.optimal_assignment, b.optimal_assignment);
    swap(a.pause_printing, b.pause_printing);
    swap(a.filament_change_count_map, b.filament_change_count_map);
    swap(a.skippable_part_time, b.skippable_part_time);
    swap(a.bed_type, b.bed_type);
    swap(a.used_filaments, b.used_filaments);
    swap(a.used_mixed_filaments, b.used_mixed_filaments);
#if ENABLE_GCODE_VIEWER_STATISTICS
    swap(a.time, b.time);
#endif
}
} // namespace Slic3r
#endif
