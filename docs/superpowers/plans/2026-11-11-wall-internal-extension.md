# Wall Internal Extension Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Outer walls that rise out of flat or curved top surfaces, or end under bottom surfaces, continue through the shell and E further into the part. A 45° chamfer sits on the surface side. Both are printed as a new "Wall extension" line type.

**Architecture:** A new libslic3r module, `WallInternalExtension`, runs at the end of `PrintObject::make_perimeters`, after every layer's walls exist.
1. It finds qualifying joint lines among each layer's outer-wall extrusions.
2. It computes, for every layer within reach, the area covered by extension and chamfer lines, clipped to the part's fill area.
3. It fills that area with Arachne lines of role `erWallExtension` and cuts the area out of the layer's fill sets.

`discover_vertical_shells` treats the area as interior, so no solid infill is added around it. Lines with only sparse infill below them carry a flag that makes G-code export print them like bridges.

**Tech Stack:** C++17, libslic3r (Clipper, Arachne `WallToolPaths`), Catch2 tests in `tests/fff_print`, Ninja build tree `build/release` in this worktree.

**Spec:** `docs/superpowers/specs/2026-11-11-wall-internal-extension-design.md`

## Global Constraints

- **Setting:**
  - key `wall_internal_extension`, label "Wall internal extension", type `coFloat` in `PrintRegionConfig`;
  - category "Strength", sidetext "mm", min 0, max 10, mode `comAdvanced`, default `1.0`;
  - `0` turns the feature off.
- **Line type:** role `erWallExtension`, name "Wall extension". It sits in `enum ExtrusionRole` directly after `erFlush` and before `erMixed`. Preview colour `{ 0.20f, 0.85f, 0.85f, 1.0f }`.
- **Bridge flag:** `CustomizeFlag::cfWallExtensionBridge` marks extension lines printed like a bridge.
- **How the lines print:**
  - Line width: the outer-wall flow of the region printing them, `LayerRegion::flow(frExternalPerimeter)`.
  - Normal lines use inner-wall speed, acceleration and jerk.
  - Flagged lines use bridging flow, bridge speed, bridge acceleration, the overhang/bridge fan, and the bridge cooling exemption.
- **Off means unchanged:** when every region has `wall_internal_extension` = 0, or in spiral vase mode, G-code must be byte-identical to master `427e841ee`. The only allowed difference is the config-listing line for the new key.
- **Inside only:** lines are never printed outside the model, on layer 0, or where that spot is an exposed top or bottom surface. Outer surfaces never change.
- **No new failures or patches:** no `SlicingError`, no warnings that stop slicing, no fallbacks, no repair passes.
- **Tolerances** come only from:
  - line widths and spacings;
  - layer heights;
  - `resolution` (`PrintConfig`) and `slice_closing_radius` (`PrintObjectConfig`);
  - `EPSILON` for z comparisons.
- **No added solid infill:** vertical-shell logic must treat the extension area as interior.
- **Fill overlap:** fill meets extension lines at normal line spacing. Cut the area out of the fill sets with no `infill_wall_overlap` into it.
- **Regions:**
  - The joint's region supplies E, the shell depth and the line width used to find the joint.
  - On each target layer, lines go only where the region at that spot has E > 0.
  - That region prints them.
- **Build:** `lockf build/release.lock nice -n 10 ninja -C build/release <target>`. Run tests from the worktree root.

## Review Focus

1. **Multi-region objects.** A modifier or second part with E = 0 or a different wall filament must get no lines inside its volume, and must not cause tool changes. Test: Task 6, "A modifier with extension 0 is an off switch for its volume".
2. **Variable layer heights.** Depths are in mm, not layers; 0.1 mm layers under a tower still give 1.8 mm. Tests: Task 3, "Shell depth follows the shell settings in millimetres"; Task 6, "Depth is measured in millimetres with variable layer heights".
3. **A lower part thinner than shell + E, or ending on the bed or a bottom surface.** Lines stop inside the part and never print on layer 0 or on an exposed surface. Test: Task 6, "On a thin plate the extension stops above the bed".
4. **Joints whose extensions overlap,** such as two towers 1 mm apart. The union is printed once; no line is printed over another. Tests: Task 4, "Overlapping extensions form one area"; Task 6, "Two close towers print their shared chamfer once".
5. **Thin fins printed as one Arachne line.** Such a fin is still a joint and gets an extension. Test: Task 6, "A one-line fin is extended".

---

### Task 1: The setting

**Files:**
- Modify: `src/libslic3r/PrintConfig.hpp:1174` (after `extra_wall_slope_angle` in `PrintRegionConfig`)
- Modify: `src/libslic3r/PrintConfig.cpp:4854` (definition after `extra_wall_slope_angle`)
- Modify: `src/libslic3r/Preset.cpp:1037` (`s_Preset_print_options`, after `"extra_wall_slope_angle"`)
- Modify: `src/slic3r/GUI/Tab.cpp:3316` (Strength → Walls group, after `extra_wall_slope_angle`)
- Modify: `src/slic3r/GUI/GUI_Factories.cpp:60` and `:107` (after `extra_wall_slope_angle`)
- Modify: `src/slic3r/GUI/ConfigManipulation.cpp:987`
- Modify: `src/libslic3r/PrintObject.cpp:1613` (`invalidate_state_by_config_options`, posPerimeters list)
- Modify: `src/libslic3r/Layer.cpp:173` (`has_compatible_layer_regions`)
- Create: `tests/fff_print/test_wall_internal_extension_config.cpp`
- Modify: `tests/fff_print/CMakeLists.txt` (new target)

**Interfaces:**
- Produces: `PrintRegionConfig::wall_internal_extension` (`ConfigOptionFloat`, mm). Produces the test target `wall_internal_extension_tests`; later tasks add their test files to it.

- [ ] **Step 1: Add the test target and the failing tests.**

  CMake target, placed after `overhang_wall_tests`:
  ```cmake
  add_executable(wall_internal_extension_tests fff_print_tests.cpp test_wall_internal_extension_config.cpp)
  target_link_libraries(wall_internal_extension_tests test_common libslic3r slicer_test_support)
  set_property(TARGET wall_internal_extension_tests PROPERTY FOLDER "tests")
  add_test(wall_internal_extension_tests wall_internal_extension_tests ${CATCH_EXTRA_ARGS})
  ```
  Tests: copy the `Workspace` helper and the preset/3MF round-trip shape from `tests/fff_print/test_slope_wall_config.cpp`.
  ```cpp
  TEST_CASE("Wall internal extension defaults to 1 mm and persists in presets and projects", "[WallExtension][Config]")
  // config.apply(PrintRegionConfig defaults): has("wall_internal_extension"), opt_float == 1.0
  // count in Preset::print_options() == 1
  // preset saved with "2.5" reloads as 2.5
  // 3MF: process 2.5, object override 0.5, part override 0 -> reload gives 2.5 / 0.5 / 0
  TEST_CASE("Changing wall internal extension regenerates walls but keeps the slices", "[WallExtension][Config]")
  // slice a 20x20x5 cube (wall_internal_extension 1), apply the same model with "2":
  // get_object(0)->is_step_done(posSlice) == true, is_step_done(posPerimeters) == false
  ```
- [ ] **Step 2: Run them and see them fail.**
  Run: `lockf build/release.lock nice -n 10 ninja -C build/release wall_internal_extension_tests` (expect compile errors or `has(...)` false), then `build/release/tests/fff_print/wall_internal_extension_tests "[Config]"`.
- [ ] **Step 3: Implement the setting.**
  - Definition, following `extra_wall_slope_angle`:
    ```cpp
    def = this->add("wall_internal_extension", coFloat);
    def->label    = L("Wall internal extension");
    def->category = L("Strength");
    def->tooltip  = L("Start outer walls that rise out of a top surface, or end under a bottom surface, inside the part: "
                      "through the top or bottom shell and this far beyond it, with a 45° chamfer on the surface side, "
                      "so the wall is anchored like a tenon in a mortise. Applies where the wall stands entirely on the "
                      "surface and stays steeper than 45° for at least this height. 0 disables this feature. "
                      "Spiral vase mode is excluded.");
    def->sidetext = L("mm");
    def->min = 0; def->max = 10;
    def->mode = comAdvanced;
    def->set_default_value(new ConfigOptionFloat(1.));
    ```
  - Add the key to the option list (Preset.cpp) and the Walls group (Tab.cpp), and to both lists at `GUI_Factories.cpp:60` and `:107` (`{"wall_internal_extension", "",1}` in the second).
  - `ConfigManipulation.cpp`: `toggle_field("wall_internal_extension", !has_spiral_vase);`.
  - `PrintObject.cpp`: add `|| opt_key == "wall_internal_extension"` to the `wall_loops` branch. It invalidates posPerimeters only.
  - `Layer.cpp`: add `&& config.wall_internal_extension == other_config.wall_internal_extension`.
- [ ] **Step 4: Build and run.** Same commands. Expected: both test cases pass.
- [ ] **Step 5: Commit** (`git add` the files above): "Add the wall internal extension setting".

---

### Task 2: The "Wall extension" line type

**Files:**
- Modify: `src/libslic3r/ExtrusionEntity.hpp:45-76`: role `erWallExtension` after `erFlush`; `CustomizeFlag::cfWallExtensionBridge` after `cfFloatingVerticalShell`.
- Modify: `src/libslic3r/ExtrusionEntity.cpp:615-690`: `role_to_string` and `string_to_role`.
- Modify: `src/libslic3r/GCode.cpp`. Speed: `get_path_speed` at 6162 and `calc_speed_by_role` at 7246. Acceleration at 7188-7199. Jerk at 7217. Bridge description at 7132. Overhang fan start at 7462 and end at 7621. Cooling exemption at 7476.
- Modify: `src/libslic3r/GCode/ToolOrdering.cpp:154,169` (wall filament)
- Modify: `src/libslic3r/PrintObject.cpp:361-381,444` (estimated print time per filament)
- Modify: `src/slic3r/GUI/GCodeRenderer/BaseRenderer.cpp:141` (colour after `erFlush`)
- Create: `tests/fff_print/test_wall_extension_role.cpp` (add to the target)

**Interfaces:**
- Produces: `ExtrusionRole::erWallExtension`. `is_perimeter`, `is_infill` and `is_bridge` all return false for it.
- Produces: `CustomizeFlag::cfWallExtensionBridge`.
- Produces: in `GCode.cpp`, a file-static `bool prints_as_bridge(const ExtrusionPath &path)`, true for `is_bridge(path.role())` or for `erWallExtension` paths carrying `cfWallExtensionBridge`.

- [ ] **Step 1: Write the failing tests.**
  ```cpp
  TEST_CASE("Wall extension is its own line type", "[WallExtension][Role]")
  // role_to_string(erWallExtension) == "Wall extension"; string_to_role("Wall extension") == erWallExtension
  // !is_perimeter, !is_infill, !is_bridge; int(erWallExtension) == int(erFlush) + 1; int(erMixed) == int(erWallExtension) + 1
  TEST_CASE("Wall extension lines print at inner-wall settings, or like a bridge when flagged", "[WallExtension][Role][GCode]")
  // config: wall_internal_extension 0, classic walls, inner_wall_speed 77, outer_wall_speed 33, bridge_speed 23,
  //   inner_wall_acceleration 900, bridge_acceleration 450, default_acceleration 2000,
  //   filament_max_volumetric_speed 100, slow_down_for_layer_cooling 0, enable_arc_fitting 0
  // slice a 20x20x6 cube; on the layer with print_z 3.2, append to region 0's perimeters one ExtrusionEntityCollection holding:
  //   loop A: 8x8 mm square around the object centre, role erWallExtension, region flow(frExternalPerimeter)
  //   loop B: 4x4 mm square, role erWallExtension, region bridging_flow(frPerimeter, thick_bridges), flag cfWallExtensionBridge
  // export_gcode(file, &result), parse with GCodeReader (pattern of test_bridge_acceleration.cpp):
  //   F values (mm/min) of extruding moves under " FEATURE: Wall extension" == {4620, 1380}
  //   M204 values in force for those moves == {900, 450}
  //   result.print_statistics.modes[0].roles_times has an erWallExtension entry with time > 0
  //   result.print_statistics.used_filaments_per_role.at(erWallExtension).first > 0
  ```
- [ ] **Step 2: Run them and see them fail** (`ninja ... wall_internal_extension_tests`; then `... "[Role]"`): `erWallExtension` is undeclared.
- [ ] **Step 3: Implement.**
  - Enum, flag and name.
  - The colour row (aqua, as in Global Constraints).
  - GCode.cpp: `erWallExtension` takes `inner_wall_speed`. With `cfWallExtensionBridge` it takes the same bridge speed expression used for `erBridgeInfill` (filament or nozzle bridge speed). Do this in both speed functions; no overhang or circle-compensation adjustments.
  - Acceleration: `inner_wall_acceleration` (when > 0) for normal lines, `bridge_acceleration` (when > 0) for flagged lines. Jerk: `inner_wall_jerk`.
  - Replace `is_bridge(path.role())` with `prints_as_bridge(path)` at the description, the overhang fan start and end markers, and the cooling exemption.
  - ToolOrdering: treat `erWallExtension` as a perimeter (wall filament) in both branches.
  - PrintObject estimated time: `erWallExtension` uses wall filament and `inner_wall_speed`.
  - Do not add the role to `is_perimeter`. That would force a lift after every extension line (`GCode.cpp:7911`) and change retraction reduction (`:7948`).
  - Leave `GCodeProcessor::set_extrusion_role` mapping it to itself.
- [ ] **Step 4: Build and run.** Run `"[WallExtension]"`; expected: all pass. Also run `ninja ... BambuStudio` to compile the GUI renderer change.
- [ ] **Step 5: Commit:** "Add the Wall extension line type, printed at inner-wall or bridge settings".

---

### Task 3: Finding joints

**Files:**
- Create: `src/libslic3r/WallInternalExtension.hpp`, `src/libslic3r/WallInternalExtension.cpp`
- Modify: `src/libslic3r/CMakeLists.txt` (after `WallPaintingSlice.cpp`)
- Create: `tests/fff_print/test_wall_internal_extension_geometry.cpp` (add to the target)

**Interfaces:**
- Produces (namespace `Slic3r::WallInternalExtension`, all geometry scaled, z in mm):
  ```cpp
  struct LayerGeometry { double bottom_z = 0., print_z = 0.; ExPolygons outline; ExPolygons walls; };
  struct JointSettings {
      double extension = 0.; int top_shell_layers = 0; double top_shell_thickness = 0.;
      int bottom_shell_layers = 0; double bottom_shell_thickness = 0.;
      coord_t line_width = 0;   // outer wall line width at the joint layer
      coord_t tolerance  = 0;   // scaled max(resolution, slice_closing_radius)
  };
  enum class JointSide { Top, Bottom };
  struct Joint { size_t layer = 0; JointSide side = JointSide::Top; Polylines lines;
                 double shell_depth = 0., extension = 0.; coord_t line_width = 0; };
  double top_shell_depth(const std::vector<LayerGeometry> &layers, size_t surface_layer, int shell_layers, double shell_thickness);
  double bottom_shell_depth(const std::vector<LayerGeometry> &layers, size_t surface_layer, int shell_layers, double shell_thickness);
  std::vector<Joint> find_joints(const std::vector<LayerGeometry> &layers, size_t layer,
                                 const Polylines &outer_wall_lines, const JointSettings &settings);
  ```
  `walls` is the area covered by every wall extrusion of the layer. `outer_wall_lines` holds the centrelines of one region's outer-wall extrusions (roles `erExternalPerimeter` and `erOverhangPerimeter`).

- [ ] **Step 1: Write the failing tests** with a synthetic stack builder in the test file. All dimensions in mm, h = 0.2 unless stated, w = 0.45, E = 1, 4/4 shell layers, thickness 0, tolerance 0.012.
  - `stack(outlines, h, walls)` builds `LayerGeometry` from per-layer outlines. Its wall area is the band within `walls·w` of each outline.
  - `outer_lines(outline)` returns the closed centrelines of `offset_ex(outline, -w/2)`.

  ```cpp
  TEST_CASE("A tower rising out of a flat top surface is a top joint", "[WallExtension][Joints]")
  // box 20x20 layers 0-14, tower 10x10 centred layers 15-29:
  //   find_joints(15, tower lines): one Top joint, total length == Approx(4*(10-0.45)).margin(0.5), shell_depth == Approx(0.8), extension == 1
  //   find_joints(16, tower lines) empty; find_joints(14, box lines) empty
  TEST_CASE("A column under a wider slab is a bottom joint", "[WallExtension][Joints]")
  // column 10x10 layers 0-14, slab 20x20 layers 15-29: find_joints(14, column lines): one Bottom joint, length ~38.2, shell_depth 0.8; at 15 empty
  TEST_CASE("A pocket's walls rising from its floor are a top joint", "[WallExtension][Joints]")
  // box 20x20 layers 0-29 with a 6x6 hole in layers 15-29: find_joints(15): Top joint, length == Approx(4*(6+0.45)).margin(0.5)
  TEST_CASE("Only walls strictly steeper than 45 degrees and at least E tall qualify", "[WallExtension][Joints]")
  // GENERATE table {per-layer change of the tower's half-width above layer 15, tower layer count, joint expected}:
  //   {0, 15, true}, {-0.1155 (lean in 60°), 15, true}, {+0.1155 (lean out 60°), 15, true},
  //   {-0.2 (45°), 15, false}, {-0.55 (20° skin), 15, false}, {0, 2, false (0.4 mm lip)}, {0, 5, true (exactly 1.0 mm)}
  TEST_CASE("A wall line deep inside the outline is not an outer wall", "[WallExtension][Joints]")
  // tower case, but pass a closed polyline 3 mm inside the tower outline: no joint
  TEST_CASE("On a two-wall lower part a line standing on the inner wall is not a joint", "[WallExtension][Joints]")
  // box 20x20 with walls=2 (0.9 mm band); tower x 0.6-10.6, y 5-15 on top: joint exists, and every joint point
  // is farther than 0.9 + 0.225 - 0.012 from the box outline (the left side, over the inner wall, is dropped)
  TEST_CASE("Shell depth follows the shell settings in millimetres", "[WallExtension][Joints]")
  // 20 layers of 0.2: top_shell_depth(10, 4, 0) == 0.8; (10, 4, 1.0) == 1.0; (10, 0, 0) == 0
  // layers 0-9 of 0.2 then 10-19 of 0.1: top_shell_depth(15, 4, 0) == 0.4; (15, 4, 0.6) == 0.6
  // bottom_shell_depth mirror: on 0.2 layers (5, 4, 0) == 0.8; on the mixed stack (10, 4, 0) == 0.4
  ```
- [ ] **Step 2: Run them and see them fail** (`"[Joints]"`): the module does not exist.
- [ ] **Step 3: Implement the shell depths and `find_joints`.**
  - **Shell depths** mirror `discover_vertical_shells`:
    - Top: a layer i ≤ t is in the shell under a top surface on t while `t - i < shell_layers` or `print_z[t] - print_z[i] < shell_thickness - EPSILON`. Return `print_z[t] - bottom_z[deepest]`, or 0 when no layer qualifies.
    - Bottom (mirror): a layer i ≥ b is in the shell while `i - b < shell_layers` or `bottom_z[i] - bottom_z[b] < shell_thickness - EPSILON`. Return `print_z[highest] - bottom_z[b]`.
  - **`find_joints`.** Return `{}` when `extension <= 0`. Let w = `line_width`, ε = `tolerance`.
    - First, keep only the pieces of the lines inside the outer band of `layers[layer]`: `diff_ex(outline, offset_ex(outline, -w))`.
    - **Top** (needs `layer ≥ 1`):
      - Keep pieces inside the support region, `diff_ex(shrink_ex(L[N-1].outline, w/2), offset(L[N-1].walls, w/2))`.
      - Then, for j = 1, 2, …, keep pieces within the band `diff_ex(offset_ex(L[N+j].outline, r), offset_ex(L[N+j].outline, -r))`, where `r = w/2 + scaled(L[N+j].print_z - L[N].print_z) - ε`.
      - Continue while `L[N+j-1].print_z - L[N].bottom_z < extension - EPSILON`.
      - Running out of layers, or emptying the pieces, means no joint.
    - **Bottom** (needs `layer + 1 < size`): the mirror image.
      - The support region is from `L[N+1]`.
      - The bands come from `L[N-j]`, with `r = w/2 + scaled(L[N].print_z - L[N-j].print_z) - ε`.
      - Continue while `L[N].print_z - L[N-j+1].bottom_z < extension - EPSILON`.
    - Drop pieces shorter than w.
    - Shell depth: `top_shell_depth(layers, N-1, …)` for Top, `bottom_shell_depth(layers, N+1, …)` for Bottom.
    - Emit at most one `Joint` per side.
- [ ] **Step 4: Build and run** `"[Joints]"`. Expected: all pass.
- [ ] **Step 5: Commit:** "Find the outer-wall joints that the wall internal extension anchors".

---

### Task 4: The extension area on each layer

**Files:**
- Modify: `src/libslic3r/WallInternalExtension.hpp/.cpp`
- Modify: `tests/fff_print/test_wall_internal_extension_geometry.cpp`

**Interfaces:**
- Consumes: `LayerGeometry`, `Joint` (Task 3).
- Produces:
  ```cpp
  struct ExtensionArea {
      ExPolygons all;      // covered by extension lines and chamfers at the target layer, before clipping to the part
      ExPolygons hanging;  // the part of `all` that is a top joint's deepest layer: nothing of the extension is below it
  };
  ExtensionArea extension_area(const std::vector<LayerGeometry> &layers, const std::vector<Joint> &joints, size_t target_layer);
  ```

- [ ] **Step 1: Write the failing tests.** Same stack builder; box 20×20 layers 0-29, tower 10×10 layers 30-44, joints from `find_joints(30, …)`.
  ```cpp
  TEST_CASE("Under a tower the extension goes through the shell and 1 mm below with a widening chamfer", "[WallExtension][Area]")
  // A(L).all non-empty exactly on layers 21-29; empty on 20 and 30
  // A(L).hanging non-empty only on layer 21 (the deepest), where it equals A(21).all
  // inside the tower outline: area(intersection(A(L).all, tower)) ~ 38.2 * 0.225 on all nine layers (margin 10%)
  // shell layers 26-29: total width area(A)/38.2 == Approx(0.45).margin(0.05)
  // chamfer layers 21-25: (area(A(L)) - area(A(L-1))) / 40 == Approx(0.2).margin(0.03) for L = 22..25;
  //   area(diff(A(25), offset(tower, 1.0 + 0.225 + 0.05))) == 0 (no wider than E)
  TEST_CASE("Over a column the extension rises through the bottom shell and 1 mm above", "[WallExtension][Area]")
  // column 10x10 layers 0-29, slab 20x20 layers 30-59: joint at 29 (Bottom); .all non-empty exactly on 30-38;
  // .hanging empty on every layer; chamfer narrows going up: area(A(L).all) > area(A(L+1).all) for L = 34..37
  TEST_CASE("Overlapping extensions form one area", "[WallExtension][Area]")
  // two 8x8 towers 1.0 mm apart on a 30x30 box; on the layer just under the shell (widest chamfer):
  //   sum of areas of the returned ExPolygons == Approx(area(union_ex(result)))
  //   area(result) < area(A_tower1) + area(A_tower2) (the shared gap is counted once)
  ```
- [ ] **Step 2: Run them and see them fail** (`"[Area]"`).
- [ ] **Step 3: Implement `extension_area`.** For each joint, with `D = shell_depth + extension` and w = `line_width`:
  - **Top joint** (target L < N): `depth = L[N].bottom_z - L[L].bottom_z`. Skip if `depth > D + EPSILON`. On chamfer layers (`depth > shell_depth + EPSILON`), `c = clamp(L[L].print_z - (L[N].bottom_z - D), 0, extension)`.
  - **Bottom joint** (target L > N): `depth = L[L].print_z - L[N].print_z`, with the same skip. `c = clamp(L[N].print_z + D - L[L].bottom_z, 0, extension)`.
  - Piece = `offset(lines, w/2)`, plus, when c > 0, `diff(offset(lines, w/2 + scaled(c)), to_polygons(L[N].outline))`. The chamfer is only outside the joint layer's material, on the surface side.
  - A top joint's piece also goes into `hanging` when L is its deepest layer, i.e. L == 0 or layer L-1 is deeper than `D + EPSILON`.
  - Return `union_ex` of each set.
- [ ] **Step 4: Build and run** `"[WallExtension]"`. Expected: all pass.
- [ ] **Step 5: Commit:** "Compute the wall extension and chamfer area on each layer".

---

### Task 5: Printing the extension

**Files:**
- Modify: `src/libslic3r/PerimeterGenerator.hpp/.cpp`: add `make_wall_tool_paths_params` and use it in `process_arachne` (replace the block at 1737-1753; same values).
- Modify: `src/libslic3r/WallInternalExtension.hpp/.cpp`: add `outer_wall_lines`, `extension_extrusions` and `apply`.
- Modify: `src/libslic3r/Layer.hpp:57-80`: add `LayerRegion::wall_extension_area`.
- Modify: `src/libslic3r/PrintObject.cpp`:
  - `make_perimeters`: call `WallInternalExtension::apply(*this)` immediately before `this->set_done(posPerimeters)` at 639;
  - `discover_vertical_shells` holes caches at 2373 and 2423.
- Modify: `src/libslic3r/Print.cpp:4201-4205,4464,4763`: slicing cache key `"wall_extension_area"`.
- Create: `tests/fff_print/test_wall_internal_extension_print.cpp` (add to the target)

**Interfaces:**
- Consumes: Task 3 and Task 4 functions; `erWallExtension` and `cfWallExtensionBridge` (Task 2).
- Produces:
  ```cpp
  // PerimeterGenerator.hpp (namespace Slic3r)
  Arachne::WallToolPathsParams make_wall_tool_paths_params(const PrintObjectConfig &object_config, const PrintConfig &print_config);
  // WallInternalExtension.hpp
  Polylines outer_wall_lines(const ExtrusionEntityCollection &perimeters);  // centrelines of erExternalPerimeter / erOverhangPerimeter paths
  // Lines filling `area`; a line with more than half its length inside `bridge_area` is printed like a bridge.
  ExtrusionEntityCollection extension_extrusions(const ExPolygon &area, const ExPolygons &bridge_area, const Flow &flow,
                                                 const Flow &bridge_flow, double layer_height, size_t inset_count,
                                                 const Arachne::WallToolPathsParams &params);
  void apply(PrintObject &object);
  // Layer.hpp, LayerRegion
  ExPolygons wall_extension_area;   // area of this region's wall extension lines on this layer; empty when none
  ```

- [ ] **Step 1: Write the failing end-to-end tests.**
  - Fixture config `extension_config(double e)`: region defaults plus `layer_height 0.2`, `initial_layer_print_height 0.2`, `wall_loops 1`, `top_shell_layers 4`, `top_shell_thickness 0`, `bottom_shell_layers 4`, `bottom_shell_thickness 0`, `sparse_infill_density 10%`, `sparse_infill_pattern grid`, `wall_generator arachne`, `brim_type no_brim`, `enable_arc_fitting 0`, `wall_internal_extension e`.
  - Models: `terrace()` is a 30×30×12 box plus a 10×10×6 tower at (10,10,12). `overhang()` is a 10×10×6 column at (10,10,0) plus a 30×30×4 slab at z 6. Both use `set_offset(80,80,0)`.
  - Use the BAC test's `collect_paths` helper.

  ```cpp
  TEST_CASE("A tower's outer wall continues 1.8 mm into the box", "[WallExtension][Print]")
  // E=1: layers holding erWallExtension paths have print_z exactly {10.4, 10.6, ..., 12.0} (9 layers)
  // all their points lie inside the box outline and within 1.0 + 0.45 + 0.05 mm of the tower outline
  // every path on 10.4 has cfWallExtensionBridge; no path on 10.6-12.0 has it
  // export_gcode: result.print_statistics.used_filaments_per_role.at(erWallExtension).first > 0
  TEST_CASE("A column's outer wall continues 1.8 mm into the slab", "[WallExtension][Print]")
  // overhang(), E=1: layers with erWallExtension print_z exactly {6.2, ..., 7.8}; no path has cfWallExtensionBridge
  TEST_CASE("Fill never covers wall extension lines", "[WallExtension][Print]")
  // terrace and overhang, E=1, every layer and region: area(intersection(fill_no_overlap_expolygons, wall_extension_area)) < 0.01 mm^2
  //   and fill_expolygons intersected with shrink(wall_extension_area, 0.01 mm) is empty
  TEST_CASE("The wall extension adds no solid infill", "[WallExtension][Print]")
  // terrace, E=0 vs E=1, every layer: area of fill_surfaces typed stInternalSolid, stInternalBridge, stTop, stBottom, stBottomBridge
  //   at E=1 <= at E=0 + 0.05 mm^2
  TEST_CASE("Extension 0 or spiral vase adds nothing", "[WallExtension][Print]")
  // terrace E=0: no erWallExtension path, every wall_extension_area empty
  // terrace E=1 with spiral_mode 1, sparse 0%, top_shell_layers 0: no erWallExtension path
  ```
- [ ] **Step 2: Run them and see them fail** (`"[Print]"`): `apply` does not exist yet, so no lines are printed.
- [ ] **Step 3: Implement.**
  - **`outer_wall_lines`:** walks the collection recursively (paths, loops, multipaths, nested collections).
  - **`extension_extrusions`:**
    - Build `Arachne::WallToolPaths(to_polygons(area), flow.scaled_spacing(), flow.scaled_spacing(), inset_count, 0, layer_height, params)`.
    - Take each `ExtrusionLine` from `getToolPaths()`. If more than half of its centreline length lies inside `bridge_area`, the line prints like a bridge:
      - Rebuild it as one constant-width path from its junction points, with `bridge_flow`'s mm3_per_mm, width and height and the flag `cfWallExtensionBridge`.
      - Make it an `ExtrusionLoop` (`elrDefault`) if the line is closed, otherwise an `ExtrusionPath`.
    - Otherwise use `extrusion_paths_append(paths, line, erWallExtension, flow)`. Make closed lines an `ExtrusionLoop` and open ones an `ExtrusionMultiPath`, as in `PerimeterGenerator.cpp:856-907`.
  - **`apply`:**
    1. Clear `wall_extension_area` in every region of every layer. Return if `print()->config().spiral_mode`, or if no printing region has `wall_internal_extension > 0`.
    2. In parallel over layers, build `LayerGeometry`:
       - `bottom_z()`, `print_z`, `lslices`;
       - `walls` = `union_ex` of every region's `perimeters.polygons_covered_by_width()`.
    3. In parallel over layers N, for each region with E > 0, call `find_joints` with:
       - `outer_wall_lines(region->perimeters)`;
       - the region's E and top/bottom shell settings;
       - `region->flow(frExternalPerimeter).scaled_width()`;
       - `tolerance = scaled(max(print config resolution, object config slice_closing_radius))`.
    4. In parallel over target layers 1 ≤ L ≤ n-2, compute `A = extension_area(…)` from the joints reaching L. For each region at L with E > 0, `W = intersection(A.all, region->fill_no_overlap_expolygons, L-1 lslices, L+1 lslices)`. Store W and `A.hanging` per (layer, region).
    5. In parallel over L, for each region with non-empty W:
       - Compute `support_below` = layer L-1 `walls` ∪ the stored W of every region at L-1.
       - `bridge_area = diff_ex(intersection_ex(A.hanging, W), offset(support_below, flow.scaled_spacing()/2))`. Only a top joint's deepest layer can hang. Bottom-joint chamfers sit on the bottom shell, whose solid infill is not known yet at this step.
       - `inset_count = ceil((w + scaled(E_max)) / spacing) + 1`, where `E_max` is the largest E over printing regions.
       - Append one collection per ExPolygon of W to `region->perimeters`, using `region->flow(frExternalPerimeter)` and `region->bridging_flow(frPerimeter, object thick_bridges)` (the same call as `LayerRegion.cpp:194`) and `make_wall_tool_paths_params(object config, print config)`.
       - Set `fill_expolygons`, `fill_no_overlap_expolygons` and `bridge_fill_expolygons` each to `diff_ex(·, W)`, and `fill_surfaces.set(fill_expolygons, stInternal)`.
       - Set `wall_extension_area = W`.
       - Call `throw_if_canceled()` per layer.
  - **Vertical shells:** next to both `polygons_append(cache.holes, to_polygons(…fill_expolygons))`, append `to_polygons(…wall_extension_area)`.
  - **Slicing cache:** save `wall_extension_area` like `bridge_fill_expolygons`. On load, use `j.contains(key) ? … : ExPolygons{}`.
- [ ] **Step 4: Build and run** `"[WallExtension]"`, then all of `wall_internal_extension_tests`, `overhang_wall_tests`, `bridge_aware_cubic_tests` and `wall_painting_tests`. Expected: the new tests pass. For the other suites, record any failure for Task 6; do not change their assertions here.
- [ ] **Step 5: Commit:** "Print outer-wall extensions and chamfers inside the part".

---

### Task 6: Hardening, existing suites, documentation

**Files:**
- Modify: `tests/fff_print/test_wall_internal_extension_print.cpp`
- Modify: fixtures of existing suites only if Step 3 requires it.
- Modify: `docs/tyr-fork.md` (new section after "Extra wall on sloped surfaces")

- [ ] **Step 1: Write the tests for the Review Focus inputs.** Use the Task 5 fixture and helpers.
  ```cpp
  TEST_CASE("A one-line fin is extended", "[WallExtension][Print]")
  // 30x30x6 box + 0.6 x 20 x 5 fin on top: erWallExtension on exactly the 9 layers below the fin
  TEST_CASE("A rib standing on a sloped skin is extended along its base", "[WallExtension][Print]")
  // a 20-degree ramp (its_make_prism or an explicit wedge mesh, 30 mm long) with a vertical 1.2 x 30 x 15 rib through it:
  //   erWallExtension paths on at least 10 distinct layers, every point within 1.0 + 0.45 + 0.05 mm of the rib's footprint,
  //   and no erWallExtension on layers where the rib does not meet the ramp
  TEST_CASE("Only the outer wall position is extended on two-wall parts", "[WallExtension][Print]")
  // terrace, wall_loops 2: on the 4 shell layers every erWallExtension point is within 0.45 + 0.05 mm of the tower outline
  TEST_CASE("The classic wall generator gets the same extension", "[WallExtension][Print]")
  // terrace, wall_generator classic: same 9 layers as with Arachne
  TEST_CASE("On a thin plate the extension stops above the bed", "[WallExtension][Print]")
  // 30x30x1.0 plate + tower: erWallExtension only on print_z {0.4, 0.6, 0.8, 1.0}; none on layer 0
  TEST_CASE("Depth is measured in millimetres with variable layer heights", "[WallExtension][Print]")
  // terrace with object layer_config_ranges {9.0, 12.0} -> layer_height 0.1:
  //   sum of heights of layers holding erWallExtension == Approx(1.8).margin(0.1)
  TEST_CASE("Two close towers print their shared chamfer once", "[WallExtension][Print]")
  // two 8x8 towers 1.0 mm apart on a 30x30x12 box; on every layer: sum over erWallExtension paths of
  //   area(polygons_covered_by_spacing) <= 1.05 * area(union of them)
  TEST_CASE("A modifier with extension 0 is an off switch for its volume", "[WallExtension][Print]")
  // terrace + PARAMETER_MODIFIER box covering x < centre (whole height) with wall_internal_extension 0:
  //   no erWallExtension point with x < centre - 0.5 mm; some with x > centre + 0.5 mm
  TEST_CASE("Continuous outer-wall speed mode still exports", "[WallExtension][Print]")
  // terrace, z_direction_outwall_speed_continuous 1: export_gcode succeeds and the result has erWallExtension time > 0
  ```
- [ ] **Step 2: Run them.** Expected: they pass. If one fails, fix the construction in `WallInternalExtension.cpp` under the Global Constraints. No special cases, no repair passes.
- [ ] **Step 3: Run the whole suite.** Run: `ctest --test-dir build/release -j 4 --output-on-failure`.
  - Expected: everything passes except the pre-existing `fff_print_tests` assertion at `test_printgcode.cpp:190`.
  - If another suite fails because its fixture's geometry now gets the 1 mm default, add `{"wall_internal_extension", "0"}` to that fixture's config with a one-line comment. Never weaken an assertion. List these in the commit message.
- [ ] **Step 4: Document** in `docs/tyr-fork.md`, in the style of the slope-wall section. Cover:
  - where the setting is;
  - what qualifies (decision 6 of the spec) and depth B;
  - the chamfer;
  - the "Wall extension" line type and the bridge-printed lowest layer;
  - the region rule;
  - that existing projects get the 1 mm default when re-sliced;
  - that older apps do not show the new line type.
- [ ] **Step 5: Commit:** "Cover wall extension edge cases and document the feature".

---

### Task 7: Acceptance (run by the controller, not an implementer)

**Files:** none in the repository. Scripts and outputs go in the session scratchpad.

- [ ] **Step 1: Build the app.** `lockf build/release.lock nice -n 10 ninja -C build/release` (all targets), then `otool -L build/release/src/BambuStudio.app/Contents/MacOS/BambuStudio | grep X11` prints nothing.
- [ ] **Step 2: Prepare two copies of the Heimdall project.** Take the local copy under `<main checkout>/build/bac-review-data/projects/`.
  - The `E1` copy is unchanged; projects without the key get the default 1 mm.
  - The `E0` copy has `"wall_internal_extension": "0"` added to `Metadata/project_settings.config` inside the 3MF.
- [ ] **Step 3: Parity.** Slice all 16 plates of `E0` with `build/master-baseline/BambuStudio.app` (master `427e841ee`) and with the feature app. Command: `--slice N --outputdir <dir> <project>`; never run the binary without `--slice`. Run at most two plates at a time and tell the user the expected duration first. Each `plate_N.gcode` must be identical after removing the `; wall_internal_extension = ` line and the header's date and version lines.
- [ ] **Step 4: Report at the default 1 mm** (`E1`, feature app, all 16 plates) against master on the same project. Per plate:
  - grams per feature, including "Wall extension";
  - estimated print time;
  - path and retraction counts;
  - instructions retired (`/usr/bin/time -l`, minus a `--slice 99` startup run).

  These are reported, not gated.
- [ ] **Step 5: Look before handing over.**
  - Render plate 1's toolpaths, retractions shown, on the layers around one top joint and one bottom joint.
  - Describe in plain words what is printed: for example "a closed loop under the rib's wall on nine layers, the lowest printed as a bridge, widening into a 1 mm chamfer under the shell".
  - Any surprise means fixing the construction, not adding a patch.
- [ ] **Step 6: Hand over for review.**
  - Export `plate_1.gcode.3mf` with `--min-save --export-3mf`, the plain `plate_1.gcode`, and a coupon. The coupon has three objects (tower on box, column under slab, pocket), each sliced at E = 0 and E = 1 with the user's H2S presets from `build/bac-review-data/user/default`.
  - Put them where the user can open them.
  - Give the launch command for this branch's app with `--datadir <main checkout>/build/bac-review-data`, which the new line type needs.
