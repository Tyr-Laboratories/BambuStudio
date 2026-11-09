# Bridge aware cubic: rewrite architecture

Status: reviewed with the user on 8 November 2026; every open question answered (9.3); awaiting the user's acceptance before the old branches are deleted. When accepted it supersedes the designs of 5 and 6 November 2026 and the Batch M designs, and the branch `codex/bridge-aware-cubic-constructive` is deleted. Base: the fork's `master` as of the evening of 8 November 2026: `origin/master` 7c81b32cd ("Close Tyr.23 delivery and allow failed benchmark retries") with the legacy fixes merged on top (dbf8c36be). Line numbers cite 28aed7650 ("Add dedicated bridge controls and publishable Tyr runtime", 5 November 2026), the commit on which the integration points were mapped and verified; the 31 commits between it and the base touch `Fill.cpp` (+137 lines: a flow mask for solid fill above internal bridges, option `flow_above_internal_bridges`), `PrintObject.cpp` (+48: support-base painting), `Print.cpp` (+16: plates sliced concurrently with isolated state), `FillFloatingConcentric.cpp` and `Lightning/TreeNode.cpp`, and none changes a mechanism this document relies on. The implementer maps each citation forward with a search for the quoted code. No code of the old branch is reused. The formulas and rules it validated are stated here in full, so that an implementer needs no access to it.

Written for the engineers and agents who build the pattern from `master`, without the old branch's history. Line numbers refer to 28aed7650, as explained above.

## 0. How to read this document

- Section 1 is the contract: what the pattern promises, what it does not, and the budgets. Everything else serves it.
- Section 2 places the pattern in the slicer and lists every line of shared code it touches.
- Section 3 is the construction, in the order it runs. Each stage says what it reads, what it makes, the exact rule, and why the contract holds.
- Section 4 collects every constant. Section 5 is the development checks. Section 6 the code units and their interfaces. Section 7 the tests and the acceptance. Section 8 the build order. Section 9 the decisions carried over, the decisions this document makes, and the open questions. Section 10 reviews the branch this replaces.

Vocabulary. A *bead* is one extruded line of width w. A *strand* is a straight piece the pattern lays along a lattice plane. A *course* is a bead the pattern prints whole along a curve (a seam wall, a rail, a solid column's line). A *plane* is a demand plane: a layer under a roof, at whose height openings are measured. *Held* means supported from the layer below by the rules of 3.7. *G* is the effective gap of a region (3.1), *G_b = G − 0.1 mm* the planning gap.

## 1. The contract

### 1.1 What the pattern is

A sparse infill pattern, `Bridge aware cubic` (enum `ipBridgeAwareCubic`, key `bridgeawarecubic`), with one setting of its own, `bridge_aware_cubic_max_gap` ("Unsupported gap size", mm, default 5). It is Cubic's lattice, exactly, everywhere. Under non-sparse material that lies over the pattern's void beyond the perimeter generator's overhang distance from what surely prints below, it adds lines so that nothing above spans more than the gap. It adds lines; it removes none of Cubic's.

Why it exists: the user's single-wall PA612-CF aircraft parts at 3% Cubic print well on every surface but the first roof layer, whose internal bridges span cells of 37 mm. The pattern is judged on those parts (16 plates, 69 objects): roof support, added grams, print time, and that nothing else about the slice changes. Their settings, read from the project: 3% (10% on 27 objects), 0.5 mm sparse lines at 0.2 mm layers, `infill_wall_overlap` 75%, Arachne walls, `thick_bridges` on, `detect_narrow_internal_solid_infill` on, `detect_floating_vertical_shell` on, `infill_combination` off, `fill_multiline` 1, `internal_bridge_layers` at its default of 1, gap 5 mm.

### 1.2 The guarantees

**K1, roofs.** For every layer n and every piece of non-sparse material of n that lies over the pattern's void of layer n−1 beyond the overhang rule's support (3.4), on the plane at layer n−1 and on the planes down to layer n−1−B (B the region's Internal bridge layers):

- (a) every opening of the void under the piece (the void within r/2 of the piece's footprint, r the piece's narrowest bead) has a span of at most G;
- (b) every unsupported run of a bead of the piece over the void is at most G, and every bead of internal bridge infill over the pattern's void ends anchored: on printed material of the layer below, or on a wall, into which the bridge's wall overlap (`bridge_line_travel_extend`, as the infill anchor does for sparse lines) fuses it (user, 8 November 2026). No hanging bridge end over the pattern's void is accepted.

Span and run are defined in 3.6 and 5.1. The verifier compares to G exactly; the plan builds to G_b.

**K2, the pattern's own lines.** Every line the pattern adds (needed strand, extension, column piece, course) is held from the layer below at every point (3.7), or is a bridge of at most G_b held at both ends. No end of a pattern line hangs over the void. Cubic's bulk lines are Cubic's and are not checked, as Cubic does not check them.

**Settings.** Choosing the pattern changes no other setting of the part. Nothing is refused, overridden or silently ignored because the pattern is chosen: not Infill combination (the surfaces combine as for Cubic; inside the single-layer zones near roofs the pattern prints at layer height, as walls do, 3.9a), Fill multiline, Internal bridge layers, top layers, 0% or 100% density, nor Conformal infill (which Cubic ignores too). Bridges, internal solid infill, floating vertical shells and gap fill may differ from Cubic's only by the stock pipeline's ordinary response to the pattern's lines. The pattern adds no special case to shared slicer logic beyond the hooks of 2.3.

**Release behaviour.** No check of the pattern runs in a release build. Nothing stops, truncates, falls back or repairs. Where a rule finds no admissible placement, the plan prints as planned and a development build lists the place. When a limit is reached (cell count, coordinate range, gap bounds) the pattern generates anyway. An isolated span a little over the gap is a development finding, never a slicing error.

### 1.3 Out of scope

- Sparse infill over sparse infill where two lattices meet horizontally (the floor or ceiling of a modifier with another density, direction or line width): as Cubic.
- Follow-up internal bridge layers resting on a bridge layer, and bridge layers printed below 100% Bridge layer density: as Cubic.
- Minimal material. Every split has a failing demanded section and every added line holds something up; the plan does not search for the lightest construction.
- Bridge orientation. Directions come from the stock detector or from Bridge direction, as for any pattern.
- Physical qualification.

### 1.4 Budgets

| Quantity | Limit | Today (old branch, 8 November 2026) |
| --- | --- | --- |
| Slicing cost | ≤ 2.0 × official Bambu Studio 2.9.1.70 regular Cubic, instructions retired, median and worst of five alternating pairs after a warm-up pair, on each of the four fixtures (small cube, large cube, rib, modifier) | ≈ 1.7 × on the large cube, loaded machine |
| Print time, 16 Heimdall plates | ≤ +17.8 % over Cubic (≤ 367.5 h against 312.0 h) | 343.4 h (+10.1 %) |
| Mass, 16 plates | reported per role per plate; direction down | 4,305.6 g against 4,255.3 g (+1.2 %) |
| Hanging internal bridge ends over the pattern's void, 16 plates (an end that overlaps a wall is anchored) | 0; any exception is a construction defect, listed by place | 106 (Cubic 93) |
| Longest hanging bridge stretch per plate | ≤ Cubic's on that plate | worse on plates 12 and 14 |
| Unheld pattern pieces, plates 1–5, by the K2 checker (5.3) | the target is 0 (K2); the gate at stage 5a is ≤ 86, the old branch's count before family changes, every piece listed with its cause (user's answer, 8 November 2026) | 153 (86 before family changes) |
| Peak RSS | reported | 4–6 GB per plate |
| Source lines in the pattern's units | ≤ 4,000 | 6,637 |
| Test lines | ≤ 2,500, ≤ 40 cases (7.2 lists 21) | 15,831, 261 cases |
| Changed lines in shared files | ≤ 150 | ≈ 400 |

Measurement rules: instructions retired, never wall time (the laptop runs in low power mode under background load); grams, the slicer's time estimate, path and retraction counts and RSS do not depend on the clock. The 16-plate run is scripted and run by the controller, not inside implementer tasks (7.4).

## 2. Where the pattern sits

### 2.1 The pipeline

`PrintObject::prepare_infill` (src/libslic3r/PrintObject.cpp:856) runs, in order: `detect_surfaces_type` (:880), `prepare_fill_surfaces` (:913), `discover_vertical_shells` (:920), `process_external_surfaces` (:933), `discover_horizontal_shells` (:954), `reset_slice_surfaces` (:969), `clip_fill_surfaces` (:976, a no-op in this fork), the Tyr snapshot of the sparse surfaces into `original_sparse` (:994-1008), `bridge_over_infill` (:1009), `add_internal_bridge_layers` (:1010), `combine_infill` (:1014), `discover_sub_top_surfaces` (:1017). The pattern's plan is built once, after `clip_fill_surfaces` and the Tyr snapshot and immediately before `bridge_over_infill`, by a new private step `prepare_bridge_aware_cubic()`. Everything after it, including the bridge generator, the Tyr internal bridge layers and `combine_infill`, runs stock, unchanged, and reads the pattern's lines through the stock previews. The fills are made later, in `Layer::make_fills` (src/libslic3r/Fill/Fill.cpp:666, called per layer in parallel from `PrintObject::infill` at src/libslic3r/PrintObject.cpp:1197-1205), one layer at a time, in parallel.

Why before the bridges (user directive, 8 November 2026: "BAC should preform all of it's generation logic before internal bridges are created"): the stock generator anchors each internal bridge on a preview of the sparse infill of the layer below (`Layer::generate_sparse_infill_polylines_for_anchoring`, src/libslic3r/Fill/Fill.cpp:859-969, called at src/libslic3r/PrintObject.cpp:3139-3156 for the bridges, at :3780-3810 for the thick bases, and at src/libslic3r/Fill/Fill.cpp:749 for floating concentric shells). When that preview shows the pattern's refined lines, the bridges are anchored at the closest of them, which makes them shorter and lighter, and nothing in the generator is touched. A plan made after the bridges would anchor them on Cubic's coarse lines and then print different lines under them.

What the bridge generator does to the surfaces after the plan (src/libslic3r/PrintObject.cpp:2922-3521: candidates :2950-3034, anchor lines :3127-3162, expansion and anchoring :3220-3438, the surface rewrite :3442-3517): it turns the candidate parts of internal solid surfaces of layer n into internal bridges, grows each bridge over the sparse fill of n along the bridging direction until it reaches anchors, and lays a solid "ensuring" strip, one solid spacing wide, on the layer below the bridge where the bridge's rim overlaps the rim along that layer's walls (:3456-3502: `additional_ensuring`, cut from `stInternal` and typed `stInternalSolid`). The Tyr step `add_internal_bridge_layers` (src/libslic3r/PrintObject.cpp:3523-3883; a no-op at `internal_bridge_layers` = 1; otherwise it projects each bridge into the sparse surfaces of the one or two layers below (:3557-3606), lays its own ensuring rim under the base (:3696-3726) and re-anchors the base on the preview of the layer below it (:3780-3810)) then projects a bridge down `internal_bridge_layers − 1` layers as thick bases. The plan does not predict these changes (decision R2 in 9.2); it tolerates them: it evaluates every roof on B + 1 planes, B the Internal bridge layers (3.4), it prints its planned lines wherever nothing else prints (3.8), and it lays rails where bridges will grow (3.5).

### 2.2 What the plan reads

All of it at plan time, before the bridges, from `Layer` and `LayerRegion` (`LayerRegion` at src/libslic3r/Layer.hpp:33-138, `Layer` at :140-220):

- `Layer::lslices`: the layer's outline.
- `LayerRegion::fill_surfaces`: the surfaces with their `surface_type` (`stInternal`, `stInternalSolid`, `stTop`, `stBottom`, `stBottomBridge`; `stInternalBridge` and `stInternalVoid` do not exist yet at plan time, since the generator and `combine_infill` make them later, and are read only by the verifier and the filler) and, at fill time, `thickness_layers`.
- `LayerRegion::fill_no_overlap_expolygons`: the fill area without the infill/wall overlap strip, bounded by the inner edge of the innermost perimeter (computed by the perimeter generator at src/libslic3r/PerimeterGenerator.cpp:1525-1576 for classic walls and :1619-1636 for Arachne, stored at src/libslic3r/Layer.cpp:259-262 and :301; `fill_expolygons` reaches `infill_wall_overlap` into the innermost wall, `fill_no_overlap_expolygons` stops at its inner edge).
- `Layer::print_z`, `Layer::height`, `Layer::id()`.
- The region configs: `sparse_infill_pattern`, `sparse_infill_density`, `infill_direction`, `fill_multiline`, `infill_wall_overlap`, `infill_combination`, `internal_bridge_layers`, `bridge_aware_cubic_max_gap`, `sparse_infill_anchor`, `sparse_infill_anchor_max`, `bridge_flow`, `wall_loops`; the flows `frInfill`, `frSolidInfill`, `frTopSolidInfill`, `frPerimeter`, `frExternalPerimeter`, `frBridge` (roles at src/libslic3r/Flow.hpp:16-25; `LayerRegion::flow` and `bridging_flow` at src/libslic3r/Layer.hpp:86-88 and src/libslic3r/LayerRegion.cpp:32-48); the extruder's nozzle diameter.
- The stock bridge candidate test, extracted into a function (2.3, item 7).

It reads no generated path. The perimeters exist at this point (`make_perimeters` ran), but the plan takes their footprint from `lslices` and `fill_no_overlap_expolygons`, not from the extrusions.

### 2.3 Shared-code hooks (the complete list)

Everything else lives in the pattern's own units (section 6). Each hook is a stock mechanism used the way other patterns use it.

| # | File | Change | Lines | Precedent |
| --- | --- | --- | --- | --- |
| 1 | `src/libslic3r/PrintConfig.hpp` | enum value `ipBridgeAwareCubic` before `ipCount`; option `bridge_aware_cubic_max_gap` in the region config | 3 | every pattern |
| 2 | `src/libslic3r/PrintConfig.cpp` | enum key and label; option definition (label, category Strength, tooltip stating the nozzle-diameter floor, sidetext mm, `min` 0.01, `max` 1000, default 5) (pattern: `infill_direction` at src/libslic3r/PrintConfig.cpp:3194-3202; the region config macro at src/libslic3r/PrintConfig.hpp:896 and :1091-1100) | ≈ 15 | every option |
| 3 | `src/libslic3r/Preset.cpp` | the key in `s_Preset_print_options` | 1 | every option |
| 4 | `src/slic3r/GUI/Tab.cpp`, `GUI_Factories.cpp`, `ConfigManipulation.cpp` | the option's line in the Strength page after `sparse_infill_pattern`, in the part and modifier setting lists, and shown only when the pattern is chosen at 0 < density < 100 (src/slic3r/GUI/Tab.cpp:3336-3339, src/slic3r/GUI/GUI_Factories.cpp:61 and :104-112, src/slic3r/GUI/ConfigManipulation.cpp:937-948) | ≈ 8 | `conformal_*` options |
| 5 | `src/libslic3r/Fill/FillBase.cpp` | `case ipBridgeAwareCubic: return new FillBridgeAwareCubic();` (src/libslic3r/Fill/FillBase.cpp:33-71, `case ipCubic` at :48; the static instantiation at :82-96 requires a case for every enum value) | 1 | every pattern |
| 6 | `src/libslic3r/Print.hpp`, `PrintObject.cpp` | member `std::unique_ptr<BridgeAwareCubic::ObjectPlan> m_bridge_aware_cubic_plan` with accessor `bridge_aware_cubic_plan()` (a shared object returns its source's); private `prepare_bridge_aware_cubic()` called once in `prepare_infill` before `bridge_over_infill`; `bridge_aware_cubic_max_gap` invalidates `posPrepareInfill` (the `posPrepareInfill` group at src/libslic3r/PrintObject.cpp:1693-1711, beside `internal_bridge_layers` at :1708; an unlisted key invalidates every step, :1841-1845); the key in `s_modifier_ignored_infill_keys` (src/libslic3r/PrintObject.cpp:3955-3996); the plan reset where the Lightning generator is reset after a cache load (src/libslic3r/Print.cpp:5402-5409) | ≈ 32 | `m_lightning_generator`, `m_adaptive_fill_octrees` |
| 7 | `src/libslic3r/PrintObject.cpp` | the candidate test of `bridge_over_infill` (src/libslic3r/PrintObject.cpp:2950-3034: `unsupported_area` of the layer below, the 1·s / 4·s / 3·s filters at :2982-2989, the per-surface test with the 9·s² threshold at :2991-3009) extracted into a function `internal_bridge_candidates(const Layer&) -> std::vector<CandidateSurface>` returning the generator's own records (surface, layer index, `worth_bridging` polygons, region), with no change of behaviour, called by the generator where the code was and by the plan (3.5) | ≈ 10 (moved, not added) | none; a pure refactor |
| 8 | `src/libslic3r/Fill/Fill.cpp` | (a) `group_fills` declared in a header so that the plan sees the islands the filler will get (3.8; the boundary function itself is the pattern's own); (b) in `make_fills` and in `generate_sparse_infill_polylines_for_anchoring`, when the filler is the pattern's, give it the object's plan and the layer (`filler.bind(*plan, layer)`), as Lightning gets its generator and Adaptive its octree (src/libslic3r/Fill/Fill.cpp:695-758 in `make_fills`, :902-911 in the preview); (c) `case ipBridgeAwareCubic:` in the preview's pattern switch (src/libslic3r/Fill/Fill.cpp:873-899); (d) one call at the end of `make_fills`, after every filler: `BridgeAwareCubic::print_uncovered(*this)` (3.8) | ≈ 40 | Lightning, Adaptive |
| 9 | `src/libslic3r/CMakeLists.txt`, `tests/fff_print/CMakeLists.txt`, top-level `CMakeLists.txt` | the units; the option `SLIC3R_BAC_DEVELOPMENT_CHECKS` (off by default and in release recipes) | ≈ 25 | |
| 10 | `src/libslic3r/Fill/Fill.cpp:296`, `src/slic3r/GUI/ConfigManipulation.cpp:944`, `src/libslic3r/Layer.cpp:606-634` | the pattern joins the lists Cubic is on: `support_multiline_infill` (so `fill_multiline` is honoured and shown) and the `ipCubic` case of `get_sparse_infill_max_void_area` (the bulk lattice is Cubic's) | 3 | Cubic |

Note on the enum: `SurfaceFillParams` sorts fill groups by bridge angle, extruder and then pattern (src/libslic3r/Fill/Fill.cpp:90-134), and each group is clipped by the earlier ones with a safety offset (:429). A new value at the end of the enum puts the pattern's sparse groups after every solid group, so their islands can differ from Cubic's by a safety offset where sparse meets solid. The contract does not care; the parity tests allow 0.001 mm.

Not touched: `combine_infill`, `Print::validate`, `PrintConfig.cpp`'s `validate()`, `bridge_over_infill`'s logic, `connect_infill`, the G-code writer and processor, `LayerRegion`. Concurrency: this master slices independent plates concurrently (pull request 11); the plan is per `PrintObject`, built inside that object's own `prepare_infill` step and read-only afterwards, so the pattern needs no shared state. If a later finding seems to need a change in any of these, the finding is reported with the smallest hook proposed, and the user decides.

## 3. The construction

### 3.0 Order

Per object, once, in `prepare_bridge_aware_cubic()`:

1. Regions and lattice groups (3.1); the lattice of each group (3.2).
2. Printed material and void of every layer (3.3).
3. Seam walls (3.5a); they join the material of their layers.
4. Roof footprints and planes (3.4).
5. Rails (3.5b); they join the material of their layers and widen the footprints.
6. Refinement of each group: open cells, needed strands, solid columns (3.6).
7. The support sweep of each group (3.7).
8. Storage, by layer: the strands and courses to print. The statistics of 5.2 are gathered only under the development flag; a release build keeps what emission needs and nothing else.

At fill time, per layer, the filler and the uncovered-lines pass (3.8). Settings the construction honours are in 3.9. Steps 1 to 8 read surfaces and the plan's own data; no step reads a generated path, and no step runs again after the bridges.

### 3.1 Regions, groups, gap

A region prints the pattern when `sparse_infill_pattern == ipBridgeAwareCubic` and 0 < `sparse_infill_density` < 100. At 100% the region is treated exactly as Cubic is: the GUI offers to switch patterns (src/slic3r/GUI/ConfigManipulation.cpp:711-745), configuration validation refuses the pattern at 100% with the stock message (src/libslic3r/PrintConfig.cpp:9816-9820; the pattern joins that list, hook 2), and a region that reaches the filler at 100% anyway, as a modifier loaded from a 3MF can, is filled as `FillCubic` fills it (3.8 step 1). At 0% it prints no sparse infill and is void to its neighbours. Spiral vase sets the density to 0.

Lattice group key: (pitch L_N of 3.2, `infill_direction`, sparse flow spacing s_f, `fill_multiline` N). Regions with equal keys share one lattice and one plan. Their gaps may differ: every piece of demand carries the gap of the region whose sparse fill the void lies beside (3.4). Different density, direction, line width or multiline makes a different group, and two groups meet only through a seam wall (3.5a).

Effective gap: G = clamp(option value, nozzle diameter of the extruder that prints the region's sparse infill, option maximum). Floor reason (user, 8 November 2026): a gap smaller than the nozzle would need solid fill, which a sparse pattern cannot print. The clamp is silent at plan time; the tooltip states the floor; no validation refuses a value. G_b = G − 0.1 mm is the planning gap; the 0.1 mm covers path simplification (`resolution`), arc fitting and coordinate rounding, which move vertices by far less.

B_region = `internal_bridge_layers` of the region (integer 1 to 3, default 1, src/libslic3r/PrintConfig.cpp:2085-2096; the user's parts use 1). For a group, B = the largest B_region of the regions whose void a footprint lies over. N is reserved for the multiline count (3.2).

### 3.2 The lattice, in closed form (Cubic's exactly)

The pattern's bulk lines are Cubic's. The implementation reproduces `FillCubic::fill_surface` (src/libslic3r/Fill/FillRectilinear.cpp:3163-3173), `fill_surface_by_multilines` (src/libslic3r/Fill/FillRectilinear.cpp:3041-3081) and `make_fill_lines` (src/libslic3r/Fill/FillRectilinear.cpp:3002-3039) in closed form, and a test pins the two against each other point for point (7.2, L1). The preview does not set `FillParams::pattern` (src/libslic3r/Fill/Fill.cpp:933-954), so the filler never reads it.

- Spacing s_f: the sparse infill flow's spacing, `Flow::spacing()` (src/libslic3r/Flow.hpp:67 and src/libslic3r/Flow.cpp:192-200, spacing = width − height · (1 − π/4); `group_fills` takes the sparse spacing at the object's `layer_height`, independent of the layer, at src/libslic3r/Fill/Fill.cpp:338), not the line width. At 0.40 mm lines on 0.2 mm layers it is 0.357 mm; at Heimdall's 0.5 mm lines, 0.457 mm.
- Pitch of one family: L_N = `coord_t(scale_(s_f) * N / (d / 3))`, with d the density as a fraction and N the multiline count. At 5%, N = 1, s_f = 0.357: L = 21.4 mm. At Heimdall's 3%: 45.7 mm. If L_N would exceed half the `coord_t` range it is clamped to that: the lattice then has at most one line of each family across the part, and refinement under roofs works as at any density.
- Families k = 0, 1, 2 with directions θ_k = `infill_direction` + π/2 + k·π/3 and signs σ = (+1, −1, +1), as FillCubic sets them (the three calls at src/libslic3r/Fill/FillRectilinear.cpp:3163-3173: angle offsets of 0, π/3, 2π/3 and the z shifts `± scale_(sqrt(0.5) * z)`); n_k = (cos θ_k, sin θ_k). The bulk lines use FillCubic's float-truncated shift `coord_t(scale_(float(σ_k · √0.5 · z)))` for parity; the plan's geometry uses the exact double σ_k · z / √2; the difference is below one scaled unit. Family k's lines on a layer at height z are { p : n_k · p + σ_k · z / √2 = m · L_N } in the object-centred scaled XY that `Fill::bounding_box` makes (src/libslic3r/Fill/Fill.cpp:678 and :689 give every filler the object's bounding box, which is centred on the origin, src/libslic3r/Print.hpp:444; `_infill_direction` uses its centre as the reference point, src/libslic3r/Fill/FillBase.cpp:216-218, and adds π/2 at :243). A line of family k therefore shifts sideways by s = h / √2 between two layers h apart (0.141 mm at 0.2 mm layers).
- The three families are mutually orthogonal planes of a cubic lattice with cell edge a_0 = L_N · √(2/3) (17.5 mm at the 5% example; 37.3 mm at Heimdall's 3%), body diagonal vertical, one vertex at the XY origin at z = 0. Cube coordinates u_k = √(2/3) · (n_k · p) + σ_k · z / √3. The cell (i_0, i_1, i_2) at depth δ is u_k ∈ [i_k · a_δ, (i_k + 1) · a_δ], a_δ = a_0 / 2^δ. Its height range is the range of z = Σ_k σ_k u_k / √3 over its corners.
- The section of a cell at height z: the convex hull of the (at most six) points where the plane cuts the twelve cube edges; a triangle or a hexagon. diam(P) = the largest vertex distance.
- Plane identity: (family k, depth δ, numerator q) for the plane u_k = q · a_δ; a refinement plane has an odd numerator at depth δ + 1 and never aliases a bulk plane. Its trace on the layer at height z is the line { p : n_k · p = q · a_δ · √(3/2) − σ_k · z / √2 }, parametrised by t along d_k = (−sin θ_k, cos θ_k). The trace of a plane of family j ≠ k crosses the trace of plane K at a point whose position t along K is linear in z (both lines drift): every pair of families gives the same closing speed, about 1.22 h per layer of height h, and its sign says whether the crossing approaches. The implementer derives it from the two line equations; test L2 pins it against brute-force intersection.
- Multiline (N > 1): family line m becomes N sub-lines at the shifts of `fill_surface_by_multilines`: sub-line i lies (i − (N − 1) / 2) · s_f from the line. A strand on a plane is likewise a band of N sub-lines at offsets o_i = (i − (N − 1) / 2) · s_f (3.9).

### 3.3 Printed material and void: one definition

Everything that judges support uses this one definition, at plan time, per layer index n:

- walls_n = the beads of the perimeters and of the gap fill between them at n, by nominal width (`polygons_covered_by_width` of `LayerRegion::perimeters` and `thin_fills`), inside lslices_n; the perimeters are final when the plan is built. With no walls (`wall_loops` 0) it is empty. (Amended 9 November 2026: the first version took lslices_n − ∪ fill_no_overlap_expolygons_n, which is wider by the gap of about 0.2 mm that the perimeter generator leaves between the last wall and the infill and that prints nothing; the K2 checker, which reads the printed beads, found strand ends that the plan held by that band 0.2 to 0.5 mm from any bead.)
- The non-sparse surfaces A of region r at n: `stInternalSolid`, `stTop`, `stBottom`, `stBottomBridge`, and `stInternal` of a region at 100% (at plan time; the verifier, which reads the final surfaces, adds `stInternalBridge`).
- printing(A) = opening(A, 0.55 · s_role, round joins, arc tolerance 0.001 mm), s_role the spacing of the flow that prints A (solid, top, bridge): the part of A that surely prints. Reason: the rectilinear fillers inset the surface by half a spacing before laying lines (src/libslic3r/Fill/FillBase.cpp:98-112; the solid fills cut their lines in `fill_surface_by_lines`, src/libslic3r/Fill/FillRectilinear.cpp:2847-2851), so a part narrower than one spacing gets none, and between 1 and 2 spacings a line lands only if the grid's phase allows; 1.1 is the old branch's validated threshold. With `detect_narrow_internal_solid_infill` on (the default) the stock pipeline hands islands narrower than about 6 mm to the concentric fillers, which print strips down to a quarter of the nozzle (src/libslic3r/Fill/Fill.cpp:519-603, FillConcentricInternal, FillFloatingConcentric); such a strip *may* print, so it is a roof below but never support.
- courses_n = the beads of the pattern's seam walls and rails at n, by their width w (3.5), added as they are planned; they surely print (the filler prints them whole, and the pass of 3.8 prints what a later surface took).
- M⁺_n = walls_n ∪ ∪_A A ∪ courses_n: what *may* print on layer n. It is the roof side of the overhang rule: a bead that may be laid there needs support.
- M⁻_n = walls_n ∪ ∪_A printing(A) ∪ courses_n: what *surely* prints on layer n. It is the support side: only a bead that is surely there can hold anything.
- V_n = lslices_n − M⁻_n, the void: sparse fill of any pattern, 0% regions, and strips that may print but are not counted on.
- S^G_n = the components of V_n that meet the `stInternal` surfaces of group G's regions at n: the void that G's lattice bounds. Seam walls separate V_n at every open boundary (3.5a), so a component never spans two groups.

One function with two sides, both conservative: a doubtful strip is a roof above and void below. Nothing is dilated, and no surface type is special beyond "may print" and "surely prints". The old branch used three notions of material at once (surface types; the fillers' output opened by the spacing; extrusion footprints), and the K1 and K2 surprises it recorded (openings beside unprinted strips, seam corridors, a 9.6 mm bridge end) all involved material that printed differently from how a notion counted it.

### 3.4 Roofs: footprints and planes

For every layer n ≥ 1 and the group G:

- h_n = half the nozzle diameter of the extruder that prints layer n's walls: the perimeter generator's overhang distance. The generator grows the whole lower outline by it, sparse fill included, so it never calls anything over sparse infill unsupported; the plan borrows the distance and applies it to what surely prints below (classic walls: src/libslic3r/PerimeterGenerator.cpp:2046-2068 and :479-492, the lower layer's outline grown by half the nozzle diameter; Arachne: :1661-1667, the same offset).
- D_n = (M⁺_n − offset_round(M⁻_{n−1}, h_n)) ∩ S^G_{n−1}: the material that may print on n and lies over G's void of n−1 beyond the support that surely prints there. Vertical walls, slopes that step in by at most h_n per layer and 45° chamfers create none; roofs, ledges, steps, solid or 100% modifiers over the pattern, and the lowest layer of a bridge stack do. Pieces of D_n over the void of no group (another pattern's sparse fill, a 0% region, air) are dropped.
- The gap of a piece of D_n is the gap of the region whose `stInternal` surface at n−1 lies nearest; pieces are split by the regions' surfaces grown by s_f where regions of one group differ in gap.
- Planes: D_n is evaluated on the layers m = n−1−j for j = 0 .. B, B as in 3.1, clamped to m ≥ 0: plane (D_n, z_m = print_z_m, G, S^G_m). The deeper planes cover the Tyr thick bases, which move the support plane of a roof down by up to B − 1 layers after the plan, and the one-spacing solid strip that the stock generator lays one layer below a bridge. Each plane is also the layer whose printed set holds the roof: openings are measured there, needed strands are laid there.
- Rails widen D_n to D⁺_n (3.5b).

On the four fixtures only the roof (and, on the modifier, the seam wall's first course) creates demand; test P1 counts the planes.

### 3.5 Courses: seam walls and rails

Courses are beads the pattern prints whole. They are material of their layer, on both sides of 3.3, from the moment they are planned: they bound openings at their own plane and, through 3.4, a course that lies more than h_n from the material below raises demand on the plane beneath it, so the lattice is refined under it as under a roof and the refinement's strands hold it. This is the mechanism that holds the length of every course. A course's *ends* are inputs to the support sweep (3.7): an end that is not within w/2 of material below gets a holder, a piece of the plane through the end point, so that no course ends free over the void (K2).

**(a) Seam walls.** Where the sparse fill of a region of G borders sparse fill of another lattice (another group or another pattern), a 0% region, or the air outside a part without walls, no lattice line belongs to both sides, and the two voids would join into one opening that no refinement can bound. For region r at layer n:

- open_r(n) = the parts of the boundary of r's `stInternal` surfaces that lie within δ_s = 0.01 mm of such a neighbour. A boundary is open by what lies next to it, never by what is missing: material of any shape (the acute tip of a solid notch, a hairline between two parts of one fill) opens nothing. A layer whose sparse fill is all of one group, each region with walls and none at 0%, has no seam and is not looked at.
- The wall is one bead of r's sparse infill inside r: the boundary of offset(S_r, −i) within i + δ_s of open_r(n), with i = w/2 − 0.05 mm beside a neighbour that deposits beads (their butt ends stop at the seam; the 0.05 mm overlap closes the channels exact touching would leave) and i = w/2 beside 0% or air. At each end of an open stretch the wall follows the inset contour round the corner for one bead width, so that it still meets the material after path simplification.
- One wall per seam. Between two of the pattern's groups the region with the lower index lays it; against another pattern, a 0% region or air, the pattern's region always lays it. Walls exist on every layer where the open boundary exists.
- A course over another pattern's void, beyond h_n of any material below, cannot be bounded by refinement: it prints as planned, and a development build warns (5.2).

**(b) Rails.** The stock generator grows a bridge past the roof's edge, over the sparse fill of the roof's layer, to the nearest anchor beyond each section's end; on the old branch that growth averaged 6.8 mm past the area to be bridged and was 43% of the bridge area, and the bridge lines hung where they turned and ended. A rail is a bead the pattern lays on the layer whose preview anchors the lowest bridge base, just outside the roof's inner edge, so that the generator finds its closest anchor there (user, 8 November 2026: rails "follow the roof edge"; like seam walls they are courses outside the lattice directions, which the user allowed).

- On which layer: the one whose preview anchors the lowest base of the bridge stack, m_r = n−1−(B−1): layer n−1 at B = 1, layer n−B otherwise. At B > 1 `add_internal_bridge_layers` rebuilds the base from the required roof at layer n−B+1, re-anchors it on the preview of layer n−B inside a domain of up to 10 mm (src/libslic3r/PrintObject.cpp:3676, :3780-3810), and trims the bridge at n to it; the growth that matters is the base's, and a rail at n−1 would bound nothing. One rail per roof component, on plane j = B−1 of the roof.
- Where on that layer: for each candidate polygon B that the stock candidate test returns for layer n (2.3 item 7; the test reads the same pre-bridge surfaces the plan reads, so it is exact up to the generator's later expansion), E = the parts of the boundary of M⁺_n within s_b of B that lie over S^G_{m_r} and border G's sparse fill at n within δ_s. s_b = the bridge flow's spacing. E is the roof's inner edge: not walls, which border perimeters and are anchors already, and not other material.
- What: the rail R = boundary of offset_round(M⁺_n, d_r) kept within d_r + 0.01 mm of E, clipped to G's fill boundary at m_r (3.8), stretches shorter than a bead (w − 1e−4, section 4) dropped; d_r = s_b + w/2 + 0.05 mm (about 0.71 mm at the fixtures' settings: the thick bridge flow of a 0.4 mm nozzle at Bridge flow 1 has w_b = 0.40 and s_b = w_b + 0.05 = 0.45, and w = 0.42). The rail then lies w/2 + 0.05 beyond the edge of the area the generator bridges, and inside the generator's expansion area wherever the sparse fill reaches d_r + s_b past E; where the fill is narrower the clip drops it, the section runs on to the fill boundary at the wall, and the bridge's wall overlap anchors it there (user: "this is an anchor"). The base's own ensuring rim (one spacing wide, at m_r) may take the rail's sparse area; the pass of 3.8 prints the rail where that rim prints nothing.
- Footprint: D⁺_n = D_n ∪ ((offset_round(M⁺_n, d_r) − M⁺_n) ∩ offset(E, d_r + 0.01) ∩ S^G_m) at every plane m of the roof. The band is the rail's own construction, not the bridge's growth (which stays no demand, 9.1): it is partitioned like the rest of the roof so that the strands of plane j = B−1 reach the rail, those of plane j = B run under it.
- The rail is material of layer m_r (3.3, both sides), so the plane at m_r − 1 holds it through 3.4, and it bounds openings at plane m_r.
- No rail along walls, along an edge over another group's void or over air, or for a surface the candidate test does not return (a roof the generator will fill with solid infill, which the needed strands bound).
- Seam rails (a one-group seam inside a candidate's footprint, where both regions' bridges turn): deferred; built only if the fixture acceptance (7.2, R1) shows a bridge end hanging at such a seam. Heimdall's objects are single-region.

Why K1 holds with rails: the band between E and the rail is demand, so its openings are within G_b; a section of the lowest base ends w_b past the rail, within w_b/2 + 0.01 of the rail's bead; the U-turn between two sections runs along the edge at that offset, over the band; the growth beyond E is at most d_r + w_b (about 1.1 mm at the fixtures' settings) instead of a cell. The bridge layers above the base rest on the base (1.3). Why K2 holds: the rail is a course (above). Test R1 covers 1, 2 and 3 internal bridge layers.

### 3.6 Refinement: open cells, needed strands, solid columns

Per group, over all its planes (3.4), ordered by (layer, roof). Let w be the sparse bead width, W_N = (N − 1) s_f + w the width of a band (w at N = 1), μ = 0.1 mm the planning margin, and r the roof's narrowest bead that will cross the void: the first internal bridge layer's strand, nozzle diameter × √(`bridge_flow`), where the roof is a bridge candidate, otherwise the narrowest wall or solid bead of the roof's layer.

Roots: the depth-0 cells whose cube meets the bounding box of some plane's footprint at that plane's height (solve the third index from the height). None if √2 · a_0 − W_N ≤ G_b for the smallest gap of the group.

```
visit(cell, candidates):                       // candidates: the planes that failed in the parent; every plane at a root
  count the visit
  failing = []
  for each plane (D, z, G, S) in candidates:
     if z is not strictly inside the cell's height range: continue
     P = section(cell, z)
     if P lies inside the fill boundary at z's layer and diam(P) - W_N <= G - μ: continue   // every face prints: the bound bounds
     if P does not meet D: continue              // bbox, then an edge-grid test, Clipper only for boundary cells
     Q = opening( (P shrunk by W_N/2 along its printed faces) ∩ S,  r/2 )   // the void a roof bead can occupy
     F = Q ∩ offset(D, r/2)                                                 // under the roof: K1(a)'s openings
     if F is empty: continue
     for each component C of F:
        span(C) = max( diam(hull(C)),  L(C) )
        if span(C) > G - μ: failing += plane; break
  if failing is empty: leaf
  else if d · 2^(depth + 1) >= 1: solid column (c), window as below
  else: split; record failing; for each of the 8 children: visit(child, failing)
```

- A *printed face* of P is an edge of P that lies inside the group's fill boundary at that layer (3.8) along at least w; an edge outside it, or whose interval inside it is shorter than w, is a gap and is not shrunk. A bulk face prints on every layer (it is a Cubic line); a refinement face prints where a needed strand lies (b).
- The quick pass holds only when every face of P prints, which is why it requires P inside the boundary (one box classification against the boundary's index). A cell with a face outside the boundary is measured in full, and the old branch let such cells pass on the quick bound. An edge outside the boundary borders no void (S ends there). An edge whose interval inside the boundary is shorter than w prints nothing; where that gap is wider than r the voids of the two cells on either side join, and the measure of each includes the other's free area along that edge. This needs the fill boundary to cut a face to between r and w, which is rare.
- The openings are measured under the roof only, F = Q ∩ offset(D, r/2), because a roof bead runs unsupported only inside D: outside D the material of n is within h_n of what surely prints below (3.4). The verifier measures the same way (5.1). A cell cut by the footprint's edge is therefore not failed for the void beyond that edge, and the strands it gets end at D⁺.
- The opening by r/2 is the overhang rule on the roof's own bead: a channel narrower than r joins no two openings and a void narrower than r is none. L(X) = per(X)/2 − 4·area(X)/per(X) is a strip's length (between l − t and l for a straight strip l × t; about the path length of a bent strip; never above a convex set's diameter): a path that follows a curved strip of floating vertical shell or top surface runs its length, not its chord. Both are computed on polygons Clipper already returned.
- Children are visited with the failing planes only. The quick bound is monotone under containment (a child's section lies in its parent's); the strip length is not, and need not be: new planes print only at planes that failed in the cell, so a plane that passes in a cell asks nothing of its children.
- Roots are descended in parallel; the result is joined in root order, so it equals a serial descent. One atomic counter counts visits; the cancellation hook is called every 1024 visits.
- No budget. The descent is finite because cells saturate (c). A development build warns when a group's count passes 1,000,000.
- Cost on the large cube roof at 5%: about 20 / 81 / 325 / 1,300 cells cross the plane at depths 0–3, about 400 splits, 3,300 visits, 150–300 Clipper intersections; 5–20 ms.

**(b) Needed strands.** A split adds three planes through the cell's centre, one of each family. They are printed only where a failing plane needs them: for each split cell and each failing plane (D⁺, m), the trace of each new plane at z_m, intersected with D⁺ and with the fill boundary at m. Each piece shorter than a bead (w − 1e−4) is lengthened to w along its trace within the boundary interval that holds it (centred on itself, then shifted inside); a piece whose boundary interval is shorter than a bead is dropped, and that face is a gap (the measure above sees it so). Recorded as a strand (plane key, layer m, [t_0, t_1], kind needed). Nothing is printed outside the footprints: the siblings of a refined cell stay coarse, and an untouched cell prints only Cubic.

**(c) Solid columns.** A cell that fails but whose children would reach 100% density (d · 2^(depth+1) ≥ 1, pitch below the line spacing) becomes a solid column: its faces print already, so the column fills between them. Its faces that are refinement planes print on every layer of the window as strands along the section's edges (bulk faces print anyway), so every course ends on a bead. On each layer of its window it lays parallel courses of one face family, alternating between two families from layer to layer, dividing the distance between two faces of that family into ⌈face spacing / s_f⌉ equal parts, none on a face, each clipped to the cell's section shrunk by W_N/2 − w/4 (a course meets the crossing faces at 60°, and its square end then stays inside the face's bead). The window opens on the layer at or just below the height where the section first grows past G_b (the section is still within the gap there, so the first course is a short bridge between faces) and closes at the highest plane that failed in the cell. A section too small for a course is covered by its faces' beads. Each course rests on the previous, slightly different one and on the faces; a lowest course over the void of another group or pattern prints as planned and a development build warns. With the gap at its floor (the nozzle diameter) saturation is reached at depth 5 at 3%; at the default 5 mm it is not reached on any fixture.

### 3.7 The support sweep

The sweep holds every strand the refinement laid, from the layer below, with lattice columns. It is a small hold rule and a greedy placement, not a search: no restarts, no rerouting, no steering toward other columns; the old branch's 1,941-line sweep was mostly such search around the same rule. It goes down each group from its highest layer that has a strand to layer 0, one layer at a time; at layer m it reads what prints at m−1: the bulk lines inside the fill boundary at m−1, the needed strands at m−1, the column pieces already placed at m−1, and the material that surely prints, M⁻_{m−1} (3.3). Solid-column courses are held by 3.6c and are not swept; seam walls and rails are held along their length by 3.5, and only their ends are items here.

Geometry, per layer pair (m, m−1) with heights h: s_m = h_m / √2, the sideways drift of a plane's trace; α_m = √((w/2)² − s_m²), the reach of a piece along the same plane (requires s_m < w/2; else the layer is too thick for the lattice to rest on itself, and a development build warns once per object); c = (w/2) / sin 60° = w/√3, the reach along a line from a crossing with a line of another family; λ = w, the length of a column piece. At 0.2 mm layers and 0.42 mm beads: s = 0.141, α = 0.155, c = 0.242. At Heimdall's 0.5 mm beads: α = 0.206, c = 0.289.

**Hold rule R.** A point at position x along a strand on plane K at layer m is held by the print set of m−1 when any of these holds:

- (i) own plane: a printed interval of K at m−1 contains a point t with |x − t| ≤ α_m (the two beads' centres are then within w/2);
- (ii) crossing: a printed line of another family at m−1 crosses K's trace at t_c with |x − t_c| ≤ c (the beads overlap at the crossing);
- (iii) material: offset_round(M⁻_{m−1}, w/2) contains the point.
- (iv) fused end (added 9 November 2026 with the anchors of 3.8): an *end* of a plan line that meets another printed line of its own layer m within w/2 (a bulk line inside the fill boundary, or a strand of another family) is fused to it by its nub and is a held point; the met line is itself held or bridging. The K2 checker reads the same rule (5.3): an end whose bead overlaps another bead of its layer is not free. Most needed strands end on the line of their cell's face, so this clause spares the column that the first version placed under nearly every strand end.

For a band (N > 1), a position is held when every sub-line is held; the closed forms are in 3.9.

**Per layer m, for each item** (a needed strand at m, an extension, or a column piece placed at m by the step above), with interval [t_0, t_1] on plane K:

1. H = the held part of [t_0, t_1] by R, as a union of intervals; gaps = [t_0, t_1] − H.
2. A gap with held points on both sides and length ≤ G_b is a bridge. Nothing to do.
3. A gap at an end of a needed strand (a free end): extend the strand along its trace, inside the fill boundary at m, to the first held point if that lies within G_b and the whole run from the last held point to it is at most G_b; the whole becomes a bridge between held points. Record the extension as a strand of kind extension at m.
4. Otherwise the gap needs holders on m−1: column pieces of length λ on plane K, under the gap, placed greedily from its ends so that every stretch between held points is at most G_b and no end is free: a piece [y − λ/2, y + λ/2] on K at m−1 holds [y − λ/2 − α, y + λ/2 + α] of the item.
   - A piece is *admissible* when it lies whole inside the fill boundary at m−1 and a chain of pieces within α of each other continues through the boundaries of the three layers below it, or lands sooner (room for the column to continue). Inside material no holder is needed; the boundary excludes material.
   - Placement is greedy from each free end inward: the first holder as near the end as an admissible position allows within λ/2 + α of it, then each next holder as far along as keeps every stretch between held points within G_b. Where no admissible position exists in a window, the stretch is unheld and recorded, and placement resumes past it.
   - If no admissible piece on K reaches the gap, a *cover* piece of another family holds it through (ii): a piece of length λ on a plane of family k' ≠ k through the point to hold, centred on the crossing. The *plane through a point* is the finest refinement plane of that family (levels 1 to 16; never a bulk plane) whose trace at that height passes within τ = (c − w/2) · sin 60° / 4 of the point; step 6 uses the same search. If none exists, the stretch prints unheld, as planned, and the place is recorded (5.2).
5. Each piece placed at m−1 is an item at the next step: a *column*. A column *lands* when its piece is held whole at m−2 by (ii) or (iii), or *merges* with a planned interval of its own plane at m−2 that lies within λ (same-plane pieces closer than λ join into one interval). Otherwise it *continues*: a new piece at m−2 within α along its trace, steered to the position in [y − α, y + α] that minimises the estimated layers to a landing: the first approaching crossing of K's trace with a bulk line of another family that prints there (closed form, 3.2: k = max(1, ⌈(g − (c − λ/2)) / (|v| + α)⌉) for a crossing g away closing at |v| per layer), the nearest material edge along the trace within 20 mm, or the bed. A joined interval (same-plane pieces merged, longer than λ) continues as the interval narrowed by α at each end until it is shorter than λ, then as a piece.
6. **Family change** (a measured addition, stage 5c of section 8; user, 8 November 2026: a column may change direction, among the three lattice directions only). A column whose piece at m is a single piece (length λ) centred at x on plane K of family k, and that has not landed, evaluates a change when its estimated layers to land by staying, k_stay, is at least k_min = 6, or when staying has no admissible piece. For each family k' ≠ k: K' = the plane of k' through the piece's centre at z_{m−1} (the search of step 4; a bulk plane does not count: had that line printed there the column would have landed); x' = the position along K' of its crossing with K's trace at z_m; the window W' = [x' − r_sw, x' + r_sw] with r_sw = (λ − c)/2; y = the admissible point of W' nearest the steering target on K'; k_sw(k') = 1 + the estimate from (K', y, m−1). The column changes family when some k_sw ≤ k_stay − 1 (or when it is trapped and some k' is a candidate), taking the smallest k_sw, then material over bulk over bed, then the lower family index; the switched piece [y − λ/2, y + λ/2] goes on K' at m−1 under the column's id. The piece above is held whole by (ii), since |x − t_c| ≤ τ ≤ c − λ/2. A joined interval longer than λ never switches. Monte Carlo on the true lattice: switching cuts mean column depth by 20–23% (41 → 32 layers at 3%, 13 → 10 at 10%), 26–31% of columns switch, k_min = 3 or 6 and a gain of 1 or 2 give the same depth.
7. **Ends.** A planned strand ends where the boundary cuts it; the sweep plans with those ends. At emission the stock connector anchors the ends on the boundary and nubs anchor the ends that meet another line (3.8 step 5, user decision of 9 November 2026); an anchor is extra material along a line that is held, so it changes no hold. A needed strand's end over the void is held like any other point of it: by a holder within α, or by an extension to a held point.

Termination: every step goes one layer down; a column ends by layer 0 at the latest, whose lines rest on the bed or the raft. No cap; nothing is cut short.

Output per layer: the strands with their kinds, and the unheld places. On the user's parts the old branch's sweep left 153 unheld column pieces on plates 1–5 (86 before family changes), all where the boundary left no room, and held every needed strand; the acceptance reports the same classes (7.4).

### 3.8 Emission

**The fill boundary.** The filler clips its lines, as Cubic does, to the fill surface inset by half a spacing (`Fill::overlap` is 0 in `make_fills`, so FillCubic's inset of overlap − spacing/2 is −s_f/2; the perimeter generator has already grown the fill surface into the wall by `infill_wall_overlap`), less the beads of the pattern's courses, cut so that the lattice overlaps a course by `infill_wall_overlap` as it overlaps a perimeter: `fill_boundary(island, courses, s_f, infill_wall_overlap)`, a function of the pattern's material unit (there is none on master; FillCubic's inset lives inside `fill_surface_by_multilines`). The plan computes the same boundary at plan time, per layer and group, from the pre-bridge surfaces: `group_fills` on the layer (2.3 item 8a) gives the islands of the pattern's sparse fills, and the pattern's own `fill_boundary` (in the material unit, 6) gives each island's boundary exactly as the filler computes it. The plan has one clip; where the bridge generator later changes a surface, the pass below prints what the change took.

**The filler** (`FillBridgeAwareCubic::fill_surface`), for one island I of the pattern's sparse fill at layer n, bound to the object's plan and the layer:

1. If the plan has nothing for I's group on any layer (no strand, no course), return `FillCubic`'s output: the group prints as Cubic (within the rounding of the fill groups' clipping order, 2.3). A filler without a plan (the 100% case; a pattern set by hand on a solid surface) does the same.
2. Bulk lines: the three families (bands of N) across I's bounding box, by the closed form, all of them: the lattice's extreme lines included, which `make_fill_lines` drops within one line width of the rotated bounding box, because a dropped edge line leaves a sliver of void unbounded.
3. Strands of layer n whose trace meets I: each clipped to the boundary along its trace (closed form, one edge-grid query per strand), pieces shorter than w − 1e−4 dropped. One threshold for "a bead long", shared with the sweep (the old branch had four).
4. Courses of layer n of the regions whose sparse fill I holds: clipped to I, printed whole, joined to nothing.
5. Every line goes through `Fill::connect_infill` (src/libslic3r/Fill/FillBase.cpp:1168-1186 and :1505; a join when the boundary arc is shorter than `sparse_infill_anchor_max` at :1600-1603 and :1641, else a hook of `sparse_infill_anchor` at :1608-1610 and :1717-1728) with the boundary and the region's anchor settings, exactly as Cubic's lines do (user, 9 November 2026, 9.1: every line the pattern adds is anchored like Cubic's, with the stock settings; Cubic's boundary behaviour accepted). The connector only touches ends that lie on the boundary; a strand end that stops within w/2 of the boundary is carried on to it first, so that it is anchored to the perimeter like a bulk line's. An interior end that meets another line of the layer (a needed strand ends on its cell's face: a bulk line or a parent strand) gets a nub along that line's centre line, `sparse_infill_anchor` long and clipped to the met segment, appended to its path; two ends that meet each other are joined into one path instead (as the connector joins two ends on the boundary), so that no piece prints twice; an end in the void, held from below, gets nothing. All of it is sparse infill of the region at the sparse flow. `no_sort()` is false. (The earlier rule, planned lines bypassing the connector so that no end is hooked along a floating boundary, is withdrawn by that decision.)
6. A combined surface (`thickness_layers` > 1): 3.9a.

**The pass** (`BridgeAwareCubic::print_uncovered(layer)`, one call at the end of `Layer::make_fills` after every filler): for each region with a plan, take the planned lines of layer n that the pattern's fillers did not print: the strands and courses outside the islands handed to them, and, under combination, the bulk lines, strands and courses inside the single-layer zones the fillers skipped (3.9a). Keep the parts outside the printing footprint of every extrusion of the layer (perimeters and fills by their nominal width; a solid fill that printed nothing has no footprint); drop pieces shorter than a bead; connect the bulk lines among them with `connect_infill` against the zone's boundary, as the filler would, and print all of it as sparse infill of the region, one collection per island of `lslices`. Why: the generator and the thick bases take sparse area after the plan (an ensuring strip one spacing wide along a bridge's edge, a bridge's growth, a thick base). Where what took it prints material, that material holds; where it prints nothing, the plan's line must still print, because the preview showed it and a bridge may be anchored on it, and because a column above may rest on it. This is not a repair of the construction: it is the emission of the plan under the user's order of operations (the plan is made before the generator, and the generator is not touched), it changes no path already made, and it runs on the few layers near roofs. It is the one piece of work this design adds to the release slice beyond the plan and the filler, and it is listed as decision R22 for the user's review. The old branch's wall-reprint pass was this pass restricted to seam walls.

Why K1 and K2 survive emission: the filler prints every planned strand inside its island up to rounding, the pass prints the rest, and the only planned material that is not printed lies under printing material, which holds whatever the strand held. The connector only joins or extends lines; it neither shortens nor drops one (src/libslic3r/Fill/FillBase.cpp:1505-1733). Anchors add material: a hook along a wall lies over the wall, a nub along a bulk line over the lower bulk line (s < w/2), a nub along a strand is held where that strand is and ends fused to it; K2 counts an end fused alongside a bead of its own layer as anchored (5.3).

### 3.9 Settings the construction honours

**(a) Infill combination.** `combine_infill` runs stock on the pattern's regions (the plan is built before it, from single layers, and plans nothing differently): the surfaces combine as Cubic's, with the same thick top surfaces and the same `stInternalVoid` layers. The pattern's own construction needs every layer (a column rests on its own piece within α; a bead n_c·h tall cannot pass over single-layer strands), so it prints that construction at layer height where it needs single layers, as walls are printed at their own layer height under combination:

- Z_l, the single-layer zone of layer l = offset(footprints at l and l+1 of the plan's strands, courses and solid-column sections, c + w) ∪ offset(the footprints D⁺ of the planes on l, w).
- For a combined group of layers a..b (the top layer b carries the thick surface), Z = Z_a ∪ … ∪ Z_b. The filler, on the combined surface at b, prints its lines of layer b at the combined thickness outside Z, and nothing inside Z.
- The pass (3.8) prints the plan's lines of layer l (bulk, strands and courses) inside Z at layer height on every layer l of the group, wherever nothing covers them, the void layers included, and connects the bulk lines as the filler would.

Outside Z there is no strand and no demand: ordinary combined Cubic. Inside Z every layer prints as without combination, so the plan's premises hold. Heimdall sets combination off; the fixtures test it (S3).

**(b) Fill multiline N.** Honoured as Cubic honours it: the pitch is L_N (3.2), every lattice line and strand is a band of N sub-lines at offsets o_i, and the measures use the band width W_N. Holds for a band of one group (every line a band of the same N): own plane α unchanged (each sub-line rests on its sub-line below); crossing reach c_N = c + (N − 1) s_f / (2√3); material: every sub-line within w/2. Cover and switched pieces have length λ_sw(N) = λ + (N − 1) s_f / sin 60°, window r_sw unchanged. Rails print as N offset curves at d_r + i·s_f, and D⁺ grows to the outer curve. Seam walls stay one bead (they close a channel). Solid-column courses are clipped by W_N/2 − w/4. The descent's depth bound becomes the smallest k with √2 · a_0 / 2^k − W_N ≤ G_b. At N = 1 every formula reduces to the single-line one and the output is identical to this document's single-line construction (stage 7b's gate).

**(c) Variable layer heights.** s_m and α_m are per layer pair; the lattice shift uses `print_z`; a layer too thick for α (s_m ≥ w/2) is a development warning, not an error.

**(d) Rafts and the first layer.** `print_z` includes the raft, so the lattice is where Cubic's is; the first object layer rests on the raft or the bed, which is material to the sweep.

**(e) Modifiers and multiple regions.** Each region has its own gap; groups follow the key of 3.1; seams between groups get walls; the gap is in `s_modifier_ignored_infill_keys`, so `modifier_ignore_infill` restores it from the parent together with the other infill keys.

**(f) Shared print objects.** An object that shares the layers of another (instances) returns its source's plan.

**(g) Conformal infill** is ignored as Cubic ignores it: only Rectilinear's filler reads it. **Spiral vase** sets the density to 0. **Internal bridge layers** and **Bridge layer density** are the stock generator's; the plan reads N from the former and treats a sub-100% bridge layer as material where its lines print (1.3).

## 4. Constants

| Name | Value | Where used | Reason |
| --- | --- | --- | --- |
| G | clamp(option, nozzle diameter, option max) | 3.1 | user: below the nozzle the pattern would have to be solid |
| G_b | G − 0.1 mm | planning (3.6, 3.7) | simplification, arc fitting and rounding move vertices far less |
| μ | 0.1 mm | 3.6 | the same margin, named for the measure |
| option min, max | 0.01 mm, 1000 mm | 2.3 item 2 | static bounds for the GUI; the runtime clamp is G |
| s_f | `Flow::spacing()` of the sparse flow | 3.2 | Cubic's pitch uses spacing, not width |
| L_N | `coord_t(scale_(s_f) · N / (d/3))` | 3.2 | FillCubic |
| a_0 | L_N · √(2/3) | 3.2 | cube edge |
| s_m | h_m / √2 | 3.7 | drift of a plane's trace per layer |
| α_m | √((w/2)² − s_m²) | 3.7 | own-plane reach |
| c | w / √3 | 3.7 | crossing reach at 60° |
| c_N | c + (N − 1) s_f / (2√3) | 3.9b | band on band |
| λ | w | 3.7 | one bead: a column piece |
| τ | (c − w/2) · sin 60° / 4 | 3.7 | cover and switch planes through a point (the old branch's validated value) |
| r_sw | (λ − c) / 2 | 3.7 | switch window |
| k_min, gain | 6, 1 | 3.7 step 6 | Monte Carlo: 3 or 6 and 1 or 2 give the same depth |
| look-ahead | 3 layers below the piece | 3.7 step 4 | room for a column to continue |
| material look-along | 20 mm | 3.7 step 5 | how far along its trace a column looks for material |
| plane levels | 1 to 16 | 3.7 steps 4, 6 | cover and switch planes; a0 / 2^16 is far below a bead |
| ε | 0.01 mm | 5.1, 5.3 | touching, for the verifier and the K2 checker |
| δ_s | 0.01 mm | 3.5a | a neighbour "next to" a boundary |
| seam inset | w/2 − 0.05 mm beside beads; w/2 beside 0% or air | 3.5a | closes butt-end channels |
| corner follow | w | 3.5a | survives path simplification |
| d_r | s_b + w/2 + 0.05 mm | 3.5b | first anchor past a bridge section's end |
| solid rule | opening by 0.55 · s_role | 3.3 | a fill lays a line where ≥ 1.1 spacings wide |
| h_n | half the nozzle diameter | 3.4, 3.5, 5.1 | the perimeter generator's overhang distance, applied to what surely prints |
| r | nozzle · √(bridge_flow) under a bridge candidate (the thick bridge flow, src/libslic3r/LayerRegion.cpp:32-48); else the narrowest wall or solid bead | 3.6 | the roof's own bead for the opening |
| round joins, arc tolerance | `jtRound`, 0.001 mm | every offset of material | the overhang rule is a distance, not a Minkowski square |
| sliver | pieces < w − 1e−4 dropped; needed strands lengthened to w | 3.6b, 3.8 | a bead shorter than its width prints a blob |
| visit warning | 1,000,000 cells per group | 3.6, 5.2 | development information only |
| cancel period | 1024 visits | 3.6 | |
| solid column course clip | W_N/2 − w/4 | 3.6c | square ends inside the faces' beads |

## 5. Development checks

All of them are compiled into the slice only with `SLIC3R_BAC_DEVELOPMENT_CHECKS` (off by default and in the release recipes `BuildMac.sh` and `scripts/build_tyr_macos_runtime.sh`). They raise `NON_CRITICAL` warnings through `active_step_add_warning`; they change no path and stop no slice. The tests call the same functions directly in any build. Messages use the slicer's own line-type names ("Sparse infill", "Bridge", "Internal solid infill", "Top surface"; there is no "support line"). Every message names the object, the region by its settings (density, gap), the heights of the roof's layer and the plane, and the measured value with its place on the bed.

### 5.1 The verifier (K1 on the final paths)

`BridgeAwareCubic::verify(const PrintObject&, cancel) -> VerificationReport`, run after `simplify_extrusion_path`, on the paths as they are exported. It reads the plan's planes (3.4, stored with their footprints) and measures each on the final paths; a roof's deeper planes are measured too, since the thick bases may have moved its support plane. For each plane (D, m, G):

- F = the footprints by nominal width of every extrusion of layer m except ironing.
- openings = the components of opening(lslices_m − F, r/2) ∩ offset(D, r/2) that meet the void of layer m (K1(a), as 3.6 measures it); check diam(hull(opening)) ≤ G and L(opening) ≤ G. A surface that printed nothing has no footprint, so a sliver of void beside it is part of the opening.
- runs: the paths of the roof's layer that meet D (perimeters as closed loops, non-sparse fills with multipaths joined, the pattern's courses, and the bridge paths, `erBridgeInfill`: this fork has no separate internal bridge role), each once; clipped to lslices_m − offset_round(F, h_n + ε) − the voids of other regions; check each run ≤ the region's G. The runs of a bridge over the void beyond the plan's footprint (its growth) are reported as information: growth is not demand (9.1).
- free ends: an end of an open path in the region's void, off the support and outside the wall overlap, counted by role; an end of internal bridge infill fails the plane (K1(b)); other roles are reported.

The report holds, per plane, the widest opening, the longest run, the free ends by role, the roof's bead r, and the first failure with its place. It is a test oracle (7.2) and a development warning source.

### 5.2 Plan checks

Reported by the plan itself, from data it gathers only in a development build:

- the places where the sweep printed a stretch or a column piece unheld (3.7 step 4), with the reason (no admissible piece, no cover plane); the plan gathers this list in every build (a few places per object), the flag governs the warning;
- a seam wall's lowest course over another pattern's void, a 0% region or air, beyond h_n of material (3.5a);
- a solid column's lowest course over the void of another group or pattern (3.6c);
- a lattice group whose refinement visited more than 1,000,000 cells (3.6);
- a layer above the first whose height gives s_m ≥ w/2 (3.7), once per object;
- the count of rails laid on surfaces the generator did not bridge in the end (prediction misses; 3.5b), from the final surfaces.

### 5.3 The K2 checker (on exported G-code)

An independent tool, `k2check`, reads an exported G-code file and, for every sparse-infill path piece over the void of the layer below that is not one of Cubic's bulk lines (told apart against the Cubic slice of the same part, every segment of the piece collinear with a Cubic line), checks the hold rule R of 3.7 exactly, with ε for touching: a parallel bead below with centre lines within w/2 (a disc about every point of the lower centre line, so round ends; not merely touching beads), a crossing within c, material whose edge is within w/2 + ε; otherwise the piece must be a stretch of at most G_b between held points. A path is measured in the parts between its vertices that lie on another bead of the layer (a junction: a line of the pattern ended on existing material there and its nub begins, or the connector joined two lines; rule (iv)), and an end fused alongside a bead of its own layer (centre lines within half the two widths and ε: the anchors of 3.8) is not a free end. It reports free ends and unheld stretches by cause. It shares no code with the planner. The carried-over script counts a point as held when its centre line is within w/2 of a lower bead's footprint, which lets parallel beads that merely touch count; it is aligned to R before stage 5a, so that the suite's K2 case and the acceptance measure the same thing. The test suite applies the same rule through the export observer (7.2, K2); the acceptance runs the checker on Heimdall's plates 1–5 (7.4). The old branch's Python implementation is carried over (`tooling/k2/k2check.py`, `causes.py`, `onbulk.py`, `classes.py` in the SDD directory), committed under `scripts/bridge_aware_cubic_acceptance/`.

## 6. Units and interfaces

All under `src/libslic3r/` in namespace `Slic3r::BridgeAwareCubic` unless noted. Line counts are targets, not gates; the total is bounded by 1.4.

| Unit | Lines | Holds |
| --- | --- | --- |
| `Fill/BridgeAwareCubicLattice.{hpp,cpp}` | ≈ 350 | `LatticeParams` (L_N, a_0, θ_k, σ_k, N, s_f, w), `Cell`, `PlaneKey`, `section()`, `height_range()`, `trace()`, `crossing()`, `bulk_lines()`, `roots()`, the sub-line offsets |
| `BridgeAwareCubicMaterial.{hpp,cpp}` | ≈ 350 | per layer: `printed_material()`, `void_of_group()`, `fill_boundary_of_group()` (through `group_fills` and `fill_boundary`), `bridge_candidates()` (through the extracted stock test); `AreaIndex` (an edge grid with `contains(point)` and `segment_inside(line, t0, t1) -> intervals`) |
| `Fill/BridgeAwareCubicPlan.{hpp,cpp}` | ≈ 800 | `ObjectPlan::build(const PrintObject&, cancel)`; regions and groups; seam walls; roofs and planes; rails; `GroupPlan::refine()` (3.6) with needed strands and solid columns; storage by layer; the queries below; the statistics of 5.2 |
| `Fill/BridgeAwareCubicSweep.{hpp,cpp}` | ≈ 700 | the hold rule, items per layer, extensions, holders, cover, columns with steering and family change, ends over the void, the unheld list |
| `Fill/FillBridgeAwareCubic.{hpp,cpp}` | ≈ 250 | the filler (3.8), `bind()`, `print_uncovered()` |
| `BridgeAwareCubicVerify.{hpp,cpp}` | ≈ 450 | 5.1 and the formatting of 5.2 |

Interfaces the shared code and the tests rely on (signatures are the contract; bodies are the implementer's):

```
// Plan
std::unique_ptr<ObjectPlan> ObjectPlan::build(const PrintObject&, const std::function<void()>& cancel);
const GroupPlan*  ObjectPlan::group_of_region(size_t region_id) const;        // nullptr when the region does not print the pattern
bool              GroupPlan::has_content() const;                              // any strand or course on any layer
Lines             GroupPlan::strands_at(size_t layer, const BoundingBox&) const;   // (PlaneKey, t0, t1, kind, sub-line) as scaled segments
const Polylines&  ObjectPlan::courses_at(size_t layer, size_t region_id) const; // seam walls, rails and solid-column courses of the region
std::vector<PlaneSummary> ObjectPlan::planes() const;                          // per plane: roof layer, plane layer, gap, footprint area (3.4); read by P1 and 5.2
ExPolygons        ObjectPlan::single_layer_zone(size_t layer) const;           // Z_l of 3.9a; empty without combination
PlanStatistics    ObjectPlan::statistics() const;                              // 5.2

// Filler
void      FillBridgeAwareCubic::bind(const ObjectPlan&, const Layer&);
Polylines FillBridgeAwareCubic::fill_surface(const Surface*, const FillParams&) override;
void      print_uncovered(Layer&);                                             // the pass of 3.8, called once per layer by make_fills

// Verification (any build; compiled into the slice only under the flag)
VerificationReport verify(const PrintObject&, const std::function<void()>& cancel);
std::vector<std::string> plan_warnings(const PrintObject&);
```

No test hooks in production code: no `allow_*` switches, no `emit_*` toggles, no statistics that exist only for a test. What a test needs it reads from the output (fills, G-code) or from the report.

## 7. Tests and acceptance

### 7.1 Principles

The old branch's 261 cases pinned coordinates, counts, digests, kind bits and hooks; 188 of its 245 commits touched tests, 85 of them nothing else, and of 30 sampled commits 21 mainly re-pinned, fixed or deleted tests after the construction moved. These rules prevent that:

1. A test asserts the contract on the output (K1, K2, settings, parity with Cubic, option plumbing) or a closed-form geometry against a brute-force oracle. Nothing else.
2. No test reads internal state: no strand coordinates, no node or column counts, no kinds, no statistics, no digests of the plan.
3. The oracles are independent of the construction: `FillCubic` for the lattice; brute-force geometry for closed forms; the G-code observer (`read_extrusions`, `footprint`, `measure`, carried over from the old export suite) and the K2 checker for the slice; the verifier's report only where the observer cannot see (the final surfaces).
4. No `[!mayfail]`, no guard values, no "known failure pinned by message". A finding the construction does not yet meet is a line in the status document, not a test.
5. A test that would need re-pinning after an implementation change that does not change the contract is deleted, not re-pinned.
6. Budget: at most 40 cases and 2,500 lines, fixtures included. A pull request that adds a case names the contract line it pins.

### 7.2 The test set

Every observer case keeps two controls, as the old export cases did: a Cubic control (Cubic exceeds the gap on the same part) and a negative control (removing the sparse lines of the plane below breaks the measure). P1 is the only case that reads the plan; it reads the plane list, which section 3.4 defines from the surfaces, and compares it with an independent computation from the same surfaces.

| Id | Case | Oracle | Fixtures |
| --- | --- | --- | --- |
| L1 | The pattern's bulk lines equal FillCubic's point for point, N = 1, 2, 3, at several heights, except within one line width of the rotated bounding box's extremes | `FillCubic` on a rectangle | synthetic |
| L2 | Cell sections, height ranges, traces and crossings agree with brute-force plane–cube clipping and line intersection; refinement plane keys never alias | brute force | synthetic |
| M1 | Printed material: a solid strip narrower than 1.1 spacings is void, the wall band is material, a seam wall separates two groups' voids | the rule applied to a sliced part's surfaces | modifier, a part with a narrow solid strip |
| P1 | Demand: the cubes have planes only under the roof, j = 0..N; a vertical wall and a 45° chamfer create none; a step wider than half the nozzle does | the plan's plane list against the surfaces | small cube, stepped cavity, chamfer |
| P2 | Siblings stay coarse: on the roof's plane layer, outside the roof footprint grown by G, the sparse lines equal Cubic's but for the lattice's extreme lines (R5) | observer against the Cubic control | large cube |
| K1 | Roof contract on exported G-code: openings ≤ G, runs ≤ G for every role over sparse infill (walls, solid, bridges grown past an edge), bridge ends on printed material, free ends by role | observer, every plane and island | the four fixtures; partial roof with 1, 2, 3 internal bridge layers, thick bridges on and off; six lattice phases; stepped cavity; variable layer height; 3% and 10%; gaps 3, 5, 8; a second instance at a fractional offset |
| K1n | The nozzle-floor gap (saturation into solid columns) holds K1 and slices | observer | small cube at gap 0.4 |
| K2 | The pattern's added lines are held: every added sparse piece (not on a bulk lattice line, told apart against the Cubic control's lines) off the plane below is at most G_b between held points, and no added line ends free over the void | observer with the hold rule of 3.7; the K2 checker on the export | small and large cube, rib, modifier, box 40 at 3% |
| R1 | Rails: at a bridged roof's inner edge the bridge reaches at most d_r + w_b + 0.05 past the edge; no bridge end over the pattern's void hangs (it lands on printed material or on a wall, which its overlap anchors); no rail where no bridge is predicted; 1, 2 and 3 internal bridge layers | observer `count_bridge_ends`, counting an end inside the wall overlap as anchored; final surfaces | partial roof, steps, fuselage top, the eight fixtures of the old preview case |
| W1 | Seam walls: the modifier seam carries one bead of the lower-index region, inset as 3.5a; each region holds its own gap (8 and 3 mm); a 0% neighbour stays empty; a floating modifier's first course is refined under; no wall against material or a same-group neighbour | observer, split by position | modifier, floating modifier, 0% half |
| S1 | Option plumbing: a scalar region option with default 5 in `print_options`; preset, 3MF and project restore; the gap change invalidates `posPrepareInfill`; `modifier_ignore_infill` restores the gap with the other infill keys | config | small cube |
| S2 | Parity where nothing needs refinement: open top; 100% (the same refusal and the same fills as Cubic, with and without combination); density under the stock limit; spiral vase; Conformal on and off; a raft moves nothing but z | `fill_paths` against Cubic within 0.001 mm; validation messages | open box, steps, small cube |
| S3 | Combination on: the pattern's surfaces (`stInternalVoid`, thick `stInternal`) equal Cubic's; inside the single-layer zone every plan line prints at layer height; K1 and K2 hold | surfaces; observer | small cube, large cube |
| S4 | Multiline N = 2: bulk bands equal Cubic's; K1 and K2 hold with band widths | `FillCubic`; observer | small cube |
| S5 | Two instances of one object measure the same; a shared object reads its source's plan | observer | small cube × 2 |
| S6 | No setting changes: the applied configuration after `process()` equals Cubic's but for the pattern and the gap; arcs are still fitted; bridge angles and internal bridge domains are kept; solid fills merge as Cubic's | config diff, surfaces, G-code | small cube, modifier, explicit bridge angle |
| S7 | The slice completes and holds K1 under stress settings: gap at the nozzle, 60%, 0.003%, raft, Arachne with narrow solid infill, variable layer height, `wall_loops` 0, combination, a 0.6 mm sparse bead, arc fitting, a blind cavity | `REQUIRE_NOTHROW`; observer | small cube, pocket block, blind cavity |
| D1 | Determinism: the same G-code on every slice; a reslice after gap, density or direction edits, and back, equals a fresh slice | exported G-code compared byte for byte; `fill_paths` | large cube, partial roof under 3 layers, modifier |
| V1 | The verifier fails a Cubic object measured at gap 5 and passes the pattern's; a release build raises no warning | report | small cube |
| X1 (hidden) | `[BACTiming]`: instructions retired by `process()` and export, Cubic against the pattern, a warm-up pair and five pairs | information only | the four fixtures |
| X2 (hidden) | `[BACBenchmarkFixtures]`: writes the native projects of the official benchmark | | |

Twenty-one cases. Most of them read the exported G-code through the observer, which shares no code with the pattern.

**Helpers carried over verbatim** (about 1,000 lines, inside the 2,500): the export observer (`tests/fff_print/test_bridge_aware_cubic_export.cpp:40-342` on the old branch: workspace, `export_config`, the exact decimal readers, `read_extrusions`, `footprint`, `measure`, `emitted_layers`, `measure_first_roof`, which is generalised to every plane and several islands and clips its openings to the footprint grown by r/2, as 5.1 does), `observed`, `count_bridge_ends`, `count_free_sparse_ends`, `exported_gcode` (`:1277-1392`, `:1438-1498`) and the observer's self-test (`:454-465`); the fixture header's `with`, geometry, configs, `paths`, `slice`, `sparse_length`, `roof_gaps` and `bridge_ends` (`bridge_aware_cubic_fixtures.hpp:35-42`, `:79-220` except `at_bed_origin`, `:228-333`); `fill_paths`, `different_layers`, `apply_with_setting` (`test_bridge_aware_cubic.cpp:33-95`); `deposited_segments`, `combined_infill_paths` and the bridge surface snapshots (`test_bridge_aware_cubic_modifiers.cpp:38-129`); the config helpers (`test_bridge_aware_cubic_config.cpp:17-53`). Dropped: every helper that re-derives the implementation (`fill_clip`, `trace_segment`, `k2_in_plan`, `Steering`, `ReferenceDescent`, `CarryingDescent`, `SliceLog`), the verifier's synthetic builders, `plan_details_kept`, `pattern_warnings`, `holds_demand`, `report_counts`, `plane_log`, `seam_hole_under_roof`, `require_exported_volume` and `preview_footprint`. The qualification runs, the CLI fixture writer, the timing gate and the perf harness move to the scripts.

### 7.3 Fixtures

The four benchmark fixtures (small cube 32 × 28 × 18, large cube 64 × 56 × 18, rib, modifier) are kept as they are, with their native projects. Of the old branch's test geometry, what a user would print is kept: the small and large cubes (one roof over sparse infill each), the partial roof (a step whose roof ends over sparse infill, the hardest everyday case, with its internal bridge layer variants), the rib (ribs on a roof, with `rib_config`'s vertical-shell thickening off as a deliberate stress), the modifier family (a modifier that densifies half a part; `modifier_between`, `floating_modifier`, the 0% half), the stepped cavity, the steps, the chamfered block, the six lattice-phase boxes, the 40 mm box at 3%, the open-top box, the pocket block, a block with a narrow solid strip, the dome, the fuselage cylinder top, the pitched and perforated roofs, the ledge and the cylinder, and the blind cavity. `fixture_config` stays as the common settings (classic walls, no arc fitting, no narrow solid, `thick_bridges` 0, `seam_gap` 0) and S7 varies them; S7's Arachne, narrow-solid and thick-bridge variants are the user's own settings, so they are not optional. Not carried over: `plan_details_kept`, `Digest`, `at_bed_origin` (a workaround for an uninitialised `Print::m_origin`, fixed on the legacy branch), and the development-check helpers listed above.

### 7.4 Acceptance on the user's parts

Every change that affects output is accepted on all 16 Heimdall plates, sliced in the release configuration, before progress is claimed, by the controller, with the scripts committed under `scripts/bridge_aware_cubic_acceptance/` (carried over from the SDD directory `tooling/`: `acc/batch.sh`, `acc/run.sh`, `acc/analyse2.sh`, `metrics/metrics.py`, `tools/table.py`, `tools/bridgecheck.py`, `k2/*.py`), on a local copy of the project (`/Users/jforbes/Documents/Development.nosync/BambuStudio-bac-data/`, never the shared drive). The scripts take their paths from the repository and from `BAC_DATA` (default `/Users/jforbes/Documents/Development.nosync/BambuStudio-bac-data`), need a Python with shapely 2 and numpy (`BAC_PYTHON`), and are described in the tooling's README. Baselines (`$BAC_DATA/baselines/cubic/pN/output/plate_N.gcode`, 16 plates) are regenerated once on the new branch by slicing the Cubic project with the same build. Reported per plate and in total: grams by role against Cubic, the estimated print time, sparse path and retraction counts, instructions retired and peak RSS; K2 classes on plates 1–5; internal bridge ends over sparse void and the longest hanging stretch on every plate, against Cubic. Thresholds are those of 1.4. A plate that fails to slice fails the acceptance.

### 7.5 The release gate

`scripts/bridge_aware_cubic_benchmark/run_benchmark.py` against the official 2.9.1.70 application, as `docs/bridge-aware-cubic-benchmark.md` describes, with no other slicing job running (the parallel 16-plate batch raised instruction counts by 4–8%): one warm-up pair and five alternating pairs per fixture, median and worst pair ratio ≤ 2.0 in instructions retired. The benchmark tooling (`scripts/bridge_aware_cubic_benchmark/`, `tests/benchmarking/`, the fixture projects, the procedure document) is carried over from the old branch verbatim as the new branch's first commit: it is measurement tooling with its own tests, validated against the official baselines, and it is not the pattern's code.

## 8. Build order

Each stage ends with a gate the controller measures by script, not inside an implementer task. Implementer tasks are about an hour each, on Opus, with one review per task; critical and important findings are fixed in the task, minor ones wait for the final review. Independent tracks run in parallel worktrees with their own build trees.

| Stage | Builds | Gate |
| --- | --- | --- |
| 0 | `codex/legacy-test-fixes` merged into master; the new branch from master; the old branches and worktrees deleted, without tags (user). First commit: this document, the benchmark tooling and fixture projects (from the SDD copy `tooling/branch/`), the acceptance scripts (7.4, 7.5), a status document (`docs/bridge-aware-cubic.md`) | benchmark-tool tests pass; the official baseline app is present; ctest green on master |
| 1 | Hooks 1–5 and 10 of 2.3 with the filler delegating to `FillCubic` (no plan) | the four fixtures' fills equal Cubic's within 0.001 mm per vertex (the enum's position changes the fill groups' clipping order by a safety offset); the option appears and persists; the four Cubic fixtures' G-code is byte-identical between master's build and the branch's (hooks 7, 8 and 10 touch code Cubic runs); legacy ctest green |
| 2 | Lattice unit; tests L1, L2 | tests pass |
| 3 | Material and roofs (3.3, 3.4) without refinement; hooks 6, 8a, 8b, 8c; tests M1 (its seam-wall assertion waits for stage 6), P1 | plane counts as predicted on the fixtures |
| 4 | Refinement with needed strands and solid columns (3.6); the filler's steps 1, 2, 3 and 5 (no sweep; no seam walls or rails yet); tests P2, K1 (openings and runs only), S2 (open top) | K1 openings and runs pass on the four fixtures; large cube ≤ 1.5× Cubic, indicative; Heimdall run 0: all 16 plates slice, mass and time reported |
| 5a | The sweep (3.7) without family changes; the K2 checker aligned to R and committed; tests K2, K1n | zero free ends and unheld stretches of added lines on the fixtures; Heimdall run 1: mass and time within 1.4, unheld pieces on plates 1–5 ≤ 86 by the checker with their causes, no free end of a needed strand |
| 5c (after 5a, optional) | Family change (3.7 step 6) | built if Heimdall run 1's estimated time is above +12% over Cubic (today +10.1%), to keep headroom under the ceiling for the rails and later stages; gate: fewer pieces and less time on the 16 plates, no new unheld place |
| 5b (parallel) | The verifier and plan checks (5.1, 5.2), the build flag; test V1 | the verifier agrees with the observer on the fixtures |
| 6 | Seam walls, rails, the uncovered-lines pass (3.5, 3.8), hooks 7 and 8d; tests W1, R1, K1 (bridge ends) | no bridge end hangs on the fixtures; Heimdall run 2: zero hanging bridge ends over the pattern's void on all 16 plates (wall-overlap ends anchored), longest hanging stretch ≤ Cubic's per plate |
| 7a | Combination (3.9a); test S3 | Heimdall unchanged (combination off); S3 passes |
| 7b (parallel) | Multiline (3.9b); test S4 | output at N = 1 byte-identical to stage 6; S4 passes |
| 8 | Tests S1, S2, S5, S6, S7, D1; the status document filled | all tests pass; ctest green; the Cubic fixtures' G-code still byte-identical to master's build |
| 9 | Release gate with no other slicing job running; docs tables; the release tracker (ask the user before posting) | 7.5 |
| 10 | Whole-branch review (on Opus, as every subagent); one fix round; `superpowers:finishing-a-development-branch` | |

Rough size: 25 to 30 tasks. Stages 2 and 3, 5a and 5b, 7a and 7b run in parallel.

## 9. Decisions

### 9.1 Carried over from the user (binding)

| Date | Decision |
| --- | --- |
| 2026-11-06 | Constructive planning from surfaces; no search, no trial regeneration, no whole-layer oracles in loops, no caches as the primary fix |
| 2026-11-07 | Checks are development-only warnings; no `SlicingError`, fallback, truncation or repair pass for the pattern; "if a limit is hit, BAC generates anyway"; fix the construction, not the output |
| 2026-11-07 | Bridges are connected at both ends; no short overhangs or cantilevered ends, however short |
| 2026-11-07 | The closed-cell requirement is dropped for mass: open cells, needed strands only where a plane fails |
| 2026-11-07, 2026-11-08 | Test on the user's real parts, all 16 plates, in the release configuration, before claiming progress; measure instructions retired, never wall time |
| 2026-11-08 | One plan, before the bridges; no re-plan after them; the previews bind the plan; the stock generator anchors at the closest point, untouched; a bridge's growth over sparse fill is not demand |
| 2026-11-08 | "No global change" means no change to the part's settings; other roles differ by the stock response; no BAC special case in shared logic; unrelated fixes on their own branch |
| 2026-11-08 | Print time ≤ +17.8% over Cubic on the 16 plates; the release gate (≤ 2.0×) is the only speed limit; memory reported only |
| 2026-11-08 | Joins only along the lattice; Cubic's edge connections stay; columns switch direction among the three lattice directions only |
| 2026-11-08 | Catch rails follow the roof edge; their columns stay on the lattice |
| 2026-11-08 | No coupon; Heimdall is the test part |
| 2026-11-08 | The gap's minimum is the nozzle diameter (clamped, no GUI check) |
| 2026-11-08 | Multiline honoured as Cubic honours it; combination honoured |
| 2026-11-08 | Subagents on Opus, tasks of about an hour; 16-plate runs, gates and ctest are the controller's scripted steps |
| 2026-11-08 | Rewrite from master on a new branch; the old worktrees and feature branches are deleted after this document is accepted |
| 2026-11-09 | Every line the pattern adds is anchored like Cubic's lines ("a small nub where it meets another extrusion, extruded along it for a small length"), with the stock `sparse_infill_anchor` / `sparse_infill_anchor_max` settings; Cubic's boundary behaviour is accepted for now. Planned lines go through the stock connector with the bulk lines (3.8 step 5 as amended); an interior end that meets another line of its layer gets a nub along that line; an end fused alongside a bead of its own layer is not a free end for K2 |
| 2026-11-09 | Geometric tolerances derive from the global precision settings (`resolution`, the arc-fitting resolution of sparse infill, `slice_closing_radius`), never from invented constants: the sweep's reaches are shortened by the path simplification the print applies plus the G-code's coordinate rounding (plan: vertex and path margins) |
| 2026-11-08 (answers to 9.3) | master (the remote head of 8 November, 7c81b32cd) takes the legacy fixes first, then the new branch; the old branches are deleted without tags; a bridge end that overlaps a wall is anchored ("this is why we have an overlap setting for bridges"), so no hanging bridge end over the pattern's void is accepted; K2 is gated by a count with causes, target zero; the uncovered-lines pass (R22), the combination zones (R10), the deferral of seam rails (R7) and the budgets (R8) are accepted; the worktrees `BambuStudio-bxfix`, `BambuStudio-sc` and the master worktree `BambuStudio-tyr17-release` are removed, the Codex worktree stays; the name, keys, label and default stay (R14, R16) |

### 9.2 Made in this document (for the user's review)

| Id | Decision | Why | Cost if wrong |
| --- | --- | --- | --- |
| R1 | One material function with two sides (3.3): what may print is a roof, what surely prints is support; demand, descent, sweep and verifier all read it | the old branch's three notions of material disagreed exactly where its K1 and K2 surprises were; the builder check found that narrow solid infill (on by default) prints strips a single rule would have called void | a conservative demand near solid slivers: a little mass |
| R2 | The plan predicts nothing about the bridge generator except the candidate test (for rails) and the plane count B + 1; ensuring strips and growth are handled by printing planned lines wherever nothing else prints (3.8) | removes the two clips, `through`, the predicted strips and the kind bits that drove the sweep to 1,941 lines | a planned line under a strip that prints solid is dropped, correctly; a strip that prints nothing gets the line |
| R3 | Courses (seam walls, rails) are held by the demand mechanism, never by the sweep (3.5) | one mechanism; no rail keys, no wall keys in the sweep | the band under a rail is refined one plane deeper than the rail-key design would; small mass |
| R4 | Cubic's bulk lines go through `connect_infill` as before; the pattern's strands and courses are appended unconnected (3.8) | joins only along the lattice (user); no hook can form on a planned end, which removes the old rule (i), the unconnected-column special case and their coupling to the connector's tolerance | a strand that ends at a wall is not chained to its neighbour by the connector: a few more travel moves |
| R5 | A group with no content prints through `FillCubic`; the pattern's own bulk lines include the lattice's extreme lines | parity where nothing is needed; the plan's measure assumes every face prints | the parity test tolerates the extreme-line difference where the plan has content |
| R6 | The stock bridge candidate test is extracted into a function and called by the plan | exact rail prediction from the same inputs, no duplicated logic | the one refactor of shared code; behaviour-preserving, pinned by the stock tests |
| R7 | Seam rails deferred until a fixture shows a hanging bridge end at a one-group seam | Heimdall's objects are single-region; K1 holds without them | a later task on the modifier and rib fixtures |
| R8 | Budgets of 4,000 source and 2,500 test lines, 40 cases, 150 shared lines (1.4) | the complaint that prompted the rewrite | a budget raised with a reason |
| R9 | Tests assert outputs only; the G-code observer and the K2 checker are the oracles; no hooks, no may-fail, no re-pinning (7.1) | 188 of 245 commits touched tests, 85 of them nothing else | a construction defect found later by the acceptance instead of a unit test |
| R10 | Combination through the single-layer zone and the pass (3.9a), not by subtracting the zone inside `combine_infill` | keeps `combine_infill` untouched and the surfaces Cubic's | the pass does more work on combined layers |
| R11 | Family change is a measured addition (stage 5c) with k_min = 6 and gain 1 (3.7 step 6); the base sweep has no merge steering, rerouting or restarts | about a third of the old sweep's lines were search around a small rule; the user permits direction changes but does not require them | a Heimdall run that misses the time budget without it, then built |
| R12 | The verifier measures the plan's own planes on the final paths and reports a bridge's growth beyond them as information (5.1) | the planes are the contract's; growth is not demand (user, 8 November 2026) | a grown bridge that hangs is seen by the free-end check, not by an opening |
| R13 | Strips are measured by their length L (3.6) | floating vertical shell rings ran 9–23 mm on the user's parts under a hull-only measure | a few more splits under curved roofs |
| R14 | The name stays "Bridge aware cubic" and the key `bridge_aware_cubic_max_gap` | the user's message wrote "bridge adaptive cubic" once; every setting, document and tracker uses the existing name | a rename before stage 1 |
| R15 | The benchmark tooling, fixture projects, procedure document and acceptance scripts are carried over verbatim as the first commit | measurement tooling, validated against official baselines; not the pattern | none |
| R16 | The option's static bounds are 0.01–1000 mm; the runtime clamp is to the nozzle diameter and the option maximum | the GUI needs static bounds; the physical floor depends on the extruder | none |
| R17 | Roof support uses the perimeter generator's overhang distance, half the nozzle diameter, applied to the material that surely prints below (3.4, 5.1); the old branch used half the widest bead | the same distance the generator uses for walls, and a hair more conservative where beads are wider than the nozzle | 0.05 mm more demand along roof edges at Heimdall's 0.5 mm beads |
| R18 | The rail lies on the layer whose preview anchors the lowest bridge base, plane j = B−1 (3.5b); the old design put it at n−1 for every B | at 2 or 3 internal bridge layers the stock step rebuilds and re-anchors the base at n−B and trims the bridge at n to it | none at B = 1, which the user's parts use; the fixture variants at B = 2 and 3 would otherwise fail R1 |
| R19 | K1(a) measures openings under the roof only, within r/2 of the footprint, in the plan and in the verifier (3.6, 5.1) | a roof bead runs unsupported only inside the footprint; the whole void component would fail every roof whose edge crosses a coarse cell without a course | the observer's run check (K1(b)) still catches any bead that does run beyond |
| R20 | K2 acceptance on the user's parts: the target is zero; the stage gate is a count (≤ 86 at 5a, the old branch's figure before family changes) with every unheld piece listed by cause; one hold rule in the suite and the checker | the release prints unheld where the fill boundary leaves no room for any holder, so a count with causes is the honest gate; two rules would disagree on the same plate. This relaxes the user's rule for those places; the user accepted it on 8 November 2026 | a plate accepted with places a stricter rule would list |
| R21 | Stage 5c (family change) is built when Heimdall run 1's time is above +12% over Cubic | a numeric trigger instead of "deep columns"; +12% leaves the rails and later stages room under +17.8% | a few tasks spent on an optimisation the budget did not need |
| R22 | The uncovered-lines pass (3.8) prints, after every filler, the planned lines that a later stock step took where nothing else prints | the plan precedes the generator by the user's order of operations and the generator is not touched, so emission must finish after it; the old branch did this for seam walls only (its decision 13) | the one addition to the release slice beyond plan and filler; accepted by the user on 8 November 2026 |
| R23 | Course ends are sweep items: a seam wall's or rail's end over the void gets a holder (3.5, 3.7) | demand holds a course's length but not its end; K2 forbids a free end | a few pieces per course end |
| R24 | The K2 checker and the suite judge only the pattern's added lines, told apart from Cubic's bulk lines against the Cubic slice (5.3, test K2) | K2 excludes Cubic's own lines and their connector hooks | an added line misclassified as bulk escapes the check; the Cubic control makes that rare |

### 9.3 Open questions for the user

All answered on 8 November 2026 and recorded in 9.1: the base branch (master with the legacy fixes merged first), the old branches (deleted without tags), bridge ends at walls (anchored by the bridge's wall overlap; the hanging-end target is zero), the K2 gate (a count with causes, target zero), the uncovered-lines pass, combination through the single-layer zones, seam rails deferred, the budgets, the worktrees to remove (`BambuStudio-bxfix`, `BambuStudio-sc` and the master worktree), and the name, keys, label and default (kept).

## 10. Review of the branch this replaces

### 10.1 What is there

`master..ef9c17704`: 245 commits between 5 and 8 November 2026. Fourteen source units of 6,637 lines (about 4,940 of code): the sweep 1,941 + 154, the plan 1,415 + 549, the verifier 907 + 253, the lattice 297 + 139, strands 193 + 146, the sweep input 249 + 93, the filler 178 + 123. Fourteen test files of 15,831 lines with 261 cases. 2,748 lines of benchmark tooling and its tests, with the fixture projects. About 400 lines of change in shared files (`Fill.cpp` +199, `PrintObject.cpp` +63, `Print.cpp` +15, `PrintConfig.*` +39, GUI +14, CMake +20, `LayerRegion.cpp` 6). The reports behind this section are `review-source.md`, `review-tests.md` (with `review-tests-cases.tsv`) and `master-map.md` in `.superpowers/sdd/2026-11-08-bac-rewrite/`.

### 10.2 What the history shows

- 188 of the 245 commits touch the tests; 85 of those change no source. By my own count 80 commits touched only BAC tests, 91 tests and source, 26 source only, 48 other files.
- Tests: 36,282 lines written, 20,451 deleted, 15,831 kept: 3.58 lines of churn per line kept. Sources: 27,978 written, 21,341 deleted, 6,637 kept: 7.4. The feature was redesigned four times (search, closed cells, open cells, the forest of columns, one plan before the bridges) and each redesign re-derived the tests that pinned the previous internals; 14,533 test lines went with the search-based design alone.
- Of 30 sampled commits, 21 mainly re-pinned, fixed or deleted tests.
- 166 of the 261 cases (6,312 of the 9,640 lines of case bodies) pin internals: plan, strand, column and cell APIs, kind bits, test switches, digests, message texts, counts. 58 cases assert the contract, and 29 of those read it through the pattern's own verifier or plan. 27 gating tests check the contract through the output.
- 17 cases expect a contract violation: pinned failure counts such as {0, 3, 5} for the partial roof under three bridge layers, ceilings, `CHECK_NOFAIL`, a predicate that lets planes with known openings pass. The literal K1 and K2 assertions that fail are `[!mayfail]`.
- In the sources, about 2,350 lines carry the contract; about 750 are search around the sweep's small hold rule (landing estimates, merge steering, a three-layer look-ahead, rerouting, a loss-restart loop, family changes with pruning); about 560 work around the pipeline order or earlier choices (about 200 copy and predict the stock bridge generator, 140 hold three different clips, 95 reprint walls the final surfaces took, plus predicted material, a roof-perimeter patch and two rules that defeat the connector); about 110 are memory plumbing for a target that does not exist; about 1,560 are diagnostics and test hooks; about 160 are dead or duplicated (the crossing algebra derived eight times, four thresholds for "a bead long", √2 and √3 defined in four files).
- Three notions of material (surface types; the fillers' output opened by the spacing; extrusion footprints) were in use at once, and the K1 and K2 surprises the branch recorded (openings beside unprinted strips, seam corridors, a 9.6 mm bridge end) all involved material that printed differently from how one of them counted it.

### 10.3 What this design keeps from it

As ideas and formulas, re-implemented: the closed-form lattice with FillCubic's sign pattern and float-truncated shift; plane keys that never alias; the demand rule (with the distance of R17 and the two sides of R1); the section bound with the strip rule (with the length measure of R13) and saturation; needed strands at failing planes only; the solid cells and their course grid; the seam-wall rules; the hold primitives α, c and material within w/2; columns that step by α with steering; the family-change closed forms; binding the anchoring preview to the plan. As code, carried over verbatim: the export observer and the other test helpers of 7.2, the K2 checker and the 16-plate scripts, the benchmark tooling with its fixture projects and procedure.

### 10.4 What it drops

- Predicting the bridge generator (the ensuring strips, the bridged classification, `through`): printing planned lines wherever nothing else prints replaces it (R2), and the one stock test the rails need is called, not copied (R6).
- Three clips: one (3.8).
- The roof-perimeter patch: the strip-length measure covers it.
- Rule (i) on strand ends and the unconnected column pieces: planned lines bypass the connector (R4).
- The sweep's search: merge steering, rerouting, the loss-restart loop. Family change becomes a measured stage.
- The kind bits `skUnheld` and `skPrinted`, the test hooks `allow_family_changes`, `allow_family_change_pruning`, `emit_intervals`, `keep_plan_details`, `PackedStrand` and the detail dropping, 23 statistics counters (the unheld list and the counts of 5.2 stay, under the flag).
- The `combine_infill` skip, the multiline refusal and the two validation blocks (settings are honoured instead); the `bridge_aware_group` fill key and the split of `group_fills` (hooks 8a–d instead).
- The 166 implementation-pinning cases, the verifier's synthetic builders, `holds_demand`, and every case that expects a contract failure.
- Its inconsistencies: miter offsets where its hold tolerance said round; the group's smallest gap used for every region in the sweep (now each plane's); "the bulk line prints" judged by three point probes (now by the boundary); four thresholds for "a bead long" (now one).

### 10.5 Shared code, old and new

| File | Old branch | This design |
| --- | --- | --- |
| `Fill/Fill.cpp` | +199: a fill key, `group_fills` split in two, three helpers, binding, the wall reprint pass | ≈ 40: `group_fills` exposed, the binding, the enum case, one pass call, the multiline list |
| `PrintObject.cpp` | +63: plan build, development checks, the `combine_infill` skip, invalidation, the modifier key | ≈ 30, plus the candidate test moved into a function |
| `Print.cpp` | +15: validation errors | 2: the plan reset after a cache load |
| `PrintConfig.cpp` / `.hpp` | +39: option, enum, a validation block, a validity helper | ≈ 18: option and enum |
| GUI | +14 | ≈ 8 |
| `LayerRegion.cpp` | 6: a copy of the config made a reference (unrelated) | 0; belongs on the legacy branch if wanted |
| `GCode.cpp`, `GCodeProcessor.cpp` | 30: fixes | already on `codex/legacy-test-fixes` |

### 10.6 Preserved outside the branch

`codex/legacy-test-fixes` (103e38608 on 8fddaaa51) holds the unrelated fixes found while qualifying the pattern (libnest2d placers, `Config.hpp`, `FilamentGroup.cpp`, Lightning `TreeNode`, the `GCode.cpp` null check, the `GCodeProcessor` Z parsing, `ModelArrange`, `PlaceholderParser`, `Print.hpp`'s origin, `PrintConfig.cpp`) and the legacy test alignments; the feature branch carries only the two G-code fixes of that set (identical on both branches) and a 6-line change in `LayerRegion.cpp` (e7f1fbce3 moved the rest). The 16-plate scripts were copied out of the session's ephemeral scratchpad into `.superpowers/sdd/2026-11-08-bac-rewrite/tooling/`, and the two local Heimdall projects into `/Users/jforbes/Documents/Development.nosync/BambuStudio-bac-data/` (the shared drive stays read-only). The Cubic baselines (G-code of the 16 plates) are regenerated by the scripts on the new branch. The new branch is `codex/bridge-aware-cubic-v2`, from master at dbf8c36be.

## Appendix A. Formulas

- Lattice line of family k at height z: n_k · p + σ_k z/√2 = m L_N. Drift per layer: s = h/√2 along n_k.
- Cube coordinates: u_k = √(2/3)(n_k · p) + σ_k z/√3; z = Σ_k σ_k u_k/√3; a_0 = L_N √(2/3); a_δ = a_0/2^δ.
- Section of a cell at z: hull of the intersections of the plane with the cube's 12 edges. Bound: diam(P) − W_N.
- Strip length: L(X) = per(X)/2 − 4 area(X)/per(X); for an l × t rectangle, (l² + t²)/(l + t).
- Hold: own plane |Δt| ≤ α = √((w/2)² − s²); crossing |Δt| ≤ c = (w/2)/sin 60° = w/√3; material within w/2.
- Band of N: W_N = (N − 1)s_f + w; offsets o_i = (i − (N − 1)/2) s_f; c_N = c + (N − 1)s_f/(2√3); λ_sw(N) = λ + (N − 1)s_f/sin 60°.
- Saturation: a cell at depth δ with d · 2^(δ+1) ≥ 1 (d the density fraction) does not split.
- Rail offset: d_r = s_b + w/2 + 0.05; growth beyond the roof's edge after a rail: at most d_r + w_b.
