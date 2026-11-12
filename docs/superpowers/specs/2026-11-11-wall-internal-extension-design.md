# Wall internal extension

## Intent

The user prints very light aircraft parts (Heimdall 2: single-wall PA612-CF, 0.2 mm layers, 0.45 mm outer wall, 4 top and 4 bottom layers, 3% cubic sparse infill). Where an outer wall meets a flat top or bottom surface at about 90°, the joint is weak. The wall bonds to the surface through a single layer interface at the corner, and light pressure splits the part along that seam. The user's photo shows an orange outer wall rising out of a red top surface; the split runs exactly where they meet.

The fix is a woodworking joint. The wall is started before it breaks out onto the surface: it continues through the surface's shell and further into the part, like a tenon in a mortise. A 45° chamfer fills the corner between that buried wall and the inside face of the shell. Together they move the stress concentration from the exterior 90° corner to a point inside the part that cannot be pressed directly.

## Terms

- **Top joint**: an outer wall rising out of a flat top surface (the part is wider below the joint). The top surface is the topmost layer of the wider part beside the wall.
- **Bottom joint**: the mirror case. An outer wall ends under a flat bottom surface (the part is wider above the joint, which starts with a bridge or overhang beside the wall).
- **Joint line**: the outer wall line at the joint layer (the first wall layer above a top surface, or the last wall layer below a bottom surface).
- **Shell depth**: the region's top (or bottom) shell depth: `top_shell_layers` layers or `top_shell_thickness`, whichever is deeper, as the slicer already computes it (`bottom_*` for bottom joints). It is measured by print height, so variable layer heights work.
- **E**: the value of the new setting (default 1 mm).

## Decided by the user

1. **New setting "wall internal extension"**, default **1 mm**.
2. **Depth is measured beyond the shell (option B).** The extension goes through the whole shell and then E further: 0.8 + 1.0 = 1.8 mm with the Heimdall settings. Measuring 1 mm from the surface would leave only one 0.2 mm layer below the shell for the chamfer.
3. **Only the outer wall is extended** ("orange lines (the outer wall)"). Inner walls on 2–3 wall objects are unchanged.
4. **45° chamfer** on the surface side of the extension, in the corner between the extension and the inside face of the shell. For top joints it hangs under the top shell, widening going up (an inverted chamfer). For bottom joints it sits on top of the bottom shell, narrowing going up. It is E wide where it meets the shell.
5. **Both top and bottom joints** are covered by the same setting.
6. **Which joints qualify** (approved after the user reviewed six example joints):
   - (a) The joint line stands entirely on the surface. For a top joint, it does not overlap the wall lines of the layer below; it sits over that layer's infill or solid fill. For a bottom joint, the layer above has no wall line over it.
   - (b) Moving away from the surface, the wall stays strictly steeper than 45° for at least E, leaning either way: at every height up to E above (or below) the joint, it has moved sideways less than that height. That means upward from a top joint and downward from a bottom joint. A wall at exactly 45° is left alone *(my call, settling a contradiction found by the builder check)*. The test is a 45° cone measured from the joint, not a check of each layer against the one below *(ruling during implementation)*: a short vertical lip that continues as a 30–40° ramp still qualifies. The comparison allows for the G-code `resolution`, the only setting that moves straight walls between layers.

   Extended: vertical walls; walls leaning in or out by less than 45° from vertical; pockets (a hole's walls rising from the pocket floor); thin fins printed as one or two lines.
   Left alone: 45° ramps; gently sloped skins where every layer steps in (each layer's wall stands on the surface, but nothing rises straight up); lips and raised text shorter than E.
   Pieces of a joint line shorter than one outer-wall line width are dropped *(my call)*.
7. **One new preview line type covers both the extension lines and the chamfer lines.** It has its own colour, visibility toggle, and time and filament statistics in the preview legend, like the existing line types.
8. **The lowest extension layer under a top surface is printed like an internal bridge.** At that layer the extension and chamfer lines have only sparse infill below them; at 3% cubic, the infill lines are up to ~46 mm apart. So the lines there use bridge flow, speed, acceleration and fan, the way internal bridges over sparse infill are printed. They still show as "Wall extension". Every line that stands on a wall or extension line below prints normally. This is the case for all bottom-joint extension layers and all top-joint layers above the lowest.
9. **Walls rising out of curved or sloped skins count too.** A rib or fin standing on a curved fuselage or wing skin is extended piece by piece, on every layer where part of its outer wall line stands on the skin and the wall rises steeper than 45°. Material is added only along such wall bases, never under bare skin. There is no separate flatness test. **Limit** *(ruling during implementation)*: where the skin slopes more than about 15° along the rib, each layer's new piece of the rib's wall is shorter than the wall corner below it. It therefore rests mostly on that wall, does not stand on the surface, and is not extended.

## Geometry

### Extension (top joint)

```
             upper part (inside)
            |W|                     W  outer wall line
            |W|                     =  top surface and top shell, 0.8 mm (4 x 0.2)
 -----------|W|==================   X  extension: copies of the joint line
            |X|==================   C  chamfer lines, 45°
            |X|==================
            |X|==================
            |X|C C C C C   <- E wide just under the shell
            |X|C C C C
            |X|C C C
            |X|C C
            |X|C           <- extension ends E below the shell
              lower part (inside)
```

- Copies of the qualifying part of the joint line go on every layer from the surface layer down to (shell depth + E) below the surface, at the same XY position.
- On each layer a copy is kept only where it is inside the part. It is clipped:
  - to that layer's slice;
  - so it does not overlap that layer's own wall lines (it stops where it meets them);
  - so it is never printed where that spot is an exposed top or bottom surface, or would hang over air.

  If the wider part is thinner than the full depth, the extension stops early. The extension never changes the outside of the part.
- Because the joint line sits just inside the upper part's outline, the copies run beside the top shell, not through it. On every layer, the shell's lines butt against them.

### Extension (bottom joint)

The mirror image. Copies go on every layer from the first layer above the joint (the bridge or overhang layer) up to (bottom shell depth + E) above the surface. The same clipping applies; in particular, a copy is never printed where it would show on a top surface. Every copy stands on the line below it.

```
             upper part (inside)
            |X|C           <- extension ends E above the shell
            |X|C C
            |X|C C C
            |X|C C C C
            |X|C C C C C   <- E wide just above the shell
            |X|==================   bottom shell, 0.8 mm
            |X|==================
            |X|==================
            |X|==================   bridge / overhang layer (over air)
 -----------|W|
            |W|   lower part
```

### Chamfer

- The chamfer sits on the surface side of each extended line: the side where the top or bottom surface is. A pocket gets it on the pocket side; a thin fin with surface on both sides gets one on each side.
- It occupies the layers between the shell and the end of the extension. Its width, measured from the extension line, grows by one layer height per layer of distance from the extension's end. It is 0 at the end and E next to the shell.
- It is printed as lines parallel to the extension line, adding lines as the band widens. In a top joint each chamfer layer overhangs the one below it. No chamfer line may overhang the line below it by more than half its width, which a 45° chamfer at the normal layer height and line width satisfies.
- It is clipped like the extension: inside the part, not over wall lines, never on an exposed surface.

### Everything else on those layers

- Fill on those layers is cut back around the extension and chamfer lines. This covers sparse infill, solid infill, shell, bridge and top/bottom surface fill. The fill meets these lines at normal line spacing, the way neighbouring solid-infill lines meet, with no extra overlap. *(My call, changed while planning.)* A wall has fill on one side only; the extension has fill on both sides. With the Heimdall preset's 75% `infill_wall_overlap` applied from both sides, the fill would cover a one-line extension almost completely and print over it.
- Where extensions or chamfers from different joints overlap, the union is printed once.
- **Different regions** (separate parts, modifiers, periodic modifiers, different wall filaments) *(my call)*:
  - The joint's region (the wall's region at the joint layer) decides whether there is a joint, and its E and shell depth.
  - On each target layer, lines are printed only where the region at that spot also has E above 0.
  - They are printed by that region, with its wall filament, inner-wall and bridge settings and outer-wall line width. That way no tool change is added.
  - So E = 0 on a part or modifier is an off switch for that volume: no extension starts from its walls, and none is printed inside it.
- **No other change.** The cut-outs must not trigger extra solid infill anywhere. The specific trap is "ensure vertical shell thickness" (on by default): it would treat the cut-out as a wall and add solid infill around and under it (`discover_vertical_shells`). Bridges in a bottom joint anchor against the extension line the way bridges anchor against walls.
- What sits there today: under a rising wall, "ensure vertical shell thickness" can turn the footprint of the wall above into solid infill for up to three layers. Its regularisation step deletes regions narrower than about 0.65 of a solid line, so whether that band survives depends on the infill-wall overlap; this is unverified for the Heimdall preset. Wherever such a band exists, the extension takes its place. The added material is otherwise the extension lines plus the chamfer.

## Line type and printing

- **Name:** "Wall extension" *(my call)*. It is a new extrusion role, carried through the following places, so the G-code reads and displays like the existing types:
  - G-code export and its feature tag;
  - the G-code processor's per-type time and filament statistics;
  - the preview legend (name, colour, visibility toggle).
- **Colour:** aqua, distinct from the existing legend colours *(my call; easy to change after review)*.
- **How it prints:** at the inner wall's speed, acceleration and cooling rules. Both extension and chamfer lines use the outer wall's line width, so the extension lines up under the wall above. *(My call; no new speed settings.)* The exception is lines with nothing but sparse infill below them (decision 8). They print with bridge flow, speed, acceleration and fan.
- **Order:** printed with the layer's walls, before that layer's infill *(my call)*, following the inner walls under the wall-sequence settings.
- **Compatibility:** older apps, including the user's installed Tyr.25, do not know the new type. Reviewing a sliced file needs the app from this branch, opened against the isolated review data folder (`build/bac-review-data`) so the production install and cloud-synced presets are untouched.

## The setting

- Key `wall_internal_extension`, label "Wall internal extension", millimetres, range 0–10 *(range is my call)*; **0 turns the feature off**.
- Tooltip in plain words: it starts outer walls that rise out of (or end under) flat surfaces inside the part, through the shell and this far beyond it, with a 45° chamfer, to strengthen the joint.
- A per-region setting (`PrintRegionConfig`), like `wall_loops`: in the process preset under **Strength → Walls**, and overridable per object, part and modifier. It is saved in presets and 3MF projects. Regions that differ only in this setting still share one perimeter group, so a modifier that only changes it adds no walls at its border. The extension assigns joint lines to regions by the regions' slices *(changed after review)*.
- Disabled in spiral vase mode *(my call, as for other wall features)*.
- **Consequence of the 1 mm default (the user's decision):** every new and existing project gets the extension when sliced, because projects saved without the key load the default.

## Goals

- Joints like the one in the photo no longer split along the exterior corner under light pressure. Physical confirmation is the user's print test.
- One setting covers top and bottom joints; 0 reproduces today's output.
- The added material is visible and measurable in the preview as its own line type.

## Non-goals

- Inner walls, gently sloped skins, 45° ramps and short lips (see decision 6).
- A separate chamfer toggle or chamfer size; the chamfer is always E.
- New speed, width or cooling settings for the new line type.
- Other weak joints, such as where a side wall meets its own top skin at the part's outer edge.

## Anti-goals (a failure even if it technically works)

- Any visible change to the part's outside: top, bottom or side surfaces.
- Material added where no qualifying joint is, especially under curved skins (wing surfaces).
- Extra solid infill triggered by the cut-outs.
- Any change to output when the setting is 0.
- New slicing errors, warnings that stop slicing, fallbacks or after-the-fact repair passes. Fix the construction instead.
- Tolerances invented as constants. Margins come from the print's own settings: line widths, layer height, `resolution`, `slice_closing_radius`.
- Breaking Bridge Aware Cubic or the fork's other wall features. These are wall painting, slope walls (`extra_wall_slope_angle`), bridge controls and periodic modifiers.

## Acceptance

1. **Focused tests** in a new test target, using fixtures:
   - a tower on a box (top joint), as in `terrace_model` in `test_bridge_aware_cubic.cpp`;
   - a pocket in a block;
   - an overhanging block (bottom joint);
   - a thin fin printed as one line and as two lines;
   - a 45° ramp, a gentle slope and a short lip (none extended);
   - a lower part thinner than shell + E (clipped);
   - a two-wall object (only the outer wall extended).

   The checks:
   - lines on the expected layers and depths;
   - chamfer width per layer;
   - nothing outside the part;
   - no added solid infill;
   - the new type in G-code with statistics;
   - Classic and Arachne;
   - setting 0 changes nothing.

   Keep the test set lean.
2. **Setting 0 is identical to master.** G-code from all 16 Heimdall plates and the fixtures must match master, apart from the line in the G-code's config listing for the new key.
3. **The 16 Heimdall plates at the default 1 mm** must slice without errors. Report per plate, against master:
   - grams by line type, including "Wall extension";
   - estimated print time;
   - path and retraction counts;
   - slicing cost in instructions retired.

   These numbers are reported, not gated: no mass or time budget has been set.
4. **Look at real toolpaths before handing over.** Open Heimdall plate 1 and the fixture coupon in this branch's app with the new type shown. Inspect a top joint and a bottom joint layer by layer and describe what is printed in plain words. Then hand the user `.gcode.3mf` files for their own review.
5. **Physical test (user):** a printable coupon with top and bottom joints, sliced at 0 and at 1 mm, for a press test.

## Left to the builder

- Where in the pipeline this runs. The code map points to a sequential pre-pass in `PrintObject::make_perimeters` (`PrintObject.cpp`), next to the wall-painting pre-pass: during the parallel perimeter loop, other layers' slices are safe to read, but other layers' perimeters and fill are not.
- How joint lines are found (from slices or from the generated walls), for both Classic and Arachne, including thin walls.
- The precise geometric tests for decision 6, with tolerances derived from the settings listed above.
- How the cut-outs are applied consistently to `fill_expolygons`, `fill_no_overlap_expolygons` and `bridge_fill_expolygons`, and how vertical-shell logic is kept from reacting.
- Start points of closed extension loops, travel and retraction order, and chamfer line layout and rounding within the half-width rule.
- Role plumbing. Templates: `erFloatingVerticalShell` for a role added after the original set; `extra_wall_slope_angle` (commit `66a45f8f7`) for a per-region wall setting with GUI, presets, invalidation and tests.
- Updating `docs/tyr-fork.md` with a user-facing section.
