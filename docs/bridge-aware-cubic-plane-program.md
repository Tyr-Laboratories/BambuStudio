# Canonical refinement strands: experimental program

The experimental program models an added cube face on a particular layer as finite intervals with stable endpoint identities. Each interval retains the cell that produces it. Removing or restoring that owner changes at most three plane rows. Contiguous active intervals form a run; a real inactive interval remains a break even when its endpoints round to the same XY point. Ordinary bulk Cubic is separate.

The raw program is not used for production emission. It does not yet establish clipping, path attachment, physical support, or end-to-end speed.

The four focused cases passed 4,079 assertions, including rollback, cancellation, negative addresses, an exact corner tie and a positive interval that rounds to a stationary edge. The small- and large-Plan diagnostics sampled 12 actual retained split IDs across 90 layers. Every incremental removal and restoration matched a fresh canonical rebuild, including ordered integer points and owners.

| Quantity across sampled removals | Small cube | Large cube |
| --- | ---: | ---: |
| Rows touched / baseline row visits | 2,154 / 20,616 | 1,594 / 38,256 |
| Face intervals removed / baseline interval visits | 2,175 / 66,960 | 1,689 / 219,636 |
| Logical update work / fresh-build work | 163,017 / 4,251,411 | 174,813 / 14,874,609 |
| Assertions | 533,521 | 1,446,102 |

Logical work counts are not elapsed time or predicted slicing speedups. The baseline scan still constructs the program per elevation. Across 90 unmodified layers the outer axes matched the legacy broad reference in both geometry and order (3,590 small; 11,433 large). Of 1,990 / 6,870 interior vertices, 1,575 / 5,771 lay off the corresponding rounded straight chord. A material clip must therefore operate on the actual canonical edges. Outer-endpoint equality alone is insufficient to replace the legacy generator.

## Clipping to the actual fill domain

The diagnostic captures each filler call's overlap-adjusted surface and actual inset material, then compares six sampled removals against freshly rebuilt canonical fragments. It compares exact crossing events, material breaks, rounded points and owners. A shared observer lease keeps concurrent fill calls safe during observer removal. The production emitter still supplies the extrusion paths in these experiments.

| Measured across six removals | Small cube | Large cube | Blind cavity |
| --- | ---: | ---: | ---: |
| Actual fill components | 84 | 84 | 64 |
| Compared component trials | 504 | 504 | 384 |
| Unavailable component trials | 0 | 0 | 0 |
| Changed component trials | 186 | 164 | 123 |
| Removed fragments / baseline fragment visits | 916 / 23,376 | 1,024 / 90,168 | 500 / 11,820 |
| Changed rows / baseline row visits | 862 / 7,560 | 821 / 15,534 | 417 / 4,452 |

All 1,392 incremental comparisons matched fresh geometry. The cavity exercised 40 holes in inset components and 87 exact gaps. Ten focused raw, clipping and lease cases passed 4,247 assertions. The comparison establishes the clipping and ownership behavior of these samples. G-code deposition and the end-to-end timing gate require separate qualification for a generator that consumes these fragments.

## First emitted roof

An opt-in, single-region generator now emits clipped partition runs as sparse paths. Each moving edge retains its source cell. Generated endpoints, including repaired terminals, participate in ordinary construction, coarsening and final support checks. This first experiment regenerates whole layers so that later incremental-generation changes can be compared with a complete reference.

The 32 × 28 × 18 mm cube with a 5 mm gap produced 2,626 actual partition paths from 3,896 source atoms and seven attachment records. The exported roof passed the existing contact, closed-opening, unsupported-run and E-per-moving-distance checks (34,777 assertions). The no-generator control failed at the required generator-selection check. This result covers one placement, layer height and relative-E mode. Expanding the experiment requires independent checks for orphan attachment records, cross-role duplicate material and post-rounding containment.
