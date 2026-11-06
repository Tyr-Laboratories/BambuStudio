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
