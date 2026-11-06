# Bridge aware cubic: proof measurements

This experiment is based on the feature at `8dbea843e`. It observes the existing verifier and runs additional checks under a separate bounded diagnostic budget. It does not use their outcomes to accept a slice or remove a cell. Diagnostic wall time is not the official speed measurement.

Enable `BAC_PROFILE_FILE` and `BAC_PROFILE_SHADOW_PROOFS=1` to write per-stage counters. Run the `[ProofShadow]` controls before interpreting them. `BAC_TRACE_FINAL_PROOF_MISS=1` reports the first failing typed comparison when a stored final certificate is available. It does not report every changed field or prove that a changed value is harmless.

| Fixture | Total collapse trials | First-sweep direct paths: empty / nonempty | Time in empty / nonempty paths | Line-only span passes / failures |
| --- | ---: | ---: | ---: | ---: |
| Small cube | 159 | 2,445 / 160 | 0.524 / 1.997 s | 41 / 51,357 |
| Large cube | 543 | 6,726 / 894 | 3.474 / 39.980 s | 69 / 356,128 |
| Rib | 302 | 2,924 / 637 | 1.203 / 4.913 s | 2,717 / 105,306 |
| Modifier | 957 | 15,731 / 655 | 3.894 / 2.653 s | 7,049 / 340,883 |

Rows count visits, not unique paths. The direct columns include only observed valid, completed decisions; unobserved suffixes after early rejection and cache hits are not empty proofs. The line-only control retains actual line anchors and omits all material masks for eligible span checks. None of its positives disagreed with the full reference. The low pass rate does not support a general line-only verifier. A complete empty direct selection was always a pass, but its measured time bounds the opportunity; many inexpensive empty paths do not explain the large-cube slowdown.

The large cube spent 113.87 s in coarsening out of 144.32 s in preparation. Physical-cell work nested in the nonempty direct checks accounted for 17.82 s in its first sweep and 7.69 s in later sweeps. The modifier spent 44.00 s collecting construction targets and 25.84 s checking them; closure layers declined emission capture 29,194 times. An optimization limited to ordinary sparse layers cannot claim to cover those closures.

The first late certificate mismatch on all four fixtures was `PrintConfig.filament_nozzle_map[0]`, changing from 1 to 0. An optimization must still prove the resolved output policy or separately verify the policy-dependent work; dropping this option from equality is not justified.

The controls passed 822 assertions in 26 cases. The small and large qualifications passed 34,106 and 79,422 assertions. On the rib and modifier, all 16,633 and 36,666 ordered, non-comment G-code commands respectively were identical to the corresponding production output.
