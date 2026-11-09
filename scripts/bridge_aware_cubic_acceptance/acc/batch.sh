#!/bin/bash
# All 16 Heimdall plates with the app in VARIANT_DIR, four at a time (each takes up to about 7 GB).
HERE=$(cd "$(dirname "$0")" && pwd); TOOLING=$(dirname "$HERE")
BAC_DATA=${BAC_DATA:-/Users/jforbes/Documents/Development.nosync/BambuStudio-bac-data}
BAC_PYTHON=${BAC_PYTHON:-python3}
BAC_WORK=${BAC_WORK:-$BAC_DATA/work}
V=$1
RUN=$TOOLING/acc/run.sh
rm -f "$V/summary.txt"
for group in "1 2 3 4" "5 6 7 8" "9 10 11 12" "13 14 15 16"; do
  for p in $group; do "$RUN" "$V" $p & done
  wait
done
echo "all done" >> "$V/summary.txt"
