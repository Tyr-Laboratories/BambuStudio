#!/bin/bash
# Controller's scripted measurement of one app on all 16 Heimdall plates: grams, estimated time, paths, retractions, instructions, RSS
# (bxfix/acc/batch.sh), K2 on plates 1-5 and the bridge-line checks on every plate (bxfix/acc/analyse2.sh), tables (bxfix/tools/table.py).
# Usage: measure.sh LABEL /path/to/BambuStudio.app   -> results in measure/LABEL (G-code deleted after the analyses).
HERE=$(cd "$(dirname "$0")" && pwd); TOOLING=$(dirname "$HERE")
BAC_DATA=${BAC_DATA:-/Users/jforbes/Documents/Development.nosync/BambuStudio-bac-data}
BAC_PYTHON=${BAC_PYTHON:-python3}
BAC_WORK=${BAC_WORK:-$BAC_DATA/work}
set -u
SP=$BAC_WORK
L=$1; APPSRC=$2
V=$BAC_WORK/measure/$L
rm -rf "${V:?}"; mkdir -p "$V/app"
ditto "$APPSRC" "$V/app/BambuStudio.app"
echo "$(date '+%F %T') start $L from $APPSRC" > "$V/log.txt"
nice -n 5 bash "$TOOLING/acc/batch.sh" "$V" >> "$V/log.txt" 2>&1
echo "$(date '+%F %T') slices done" >> "$V/log.txt"
bash "$TOOLING/acc/analyse2.sh" "$V" >> "$V/log.txt" 2>&1
echo "$(date '+%F %T') analyses done" >> "$V/log.txt"
"$BAC_PYTHON" "$TOOLING/tools/table.py" "$V/summary.txt" > "$V/table.md" 2>&1
rm -f "$V"/p*/output/*.gcode
echo "$(date '+%F %T') done" >> "$V/log.txt"
