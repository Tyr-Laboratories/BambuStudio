#!/bin/bash
HERE=$(cd "$(dirname "$0")" && pwd); TOOLING=$(dirname "$HERE")
BAC_DATA=${BAC_DATA:-/Users/jforbes/Documents/Development.nosync/BambuStudio-bac-data}
BAC_PYTHON=${BAC_PYTHON:-python3}
BAC_WORK=${BAC_WORK:-$BAC_DATA/work}
cd "$(dirname "$0")"
PY=../bacvenv/bin/python3
H=../batchM-spike/heimdall
for p in 1 2 3 4 5; do
  $PY k2check.py $H/timing/open/p$p/output/plate_$p.gcode $H/timing/cubic/p$p/output/plate_$p.gcode 5 k2/open-p$p.tsv > k2/open-p$p.out 2>&1 &
done
$PY k2check.py $H/timing/cubic/p5/output/plate_5.gcode $H/timing/cubic/p5/output/plate_5.gcode 5 k2/cubic-p5.tsv > k2/cubic-p5.out 2>&1 &
$PY k2check.py $H/timing/base/p5/output/plate_5.gcode $H/timing/cubic/p5/output/plate_5.gcode 5 k2/base-p5.tsv > k2/base-p5.out 2>&1 &
wait
echo all-done > k2/done.txt
