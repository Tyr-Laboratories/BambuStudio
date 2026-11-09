#!/bin/bash
HERE=$(cd "$(dirname "$0")" && pwd); TOOLING=$(dirname "$HERE")
BAC_DATA=${BAC_DATA:-/Users/jforbes/Documents/Development.nosync/BambuStudio-bac-data}
BAC_PYTHON=${BAC_PYTHON:-python3}
BAC_WORK=${BAC_WORK:-$BAC_DATA/work}
D=$BAC_WORK
export PY=$D/bacvenv/bin/python3
export H=$D/batchM-spike/heimdall
export C=$D/checkM
run() { v=$1; p=$2; $PY $C/runcheck.py $H/timing/$v/p$p/output/plate_$p.gcode 5 $C/runs/$v-p$p.tsv > $C/runs/$v-p$p.out 2>&1; }
export -f run
for p in 1 2 3 4 5; do for v in open base cubic; do echo "$v $p"; done; done | xargs -P 8 -L 1 bash -c 'run $0 $1'
echo all-done > $C/runs/done.txt
