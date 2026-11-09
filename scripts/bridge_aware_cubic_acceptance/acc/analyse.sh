#!/bin/bash
# Batch BX-fix: per-plate analyses of the acceptance runs in VARIANT_DIR (default acc/final): K2 on plates 1-5 (k2check, causes, onbulk,
# against M-2a's Cubic G-code, gap 5) and the bridge-line checks on every plate against Cubic's G-code of investigation BX.
HERE=$(cd "$(dirname "$0")" && pwd); TOOLING=$(dirname "$HERE")
BAC_DATA=${BAC_DATA:-/Users/jforbes/Documents/Development.nosync/BambuStudio-bac-data}
BAC_PYTHON=${BAC_PYTHON:-python3}
BAC_WORK=${BAC_WORK:-$BAC_DATA/work}
SP=$BAC_WORK
V=${1:-$TOOLING/acc/final}
PY=$BAC_PYTHON
K=$TOOLING/k2
mkdir -p "$V/k2" "$V/bridges"
for p in 1 2 3 4 5; do
  g="$V/p$p/output/plate_$p.gcode"; c="$BAC_DATA/baselines/cubic/p$p/output/plate_$p.gcode"
  [ -f "$g" ] || continue
  [ -f "$V/k2/bac-p$p.out" ] || $PY -I "$K/k2check.py" "$g" "$c" 5 "$V/k2/bac-p$p.tsv" > "$V/k2/bac-p$p.out" 2>&1
  [ -f "$V/k2/bac-p$p-causes.tsv" ] || $PY -I "$K/causes.py" "$g" "$V/k2/bac-p$p.tsv" "$V/k2/bac-p$p-causes.tsv" > "$V/k2/bac-p$p-causes.out" 2>&1
  [ -f "$V/k2/bac-p$p-lines.tsv" ] || $PY -I "$K/onbulk.py" "$g" "$c" "$V/k2/bac-p$p-causes.tsv" "$V/k2/bac-p$p-lines.tsv" > "$V/k2/bac-p$p-lines.out" 2>&1
done
$PY -I "$K/classes.py" "$V"/k2/bac-p[1-5]-lines.tsv > "$V/k2/classes.txt" 2>&1
for p in $(seq 1 16); do
  g="$V/p$p/output/plate_$p.gcode"; c="$BAC_DATA/baselines/cubic/p$p/output/plate_$p.gcode"
  [ -f "$g" ] || continue
  [ -f "$V/bridges/p$p.json" ] || $PY "$TOOLING/tools/bridgecheck.py" $p "$V/bridges/p$p.json" "cubic=$c" "bac=$g" > "$V/bridges/p$p.txt" 2>&1
done
echo analyse-done
