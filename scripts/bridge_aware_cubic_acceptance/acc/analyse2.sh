#!/bin/bash
# Batch BX-fix: the per-plate analyses of acceptance runs in VARIANT_DIR, in parallel: K2 on plates 1-5 (k2check, causes, onbulk against
# M-2a's Cubic G-code, gap 5; then classes) and the bridge-line checks on every plate against investigation BX's Cubic G-code.
HERE=$(cd "$(dirname "$0")" && pwd); TOOLING=$(dirname "$HERE")
BAC_DATA=${BAC_DATA:-/Users/jforbes/Documents/Development.nosync/BambuStudio-bac-data}
BAC_PYTHON=${BAC_PYTHON:-python3}
BAC_WORK=${BAC_WORK:-$BAC_DATA/work}
SP=$BAC_WORK
V=${1:-$TOOLING/acc/final}
PY=$BAC_PYTHON
K=$TOOLING/k2
mkdir -p "$V/k2" "$V/bridges"
k2() {
  local p=$1 g="$V/p$1/output/plate_$1.gcode" c="$BAC_DATA/baselines/cubic/p$1/output/plate_$1.gcode"
  $PY -I "$K/k2check.py" "$g" "$c" 5 "$V/k2/bac-p$p.tsv" > "$V/k2/bac-p$p.out" 2>&1
  $PY -I "$K/causes.py" "$g" "$V/k2/bac-p$p.tsv" "$V/k2/bac-p$p-causes.tsv" > "$V/k2/bac-p$p-causes.out" 2>&1
  $PY -I "$K/onbulk.py" "$g" "$c" "$V/k2/bac-p$p-causes.tsv" "$V/k2/bac-p$p-lines.tsv" > "$V/k2/bac-p$p-lines.out" 2>&1
  echo "k2 p$p done"
}
bridges() {
  local p=$1 g="$V/p$1/output/plate_$1.gcode" c="$BAC_DATA/baselines/cubic/p$1/output/plate_$1.gcode"
  $PY "$TOOLING/tools/bridgecheck.py" $p "$V/bridges/p$p.json" "cubic=$c" "bac=$g" > "$V/bridges/p$p.txt" 2>&1
  echo "bridges p$p done"
}
for p in 1 2 3 4 5; do k2 $p & done
for p in 6 7 8; do bridges $p & done
wait
$PY -I "$K/classes.py" "$V"/k2/bac-p[1-5]-lines.tsv > "$V/k2/classes.txt" 2>&1
for group in "1 2 3 4 5 9 10 11" "12 13 14 15 16"; do
  for p in $group; do bridges $p & done
  wait
done
echo analyse-done
