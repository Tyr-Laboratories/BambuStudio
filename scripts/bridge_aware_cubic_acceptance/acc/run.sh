#!/bin/bash
# Batch BX-fix: Heimdall plates with one app. Usage: run.sh VARIANT_DIR PLATE... (VARIANT_DIR holds app/BambuStudio.app; results go to
# VARIANT_DIR/pN, one line per plate to VARIANT_DIR/summary.txt), as M-2a's fx1/ab/run.sh, on a read-only local copy of the project.
HERE=$(cd "$(dirname "$0")" && pwd); TOOLING=$(dirname "$HERE")
BAC_DATA=${BAC_DATA:-/Users/jforbes/Documents/Development.nosync/BambuStudio-bac-data}
BAC_PYTHON=${BAC_PYTHON:-python3}
BAC_WORK=${BAC_WORK:-$BAC_DATA/work}
V=$1; shift
APP="$V/app/BambuStudio.app/Contents/MacOS/BambuStudio"
# The project to slice: the BAC review copy by default; the Cubic baseline is made with BAC_PROJECT set to the Cubic copy.
IN=${BAC_PROJECT:-"$BAC_DATA/Heimdall2 - BAC review.3mf"}
METRICS=$TOOLING/metrics/metrics.py
for p in "$@"; do
  d="$V/p$p"
  rm -rf "${d:?}"; mkdir -p "${d:?}/data" "${d:?}/output" "${d:?}/tmp" "${d:?}/cocoa-home"
  env $BX_ENV TMPDIR="${d:?}/tmp/" CFFIXED_USER_HOME="${d:?}/cocoa-home" LC_ALL=C /usr/bin/time -l "$APP" --datadir "${d:?}/data" \
    --outputdir "${d:?}/output" --arrange 0 --orient 0 --debug 1 --no-check --slice $p "$IN" > "${d:?}/stdout.log" 2> "${d:?}/time.log"
  rss=$(awk '/maximum resident set size/ {printf "%d", $1 / 1048576}' "${d:?}/time.log")
  instr=$(awk '/instructions retired/ {printf "%.0f", $1}' "${d:?}/time.log")
  rc=$(python3 -I -c "import json,sys;print(json.load(open(sys.argv[1])).get('return_code', ''))" "${d:?}/output/result.json" 2>/dev/null)
  m=""; [ -f "${d:?}/output/plate_$p.gcode" ] && m=$(python3 -I "$METRICS" "${d:?}/output/plate_$p.gcode")
  echo "bac plate $p rc=$rc instructions=$instr rss_mb=$rss $m" >> "$V/summary.txt"
  rm -rf "${d:?}/data" "${d:?}/tmp" "${d:?}/cocoa-home"
done
