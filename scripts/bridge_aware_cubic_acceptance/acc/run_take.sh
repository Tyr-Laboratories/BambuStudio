#!/bin/bash
# Batch BX-fix fix round 1: the measurement app on Heimdall plates, with BX_TAKE and a dump directory per plate; the G-code is deleted.
HERE=$(cd "$(dirname "$0")" && pwd); TOOLING=$(dirname "$HERE")
BAC_DATA=${BAC_DATA:-/Users/jforbes/Documents/Development.nosync/BambuStudio-bac-data}
BAC_PYTHON=${BAC_PYTHON:-python3}
BAC_WORK=${BAC_WORK:-$BAC_DATA/work}
V=$1; shift
APP="$V/app/BambuStudio.app/Contents/MacOS/BambuStudio"
IN="$BAC_DATA/Heimdall2 - BAC review.3mf"
for p in "$@"; do
  d="$V/p$p"
  rm -rf "${d:?}"; mkdir -p "${d:?}/data" "${d:?}/output" "${d:?}/tmp" "${d:?}/cocoa-home" "${d:?}/dump"
  BX_TAKE=1 BX_TAKE_DUMP="${d:?}/dump" TMPDIR="${d:?}/tmp/" CFFIXED_USER_HOME="${d:?}/cocoa-home" LC_ALL=C "$APP" --datadir "${d:?}/data" \
    --outputdir "${d:?}/output" --arrange 0 --orient 0 --debug 1 --no-check --slice $p "$IN" > "${d:?}/stdout.log" 2> "${d:?}/stderr.log"
  echo "take plate $p rc=$?" >> "$V/summary.txt"
  rm -rf "${d:?}/data" "${d:?}/tmp" "${d:?}/cocoa-home" "${d:?}"/output/*.gcode
done
