#!/bin/bash
# Batch BX-fix fix round 2: the measurement app (take_patch6) on Heimdall plates, with BX_TAKE, BX_TAKE_Z and a dump directory per plate;
# the G-code is kept for parsing, then deleted by the caller.
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
  BX_TAKE=1 BX_TAKE_DUMP="${d:?}/dump" BX_TAKE_Z="${BX_TAKE_Z_LIST}" TMPDIR="${d:?}/tmp/" CFFIXED_USER_HOME="${d:?}/cocoa-home" LC_ALL=C /usr/bin/time -l "$APP" --datadir "${d:?}/data" \
    --outputdir "${d:?}/output" --arrange 0 --orient 0 --debug 1 --no-check --slice $p "$IN" > "${d:?}/stdout.log" 2> "${d:?}/stderr.log"
  echo "take plate $p rc=$?" >> "$V/summary.txt"
  rm -rf "${d:?}/data" "${d:?}/tmp" "${d:?}/cocoa-home"
done
