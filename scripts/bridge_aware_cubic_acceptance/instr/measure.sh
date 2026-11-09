#!/bin/zsh
# measure.sh APP FIXTURE MODE OUT: one CLI run under /usr/bin/time -l with fresh data, output, tmp and Cocoa home directories.
# MODE is "slice" (--slice 1) or "load" (the same arguments without --slice). Prints: rc instructions cycles peak_footprint
HERE=$(cd "$(dirname "$0")" && pwd); TOOLING=$(dirname "$HERE")
BAC_DATA=${BAC_DATA:-/Users/jforbes/Documents/Development.nosync/BambuStudio-bac-data}
BAC_PYTHON=${BAC_PYTHON:-python3}
BAC_WORK=${BAC_WORK:-$BAC_DATA/work}
app=$1; fixture=$2; mode=$3; out=$4
rm -rf $out; mkdir -p $out/data $out/output $out/tmp $out/home
args=(--datadir $out/data --outputdir $out/output --arrange 0 --orient 0 --debug 1)
[[ $mode == slice ]] && args+=(--slice 1)
TMPDIR=$out/tmp/ LC_ALL=C CFFIXED_USER_HOME=$out/home /usr/bin/time -l $app $args $fixture >$out/stdout.log 2>$out/time.log
rc=$?
ins=$(awk '/instructions retired/{print $1}' $out/time.log)
cyc=$(awk '/cycles elapsed/{print $1}' $out/time.log)
mem=$(awk '/peak memory footprint/{print $1}' $out/time.log)
echo "$rc $ins $cyc $mem"
