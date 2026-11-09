#!/bin/zsh
# matrix.sh OUT [FORK_APP] [ITERATIONS]: instructions retired for the four benchmark fixtures, official 2.9.1.70 Cubic against the fork's Cubic and BAC
# (default: M-1's release app), each as a full slice (--slice 1) and as startup and load only (--slice 99, which loads and exits on the
# missing plate). Three iterations, sides interleaved. Writes OUT/matrix.tsv: iter size app pattern mode rc instructions cycles.
HERE=$(cd "$(dirname "$0")" && pwd); TOOLING=$(dirname "$HERE")
BAC_DATA=${BAC_DATA:-/Users/jforbes/Documents/Development.nosync/BambuStudio-bac-data}
BAC_PYTHON=${BAC_PYTHON:-python3}
BAC_WORK=${BAC_WORK:-$BAC_DATA/work}
SP=${0:h}
OFF=/private/tmp/bridge-aware-official-baseline/2.9.1.70/BambuStudio.app/Contents/MacOS/BambuStudio
FORK=${2:-$BAC_WORK/../m1/final/app/BambuStudio.app/Contents/MacOS/BambuStudio}
iters=${3:-3}
out=$1; mkdir -p $out; tsv=$out/matrix.tsv; : > $tsv
run() { # iter size appname app pattern mode
  local dir=$out/runs/$1-$2-$3-$5-$6; rm -rf $dir; mkdir -p $dir/data $dir/output $dir/tmp $dir/home
  local plate=1; [[ $6 == load ]] && plate=99
  TMPDIR=$dir/tmp/ LC_ALL=C CFFIXED_USER_HOME=$dir/home perl -e 'alarm 600; exec @ARGV' /usr/bin/time -l $4 --datadir $dir/data \
    --outputdir $dir/output --arrange 0 --orient 0 --debug 1 --slice $plate $BAC_WORK/projects/$2-$5.3mf >$dir/stdout.log 2>$dir/time.log
  local rc=$?
  print -r -- "$1	$2	$3	$5	$6	$rc	$(awk '/instructions retired/{print $1}' $dir/time.log)	$(awk '/cycles elapsed/{print $1}' $dir/time.log)" >> $tsv
  rm -f $dir/output/plate_1.gcode
}
for iter in $(seq 1 $iters); do
  for size in small large rib modifier; do
    for mode in slice load; do
      run $iter $size official $OFF cubic $mode
      run $iter $size fork $FORK cubic $mode
      run $iter $size fork $FORK bridgeawarecubic $mode
    done
  done
done
echo done >> $out/matrix.done
