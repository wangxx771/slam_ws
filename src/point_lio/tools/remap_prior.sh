#!/bin/bash
# Multi-pass mapping: build normally on the first pass, then feed the previous pass's map into ivox as a prior and re-register.
#
# In a single pass the same place is observed at different times, and the two
# observations are offset by however far the front end drifted, showing up as one
# surface built into several layers. A second pass fixes this because every frame
# registers against the same fixed reference map. Absolute accuracy does not
# improve (it inherits the first pass's shape) but self-consistency does, and
# self-consistency is exactly what layering measures.
#
# Three hard constraints:
#   1. Loop closure must be off. With it on, pass 2 went from 330951 to 3184188 points and blew up.
#   2. Only useful on bags with revisits. A flat one-way bag lost just 0.2% of
#      points over three passes while xy closure went from 1.808 m to 1.945 m -
#      pure loss.
#   3. This is not a convergent iteration. Stop and fall back to the previous
#      pass once the point count stops dropping - a rebound signals divergence.
#      On the stair-climb bag pass 2 took landing |dz| from 0.363 to 0.085 m and
#      pass 3 diverged.
#
# The metrics use physical anchors, since a stairwell's self-similar geometry
# fools conventional ones: landing height and same-flight height. Do not use
# landing xy - where the dog stands on a landing is its own choice and a 0.5 m
# difference between up and down is normal.
#
# Offline prior-map building from a bag only; live mapping on hardware cannot run a second pass.
#
# Usage: remap_prior.sh <bag> <out_dir> [config] [max_passes=3] [extra --ros-args ...]

set -u
BAG=$1; OUT=$2; CFG=${3:-}; PASSES=${4:-3}; shift 4 2>/dev/null || shift $#
WS=${WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}
CFG=${CFG:-$WS/src/point_lio/config/mid360.yaml}
REPLAY="$(dirname "${BASH_SOURCE[0]}")/run_replay.sh"

rm -rf "$OUT"; mkdir -p "$OUT"
PREV=""; PREVN=""
for i in $(seq 1 "$PASSES"); do
  PDIR="$OUT/pass$i"
  mkdir -p "$PDIR"
  EXTRA=()
  if [ -n "$PREV" ]; then
    # freeze: this pass localizes against the prior map only and does not merge
    # its own new points into the registration reference. The prior map is already
    # more consistent than this pass's front end, so merging drifting new points
    # would use it as a baseline and corrupt it at the same time. Two frozen
    # passes reached 0.085/0.092 m, better than three unfrozen ones
    # (0.102/0.110) and one pass shorter.
    EXTRA=(-p prior_pcd.enable:=true -p prior_pcd.prior_pcd_map_path:="$PREV"
           -p prior_pcd.freeze:=true)
  fi
  echo "--- pass $i ${PREV:+(prior: $PREV)}"
  OUT_ROOT="$OUT/replay" bash "$REPLAY" "$BAG" "pass$i" "$CFG" \
    -p pcd_save.pcd_save_en:=true -p pcd_save.output_dir:="$PDIR" \
    "${EXTRA[@]}" "$@"
  MAP=$(find "$PDIR" -name map.pcd | head -1)
  if [ -z "$MAP" ]; then echo "pass $i produced no map, aborting"; exit 1; fi
  N=$(grep -ao '([0-9]* points)' "$OUT/replay/pass$i/node.log" | tail -1 | tr -dc '0-9')
  echo "    -> $MAP  ${N:-?} points"
  # Automatic brake: a point count that stops dropping signals divergence, so stop at the previous pass
  if [ -n "$PREVN" ] && [ -n "$N" ] && [ "$N" -ge "$PREVN" ]; then
    echo "    !! point count stopped dropping ($PREVN -> $N), judged divergent, discarding this pass"
    break
  fi
  PREVN="$N"; PREV="$MAP"
done

# Put the selected pass straight into the output root rather than making callers
# count passes: on divergence the final map is not the last pass, which is the
# easiest thing to get wrong. Every pass is kept for tracing back.
if [ -n "$PREV" ]; then
  cp -f "$PREV" "$OUT/map.pcd"
  [ -f "$PREV.sc" ] && cp -f "$PREV.sc" "$OUT/map.pcd.sc"
fi

echo
echo "final map: $OUT/map.pcd  (from $PREV)"
