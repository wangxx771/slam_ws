#!/bin/bash
# Deterministic offline replay: the node reads the bag in order and invokes callbacks directly, bypassing ros2 bag play and subscriptions.
#
# Why: live replay randomly drops frames at startup (the same binary produced
# 1460/1463/1093 frames), and a different starting frame means a different initial
# map, diverging trajectories, and endpoint errors of 1.54/6.70/5.12 m across
# three runs of the same configuration. That variance exceeds the effect of the
# parameters, making any tuning conclusion worthless. Offline replay processes
# every message every time.
#
# The node writes the CSV itself (replay.odom_csv); do not also attach
# odom_logger. Writing the same file, the logger opens with "w" and truncates the
# rows the node already wrote - the node finished 147 s of data in 15 s while the
# logger started 3 s late, so the CSV began at t~45 s.
#
# Usage: run_replay.sh <bag> <tag> <config> [extra --ros-args ...]

BAG=$1; TAG=$2; CFG=$3; shift 3
# Derive the workspace root from the script's own location; never hard-code an absolute path (the old /home/toe/slam_ws no longer exists)
WS=${WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}
OUT=${OUT_ROOT:-/tmp/replay}/$TAG
rm -rf "$OUT"; mkdir -p "$OUT"

export ROS_DOMAIN_ID=$(( (RANDOM % 90) + 10 ))
export ROS_LOCALHOST_ONLY=1
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
# Dependencies may live in another overlay (livox_ros_driver2, for instance); pass it via EXTRA_SETUP
for s in ${EXTRA_SETUP}; do source "$s"; done
source $WS/install/setup.bash

"$WS/install/point_lio/lib/point_lio/pointlio_mapping" --ros-args \
  --params-file "$CFG" -p replay.bag:="$BAG" -p replay.odom_csv:="$OUT/odom.csv" \
  "$@" > "$OUT/node.log" 2>&1 &
NODE_PID=$!

# Resource sampling: cumulative CPU time and RSS every 0.5 s (VmHWM is the kernel's own peak and the most trustworthy)
CLK=$(getconf CLK_TCK)
(
  echo "t,cpu_s,rss_mb,hwm_mb,threads"
  t=0
  while kill -0 $NODE_PID 2>/dev/null; do
    read -r _ _ _ _ _ _ _ _ _ _ _ _ _ ut st _ < /proc/$NODE_PID/stat 2>/dev/null || break
    rss=$(awk '/VmRSS/{print $2}' /proc/$NODE_PID/status 2>/dev/null)
    hwm=$(awk '/VmHWM/{print $2}' /proc/$NODE_PID/status 2>/dev/null)
    thr=$(awk '/Threads/{print $2}' /proc/$NODE_PID/status 2>/dev/null)
    echo "$t,$(awk -v a=$ut -v b=$st -v c=$CLK 'BEGIN{printf "%.2f",(a+b)/c}'),$((${rss:-0}/1024)),$((${hwm:-0}/1024)),${thr:-0}"
    t=$(awk -v x=$t 'BEGIN{printf "%.1f",x+0.5}')
    sleep 0.5
  done
) > "$OUT/res.csv" &
SAMPLER=$!

wait $NODE_PID
RC=$?
wait $SAMPLER 2>/dev/null

# Reap only this process. This used to be `pkill -9 -f pointlio_mapping`, which
# killed other replays running in parallel - it once cut off a loop-closure test
# 20 minutes in.
kill -9 $NODE_PID 2>/dev/null

# Integrity check: an interrupted run leaves a half-written CSV, and the
# evaluation scripts silently skip bad rows rather than erroring - two wrong
# conclusions came from such truncated files (2.5 m once, 0.21 m the other).
BAD=$(awk -F, "NF!=14{n++}END{print n+0}" "$OUT/odom.csv" 2>/dev/null)
if [ "${BAD:-0}" -gt 1 ]; then
  echo "  !! $TAG: odom.csv has $BAD rows with a bad column count; the file is incomplete and the result unusable"
fi

PEAK=$(awk -F, 'NR>1{if($4>m)m=$4}END{printf "%d",m}' "$OUT/res.csv" 2>/dev/null)
CPU=$(awk -F, 'NR>1{c=$2}END{printf "%.1f",c}' "$OUT/res.csv" 2>/dev/null)
echo "  $TAG: rc=$RC frames=$(grep -o '([0-9]* frames)' "$OUT/node.log" | tail -1 | tr -dc '0-9') \
odom=$(( $(wc -l < "$OUT/odom.csv" 2>/dev/null || echo 1) - 1 )) \
peakRSS=${PEAK}MB CPU=${CPU}s"
