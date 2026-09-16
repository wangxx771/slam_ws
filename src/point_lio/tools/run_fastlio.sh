#!/bin/bash
# Deterministic offline replay for FAST_LIO_GPU, same interface as point_lio's run_replay.sh
# Usage: run_fastlio.sh <bag> <tag> <config> [extra --ros-args ...]
BAG=$1; TAG=$2; CFG=$3; shift 3
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
OUT=${OUT_ROOT:-/tmp/replay}/$TAG
rm -rf "$OUT"; mkdir -p "$OUT"
export ROS_DOMAIN_ID=$(( (RANDOM % 90) + 10 ))
export ROS_LOCALHOST_ONLY=1
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

"$WS/install/fast_lio/lib/fast_lio/fastlio_mapping" --ros-args --params-file "$CFG" \
  -p replay.bag:="$BAG" -p replay.odom_csv:="$OUT/odom.csv" "$@" > "$OUT/node.log" 2>&1 &
NODE_PID=$!
CLK=$(getconf CLK_TCK)
( echo "t,cpu_s,rss_mb,hwm_mb"
  t=0
  while kill -0 $NODE_PID 2>/dev/null; do
    read -r _ _ _ _ _ _ _ _ _ _ _ _ _ ut st _ < /proc/$NODE_PID/stat 2>/dev/null || break
    rss=$(awk '/VmRSS/{print $2}' /proc/$NODE_PID/status 2>/dev/null)
    hwm=$(awk '/VmHWM/{print $2}' /proc/$NODE_PID/status 2>/dev/null)
    echo "$t,$(awk -v a=$ut -v b=$st -v c=$CLK 'BEGIN{printf "%.2f",(a+b)/c}'),$((${rss:-0}/1024)),$((${hwm:-0}/1024))"
    t=$(awk -v x=$t 'BEGIN{printf "%.1f",x+0.5}'); sleep 0.5
  done ) > "$OUT/res.csv" &
SAMPLER=$!
wait $NODE_PID; RC=$?
kill -9 $NODE_PID 2>/dev/null; wait $SAMPLER 2>/dev/null
PEAK=$(awk -F, 'NR>1{if($4>m)m=$4}END{printf "%d",m}' "$OUT/res.csv" 2>/dev/null)
CPU=$(awk -F, 'NR>1{c=$2}END{printf "%.1f",c}' "$OUT/res.csv" 2>/dev/null)
echo "  $TAG: rc=$RC odom=$(( $(wc -l < "$OUT/odom.csv" 2>/dev/null || echo 1) - 1 )) peakRSS=${PEAK}MB CPU=${CPU}s"
