#!/bin/bash
# Run FreeDOM end to end on labelled HeLiMOS data and save the static map.
#
# Node, playback and map saving live in one script so processes are not reaped between tool calls.
#
# The workspace is derived from the script's own location rather than hard-coded:
# the previously hard-coded /home/toe/slam_ws was an old workspace and produced
# results from an old version.
#
# Usage: run_freedom.sh [out_dir] [bag] [config]

WS=${WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}
OUT=${1:-/tmp/freedom_out}
BAG=${2:-/home/toe/slam_ws/bags/ouster_ros2}
CFG=${3:-$WS/src/FreeDOM/config/helimos_ouster.yaml}
RATE=${RATE:-0.4}

rm -rf "$OUT"; mkdir -p "$OUT"

source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=$(( (RANDOM % 90) + 10 ))
export ROS_LOCALHOST_ONLY=1

# Check for leftovers from the previous run first: two freedom_nodes subscribing
# to the same topic overwrite each other's saved map while both logs look
# normal.
for pid in $(ps -eo pid,stat,comm --no-headers |
             awk '$2 !~ /Z/ && $3 ~ /^freedom_node/ {print $1}'); do
  kill -9 "$pid" 2>/dev/null
done

stdbuf -oL -eL "$WS/install/freedom/lib/freedom/freedom_node" --ros-args \
  --params-file "$CFG" \
  -p pointcloud_topic:=/velodyne_points -p map_tf_frame:=map -p sensor_tf_frame:=velodyne \
  -p save_map_topic:=/save_map -p save_map_path:="$OUT/" -p enable_visualization:=false \
  -p use_sim_time:=true \
  > "$OUT/node.log" 2>&1 &
NODE=$!
sleep 8
if ! kill -0 $NODE 2>/dev/null; then
  echo "freedom_node died immediately, log:"; tail -20 "$OUT/node.log"; exit 1
fi

# The simulated clock is required. HeLiMOS bag timestamps start at 0.1 s (1970)
# while the node defaults to system time (2026), so tf2 considers every transform
# decades stale and all lookups fail. The symptom is a node running normally with
# a clean log that saves an empty map - easily mistaken for very thorough
# removal.
#
# Play slowly to leave FreeDOM real-time headroom. Playing too fast drops frames, and dropped frames are also silent.
CPU0=$(awk '{print $14+$15}' /proc/$NODE/stat)
# These bags were recorded with point_lio running, so besides the sensor topics
# they also contain that session's own output: /lio/odom (22821 messages),
# /tf, /path and the /cloud_registered* clouds. Replaying those republishes an
# old trajectory onto the very topics the live node produces - two interleaved
# streams 1.9 m apart, and a stale map->odom overwriting the live one. Exclude
# every output topic and play only what a sensor would produce.
EXCLUDE_OUT='-x ^/(lio|Laser_map|cloud_effected|cloud_registered.*|path|tf|tf_static|rosout|parameter_events|events)'
ros2 bag play "$BAG" $EXCLUDE_OUT --clock --rate "$RATE" > "$OUT/play.log" 2>&1
sleep 20
CPU1=$(awk '{print $14+$15}' /proc/$NODE/stat 2>/dev/null || echo "$CPU0")
HWM=$(awk '/VmHWM/{print $2}' /proc/$NODE/status 2>/dev/null)

ros2 topic pub --once /save_map std_msgs/msg/Empty "{}" > /dev/null 2>&1
sleep 60
kill -9 $NODE 2>/dev/null
wait 2>/dev/null

CLK=$(getconf CLK_TCK)
echo "--- resources"
awk -v a="$CPU0" -v b="$CPU1" -v c="$CLK" -v h="${HWM:-0}" \
  'BEGIN{printf "CPU time %.1f s, peak RSS %d MB\n",(b-a)/c,h/1024}'
echo "--- output"
ls -la "$OUT"/*.pcd 2>/dev/null || echo "no map was saved"
echo "--- log tail"
tail -6 "$OUT/node.log"
