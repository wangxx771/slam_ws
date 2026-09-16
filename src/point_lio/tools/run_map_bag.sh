#!/bin/bash
# Build a map from a recorded bag: point_lio provides odometry, FreeDOM the static map.
#
# Differences from map.launch.py: the livox driver is not started (no hardware
# during playback), and the map output dir is overridden by argument (the config
# hard-codes a path from another machine).
#
# Usage: run_map_bag.sh <bag> [out_dir] [playback_rate] [indoor|outdoor]
#
# Pick the right scene: FreeDOM's indoor sensor.max_z is 8 m, and using it
# outdoors cuts away everything above 8 m (the Z distribution stops dead at
# 8.0-8.5 rather than tapering). These overrides match OUTDOOR in map.launch.py.

BAG=${1:?Usage: run_map_bag.sh <bag> [outdir] [rate] [indoor|outdoor]}
OUT=${2:-/home/toe/slam_ws/PCD/$(date +%Y%m%d_%H%M%S)}
RATE=${3:-1.0}
SCENE=${4:-indoor}

SCENE_ARGS=""
if [ "$SCENE" = "outdoor" ]; then
  SCENE_ARGS="-p sensor.max_range:=60.0 -p sensor.min_z:=-10.0 -p sensor.max_z:=25.0 \
    -p map.raycast_max_range:=30.0 -p map.raycast_min_z:=-5.0 -p map.raycast_max_z:=10.0 \
    -p map.local_map_range:=80.0 -p map.local_map_max_z:=30.0"
elif [ "$SCENE" != "indoor" ]; then
  echo "scene must be indoor or outdoor, got '$SCENE'"; exit 1
fi
WS=${WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}   # derived from the script location; never hard-code an old workspace
mkdir -p "$OUT"

source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash
export ROS_DOMAIN_ID=$(( (RANDOM % 90) + 10 ))
export ROS_LOCALHOST_ONLY=1

PREFIX="$OUT/map_"
SC="${PREFIX}static_map_point.pcd.sc"

stdbuf -oL "$WS/install/point_lio/lib/point_lio/pointlio_mapping" --ros-args \
  --params-file "${LIO_CFG:-$WS/src/point_lio/config/mid360.yaml}" \
  -p pcd_save.pcd_save_en:=false \
  -p dynamic_filter.enable:=false \
  -p scan_context.save_path:="$SC" \
  > "$OUT/point_lio.log" 2>&1 &
LIO=$!

stdbuf -oL "$WS/install/freedom/lib/freedom/freedom_node" --ros-args \
  --params-file "${FD_CFG:-$WS/src/FreeDOM/config/mid360_pointlio.yaml}" \
  -p save_map_path:="$PREFIX" $SCENE_ARGS \
  > "$OUT/freedom.log" 2>&1 &
FD=$!

sleep 6
echo "nodes up (point_lio=$LIO freedom=$FD), scene=$SCENE, starting playback at rate=$RATE"
# These bags were recorded with point_lio running, so besides the sensor topics
# they also contain that session's own output: /lio/odom (22821 messages),
# /tf, /path and the /cloud_registered* clouds. Replaying those republishes an
# old trajectory onto the very topics the live node produces - two interleaved
# streams 1.9 m apart, and a stale map->odom overwriting the live one. Exclude
# every output topic and play only what a sensor would produce.
EXCLUDE_OUT='-x ^/(lio|Laser_map|cloud_effected|cloud_registered.*|path|tf|tf_static|rosout|parameter_events|events)'
ros2 bag play "$BAG" $EXCLUDE_OUT --rate "$RATE" > "$OUT/play.log" 2>&1
echo "playback done, processed $(grep -ac 'pointcloud received' "$OUT/freedom.log") frames"
sleep 15

ros2 topic pub --once /save_map std_msgs/msg/Empty "{}" > /dev/null 2>&1
sleep 60
kill -9 $LIO $FD 2>/dev/null
wait 2>/dev/null

echo "--- output"
ls -la "$OUT"/*.pcd "$OUT"/*.sc 2>/dev/null
echo "--- point_lio tail"; tail -4 "$OUT/point_lio.log"
echo "--- freedom tail";  tail -4 "$OUT/freedom.log"
