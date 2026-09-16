#!/bin/bash
# Live relocalization test for small_gicp_relocalization.
#
# Unlike run_replay.sh this must use live playback: small_gicp is a separate node
# driven by /cloud_registered, and in offline replay point_lio runs ten times
# faster than real time so it cannot keep up. The cost is that live playback drops
# frames and the trajectory is not bit-reproducible - but this test asks whether
# it can lock from an arbitrary position and how accurate the pose is afterwards,
# which dropped frames do not affect.
#
# Note that several parameters take the launch file's values, not the node defaults, which differ:
#   num_neighbors 10 vs 20、feature_leaf_size 0.3 vs 0.5、
#   ransac_max_tilt_deg 180 vs 15. The last one matters most: at the node default
#   of 15 deg, the global seed poses FPFH produces in the AVIA corridor scene are
#   often 19-30 deg off and all get rejected.
#
# Usage: run_reloc_test.sh <bag> <tag> <point_lio_config> <prior_map.pcd> <start_offset_s> <duration_s>
#       [truth.csv]  when given, at bag time + 8 s it takes the truth at that
#       instant, adds a deliberate error (2 m / 12 deg) and publishes a coarse
#       guess to /initialpose, equivalent to RViz's 2D Pose Estimate. Without it,
#       relocalization relies entirely on automatic FPFH+RANSAC.

BAG=$1; TAG=$2; CFG=$3; MAP=$4; OFF=$5; DUR=$6
SEED_CSV=${7:--}
# Derive the workspace from the script's own location; never hard-code it. The
# previously hard-coded /home/toe/slam_ws was an old workspace, so running it
# tested an old binary (run_freedom.sh hit the same trap).
WS=${WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}
OUT=${OUT_ROOT:-/tmp/reloc}/$TAG
rm -rf "$OUT"; mkdir -p "$OUT"

export ROS_DOMAIN_ID=$(( (RANDOM % 90) + 10 ))
export ROS_LOCALHOST_ONLY=1
source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash

"$WS/install/point_lio/lib/point_lio/pointlio_mapping" --ros-args \
  --params-file "$CFG" -p pcd_save.pcd_save_en:=false ${LIO_EXTRA:-} > "$OUT/lio.log" 2>&1 &
LIO=$!

"$WS/install/small_gicp_relocalization/lib/small_gicp_relocalization/small_gicp_relocalization_node" \
  --ros-args \
  -p map_frame:=map -p odom_frame:=odom -p base_frame:=body -p lidar_frame:=body \
  -p input_cloud_topic:=cloud_registered -p prior_pcd_file:="$MAP" \
  -p continuous_correction:=true \
  -p num_neighbors:=10 -p feature_leaf_size:=0.3 -p ransac_max_tilt_deg:=${TILT:-180.0} ${GICP_EXTRA:-} \
  > "$OUT/gicp.log" 2>&1 &
GICP=$!

python3 "$WS/src/point_lio/tools/reloc_recorder.py" "$OUT/odom.csv" "$DUR" "$SEED_CSV" 8 \
  > "$OUT/rec.log" 2>&1 &
REC=$!

sleep 3
  # Playback rate. Default 1.0 (real time, closest to hardware); set RATE=0.4 for evaluation.
  #
  # Five runs each on the flat bag:
  #     1.0x   position error 0.257-0.260 m, but lock time 10.9-36.2 s (a factor of 3.3)
  #     0.4x   position error 0.268 m on all five, lock time 18.4-18.9 s (2.7% spread)
  # Slowed down it is essentially deterministic, which is what tuning needs.
  #
  # Position error is already stable at 1.0x. A 0.017 vs 0.258 m gap was seen
  # once; that was not run variance but landing on a different solution
  # (multimodal), and must not be averaged away as noise.
  # Sensor topics only. These bags were recorded with point_lio running, so they
  # also contain /lio/odom (22821 messages) and /tf from that session. Replaying
  # those puts a second, unrelated trajectory on the very topic the test measures:
  # the recorder saw two interleaved streams 1.9 m apart, and the stale map->odom
  # in the replayed /tf overwrote the live one. That is where the "1.9 m
  # relocalization error" came from - it was never relocalization.
  ros2 bag play "$BAG" --start-offset "$OFF" --rate "${RATE:-1.0}" \
    --topics /livox/lidar /livox/imu > "$OUT/play.log" 2>&1 &
PLAY=$!

wait $REC
kill -9 $PLAY $GICP $LIO 2>/dev/null
wait 2>/dev/null

LOCK=$(grep -o "Relocalization LOCKED.*" "$OUT/gicp.log" | head -1)
echo "  $TAG: odom=$(( $(wc -l < "$OUT/odom.csv") - 1 ))  ${LOCK:-no lock}"
