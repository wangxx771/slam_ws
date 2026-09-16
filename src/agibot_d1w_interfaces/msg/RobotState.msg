std_msgs/Header header

bool connected
uint8 control_mode
uint8 battery_percent

# SDK quaternion order is [w, x, y, z].
float32[4] orientation_wxyz
float32[3] rpy
float32[3] position
float32[3] world_velocity
float32[3] body_velocity
float32[3] angular_velocity
float32[3] linear_acceleration

# Joint array leg order: [FR, FL, RR, RL].
float32[4] abad_joint_angles
float32[4] hip_joint_angles
float32[4] knee_joint_angles
float32[4] foot_joint_angles

float32[4] abad_joint_velocities
float32[4] hip_joint_velocities
float32[4] knee_joint_velocities
float32[4] foot_joint_velocities

float32[4] abad_joint_torques
float32[4] hip_joint_torques
float32[4] knee_joint_torques
float32[4] foot_joint_torques
