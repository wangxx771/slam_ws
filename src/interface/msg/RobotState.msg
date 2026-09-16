# 机器狗状态消息
std_msgs/Header header
float32[3] position          # x, y, z (m)
float32[4] orientation       # w, x, y, z 四元数
float32[3] rpy               # roll, pitch, yaw (rad)
float32[3] linear_velocity   # 世界坐标系速度 (m/s)
float32[3] body_velocity     # 机体坐标系速度 (m/s)
float32[3] angular_velocity  # 机体角速度 (rad/s)
float32[3] linear_acceleration  # 机体加速度 (m/s²)
uint8 battery_percent        # 电量 0-100
uint8 control_mode           # 当前控制模式
float32[4] abad_joint_angles # 髋外展关节角度 (rad)
float32[4] hip_joint_angles  # 髋关节角度 (rad)
float32[4] knee_joint_angles # 膝关节角度 (rad)
