//
// Created by ZhiangQi on 25-5-5.
//
#include <FreeDOM-ROS2/freenode.h>

using namespace freedom;
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
  // Logs go to <base>/<date>; SLAM_LOG_DIR overrides the default root.
  {
    const char * e = std::getenv("SLAM_LOG_DIR");
    slam_log::init(e ? e : "/var/log/slam", argv[0]);
  }
    auto node = std::make_shared<FreeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}