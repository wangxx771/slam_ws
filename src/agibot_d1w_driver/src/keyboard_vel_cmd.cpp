#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h> 

static float linear_vel = 0.8;  // 默认线速度
static float angular_vel = 0.5; // 默认角速度


int GetChWithTimeout()
{
  struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  // 禁用标准模式(ICANON)和回显(ECHO，防止屏幕上打印满屏的wasd)
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);

  // 设置 select 超时时间为 0.1 秒
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 100000; // 100 毫秒

  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(STDIN_FILENO, &read_fds);

  int c = -1; // 默认返回 -1 表示没有按键按下
  
  // select 监听键盘输入
  if (select(STDIN_FILENO + 1, &read_fds, NULL, NULL, &tv) > 0)
  {
    c = getchar();
  }

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return c;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("keyboard_vel_cmd");

  printf("键盘控制WPR机器人 (ROS 2 版) - [按下走，松开停] \n");
  printf("w - 向前 \n");
  printf("s - 向后 \n");
  printf("a - 向左 \n");
  printf("d - 向右 \n");
  printf("q - 左旋 \n");
  printf("e - 右旋 \n");
  printf("x - 退出 \n");
  printf("------------- \n");

  auto cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("/d1w/cmd_vel", 10);

  bool is_moving = false; // 记录机器人当前是否处于运动状态

  while(rclcpp::ok())
  {
    // 使用带超时的非阻塞读取
    int cKey = GetChWithTimeout();
    
    geometry_msgs::msg::Twist base_cmd;
    base_cmd.linear.x = 0.0;
    base_cmd.linear.y = 0.0;
    base_cmd.angular.z = 0.0;

    bool valid_key_pressed = true;

    // 不再使用 += 累加速度，而是直接赋固定速度
    if(cKey == 'w') {
      base_cmd.linear.x = linear_vel;
    } 
    else if(cKey == 's') {
      base_cmd.linear.x = -linear_vel;
    } 
    else if(cKey == 'a') {
      base_cmd.linear.y = linear_vel;
    } 
    else if(cKey == 'd') {
      base_cmd.linear.y = -linear_vel;
    } 
    else if(cKey == 'q') {
      base_cmd.angular.z = angular_vel;
    } 
    else if(cKey == 'e') {
      base_cmd.angular.z = -angular_vel;
    } 
    else if(cKey == 'x') {
      printf("退出！ \n");
      rclcpp::shutdown();
      return 0;
    } 
    else {
      valid_key_pressed = false;
    }

    // 发布逻辑
    if (valid_key_pressed) {
      // 只要一直按着按键，就持续下发速度
      cmd_vel_pub->publish(base_cmd);
      is_moving = true;
      printf("\r 发送指令: linear.x= %.2f linear.y= %.2f angular.z= %.2f   ", base_cmd.linear.x, base_cmd.linear.y, base_cmd.angular.z);
      fflush(stdout);
    } 
    else {
      // 如果没有按键按下（超时返回-1），且机器人之前在动，就发送一次刹车指令
      if (is_moving) {
        cmd_vel_pub->publish(base_cmd); // 此时 base_cmd 全为 0
        is_moving = false;
        printf("\r [松开按键] 刹车停止！                                  ");
        fflush(stdout);
      }
    }
  }
  
  rclcpp::shutdown();
  return 0;
}
