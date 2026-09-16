#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "agibot_d1w_interfaces/msg/robot_state.hpp"
#include "agibot_d1w_interfaces/srv/attitude_control.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "zsl-1w/highlevel.h"

namespace agibot_d1w_driver
{

class RobotDriver final : public rclcpp::Node
{
public:
  using SteadyClock = std::chrono::steady_clock;

  RobotDriver()
  : Node("agibot_d1w_driver")
  {
    declareParameters();
    loadParameters();

    if (control_rate_ <= 0.0 || state_rate_ <= 0.0 ||
      imu_rate_ <= 0.0 || connection_rate_ <= 0.0)
    {
      throw std::runtime_error(
              "control_rate, state_rate, imu_rate and connection_rate must be positive");
    }

    if (cmd_vel_timeout_ < 0.0 || body_cmd_timeout_ <= 0.0 ||
      action_timeout_ <= 0.0 || auto_stand_delay_ < 0.0 ||
      stand_settle_duration_ < 0.0 || stop_hold_duration_ <= 0.0 ||
      move_transition_retry_interval_ <= 0.0 || action_retry_interval_ <= 0.0)
    {
      throw std::runtime_error(
              "timeout/rate parameters are invalid");
    }

    sdk_ = std::make_unique<mc_sdk::zsl_1w::HighLevel>();
    sdk_->initRobot(local_ip_, local_port_, dog_ip_);

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(10),
      std::bind(&RobotDriver::cmdVelCallback, this, std::placeholders::_1));

    body_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "body_cmd", rclcpp::QoS(10),
      std::bind(&RobotDriver::bodyCmdCallback, this, std::placeholders::_1));

    high_level_cmd_sub_ = create_subscription<std_msgs::msg::String>(
      "high_level_cmd", rclcpp::QoS(10),
      std::bind(&RobotDriver::highLevelCmdCallback, this, std::placeholders::_1));

    stand_up_srv_ = create_service<std_srvs::srv::Trigger>(
      "stand_up",
      std::bind(&RobotDriver::standUpCallback, this,
        std::placeholders::_1, std::placeholders::_2));

    lie_down_srv_ = create_service<std_srvs::srv::Trigger>(
      "lie_down",
      std::bind(&RobotDriver::lieDownCallback, this,
        std::placeholders::_1, std::placeholders::_2));

    passive_srv_ = create_service<std_srvs::srv::Trigger>(
      "passive",
      std::bind(&RobotDriver::passiveCallback, this,
        std::placeholders::_1, std::placeholders::_2));

    stop_srv_ = create_service<std_srvs::srv::Trigger>(
      "stop",
      std::bind(&RobotDriver::stopCallback, this,
        std::placeholders::_1, std::placeholders::_2));

    shake_hand_srv_ = create_service<std_srvs::srv::Trigger>(
      "shake_hand",
      std::bind(&RobotDriver::shakeHandCallback, this,
        std::placeholders::_1, std::placeholders::_2));

    crawl_srv_ = create_service<std_srvs::srv::SetBool>(
      "enable_crawl",
      std::bind(&RobotDriver::crawlCallback, this,
        std::placeholders::_1, std::placeholders::_2));

    climb_srv_ = create_service<std_srvs::srv::SetBool>(
      "enable_climb",
      std::bind(&RobotDriver::climbCallback, this,
        std::placeholders::_1, std::placeholders::_2));

    attitude_control_srv_ =
      create_service<agibot_d1w_interfaces::srv::AttitudeControl>(
      "attitude_control",
      std::bind(&RobotDriver::attitudeControlCallback, this,
        std::placeholders::_1, std::placeholders::_2));

    state_pub_ =
      create_publisher<agibot_d1w_interfaces::msg::RobotState>("robot_state", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    joint_state_pub_ =
      create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    const auto ros_now = now();
    const auto steady_now = SteadyClock::now();

    last_cmd_vel_time_ = steady_now;
    body_cmd_expire_time_ = steady_now;
    action_start_time_ = ros_now;
    gait_transition_start_time_ = ros_now;
    auto_stand_ready_time_ = steady_now;
    motion_ready_time_ = steady_now;
    stop_until_time_ = steady_now;
    next_move_attempt_time_ = steady_now;
    next_action_attempt_time_ = steady_now;

    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_rate_),
      std::bind(&RobotDriver::controlLoop, this));

    state_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / state_rate_),
      std::bind(&RobotDriver::stateLoop, this));

    // IMU独立200Hz发布，不再跟随state_rate。
    imu_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / imu_rate_),
      std::bind(&RobotDriver::imuLoop, this));

    connection_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / connection_rate_),
      std::bind(&RobotDriver::connectionLoop, this));

    RCLCPP_INFO(
      get_logger(),
      "D1 ZSL-1w bridge started: local=%s:%d dog=%s "
      "control=%.1fHz state=%.1fHz imu=%.1fHz auto_stand_up=%s",
      local_ip_.c_str(), local_port_, dog_ip_.c_str(),
      control_rate_, state_rate_, imu_rate_,
      auto_stand_up_ ? "true" : "false");
  }

  ~RobotDriver() override
  {
    shutdownRobotSafely();
  }

private:
  enum class Action
  {
    NONE,
    STAND_UP,
    LIE_DOWN,
    PASSIVE,
    SHAKE_HAND
  };

  enum class Gait
  {
    NORMAL,
    CRAWL,
    CLIMB
  };

  struct VelocityCommand
  {
    float vx{0.0F};
    float vy{0.0F};
    float yaw_rate{0.0F};
  };

  struct AttitudeCommand
  {
    float roll_vel{0.0F};
    float pitch_vel{0.0F};
    float yaw_vel{0.0F};
    float height_vel{0.0F};
  };

  void declareParameters()
  {
    declare_parameter<std::string>("local_ip", "192.168.168.120");
    declare_parameter<int>("local_port", 43988);
    declare_parameter<std::string>("dog_ip", "192.168.168.168");

    declare_parameter<double>("control_rate", 500.0);
    declare_parameter<double>("state_rate", 50.0);
    declare_parameter<double>("imu_rate", 200.0);
    declare_parameter<double>("connection_rate", 1.0);

    declare_parameter<bool>("auto_stand_up", true);
    declare_parameter<double>("auto_stand_delay", 1.00);
    declare_parameter<double>("stand_settle_duration", 2.00);

    // 0.0 = 关闭按消息间隔超时停车。
    // SCAN-Planner可能只在速度变化时发布，SDK的move命令本身具有保持语义。
    declare_parameter<double>("cmd_vel_timeout", 0.0);
    declare_parameter<double>("stop_hold_duration", 1.00);
    declare_parameter<double>("body_cmd_timeout", 0.25);
    declare_parameter<double>("action_timeout", 10.0);
    declare_parameter<double>("action_retry_interval", 0.10);
    declare_parameter<double>("gait_switch_settle", 0.30);
    declare_parameter<double>("move_transition_retry_interval", 0.20);

    declare_parameter<double>("max_vx", 0.30);
    declare_parameter<double>("max_vy", 0.20);
    declare_parameter<double>("max_yaw_rate", 0.50);
    declare_parameter<double>("vx_deadband", 0.05);
    declare_parameter<double>("vy_deadband", 0.10);
    declare_parameter<double>("yaw_deadband", 0.10);

    declare_parameter<double>("max_roll_vel", 0.30);
    declare_parameter<double>("max_pitch_vel", 0.30);
    declare_parameter<double>("max_attitude_yaw_vel", 0.30);
    declare_parameter<double>("max_height_vel", 0.20);

    declare_parameter<bool>("publish_odom", true);
    declare_parameter<bool>("publish_imu", true);
    declare_parameter<bool>("publish_joint_states", true);
    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<std::string>("base_frame", "base_link");

    declare_parameter<int>("shutdown_zero_command_count", 20);

    // 正常退出 / Ctrl+C / SIGTERM 时的安全关闭参数。
    declare_parameter<bool>("shutdown_lie_down", true);
    declare_parameter<bool>("shutdown_passive_after_lie_down", true);
    declare_parameter<double>("shutdown_stop_duration", 0.50);
    declare_parameter<double>("shutdown_lie_down_timeout", 5.00);
    declare_parameter<double>("shutdown_lie_down_retry_interval", 0.10);
    declare_parameter<double>("shutdown_lie_down_settle_duration", 2.00);
    declare_parameter<double>("shutdown_passive_timeout", 3.00);
    declare_parameter<double>("shutdown_passive_retry_interval", 0.10);
  }

  void loadParameters()
  {
    local_ip_ = get_parameter("local_ip").as_string();
    local_port_ = static_cast<int>(get_parameter("local_port").as_int());
    dog_ip_ = get_parameter("dog_ip").as_string();

    control_rate_ = get_parameter("control_rate").as_double();
    state_rate_ = get_parameter("state_rate").as_double();
    imu_rate_ = get_parameter("imu_rate").as_double();
    connection_rate_ = get_parameter("connection_rate").as_double();

    auto_stand_up_ = get_parameter("auto_stand_up").as_bool();
    auto_stand_delay_ = get_parameter("auto_stand_delay").as_double();
    stand_settle_duration_ = get_parameter("stand_settle_duration").as_double();

    cmd_vel_timeout_ = get_parameter("cmd_vel_timeout").as_double();
    stop_hold_duration_ = get_parameter("stop_hold_duration").as_double();
    body_cmd_timeout_ = get_parameter("body_cmd_timeout").as_double();
    action_timeout_ = get_parameter("action_timeout").as_double();
    action_retry_interval_ = get_parameter("action_retry_interval").as_double();
    gait_switch_settle_ = get_parameter("gait_switch_settle").as_double();
    move_transition_retry_interval_ =
      get_parameter("move_transition_retry_interval").as_double();

    max_vx_ = get_parameter("max_vx").as_double();
    max_vy_ = get_parameter("max_vy").as_double();
    max_yaw_rate_ = get_parameter("max_yaw_rate").as_double();
    vx_deadband_ = get_parameter("vx_deadband").as_double();
    vy_deadband_ = get_parameter("vy_deadband").as_double();
    yaw_deadband_ = get_parameter("yaw_deadband").as_double();

    max_roll_vel_ = get_parameter("max_roll_vel").as_double();
    max_pitch_vel_ = get_parameter("max_pitch_vel").as_double();
    max_attitude_yaw_vel_ = get_parameter("max_attitude_yaw_vel").as_double();
    max_height_vel_ = get_parameter("max_height_vel").as_double();

    publish_odom_ = get_parameter("publish_odom").as_bool();
    publish_imu_ = get_parameter("publish_imu").as_bool();
    publish_joint_states_ = get_parameter("publish_joint_states").as_bool();
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();

    shutdown_zero_command_count_ = static_cast<int>(
      get_parameter("shutdown_zero_command_count").as_int());
    shutdown_zero_command_count_ = std::max(0, shutdown_zero_command_count_);

    shutdown_lie_down_ = get_parameter("shutdown_lie_down").as_bool();
    shutdown_passive_after_lie_down_ =
      get_parameter("shutdown_passive_after_lie_down").as_bool();
    shutdown_stop_duration_ =
      std::max(0.0, get_parameter("shutdown_stop_duration").as_double());
    shutdown_lie_down_timeout_ =
      std::max(0.0, get_parameter("shutdown_lie_down_timeout").as_double());
    shutdown_lie_down_retry_interval_ =
      std::max(0.01, get_parameter("shutdown_lie_down_retry_interval").as_double());
    shutdown_lie_down_settle_duration_ =
      std::max(0.0, get_parameter("shutdown_lie_down_settle_duration").as_double());
    shutdown_passive_timeout_ =
      std::max(0.0, get_parameter("shutdown_passive_timeout").as_double());
    shutdown_passive_retry_interval_ =
      std::max(0.01, get_parameter("shutdown_passive_retry_interval").as_double());
  }

  static std::string trim(const std::string & input)
  {
    const auto first = input.find_first_not_of(" \t\n\r");
    if (first == std::string::npos) {
      return {};
    }

    const auto last = input.find_last_not_of(" \t\n\r");
    return input.substr(first, last - first + 1);
  }

  static float clampWithDeadband(
    const float value,
    const float limit,
    const float deadband)
  {
    const float clamped = std::clamp(value, -limit, limit);
    return std::abs(clamped) < deadband ? 0.0F : clamped;
  }

  void queueAction(const Action action)
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    action_ = action;
    action_initialized_ = false;
    action_start_time_ = now();
    next_action_attempt_time_ = SteadyClock::now();

    velocity_cmd_ = VelocityCommand{};
    attitude_cmd_ = AttitudeCommand{};
    motion_active_ = false;
    body_active_ = false;
    stop_active_ = false;
    cmd_vel_watchdog_armed_ = false;
    cmd_vel_dirty_ = false;
  }

  void clearMotionCommand()
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    velocity_cmd_ = VelocityCommand{};
    attitude_cmd_ = AttitudeCommand{};
    motion_active_ = false;
    body_active_ = false;
    stop_active_ = false;
    cmd_vel_watchdog_armed_ = false;
    cmd_vel_dirty_ = false;
    publisher_loss_stop_sent_ = false;
    last_cmd_vel_time_ = SteadyClock::now();
  }

  void requestStopHold()
  {
    std::lock_guard<std::mutex> lock(command_mutex_);

    velocity_cmd_ = VelocityCommand{};
    attitude_cmd_ = AttitudeCommand{};
    motion_active_ = false;
    body_active_ = false;

    // 进入有限时长的零速度保持。
    // 看门狗已经修复，因此无需永久发送零速度。
    stop_active_ = true;
    cmd_vel_watchdog_armed_ = false;
    last_cmd_vel_time_ = SteadyClock::now();
    stop_until_time_ =
      SteadyClock::now() +
      std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(stop_hold_duration_));
  }

  uint32_t sendZeroVelocity()
  {
    const VelocityCommand zero{};
    return sendVelocity(active_gait_, zero);
  }

  void shutdownRobotSafely() noexcept
  {
    try {
      if (!sdk_) {
        return;
      }

      bool connected = connected_.load();
      try {
        connected = sdk_->checkConnect();
      } catch (...) {
        // 保留connected_中的最后状态。
      }

      if (!connected) {
        return;
      }

      RCLCPP_INFO(
        get_logger(),
        "Driver shutdown requested: stop -> lieDown -> passive");

      // 1) 如果机器人运动过，先发送零速度，让机器人停止。
      if (motion_command_sent_.load()) {
        const auto stop_end =
          SteadyClock::now() +
          std::chrono::duration_cast<SteadyClock::duration>(
          std::chrono::duration<double>(shutdown_stop_duration_));

        while (SteadyClock::now() < stop_end) {
          const uint32_t ret = sendZeroVelocity();
          if (ret != 0U) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 500,
              "Shutdown zero-velocity command returned code %u", ret);
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
      }

      // 特殊步态先退出，避免crawl/climb状态影响趴下。
      try {
        cancelActiveGait();
      } catch (...) {
      }

      // 2) 先执行趴下。
      if (!shutdown_lie_down_) {
        RCLCPP_INFO(
          get_logger(),
          "shutdown_lie_down=false; skip lieDown/passive shutdown sequence");
        return;
      }

      bool lie_down_accepted = false;
      uint32_t lie_down_ret = 0U;

      const auto lie_down_deadline =
        SteadyClock::now() +
        std::chrono::duration_cast<SteadyClock::duration>(
        std::chrono::duration<double>(shutdown_lie_down_timeout_));

      while (SteadyClock::now() < lie_down_deadline) {
        lie_down_ret = sdk_->lieDown();

        if (lie_down_ret == 0U) {
          lie_down_accepted = true;
          RCLCPP_INFO(
            get_logger(),
            "Shutdown lieDown command accepted; waiting %.2f s for lying-down motion",
            shutdown_lie_down_settle_duration_);
          break;
        }

        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Shutdown lieDown returned code %u", lie_down_ret);

        std::this_thread::sleep_for(
          std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(shutdown_lie_down_retry_interval_)));
      }

      if (!lie_down_accepted) {
        RCLCPP_WARN(
          get_logger(),
          "Shutdown lieDown was not accepted within %.2f s; "
          "passive will not be sent",
          shutdown_lie_down_timeout_);
        return;
      }

      // lieDown()返回0只表示命令被接受，不代表机械动作瞬间完成。
      // 因此先等待趴下动作，再发送passive()。
      if (shutdown_lie_down_settle_duration_ > 0.0) {
        std::this_thread::sleep_for(
          std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(shutdown_lie_down_settle_duration_)));
      }

      // 3) 趴下之后再进入阻尼。
      if (!shutdown_passive_after_lie_down_) {
        RCLCPP_INFO(
          get_logger(),
          "Robot lieDown completed; passive-after-lie-down is disabled");
        return;
      }

      bool passive_accepted = false;
      uint32_t passive_ret = 0U;

      const auto passive_deadline =
        SteadyClock::now() +
        std::chrono::duration_cast<SteadyClock::duration>(
        std::chrono::duration<double>(shutdown_passive_timeout_));

      while (SteadyClock::now() < passive_deadline) {
        passive_ret = sdk_->passive();

        if (passive_ret == 0U) {
          passive_accepted = true;
          break;
        }

        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Shutdown passive returned code %u", passive_ret);

        std::this_thread::sleep_for(
          std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(shutdown_passive_retry_interval_)));
      }

      if (passive_accepted) {
        RCLCPP_INFO(
          get_logger(),
          "Shutdown sequence completed: lieDown -> passive");
      } else {
        RCLCPP_WARN(
          get_logger(),
          "Shutdown passive was not accepted within %.2f s; last return code=%u",
          shutdown_passive_timeout_, passive_ret);
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(),
        "Exception during shutdown lieDown->passive sequence: %s",
        error.what());
    } catch (...) {
      RCLCPP_ERROR(
        get_logger(),
        "Unknown exception during shutdown lieDown->passive sequence");
    }
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const float vx = static_cast<float>(msg->linear.x);
    const float vy = static_cast<float>(msg->linear.y);
    const float yaw = static_cast<float>(msg->angular.z);

    const bool has_motion =
      std::abs(vx) >= static_cast<float>(vx_deadband_) ||
      std::abs(vy) >= static_cast<float>(vy_deadband_) ||
      std::abs(yaw) >= static_cast<float>(yaw_deadband_);

    std::lock_guard<std::mutex> lock(command_mutex_);

    last_cmd_vel_time_ = SteadyClock::now();
    body_active_ = false;

    // SCAN-Planner适配：
    // 每收到一条ROS cmd_vel，只让controlLoop向SDK发送一次。
    // SDK demo也是按键时调用一次move()，之后由底层保持命令，
    // 不需要500Hz/5Hz持续重复调用move()。
    if (has_motion) {
      velocity_cmd_.vx = vx;
      velocity_cmd_.vy = vy;
      velocity_cmd_.yaw_rate = yaw;

      motion_active_ = true;
      stop_active_ = false;
      cmd_vel_dirty_ = true;
      publisher_loss_stop_sent_ = false;

      // cmd_vel_timeout=0时禁用按消息间隔停车。
      cmd_vel_watchdog_armed_ = cmd_vel_timeout_ > 0.0;
    } else {
      // 上游明确发布零速度时，只向SDK发送一次move(0,0,0)。
      velocity_cmd_ = VelocityCommand{};
      motion_active_ = false;
      stop_active_ = false;
      cmd_vel_dirty_ = true;
      cmd_vel_watchdog_armed_ = false;
      publisher_loss_stop_sent_ = false;
    }
  }

  void bodyCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (!standing_.load()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring body_cmd because robot is not standing");
      return;
    }

    std::lock_guard<std::mutex> lock(command_mutex_);

    attitude_cmd_.roll_vel = static_cast<float>(msg->angular.x);
    attitude_cmd_.pitch_vel = static_cast<float>(msg->angular.y);
    attitude_cmd_.yaw_vel = static_cast<float>(msg->angular.z);
    attitude_cmd_.height_vel = static_cast<float>(msg->linear.z);

    body_cmd_expire_time_ =
      SteadyClock::now() +
      std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(body_cmd_timeout_));

    body_active_ = true;
    motion_active_ = false;
    stop_active_ = false;
  }

  void highLevelCmdCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    const auto command = trim(msg->data);

    if (command == "stand_up") {
      queueAction(Action::STAND_UP);
    } else if (command == "lie_down") {
      auto_stand_pending_ = false;
      queueAction(Action::LIE_DOWN);
    } else if (command == "passive") {
      auto_stand_pending_ = false;
      queueAction(Action::PASSIVE);
    } else if (command == "stop") {
      requestStopHold();
    } else if (
      command == "shake_hand" || command == "greet" || command == "hello")
    {
      queueAction(Action::SHAKE_HAND);
    } else if (command == "crawl_on") {
      requestGait(Gait::CRAWL);
    } else if (command == "crawl_off") {
      requestGait(Gait::NORMAL);
    } else if (command == "climb_on") {
      requestGait(Gait::CLIMB);
    } else if (command == "climb_off") {
      requestGait(Gait::NORMAL);
    } else {
      RCLCPP_WARN(
        get_logger(), "Unknown high_level_cmd: '%s'", command.c_str());
    }
  }

  void standUpCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (!connected_.load()) {
      response->success = false;
      response->message = "SDK is not connected";
      return;
    }

    auto_stand_pending_ = false;
    queueAction(Action::STAND_UP);

    response->success = true;
    response->message = "standUp command queued";
  }

  void lieDownCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (!connected_.load()) {
      response->success = false;
      response->message = "SDK is not connected";
      return;
    }

    auto_stand_pending_ = false;
    queueAction(Action::LIE_DOWN);

    response->success = true;
    response->message = "lieDown command queued";
  }

  void passiveCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (!connected_.load()) {
      response->success = false;
      response->message = "SDK is not connected";
      return;
    }

    auto_stand_pending_ = false;
    queueAction(Action::PASSIVE);

    response->success = true;
    response->message = "passive command queued";
  }

  void stopCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    requestStopHold();

    response->success = connected_.load();
    response->message = connected_.load() ?
      "Zero-velocity hold requested" :
      "Motion command cleared, but SDK is not connected";
  }

  void shakeHandCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (!standing_.load()) {
      response->success = false;
      response->message = "Robot must be standing before shakeHand";
      return;
    }

    queueAction(Action::SHAKE_HAND);

    response->success = true;
    response->message = "shakeHand command queued";
  }

  void requestGait(const Gait gait)
  {
    std::lock_guard<std::mutex> lock(command_mutex_);

    if (requested_gait_ == gait && !gait_transition_pending_) {
      return;
    }

    requested_gait_ = gait;
    gait_transition_pending_ = true;
    gait_transition_start_time_ = now();

    velocity_cmd_ = VelocityCommand{};
    motion_active_ = false;
    stop_active_ = false;
  }

  void crawlCallback(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    requestGait(request->data ? Gait::CRAWL : Gait::NORMAL);

    response->success = true;
    response->message = request->data ? "crawl gait requested" : "normal gait requested";
  }

  void climbCallback(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    requestGait(request->data ? Gait::CLIMB : Gait::NORMAL);

    response->success = true;
    response->message = request->data ? "climb gait requested" : "normal gait requested";
  }

  void attitudeControlCallback(
    const agibot_d1w_interfaces::srv::AttitudeControl::Request::SharedPtr request,
    agibot_d1w_interfaces::srv::AttitudeControl::Response::SharedPtr response)
  {
    if (!standing_.load()) {
      response->success = false;
      response->message = "Robot must be standing before attitude control";
      return;
    }

    const double duration =
      request->duration_sec > 0.0F ?
      static_cast<double>(request->duration_sec) : body_cmd_timeout_;

    std::lock_guard<std::mutex> lock(command_mutex_);

    attitude_cmd_.roll_vel = request->roll_vel;
    attitude_cmd_.pitch_vel = request->pitch_vel;
    attitude_cmd_.yaw_vel = request->yaw_vel;
    attitude_cmd_.height_vel = request->height_vel;

    body_cmd_expire_time_ =
      SteadyClock::now() +
      std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(duration));

    body_active_ = true;
    motion_active_ = false;
    stop_active_ = false;

    response->success = true;
    response->message = "attitude command accepted";
  }

  uint32_t sendVelocity(const Gait gait, const VelocityCommand & command)
  {
    switch (gait) {
      case Gait::CRAWL:
        return sdk_->crawl(command.vx, command.vy, command.yaw_rate);

      case Gait::CLIMB:
        return sdk_->climb(command.vx, command.vy, command.yaw_rate);

      case Gait::NORMAL:
      default:
        return sdk_->move(command.vx, command.vy, command.yaw_rate);
    }
  }

  void cancelActiveGait()
  {
    if (active_gait_ == Gait::CRAWL) {
      sdk_->cancelCrawl();
    } else if (active_gait_ == Gait::CLIMB) {
      sdk_->cancelClimb();
    }

    active_gait_ = Gait::NORMAL;
  }

  void completeAction(const char * message)
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    action_ = Action::NONE;
    action_initialized_ = false;

    RCLCPP_INFO(get_logger(), "%s", message);
  }

  void processAction(const Action action, const rclcpp::Time & action_start)
  {
    if ((now() - action_start).seconds() > action_timeout_) {
      completeAction("High-level action timed out");
      return;
    }

    const bool first_action_cycle = !action_initialized_;

    if (first_action_cycle) {
      // 只有机器人已经进入运动模式时才发送零速度。
      // 站立模式下调用move(0,0,0)会触发不必要的Move状态切换。
      if (control_mode_.load() == 3U || motion_command_sent_.load()) {
        (void)sendZeroVelocity();
      }

      cancelActiveGait();
      requested_gait_ = Gait::NORMAL;
      action_initialized_ = true;
      next_action_attempt_time_ = SteadyClock::now();
    }

    if (action == Action::STAND_UP && !first_action_cycle && standing_.load()) {
      completeAction("Robot reports standing; waiting for stand settle time before motion");
      return;
    }

    if (action == Action::LIE_DOWN && control_mode_.load() == 0U) {
      standing_.store(false);
      completeAction("Robot entered damping/lying state");
      return;
    }

    if (action == Action::SHAKE_HAND && !standing_.load()) {
      completeAction("shakeHand cancelled because robot is not standing");
      return;
    }

    if (SteadyClock::now() < next_action_attempt_time_) {
      return;
    }

    next_action_attempt_time_ =
      SteadyClock::now() +
      std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(action_retry_interval_));

    uint32_t ret = 0U;

    switch (action) {
      case Action::STAND_UP:
        ret = sdk_->standUp();
        break;

      case Action::LIE_DOWN:
        ret = sdk_->lieDown();
        break;

      case Action::PASSIVE:
        ret = sdk_->passive();
        if (ret == 0U) {
          standing_.store(false);
          completeAction("Robot entered passive mode");
          return;
        }
        break;

      case Action::SHAKE_HAND:
        ret = sdk_->shakeHand();
        if (ret == 0U) {
          completeAction("shakeHand command accepted");
          return;
        }
        break;

      case Action::NONE:
      default:
        return;
    }

    if (ret != 0U) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "High-level action returned code %u", ret);
    }
  }

  void maybeQueueAutoStand()
  {
    if (!auto_stand_pending_ || !connected_.load()) {
      return;
    }

    if (SteadyClock::now() < auto_stand_ready_time_) {
      return;
    }

    auto_stand_pending_ = false;

    RCLCPP_INFO(
      get_logger(),
      "Auto stand-up command queued (current mode=%u)",
      control_mode_.load());
    queueAction(Action::STAND_UP);
  }

  void controlLoop()
  {
    if (!connected_.load()) {
      return;
    }

    maybeQueueAutoStand();

    // ------------------------------------------------------------
    // SCAN-Planner适配
    //
    // cmd_vel_timeout=0：不根据消息间隔停车，保持SDK最后一次move命令。
    // cmd_vel_timeout>0：超过指定时间后只生成“一次”零速度命令，
    // 不再进入500Hz零速度保持，避免反复Move/Stop造成踏步。
    // ------------------------------------------------------------
    bool watchdog_timed_out = false;
    double watchdog_age = 0.0;

    {
      std::lock_guard<std::mutex> lock(command_mutex_);

      if (cmd_vel_timeout_ > 0.0 && cmd_vel_watchdog_armed_) {
        watchdog_age = std::chrono::duration<double>(
          SteadyClock::now() - last_cmd_vel_time_).count();

        if (watchdog_age > cmd_vel_timeout_) {
          velocity_cmd_ = VelocityCommand{};
          motion_active_ = false;
          cmd_vel_dirty_ = true;
          cmd_vel_watchdog_armed_ = false;
          watchdog_timed_out = true;
        }
      }
    }

    if (watchdog_timed_out) {
      RCLCPP_WARN(
        get_logger(),
        "cmd_vel stale for %.3f s; sending one zero-velocity command",
        watchdog_age);
    }

    // 发布者消失（例如SCAN-Planner退出、ros2 topic pub Ctrl+C）时停车。
    // 这比0.25s消息间隔watchdog更适合“事件式/稀疏cmd_vel”。
    if (count_publishers("cmd_vel") == 0U &&
      motion_command_sent_.load() &&
      !publisher_loss_stop_sent_.load())
    {
      {
        std::lock_guard<std::mutex> lock(command_mutex_);
        velocity_cmd_ = VelocityCommand{};
        motion_active_ = false;
        cmd_vel_dirty_ = true;
        cmd_vel_watchdog_armed_ = false;
      }

      publisher_loss_stop_sent_.store(true);

      RCLCPP_WARN(
        get_logger(),
        "cmd_vel publisher disappeared; sending one zero-velocity command");
    }

    Action action;
    rclcpp::Time action_start(0, 0, get_clock()->get_clock_type());
    VelocityCommand velocity;
    AttitudeCommand attitude;
    bool motion_active = false;
    bool body_active = false;
    bool stop_active = false;
    bool cmd_vel_dirty = false;
    bool gait_transition_pending = false;
    Gait requested_gait;
    SteadyClock::time_point body_expire;
    SteadyClock::time_point stop_until;
    rclcpp::Time gait_transition_start(0, 0, get_clock()->get_clock_type());

    {
      std::lock_guard<std::mutex> lock(command_mutex_);

      action = action_;
      action_start = action_start_time_;
      velocity = velocity_cmd_;
      attitude = attitude_cmd_;
      motion_active = motion_active_;
      body_active = body_active_;
      stop_active = stop_active_;
      cmd_vel_dirty = cmd_vel_dirty_;
      gait_transition_pending = gait_transition_pending_;
      requested_gait = requested_gait_;
      body_expire = body_cmd_expire_time_;
      stop_until = stop_until_time_;
      gait_transition_start = gait_transition_start_time_;
    }

    if (action != Action::NONE) {
      processAction(action, action_start);
      return;
    }

    if (gait_transition_pending) {
      if (control_mode_.load() == 3U || motion_command_sent_.load()) {
        (void)sendZeroVelocity();
      }

      if ((now() - gait_transition_start).seconds() >= gait_switch_settle_) {
        cancelActiveGait();
        active_gait_ = requested_gait;

        std::lock_guard<std::mutex> lock(command_mutex_);
        gait_transition_pending_ = false;

        RCLCPP_INFO(get_logger(), "Gait switched");
      }

      return;
    }

    // 停车保持放在 standing 判断之前，保证任何模式下都能发送零速度。
    if (stop_active) {
      if (SteadyClock::now() <= stop_until) {
        const uint32_t ret = sendZeroVelocity();

        if (ret != 0U) {
          RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "zero-velocity hold returned code %u (mode=%u)",
            ret,
            control_mode_.load());
        }
      } else {
        std::lock_guard<std::mutex> lock(command_mutex_);
        stop_active_ = false;

        RCLCPP_INFO(
          get_logger(),
          "Zero-velocity hold completed; ready for a new cmd_vel");
      }

      return;
    }

    // 上游显式零速度、可选超时、publisher消失：
    // 只向SDK发送一次move(0,0,0)，然后等待下一条ROS命令。
    if (cmd_vel_dirty && !motion_active) {
      // mode=0时已经是阻尼/非运动状态，无需调用move(0)。
      if (control_mode_.load() == 0U) {
        std::lock_guard<std::mutex> lock(command_mutex_);
        cmd_vel_dirty_ = false;
        motion_command_sent_.store(false);
        return;
      }

      const uint32_t ret = sendZeroVelocity();

      if (ret == 0U) {
        std::lock_guard<std::mutex> lock(command_mutex_);
        cmd_vel_dirty_ = false;
        motion_command_sent_.store(false);
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "zero velocity command returned code %u (mode=%u)",
          ret, control_mode_.load());
      }

      return;
    }

    // 只有明确处于阻尼/趴下模式(mode=0)时才重新请求站立。
    // SDK可能在站立和Move切换期间短暂报告其他过渡模式；
    if (!standing_.load()) {
      if (motion_active && control_mode_.load() == 0U) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Non-zero cmd_vel received in damping mode; requesting stand-up");
        queueAction(Action::STAND_UP);
      } else if (motion_active) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Non-zero cmd_vel is waiting for a valid standing mode (current mode=%u)",
          control_mode_.load());
      }
      return;
    }

    if (body_active) {
      if (SteadyClock::now() <= body_expire) {
        const float roll = clampWithDeadband(
          attitude.roll_vel, static_cast<float>(max_roll_vel_), 0.0F);
        const float pitch = clampWithDeadband(
          attitude.pitch_vel, static_cast<float>(max_pitch_vel_), 0.0F);
        const float yaw = clampWithDeadband(
          attitude.yaw_vel, static_cast<float>(max_attitude_yaw_vel_), 0.0F);
        const float height = clampWithDeadband(
          attitude.height_vel, static_cast<float>(max_height_vel_), 0.0F);

        const uint32_t ret = sdk_->attitudeControl(roll, pitch, yaw, height);

        if (ret != 0U) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "attitudeControl returned code %u", ret);
        }
      } else {
        (void)sdk_->attitudeControl(0.0F, 0.0F, 0.0F, 0.0F);

        std::lock_guard<std::mutex> lock(command_mutex_);
        attitude_cmd_ = AttitudeCommand{};
        body_active_ = false;
      }

      return;
    }

    if (!motion_active || !cmd_vel_dirty) {
      return;
    }

    if (SteadyClock::now() < motion_ready_time_) {
      const double wait = std::chrono::duration<double>(
        motion_ready_time_ - SteadyClock::now()).count();

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Robot is standing but not motion-ready; waiting %.2f s",
        std::max(0.0, wait));
      return;
    }

    VelocityCommand output;
    output.vx = clampWithDeadband(
      velocity.vx, static_cast<float>(max_vx_), static_cast<float>(vx_deadband_));
    output.vy = clampWithDeadband(
      velocity.vy, static_cast<float>(max_vy_), static_cast<float>(vy_deadband_));
    output.yaw_rate = clampWithDeadband(
      velocity.yaw_rate,
      static_cast<float>(max_yaw_rate_),
      static_cast<float>(yaw_deadband_));

    if (SteadyClock::now() < next_move_attempt_time_) {
      return;
    }

    const uint32_t ret = sendVelocity(active_gait_, output);

    if (ret == 0U) {
      // SDK已经接受本次速度命令。
      // 清除dirty，后续500Hz controlLoop不会重复调用move()。
      std::lock_guard<std::mutex> lock(command_mutex_);
      cmd_vel_dirty_ = false;
      motion_command_sent_.store(true);
      next_move_attempt_time_ = SteadyClock::now();
    } else {
      // 只有SDK拒绝本次命令时才重试。
      next_move_attempt_time_ =
        SteadyClock::now() +
        std::chrono::duration_cast<SteadyClock::duration>(
        std::chrono::duration<double>(move_transition_retry_interval_));

      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "velocity command returned code %u (mode=%u); will retry",
        ret, control_mode_.load());
    }
  }

  template<std::size_t N>
  static bool copyVector(
    const std::vector<float> & source,
    std::array<float, N> & destination)
  {
    if (source.size() < N) {
      return false;
    }

    std::copy_n(source.begin(), N, destination.begin());
    return true;
  }

  void stateLoop()
  {
    auto state = agibot_d1w_interfaces::msg::RobotState();

    state.header.stamp = now();
    state.header.frame_id = base_frame_;
    state.connected = connected_.load();

    if (!state.connected) {
      state_pub_->publish(state);
      return;
    }

    try {
      state.control_mode = static_cast<uint8_t>(sdk_->getCurrentCtrlmode());

      const uint32_t previous_mode =
        control_mode_.exchange(static_cast<uint32_t>(state.control_mode));

      if (previous_mode != static_cast<uint32_t>(state.control_mode)) {
        RCLCPP_INFO(
          get_logger(),
          "Control mode changed: %u -> %u",
          previous_mode,
          static_cast<uint32_t>(state.control_mode));
      }

      const bool was_standing = standing_.load();

      if (state.control_mode == 1U || state.control_mode == 3U) {
        // 1=站立，3=运动：明确属于可运动支撑状态。
        standing_.store(true);

        if (!was_standing) {
          motion_ready_time_ =
            SteadyClock::now() +
            std::chrono::duration_cast<SteadyClock::duration>(
            std::chrono::duration<double>(stand_settle_duration_));

          RCLCPP_INFO(
            get_logger(),
            "Standing mode detected; motion will be enabled after %.2f s",
            stand_settle_duration_);
        }

        if (state.control_mode == 3U) {
          motion_command_sent_.store(true);
          motion_ready_time_ = SteadyClock::now();
        }
      } else if (state.control_mode == 0U) {
        // 0=阻尼/趴下：明确不是站立状态。
        standing_.store(false);
        motion_command_sent_.store(false);
      } else {
          // 其他值按SDK过渡模式处理。
          // 保留之前的standing状态，避免1 -> 过渡模式 -> 1时反复重置等待时间。
          // 不输出警告，避免固件长期返回未公开模式（例如18）时刷屏。
      }

      state.battery_percent = static_cast<uint8_t>(
        std::min<uint32_t>(100U, sdk_->getBatteryPower()));

      copyVector(sdk_->getQuaternion(), state.orientation_wxyz);
      copyVector(sdk_->getRPY(), state.rpy);
      copyVector(sdk_->getPosition(), state.position);
      copyVector(sdk_->getWorldVelocity(), state.world_velocity);
      copyVector(sdk_->getBodyVelocity(), state.body_velocity);
      copyVector(sdk_->getBodyGyro(), state.angular_velocity);
      copyVector(sdk_->getBodyAcc(), state.linear_acceleration);

      copyVector(sdk_->getLegAbadJoint(), state.abad_joint_angles);
      copyVector(sdk_->getLegHipJoint(), state.hip_joint_angles);
      copyVector(sdk_->getLegKneeJoint(), state.knee_joint_angles);
      copyVector(sdk_->getLegFootJoint(), state.foot_joint_angles);

      copyVector(sdk_->getLegAbadJointVel(), state.abad_joint_velocities);
      copyVector(sdk_->getLegHipJointVel(), state.hip_joint_velocities);
      copyVector(sdk_->getLegKneeJointVel(), state.knee_joint_velocities);
      copyVector(sdk_->getLegFootJointVel(), state.foot_joint_velocities);

      copyVector(sdk_->getLegAbadJointTorque(), state.abad_joint_torques);
      copyVector(sdk_->getLegHipJointTorque(), state.hip_joint_torques);
      copyVector(sdk_->getLegKneeJointTorque(), state.knee_joint_torques);
      copyVector(sdk_->getLegFootJointTorque(), state.foot_joint_torques);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "SDK state read failed: %s", error.what());
    } catch (...) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "SDK state read failed");
    }

    state_pub_->publish(state);
    publishStandardMessages(state);
  }

  void imuLoop()
  {
    if (!publish_imu_ || !connected_.load()) {
      return;
    }

    try {
      // 直接从SDK读取IMU相关数据，独立于50Hz stateLoop。
      const auto quaternion = sdk_->getQuaternion();
      const auto gyro = sdk_->getBodyGyro();
      const auto acc = sdk_->getBodyAcc();

      if (quaternion.size() < 4 || gyro.size() < 3 || acc.size() < 3) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "SDK IMU data size is invalid");
        return;
      }

      sensor_msgs::msg::Imu imu;
      imu.header.stamp = now();
      imu.header.frame_id = base_frame_;

      // SDK四元数顺序：[w, x, y, z]
      imu.orientation.w = quaternion[0];
      imu.orientation.x = quaternion[1];
      imu.orientation.y = quaternion[2];
      imu.orientation.z = quaternion[3];

      imu.angular_velocity.x = gyro[0];
      imu.angular_velocity.y = gyro[1];
      imu.angular_velocity.z = gyro[2];

      imu.linear_acceleration.x = acc[0];
      imu.linear_acceleration.y = acc[1];
      imu.linear_acceleration.z = acc[2];

      imu_pub_->publish(imu);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "SDK IMU read failed: %s", error.what());
    } catch (...) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "SDK IMU read failed");
    }
  }

  void publishStandardMessages(
    const agibot_d1w_interfaces::msg::RobotState & state)
  {
    if (publish_odom_) {
      nav_msgs::msg::Odometry odom;

      odom.header = state.header;
      odom.header.frame_id = odom_frame_;
      odom.child_frame_id = base_frame_;

      odom.pose.pose.position.x = state.position[0];
      odom.pose.pose.position.y = state.position[1];
      odom.pose.pose.position.z = state.position[2];

      odom.pose.pose.orientation.w = state.orientation_wxyz[0];
      odom.pose.pose.orientation.x = state.orientation_wxyz[1];
      odom.pose.pose.orientation.y = state.orientation_wxyz[2];
      odom.pose.pose.orientation.z = state.orientation_wxyz[3];

      odom.twist.twist.linear.x = state.body_velocity[0];
      odom.twist.twist.linear.y = state.body_velocity[1];
      odom.twist.twist.linear.z = state.body_velocity[2];

      odom.twist.twist.angular.x = state.angular_velocity[0];
      odom.twist.twist.angular.y = state.angular_velocity[1];
      odom.twist.twist.angular.z = state.angular_velocity[2];

      odom_pub_->publish(odom);
    }

    if (publish_joint_states_) {
      sensor_msgs::msg::JointState joint_state;

      joint_state.header = state.header;
      joint_state.name = {
        "fr_abad_joint", "fr_hip_joint", "fr_knee_joint", "fr_foot_joint",
        "fl_abad_joint", "fl_hip_joint", "fl_knee_joint", "fl_foot_joint",
        "rr_abad_joint", "rr_hip_joint", "rr_knee_joint", "rr_foot_joint",
        "rl_abad_joint", "rl_hip_joint", "rl_knee_joint", "rl_foot_joint"};

      joint_state.position.resize(16);
      joint_state.velocity.resize(16);
      joint_state.effort.resize(16);

      for (std::size_t leg = 0; leg < 4; ++leg) {
        const std::size_t offset = leg * 4;

        joint_state.position[offset + 0] = state.abad_joint_angles[leg];
        joint_state.position[offset + 1] = state.hip_joint_angles[leg];
        joint_state.position[offset + 2] = state.knee_joint_angles[leg];
        joint_state.position[offset + 3] = state.foot_joint_angles[leg];

        joint_state.velocity[offset + 0] = state.abad_joint_velocities[leg];
        joint_state.velocity[offset + 1] = state.hip_joint_velocities[leg];
        joint_state.velocity[offset + 2] = state.knee_joint_velocities[leg];
        joint_state.velocity[offset + 3] = state.foot_joint_velocities[leg];

        joint_state.effort[offset + 0] = state.abad_joint_torques[leg];
        joint_state.effort[offset + 1] = state.hip_joint_torques[leg];
        joint_state.effort[offset + 2] = state.knee_joint_torques[leg];
        joint_state.effort[offset + 3] = state.foot_joint_torques[leg];
      }

      joint_state_pub_->publish(joint_state);
    }
  }

  void connectionLoop()
  {
    bool connected = false;

    try {
      connected = sdk_->checkConnect();
    } catch (...) {
      connected = false;
    }

    const bool previous = connected_.exchange(connected);

    if (connected && !previous) {
      RCLCPP_INFO(get_logger(), "SDK connection established");

      clearMotionCommand();

      if (auto_stand_up_) {
        auto_stand_pending_ = true;
        auto_stand_ready_time_ =
          SteadyClock::now() +
          std::chrono::duration_cast<SteadyClock::duration>(
          std::chrono::duration<double>(auto_stand_delay_));

        RCLCPP_INFO(
          get_logger(),
          "Auto stand-up will be checked after %.2f s",
          auto_stand_delay_);
      }
    } else if (!connected && previous) {
      RCLCPP_ERROR(get_logger(), "SDK connection lost");

      standing_.store(false);
      motion_command_sent_.store(false);
      auto_stand_pending_ = false;
      motion_ready_time_ = SteadyClock::now();

      clearMotionCommand();

      std::lock_guard<std::mutex> lock(command_mutex_);
      action_ = Action::NONE;
      action_initialized_ = false;
    }
  }

  std::unique_ptr<mc_sdk::zsl_1w::HighLevel> sdk_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr body_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr high_level_cmd_sub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stand_up_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr lie_down_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr passive_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr shake_hand_srv_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr crawl_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr climb_srv_;

  rclcpp::Service<agibot_d1w_interfaces::srv::AttitudeControl>::SharedPtr
    attitude_control_srv_;

  rclcpp::Publisher<agibot_d1w_interfaces::msg::RobotState>::SharedPtr state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr imu_timer_;
  rclcpp::TimerBase::SharedPtr connection_timer_;

  std::string local_ip_;
  int local_port_{43988};
  std::string dog_ip_;

  double control_rate_{500.0};
  double state_rate_{50.0};
  double imu_rate_{200.0};
  double connection_rate_{1.0};

  bool auto_stand_up_{true};
  double auto_stand_delay_{1.00};
  double stand_settle_duration_{2.00};
  bool auto_stand_pending_{false};
  SteadyClock::time_point auto_stand_ready_time_;
  SteadyClock::time_point motion_ready_time_;

  double cmd_vel_timeout_{0.0};
  double stop_hold_duration_{1.00};
  double body_cmd_timeout_{0.25};
  double action_timeout_{10.0};
  double action_retry_interval_{0.10};
  double gait_switch_settle_{0.30};
  double move_transition_retry_interval_{0.20};

  double max_vx_{0.30};
  double max_vy_{0.20};
  double max_yaw_rate_{0.50};
  double vx_deadband_{0.05};
  double vy_deadband_{0.10};
  double yaw_deadband_{0.10};

  double max_roll_vel_{0.30};
  double max_pitch_vel_{0.30};
  double max_attitude_yaw_vel_{0.30};
  double max_height_vel_{0.20};

  bool publish_odom_{true};
  bool publish_imu_{true};
  bool publish_joint_states_{true};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};

  int shutdown_zero_command_count_{20};

  bool shutdown_lie_down_{true};
  bool shutdown_passive_after_lie_down_{true};
  double shutdown_stop_duration_{0.50};
  double shutdown_lie_down_timeout_{5.00};
  double shutdown_lie_down_retry_interval_{0.10};
  double shutdown_lie_down_settle_duration_{2.00};
  double shutdown_passive_timeout_{3.00};
  double shutdown_passive_retry_interval_{0.10};

  std::atomic_bool connected_{false};
  std::atomic_bool standing_{false};
  std::atomic_bool motion_command_sent_{false};
  std::atomic<uint32_t> control_mode_{0U};

  std::mutex command_mutex_;

  Action action_{Action::NONE};
  bool action_initialized_{false};
  rclcpp::Time action_start_time_;

  VelocityCommand velocity_cmd_;
  AttitudeCommand attitude_cmd_;

  bool motion_active_{false};
  bool body_active_{false};
  bool stop_active_{false};

  // 收到非零 cmd_vel 后置为 true；超时或停车后清除。
  // 受 command_mutex_ 保护。
  bool cmd_vel_watchdog_armed_{false};

  // 每收到一条新的cmd_vel置true；SDK成功接收后清false。
  // 让move()从“控制循环持续调用”变成“ROS消息事件触发一次”。
  bool cmd_vel_dirty_{false};

  // cmd_vel publisher消失时只发送一次零速度。
  std::atomic_bool publisher_loss_stop_sent_{false};

  SteadyClock::time_point last_cmd_vel_time_;
  SteadyClock::time_point body_cmd_expire_time_;
  SteadyClock::time_point stop_until_time_;
  SteadyClock::time_point next_move_attempt_time_;
  SteadyClock::time_point next_action_attempt_time_;

  Gait active_gait_{Gait::NORMAL};
  Gait requested_gait_{Gait::NORMAL};
  bool gait_transition_pending_{false};
  rclcpp::Time gait_transition_start_time_;
};

}  // namespace agibot_d1w_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<agibot_d1w_driver::RobotDriver>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("agibot_d1w_driver"),
      "Fatal error: %s", error.what());
  }

  rclcpp::shutdown();
  return 0;
}