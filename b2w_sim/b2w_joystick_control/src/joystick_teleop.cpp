#include "b2w_joystick_control/joystick_teleop.hpp"
#include <cmath>

namespace b2w_joystick_control
{

JoystickTeleop::JoystickTeleop()
: Node("joystick_teleop"), enabled_(false), prev_turbo_(false),
  prev_enable_button_state_(false), prev_turbo_button_state_(false)
{
  // Declare parameters with defaults
  this->declare_parameter("axis_linear_x", 1);
  this->declare_parameter("axis_linear_y", 0);
  this->declare_parameter("axis_angular_z", 2);
  this->declare_parameter("enable_button", 4);  // L1/LB button
  this->declare_parameter("turbo_button", 5);   // R1/RB button

  this->declare_parameter("scale_linear", 0.5);
  this->declare_parameter("scale_linear_turbo", 1.0);
  this->declare_parameter("scale_angular", 1.0);
  this->declare_parameter("scale_angular_turbo", 1.5);
  this->declare_parameter("deadzone", 0.1);

  // Get parameters
  axis_linear_x_ = this->get_parameter("axis_linear_x").as_int();
  axis_linear_y_ = this->get_parameter("axis_linear_y").as_int();
  axis_angular_z_ = this->get_parameter("axis_angular_z").as_int();
  enable_button_ = this->get_parameter("enable_button").as_int();
  turbo_button_ = this->get_parameter("turbo_button").as_int();

  scale_linear_ = this->get_parameter("scale_linear").as_double();
  scale_linear_turbo_ = this->get_parameter("scale_linear_turbo").as_double();
  scale_angular_ = this->get_parameter("scale_angular").as_double();
  scale_angular_turbo_ = this->get_parameter("scale_angular_turbo").as_double();
  deadzone_ = this->get_parameter("deadzone").as_double();

  // Create publisher and subscriber
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/path_manager/path_manager_ros/nav_vel", 10);
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 10,
    std::bind(&JoystickTeleop::joyCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "===========================================");
  RCLCPP_INFO(this->get_logger(), "Joystick Teleop Node Started");
  RCLCPP_INFO(this->get_logger(), "Publishing to: /path_manager/path_manager_ros/nav_vel");
  RCLCPP_INFO(this->get_logger(), "===========================================");
  RCLCPP_INFO(this->get_logger(), "Controls:");
  RCLCPP_INFO(this->get_logger(), "  L1 (Button %d): Press to TOGGLE control ON/OFF", enable_button_);
  RCLCPP_INFO(this->get_logger(), "  R1 (Button %d): Press to TOGGLE TURBO mode", turbo_button_);
  RCLCPP_INFO(this->get_logger(), "  Left Stick: Forward/Backward + Strafe");
  RCLCPP_INFO(this->get_logger(), "  Right Stick: Rotation");
  RCLCPP_INFO(this->get_logger(), "===========================================");
}

void JoystickTeleop::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // Check if enable button is pressed (toggle on button press)
  if (enable_button_ >= 0 &&
      static_cast<size_t>(enable_button_) < msg->buttons.size())
  {
    bool enable_button_state = msg->buttons[enable_button_] == 1;

    // Detect rising edge (button press)
    if (enable_button_state && !prev_enable_button_state_)
    {
      enabled_ = !enabled_;  // Toggle
      if (enabled_)
      {
        RCLCPP_INFO(this->get_logger(), ">>> L1 PRESSED - Control ENABLED <<<");
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), ">>> L1 PRESSED - Control DISABLED <<<");
      }
    }
    prev_enable_button_state_ = enable_button_state;
  }
  else
  {
    // If no enable button configured, always enabled
    enabled_ = true;
  }

  auto twist = geometry_msgs::msg::Twist();

  if (enabled_)
  {
    // Check if turbo button is pressed (toggle on button press)
    if (turbo_button_ >= 0 &&
        static_cast<size_t>(turbo_button_) < msg->buttons.size())
    {
      bool turbo_button_state = msg->buttons[turbo_button_] == 1;

      // Detect rising edge (button press)
      if (turbo_button_state && !prev_turbo_button_state_)
      {
        prev_turbo_ = !prev_turbo_;  // Toggle
        if (prev_turbo_)
        {
          RCLCPP_INFO(this->get_logger(), ">>> R1 PRESSED - TURBO MODE ON <<<");
        }
        else
        {
          RCLCPP_INFO(this->get_logger(), ">>> R1 PRESSED - TURBO MODE OFF <<<");
        }
      }
      prev_turbo_button_state_ = turbo_button_state;
    }

    // Select scaling factors
    double linear_scale = prev_turbo_ ? scale_linear_turbo_ : scale_linear_;
    double angular_scale = prev_turbo_ ? scale_angular_turbo_ : scale_angular_;

    // Get axis values with deadzone
    double linear_x = 0.0;
    double linear_y = 0.0;
    double angular_z = 0.0;

    if (static_cast<size_t>(axis_linear_x_) < msg->axes.size())
    {
      linear_x = msg->axes[axis_linear_x_];
      if (std::abs(linear_x) < deadzone_) linear_x = 0.0;
    }

    if (static_cast<size_t>(axis_linear_y_) < msg->axes.size())
    {
      linear_y = msg->axes[axis_linear_y_];
      if (std::abs(linear_y) < deadzone_) linear_y = 0.0;
    }

    if (static_cast<size_t>(axis_angular_z_) < msg->axes.size())
    {
      angular_z = msg->axes[axis_angular_z_];
      if (std::abs(angular_z) < deadzone_) angular_z = 0.0;
    }

    // Apply scaling
    twist.linear.x = linear_x * linear_scale;
    twist.linear.y = linear_y * linear_scale;
    twist.angular.z = angular_z * angular_scale;

    // Print velocity commands when there's movement
    if (std::abs(twist.linear.x) > 0.01 || std::abs(twist.linear.y) > 0.01 ||
        std::abs(twist.angular.z) > 0.01)
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "Cmd vel: [%.2f, %.2f, %.2f] %s",
        twist.linear.x, twist.linear.y, twist.angular.z,
        prev_turbo_ ? "[TURBO]" : "");
    }
  }
  else
  {
    // Send zero velocity when disabled
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;
  }

  cmd_vel_pub_->publish(twist);
}

}  // namespace b2w_joystick_control
