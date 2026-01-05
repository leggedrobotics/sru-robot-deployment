#ifndef B2W_JOYSTICK_CONTROL__JOYSTICK_TELEOP_HPP_
#define B2W_JOYSTICK_CONTROL__JOYSTICK_TELEOP_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace b2w_joystick_control
{

class JoystickTeleop : public rclcpp::Node
{
public:
  JoystickTeleop();

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // Joystick axis and button mappings
  int axis_linear_x_;
  int axis_linear_y_;
  int axis_angular_z_;
  int enable_button_;
  int turbo_button_;

  // Velocity scaling factors
  double scale_linear_;
  double scale_linear_turbo_;
  double scale_angular_;
  double scale_angular_turbo_;

  // Deadzone threshold
  double deadzone_;

  // State tracking
  bool enabled_;
  bool prev_turbo_;
  bool prev_enable_button_state_;
  bool prev_turbo_button_state_;
};

}  // namespace b2w_joystick_control

#endif  // B2W_JOYSTICK_CONTROL__JOYSTICK_TELEOP_HPP_
