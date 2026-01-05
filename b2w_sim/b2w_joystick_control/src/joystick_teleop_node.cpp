#include <rclcpp/rclcpp.hpp>
#include "b2w_joystick_control/joystick_teleop.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<b2w_joystick_control::JoystickTeleop>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
