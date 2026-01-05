# b2w_joystick_control

ROS 2 package for controlling the B2W quadrupedal robot using a PS5 (or other) joystick controller.

## Overview

This package provides joystick teleoperation for the B2W robot by:
1. Running the `joy_node` to read raw joystick inputs
2. Converting joystick messages to `geometry_msgs/Twist` commands
3. Publishing velocity commands to `/path_manager/path_manager_ros/nav_vel` topic

## Features

- **PS5 Controller Support**: Pre-configured button/axis mappings for PS5 DualSense
- **Safety Enable Button**: Hold L1 to enable movement (deadman switch)
- **Turbo Mode**: Hold R1 for increased speed
- **Configurable Mappings**: Easy-to-edit YAML configuration
- **Deadzone Handling**: Filters small joystick movements
- **Multi-axis Control**: Forward/backward, strafe left/right, rotate

## PS5 Controller Layout

### Axes
- **Left Stick Up/Down**: Forward/Backward movement (`linear.x`)
- **Left Stick Left/Right**: Strafe movement (`linear.y`)
- **Right Stick Left/Right**: Rotation (`angular.z`)

### Buttons
- **L1 (Press)**: Toggle control ON/OFF
- **R1 (Press)**: Toggle turbo mode ON/OFF

## Installation

### Prerequisites

```bash
# Install joy package if not already installed
sudo apt-get install ros-jazzy-joy
```

### Build

```bash
# Navigate to your workspace root
cd /home/fanyang/Projets/sru_repos/sru-ws

# Build the package
colcon build --packages-select b2w_joystick_control --symlink-install

# Source the workspace
source install/setup.bash
```

## Usage

### Connect PS5 Controller

#### Wired Connection
Simply plug in the controller via USB cable.

#### Bluetooth Connection
```bash
# Put controller in pairing mode (hold PS + Share buttons until light flashes)
bluetoothctl
> scan on
> pair [CONTROLLER_MAC_ADDRESS]
> connect [CONTROLLER_MAC_ADDRESS]
> trust [CONTROLLER_MAC_ADDRESS]
> exit
```

### Launch Joystick Control

```bash
# Default launch (uses /dev/input/js0 and ps5_config.yaml)
ros2 launch b2w_joystick_control joystick_teleop.launch.py

# Specify custom joystick device
ros2 launch b2w_joystick_control joystick_teleop.launch.py joy_dev:=/dev/input/js1

# Use different config file
ros2 launch b2w_joystick_control joystick_teleop.launch.py config_file:=custom_config.yaml
```

### Use with B2W Simulation

```bash
# Terminal 1: Launch the B2W simulation
ros2 launch b2w_gazebo_ros2 b2w_gazebo.launch.py

# Terminal 2: Launch joystick control
ros2 launch b2w_joystick_control joystick_teleop.launch.py
```

## Configuration

Edit [config/ps5_config.yaml](config/ps5_config.yaml) to customize behavior:

```yaml
/**:
  ros__parameters:
    # Axis mappings (PS5 defaults)
    axis_linear_x: 1      # Left stick Y
    axis_linear_y: 0      # Left stick X
    axis_angular_z: 3     # Right stick X

    # Button mappings
    enable_button: 4      # L1
    turbo_button: 5       # R1

    # Speed scaling
    scale_linear: 1.0         # Normal speed (m/s)
    scale_linear_turbo: 1.5   # Turbo speed (m/s)
    scale_angular: 0.75        # Normal rotation (rad/s)
    scale_angular_turbo: 1.0  # Turbo rotation (rad/s)

    # Deadzone
    deadzone: 0.15
```

## Troubleshooting

### Controller Not Detected

```bash
# Check if joystick device exists
ls -l /dev/input/js*

# Test raw joystick input
jstest /dev/input/js0

# Check ROS joy messages
ros2 topic echo /joy
```

### Permission Issues

```bash
# Add user to input group
sudo usermod -a -G input $USER

# Logout and login again for changes to take effect
```

### Wrong Button Mappings

Different controllers have different layouts. To find your button/axis mappings:

```bash
# Run joy_node and view output
ros2 run joy joy_node

# In another terminal, watch the messages
ros2 topic echo /joy

# Press buttons and move axes to see which indices they correspond to
```

Update the config file with the correct indices.

## Topics

### Subscribed Topics
- `/joy` ([sensor_msgs/Joy](http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html)): Raw joystick input

### Published Topics
- `/path_manager/path_manager_ros/nav_vel` ([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)): Velocity commands for robot

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `axis_linear_x` | int | 1 | Joystick axis for forward/backward |
| `axis_linear_y` | int | 0 | Joystick axis for left/right strafe |
| `axis_angular_z` | int | 3 | Joystick axis for rotation |
| `enable_button` | int | 4 | Button to enable movement |
| `turbo_button` | int | 5 | Button for turbo mode |
| `scale_linear` | double | 1.0 | Normal linear velocity scale (m/s) |
| `scale_linear_turbo` | double | 1.5 | Turbo linear velocity scale (m/s) |
| `scale_angular` | double | 0.75 | Normal angular velocity scale (rad/s) |
| `scale_angular_turbo` | double | 1.0 | Turbo angular velocity scale (rad/s) |
| `deadzone` | double | 0.15 | Joystick deadzone threshold (0.0-1.0) |

## Other Controllers

To use Xbox, Logitech, or other controllers, create a new config file with appropriate button/axis mappings:

```bash
# Copy the PS5 config as a template
cp config/ps5_config.yaml config/xbox_config.yaml

# Edit the new file with correct mappings
# Launch with custom config
ros2 launch b2w_joystick_control joystick_teleop.launch.py config_file:=xbox_config.yaml
```

## License

MIT
