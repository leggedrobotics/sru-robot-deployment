# B2W Robot Gazebo Simulation

[![Paper](https://img.shields.io/badge/IJRR-2025-blue)](https://journals.sagepub.com/home/ijr)
[![Website](https://img.shields.io/badge/Project-Website-green)](https://michaelfyang.github.io/sru-project-website/)
[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://docs.ros.org/en/jazzy/index.html)

> **📌 Important Note**: This repository contains the **Gazebo simulation environment** for the Unitree B2W quadrupedal robot with learning-based locomotion control. This is part of the [sru-robot-deployment](https://michaelfyang.github.io/sru-project-website/) project for real robot deployment.

## About This Repository

This repository provides the **Gazebo simulation environment** for the Unitree B2W quadrupedal robot with learning-based locomotion control. The simulation stack enables sim-to-real transfer for navigation policies and provides a complete testing environment with ONNX-based neural network inference running on ROS 2 Jazzy.

**Scope of this repository:**

- ✅ Complete robot URDF/Xacro models with 23 DAE mesh files
- ✅ Gazebo simulation with ros_gz_bridge integration (46 topics/services)
- ✅ Neural network-based high-level controller (ONNX Runtime)
- ✅ Custom RViz2 panel for interactive robot control
- ✅ Training environment world file (ISAACLAB_TRAIN)
- ✅ Joystick control integration for teleoperation

**Not included in this repository:**

- ❌ Depth perception training and VAE models (see sru-depth-pretraining)
- ❌ Navigation policy training infrastructure (see project website)
- ❌ Hardware integration and ZedX camera drivers

For the **complete real robot deployment system** and **navigation experiments**, please refer to the related repositories on the [project website](https://michaelfyang.github.io/sru-project-website/).

### Related Repositories

- [sru-pytorch-spatial-learning](https://github.com/michaelfyang/sru-pytorch-spatial-learning) - Core SRU architecture
- [sru-depth-pretraining](https://github.com/michaelfyang/sru-depth-pretraining) - Depth perception pretraining
- [SRU Project Website](https://michaelfyang.github.io/sru-project-website/) - Complete navigation system

## Robot Specifications

The B2W robot features **16 degrees of freedom** across 4 legs (FL/FR/RL/RR):
- **Hip joint**: Lateral abduction/adduction (position control)
- **Thigh joint**: Forward/backward swing (position control)
- **Calf joint**: Knee flexion/extension (position control)
- **Foot joint**: Wheel rotation (velocity control)

Joint ordering: `[FL, FR, RL, RR] × [hip, thigh, calf, foot]`

## Package Structure

```
b2w_sim/
├── b2w_description_ros2/          # Robot URDF/Xacro models and meshes
│   ├── xacro/                     # Modular robot description
│   │   ├── robot.xacro           # Main robot assembly
│   │   ├── leg.xacro             # Leg macro (4 instances)
│   │   ├── const.xacro           # Physical constants
│   │   └── gazebo.xacro          # Gazebo plugins
│   └── meshes/                    # 23 DAE visual/collision meshes
│
├── b2w_gazebo_ros2/               # Gazebo launch files and configuration
│   ├── config/
│   │   └── b2w_gz_bridge.yaml    # 46 Gazebo↔ROS topic bridges
│   └── launch/
│       ├── gazebo.launch.py      # Minimal Gazebo spawn
│       └── b2w_gazebo.launch.py  # Full simulation stack
│
├── b2w_controllers/               # Neural network locomotion controller (C++)
│   ├── src/
│   │   ├── b2w_controllers.cpp   # Main controller node
│   │   └── policy/               # ONNX policy files
│   ├── third_party/
│   │   └── onnxruntime/          # Bundled ONNX Runtime library
│   └── config/
│       └── b2w_controllers.yaml  # Controller configuration
│
├── b2w_joystick_control/          # Teleoperation via game controller
│   └── config/
│       └── ps5_config.yaml       # PS5 DualSense button mapping
│
├── b2w_collision_monitor/         # Collision detection system
│   └── src/                       # Force-torque & pointcloud monitoring
│
└── b2w_sim_worlds/                # Gazebo world files
    └── worlds/
        └── ISAACLAB_TRAIN.world  # Default training environment
```

## Installation

### Prerequisites

- **ROS 2 Jazzy** (or compatible distribution)
- **Gazebo Harmonic** (or compatible version)
- **ros_gz bridge** for ROS-Gazebo communication

### Install Dependencies

```bash
# Install ros_gz bridge (replace <ros-distro> with your ROS 2 version)
sudo apt install ros-<ros-distro>-ros-gz

# For ROS 2 Jazzy
sudo apt install ros-jazzy-ros-gz
```

### Build Instructions

```bash
# Create workspace (if needed)
mkdir -p ~/b2w_ws/src
cd ~/b2w_ws/src

# Clone this repository
git clone <repo-url> b2w_sim

# Build workspace
cd ~/b2w_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source workspace
source install/setup.bash
```

**Clean build** (if needed):
```bash
rm -rf build install log
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Quick Start

### 1. Launch Robot Description Only

Load URDF and robot_state_publisher without simulation:
```bash
ros2 launch b2w_description_ros2 load.launch.py
```

### 2. Launch Minimal Gazebo Simulation

Spawn robot in Gazebo without controllers:
```bash
ros2 launch b2w_gazebo_ros2 gazebo.launch.py
```

### 3. Launch Full Simulation Stack (Recommended)

Run complete system with neural network controller:
```bash
ros2 launch b2w_gazebo_ros2 b2w_gazebo.launch.py
```

### 4. Advanced Launch Options

**Enable RViz visualization:**
```bash
ros2 launch b2w_gazebo_ros2 b2w_gazebo.launch.py \
  enable_rviz:=true \
  enable_mesh_publisher:=true
```

**Use custom environment profile:**
```bash
ros2 launch b2w_gazebo_ros2 b2w_gazebo.launch.py \
  controller_environment_profile:=crouched
```

**Load custom policy file:**
```bash
ros2 launch b2w_gazebo_ros2 b2w_gazebo.launch.py \
  controller_policy_path:=/absolute/path/to/policy.onnx
```

**Use default simulation world:**
```bash
ros2 launch b2w_gazebo_ros2 gazebo.launch.py \
  world_file:=ISAACLAB_TRAIN.world
```

**Enable RViz with robot repositioning service:**
```bash
ros2 launch b2w_gazebo_ros2 b2w_gazebo.launch.py \
  enable_set_pose_bridge:=true \
  enable_rviz:=true
```

Note: The `enable_set_pose_bridge` argument bridges Gazebo's `/world/ISAACLAB_TRAIN_world/set_pose` service to ROS 2, allowing external tools to reposition the robot during simulation.

## System Architecture

### Data Flow

```
Gazebo Simulation
    ↓↑ (ros_gz_bridge - 46 topics/services)
ROS 2 Topics
    ├→ /joint_states (sensor_msgs/JointState)
    ├→ /dlio/odom_node/odom (nav_msgs/Odometry)
    ├→ /clock, /imu, /camera, /pointcloud
    └→ 16× joint command topics (std_msgs/Float64)
        ↓↑
b2w_controllers Node (C++)
    ├─ ONNX Policy Inference (50 Hz)
    │   ├─ Input: 60 observations (joint states, odometry, commands)
    │   ├─ Hidden: 256-dim LSTM cells
    │   └─ Output: 16 joint actions
    └─ Command Publishing (200 Hz)
        ├─ 12× position commands (hip/thigh/calf)
        └─ 4× velocity commands (foot joints)
```

### Key Configuration Files

**[b2w_controllers/config/b2w_controllers.yaml](b2w_controllers/config/b2w_controllers.yaml)**
- Policy file path (default: `policy/policy_force_new.onnx`)
- 16 joint names and command topic mappings
- Default joint positions and scales
- Environment profiles: "default" and "crouched"
- Topic subscriptions: odometry, cmd_vel, joint_states

**[b2w_gazebo_ros2/config/b2w_gz_bridge.yaml](b2w_gazebo_ros2/config/b2w_gz_bridge.yaml)**
- 46 topic/service bridges between Gazebo and ROS 2
- Message type mappings (e.g., gz.msgs.Odometry → nav_msgs/Odometry)
- Camera/sensor bridges (depth, RGB, IMU, force/torque)

**[b2w_description_ros2/xacro/](b2w_description_ros2/xacro/)**
- `robot.xacro`: Main robot assembly
- `leg.xacro`: Leg macro (instantiated 4 times)
- `const.xacro`: Physical constants and dimensions
- `gazebo.xacro`: Gazebo plugins (odometry, joint states, sensors)

### ONNX Runtime Integration

The controller uses a **bundled ONNX Runtime** library:
- Location: [b2w_controllers/third_party/onnxruntime/](b2w_controllers/third_party/onnxruntime/)
- Library: `lib64/libonnxruntime.so`
- Headers: `include/onnxruntime_cxx_api.h`
- Not system-installed, linked directly via CMakeLists.txt

**Neural Network Policy:**
- Location: [b2w_controllers/src/policy/policy_force_new.onnx](b2w_controllers/src/policy/policy_force_new.onnx)
- Input size: 60 observations
- Hidden state: 256-dim LSTM
- Output: 16 joint actions

## Development

### Modifying Robot Description

1. Edit Xacro files in [b2w_description_ros2/xacro/](b2w_description_ros2/xacro/)
2. Rebuild package: `colcon build --packages-select b2w_description_ros2 --symlink-install`
3. URDFs are generated at build time from Xacro templates
4. With `--symlink-install`, Xacro changes are picked up without reinstall

### Changing Neural Network Policy

**Option 1: Update configuration**
1. Place new `.onnx` file in [b2w_controllers/src/policy/](b2w_controllers/src/policy/)
2. Update [b2w_controllers.yaml](b2w_controllers/config/b2w_controllers.yaml): `policy.relative_path: policy/<your_policy>.onnx`

**Option 2: Use launch argument**
```bash
ros2 launch b2w_gazebo_ros2 b2w_gazebo.launch.py \
  controller_policy_path:=/absolute/path/to/policy.onnx
```

**Policy requirements:**
- Input: 60 observations
- Output: 16 actions
- LSTM hidden state size: 256

### Adding Gazebo-ROS Bridges

Edit [b2w_gazebo_ros2/config/b2w_gz_bridge.yaml](b2w_gazebo_ros2/config/b2w_gz_bridge.yaml):
```yaml
- topic_name: /new_topic
  ros_type_name: std_msgs/msg/Float64
  gz_type_name: gz.msgs.Double
  direction: GZ_TO_ROS  # or ROS_TO_GZ or BIDIRECTIONAL
```

### Modifying Controller Behavior

Main files:
- [b2w_controllers/src/b2w_controllers.cpp](b2w_controllers/src/b2w_controllers.cpp)
- [b2w_controllers/include/b2w_controllers/b2w_controllers.hpp](b2w_controllers/include/b2w_controllers/b2w_controllers.hpp)

**Key timing parameters:**
- Inference loop: 50 Hz (20ms period) - `inference_timer_`
- Publishing loop: 200 Hz (5ms period) - `publish_timer_`

### Collision Detection

The [b2w_collision_monitor](b2w_collision_monitor/) monitors force-torque sensors to detect collisions with the robot:

**Key Features:**
- Subscribes to `/collision_plate/force_torque` topic
- Calculates torque magnitude: `√(tx² + ty² + tz²)`
- Configurable threshold (default: 200 N⋅m) and cooldown period (default: 5000ms)
- Optional pointcloud-based collision detection support

**Configuration in launch file:**
```bash
ros2 launch b2w_gazebo_ros2 b2w_gazebo.launch.py
```

The collision monitor is automatically launched with default parameters. Customize via launch arguments if needed.

## Troubleshooting

### Simulation Time Not Synchronized
**Symptom:** Nodes use different time stamps
**Solution:** Ensure all nodes have `use_sim_time: true` and `/clock` topic is bridged in [b2w_gz_bridge.yaml](b2w_gazebo_ros2/config/b2w_gz_bridge.yaml)

### ONNX Runtime Errors
**Symptom:** Controller fails to load policy
**Solution:** The library is bundled in [third_party/onnxruntime/](b2w_controllers/third_party/onnxruntime/). Do not install system ONNX Runtime.

### Joint Commands Not Reaching Gazebo
**Symptom:** Robot doesn't move in simulation
**Solution:** Verify all 16 joint command topics are listed in [b2w_gz_bridge.yaml](b2w_gazebo_ros2/config/b2w_gz_bridge.yaml) with direction `ROS_TO_GZ`

### Robot Falls Through Ground
**Symptom:** Robot spawns and immediately falls
**Solution:** Check spawn height in [gazebo.launch.py](b2w_gazebo_ros2/launch/gazebo.launch.py) (default z=2.5m). Ensure Gazebo physics is not paused.

### World File Not Found
**Symptom:** Launch fails with "world file not found"
**Solution:** The default world file is [b2w_sim_worlds/worlds/ISAACLAB_TRAIN.world](b2w_sim_worlds/worlds/ISAACLAB_TRAIN.world). Use filename only (e.g., `ISAACLAB_TRAIN.world`), not full path.

## Testing

```bash
# Run linting tests (configured in CMakeLists.txt)
colcon test --packages-select b2w_controllers

# View test results
colcon test-result --verbose
```

**Note:** Copyright and cpplint checks are currently disabled in [b2w_controllers/CMakeLists.txt](b2w_controllers/CMakeLists.txt).

## License

All packages are licensed under the **Apache 2.0 License**.

## Citation

If you use this simulation in your research, please cite:

```bibtex
@article{yang2025sru,
  author = {Yang, Fan and Frivik, Per and Hoeller, David and Wang, Chen and Cadena, Cesar and Hutter, Marco},
  title = {Spatially-enhanced recurrent memory for long-range mapless navigation via end-to-end reinforcement learning},
  journal = {The International Journal of Robotics Research},
  year = {2025},
  doi = {10.1177/02783649251401926},
  url = {https://doi.org/10.1177/02783649251401926}
}
```

## Contact

**Authors:**
- Per Frivik (pfrivik@ethz.ch)
- Fan Yang (fanyang1@ethz.ch)

**Affiliation:** Robotic Systems Lab, ETH Zurich

## Acknowledgments

This simulation environment is built with:
- ROS 2 and ros_gz bridge for robot simulation
- ONNX Runtime for neural network inference
- Unitree B2W robot platform
