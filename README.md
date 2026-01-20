# Robot Deployment - SRU Navigation on Unitree B2W

[![Paper](https://img.shields.io/badge/IJRR-2025-blue)](https://journals.sagepub.com/home/ijr)
[![Website](https://img.shields.io/badge/Project-Website-green)](https://michaelfyang.github.io/sru-project-website/)
[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://docs.ros.org/en/jazzy/index.html)

> **📌 Important Note**: This repository is the **integrated deployment system** for the Unitree B2W quadrupedal robot with learning-based navigation and locomotion control. It combines the low-level simulation environment, high-level navigation controller, and teleoperation interface for autonomous navigation research and real robot deployment.

## Overview

This project provides a complete stack for quadrupedal robot navigation with **two alternative control modes**:

1. **[b2w_sim](b2w_sim/)** - Gazebo simulation with neural network locomotion control (50 Hz inference, 200 Hz output)
2. **[rl_nav_controller](rl_nav_controller/)** - Autonomous navigation using trained SRU network (5 Hz) **OR**
3. **[b2w_joystick_control](b2w_sim/b2w_joystick_control/)** - Manual teleoperation interface (10 Hz)

**Control Modes:** Choose either autonomous navigation with the RL controller OR manual control with joystick - only one runs at a time. Both send velocity commands to the same low-level locomotion controller.

> **🎮 Joystick Required:** A joystick (e.g., PS5 controller) is required for **both** control modes. In autonomous navigation mode, the joystick serves as a safety deadman switch. In manual teleoperation mode, it provides direct control. The robot will not move without a connected joystick.

The `rl_nav_controller` package loads and deploys the trained **Spatially-enhanced Recurrent Unit (SRU)** neural network, enabling vision-based autonomous navigation to goal positions. See the [SRU project website](https://michaelfyang.github.io/sru-project-website/) for more details.

### System Architecture

```
┌──────────────────────────┐     ┌──────────────────────────┐
│  RL Navigation (5 Hz)    │     │  Joystick Control (10 Hz)│
│  Depth + Goal → Velocity │     │   Joy Input → Velocity   │
└──────────┬───────────────┘     └──────────┬───────────────┘
           │                                │
           └────────────┬───────────────────┘
                        │ Either/Or
           /path_manager/path_manager_ros/nav_vel (Twist)
                        │
                        ↓
┌─────────────────────────────────────────────────────────────┐
│              B2W Simulation / Real Robot                    │
│  ┌──────────────────────────────────────────────────────┐   │
│  │   Low-level Locomotion Controller (ONNX/C++)        │   │
│  │   - Joint state feedback (50 Hz inference)          │   │
│  │   - 16 joint action outputs (200 Hz publish)        │   │
│  └──────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

## Quick Start

### Installation

```bash
# Clone repository
git clone <repo-url> ~/sru_ws/src/sru-robot-deployment
cd ~/sru_ws/src/sru-robot-deployment

# Option 1: Use setup script (recommended - auto-detects GPU)
./setup_ros2_onnx.sh

# Option 2: Manual installation
pip install numpy scipy opencv-python onnxruntime  # or onnxruntime-gpu for CUDA

# Build
cd ~/sru_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### Simulation

**Choose one control mode:**

**Option A: Autonomous navigation with RL controller (5 Hz)**

```bash
# Terminal 1: Launch full B2W simulation with controller
ros2 launch b2w_gazebo_ros2 b2w_gazebo.launch.py enable_rviz:=true

# Terminal 2: Launch navigation controller (loads SRU network via ONNX Runtime)
ros2 launch rl_nav_controller rl_nav_controller.launch.py sim:=true
```

**Option B: Direct joystick teleoperation (10 Hz)**

```bash
# Terminal 1: Launch full B2W simulation with controller
ros2 launch b2w_gazebo_ros2 b2w_gazebo.launch.py enable_rviz:=true

# Terminal 2: Launch joystick control (instead of navigation controller)
ros2 launch b2w_joystick_control joystick_teleop.launch.py
```

### Real Robot Deployment

**Prerequisites:**
```bash
# Install Python dependencies
pip install numpy scipy opencv-python onnxruntime  # or onnxruntime-gpu for CUDA
```

**Choose one control mode:**

**Option A: Autonomous navigation with RL controller (5 Hz)**

```bash
# Terminal 1: Launch real robot (requires hardware setup)
ros2 launch b2w_gazebo_ros2 b2w_gazebo.launch.py  # Adjust for real hardware

# Terminal 2: Launch navigation controller
ros2 launch rl_nav_controller rl_nav_controller.launch.py

# Terminal 3: Send navigation goal (choose one method)

# Method 1: Click in RViz to set goal
# In RViz, use the "Publish Point" tool to click on the map where you want the robot to navigate
# Make sure the topic is set to /goal_pose

# Method 2: Command line
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'odom'},
  pose: {position: {x: 5.0, y: 2.0, z: 0.0}}
}"
```

**Option B: Direct joystick teleoperation (10 Hz)**

```bash
# Terminal 1: Launch real robot (requires hardware setup)
ros2 launch b2w_gazebo_ros2 b2w_gazebo.launch.py  # Adjust for real hardware

# Terminal 2: Launch joystick control (instead of navigation controller)
ros2 launch b2w_joystick_control joystick_teleop.launch.py
```

## Module Details

### B2W Simulation ([b2w_sim/](b2w_sim/))

Gazebo simulation environment with ONNX-based neural network locomotion controller for the Unitree B2W quadrupedal robot. Includes robot URDF models, ROS-Gazebo bridge configuration, and RViz2 visualization.

**Main launch file:** `b2w_gazebo.launch.py` - Full simulation stack with controller and RViz

See [b2w_sim/README.md](b2w_sim/README.md) for detailed documentation.

### Navigation Controller ([rl_nav_controller/](rl_nav_controller/))

Deployment of the trained SRU network for autonomous goal-directed navigation:

- **Loads trained SRU policy** with VAE depth preprocessing for vision-based navigation
- **Multi-modal control**: autonomous goal navigation, smart joystick, waypoint playback
- **Real-time inference** at 5 Hz with depth perception
- **RViz2 visualization** for debugging and monitoring

This package deploys the trained **Spatially-enhanced Recurrent Unit (SRU)** neural network, enabling the robot to autonomously navigate to goal positions using depth camera input.

**Launch modes:**
- `sim:=true` - Simulation mode
- `launch_zed:=true` - Enable ZedX camera integration

See [rl_nav_controller/README.md](rl_nav_controller/README.md) for detailed documentation.

### Joystick Control ([b2w_joystick_control/](b2w_sim/b2w_joystick_control/))

PS5 controller teleoperation interface:

- **Safety enable button** (L1 deadman switch)
- **Turbo mode** (R1) for increased speed
- **Configurable button/axis mappings**

See [b2w_joystick_control/README.md](b2w_sim/b2w_joystick_control/README.md) for detailed documentation.

## Control Hierarchy

| Level | Component | Frequency | Input | Output |
|-------|-----------|-----------|-------|--------|
| **High (either)** | Navigation Controller | 5 Hz | Depth + Goal pose | Velocity commands |
| **High (either)** | Joystick Control | 10 Hz | Joy input | Velocity commands |
| **Low** | Locomotion Controller | 50-200 Hz | Velocity commands | 16 joint actions |

**Note:** Navigation Controller and Joystick Control are alternative input sources - only one runs at a time. Both send velocity commands to the same topic that the Locomotion Controller subscribes to.

## Setting Navigation Goals

### Using RViz to Click Goals

1. In RViz, locate the "2D Goal Pose" tool in the toolbar (or press 'G')
2. Configure the tool to publish to `/goal_pose` topic with `PoseStamped` message type
3. Click on the map where you want the robot to navigate
4. The robot will autonomously navigate to the clicked location

### Using Command Line

```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'odom'},
  pose: {position: {x: 5.0, y: 2.0, z: 0.0}}
}"
```

## Key Topics & Services

**Main command topics:**
- `/goal_pose` (geometry_msgs/PoseStamped) - Navigation goal input (RViz or command line)
- `/path_manager/path_manager_ros/nav_vel` (geometry_msgs/Twist) - Navigation output
- `/nav_vel` (geometry_msgs/Twist) - Alternative navigation topic
- `/joy` (sensor_msgs/Joy) - Joystick input

**Simulation-specific:**
- `/clock` - Gazebo simulation time
- `/joint_states` - Robot joint feedback
- `/dlio/odom_node/odom` - Odometry

**Sensors:**
- `/zed/zed_node/depth/depth_registered` - Depth camera input
- `/collision_plate/force_torque` - Force/torque feedback

See individual module READMEs for complete topic lists.

## Configuration & Customization

### Locomotion Policy

Edit [b2w_controllers/config/b2w_controllers.yaml](b2w_sim/b2w_controllers/config/b2w_controllers.yaml):
```yaml
policy:
  relative_path: policy/policy_force_new.onnx
environment_profile: default  # or 'crouched'
```

### Navigation Parameters

Edit [rl_nav_controller/constants.py](rl_nav_controller/rl_nav_controller/constants.py):
```python
DEFAULT_CONTROL_FREQUENCY = 5.0        # Hz
ARRIVE_GOAL_THRESHOLD = 0.75           # meters
SMART_JOYSTICK_SCALE = 5.0             # meters
```

### Joystick Mappings

Edit [b2w_joystick_control/config/ps5_config.yaml](b2w_sim/b2w_joystick_control/config/ps5_config.yaml) for custom controller layouts.

## Development

### Adding Features

**For locomotion (low-level):**
- Modify [b2w_controllers/src/b2w_controllers.cpp](b2w_sim/b2w_controllers/src/b2w_controllers.cpp)
- Update ONNX policy path in configuration
- Rebuild: `colcon build --packages-select b2w_controllers`

**For navigation (high-level):**
- Modify [rl_nav_controller/rl_nav_controller.py](rl_nav_controller/rl_nav_controller/rl_nav_controller.py)
- Update control modes in [utils.py](rl_nav_controller/rl_nav_controller/utils.py)
- Rebuild: `colcon build --packages-select rl_nav_controller` (without --symlink-install for venv compatibility)

**For Gazebo bridges:**
- Edit [b2w_gazebo_ros2/config/b2w_gz_bridge.yaml](b2w_sim/b2w_gazebo_ros2/config/b2w_gz_bridge.yaml)
- No rebuild needed with `--symlink-install`

## Troubleshooting

**Simulation time not synchronized:**
- Ensure `/clock` topic is bridged in b2w_gz_bridge.yaml
- All nodes must have `use_sim_time: true`

**Navigation policy runs slowly:**
- Verify GPU: `python -c "import onnxruntime as ort; print(ort.get_available_providers())"`
- Check depth input rate: `ros2 topic hz /zed/zed_node/depth/depth_registered`

**Robot doesn't move:**
- Check joystick enables (L1 button in PS5 controller)
- Verify `/nav_vel` or `/path_manager/path_manager_ros/nav_vel` topics
- Check joint command topics in b2w_gz_bridge.yaml

**ONNX Runtime errors:**
- Do not install system ONNX Runtime—library is bundled in `b2w_controllers/third_party/`

See individual module READMEs for more troubleshooting guidance.

## Related Repositories

- [sru-pytorch-spatial-learning](https://github.com/michaelfyang/sru-pytorch-spatial-learning) - Core SRU architecture
- [sru-depth-pretraining](https://github.com/michaelfyang/sru-depth-pretraining) - Depth perception pretraining
- [SRU Project Website](https://michaelfyang.github.io/sru-project-website/) - Complete system overview

## Requirements

- **ROS 2 Jazzy** (or compatible)
- **Gazebo Harmonic** (simulation only)
- **Python 3.10+** with ONNX Runtime
- **ZedX Camera** (hardware deployment only)

## License

This project is licensed under the MIT License. See the LICENSE file in each package for details.

## Citation

If you use this system in your research, please cite:

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
- Fan Yang (fanyang1@ethz.ch)
- Per Frivik (pfrivik@ethz.ch)

**Affiliation:** Robotic Systems Lab, ETH Zurich
