# RL Navigation Controller

[![Paper](https://img.shields.io/badge/IJRR-2025-blue)](https://journals.sagepub.com/home/ijr)
[![Website](https://img.shields.io/badge/Project-Website-green)](https://michaelfyang.github.io/sru-project-website/)

> **📌 Important Note**: This repository contains the **high-level navigation controller** for autonomous robot navigation with learning-based vision policies. This is part of the [SRU Navigation Project](https://michaelfyang.github.io/sru-project-website/) for real robot deployment.

## About This Repository

This repository provides the **learned navigation policy controller** that enables autonomous navigation using depth perception and reinforcement learning. The controller integrates with the quadrupedal robot's locomotion system to provide goal-directed navigation with waypoint recording and smart joystick control.

**Scope of this repository:**

- ✅ PyTorch-based navigation policy with depth preprocessing (VAE)
- ✅ ROS 2 integration with real-time inference at 5 Hz
- ✅ Multi-modal control: autonomous, smart joystick, waypoint playback
- ✅ Real-time visualization markers for debugging (RViz2)
- ✅ Hardware/simulation compatibility with unified interface
- ✅ Joystick integration for teleoperation and goal setting

**Not included in this repository:**

- ❌ Low-level locomotion control (see b2w_sim repository)
- ❌ Policy training infrastructure (see project website)
- ❌ Depth perception training and VAE models (see sru-depth-pretraining)
- ❌ Hardware drivers for ZedX camera and localization

For the **complete navigation system** and **simulation environment**, please refer to the related repositories on the [project website](https://michaelfyang.github.io/sru-project-website/).

### Related Repositories

- [sru-pytorch-spatial-learning](https://github.com/michaelfyang/sru-pytorch-spatial-learning) - Core SRU architecture
- [sru-depth-pretraining](https://github.com/michaelfyang/sru-depth-pretraining) - Depth perception pretraining
- [b2w_sim](../b2w_sim/) - Quadrupedal robot locomotion simulation
- [SRU Project Website](https://michaelfyang.github.io/sru-project-website/) - Complete system overview

## Package Structure

```
rl_nav_controller/
├── rl_nav_controller/
│   ├── rl_nav_controller.py    # Main navigation node (916 lines)
│   ├── utils.py                # Geometric transforms (281 lines)
│   ├── constants.py            # Configuration parameters (61 lines)
│   ├── visualization.py        # RViz marker management (189 lines)
│   └── waypoint_manager.py     # Waypoint recording/playback (165 lines)
│
├── launch/
│   └── rl_nav_controller.launch.py  # Launch with sim/hardware modes
│
├── deployment_policies/
│   ├── vae_pretrain_new_jit.pt           # Depth encoder (40×64 latent)
│   └── nav_policy_new_encoder_gamma.pt  # Navigation policy (SRU-based)
│
└── package.xml / setup.py      # ROS 2 package configuration
```

## Installation

### Prerequisites

- **ROS 2 Jazzy** (or compatible distribution)
- **Python 3.10+** with PyTorch support
- **ZedX Camera** (for hardware deployment)
- **DLIO Localization** system (odometry provider)

### Python Environment Setup

The navigation controller requires PyTorch with GPU support (CUDA) for optimal performance. Use the provided setup script to create a Python virtual environment with all necessary dependencies:

```bash
# Navigate to repository root
cd ~/sru_ws/src/sru-robot-deployment

# Run setup script (creates 'ros2_torch' virtual environment in home folder)
bash setup_ros2_torch.sh
```

**What the setup script does:**
- Creates a Python virtual environment named `ros2_torch` in `~/ros2_torch`
- Detects GPU availability and installs appropriate PyTorch version
  - **With GPU**: PyTorch with CUDA 12.1 or CUDA 11.8 support
  - **Without GPU**: CPU-only PyTorch (slower inference)
- Installs required dependencies: numpy, scipy, opencv-python, pyyaml
- Verifies PyTorch installation and GPU availability

**After setup:**
```bash
# Activate the environment
source ~/ros2_torch/bin/activate

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Verify PyTorch installation
python -c "import torch; print(f'PyTorch: {torch.__version__}, CUDA: {torch.cuda.is_available()}')"
```

### Build Instructions

```bash
# Navigate to workspace
cd ~/sru_ws

# Build package (without symlink install for working with virtual env)
colcon build --packages-select rl_nav_controller --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source workspace
source install/setup.bash
```

## Quick Start

### Launch Navigation Controller

**Important:** Always activate the Python environment before launching:
```bash
# Activate virtual environment
source ~/ros2_torch/bin/activate

# Source ROS 2 and workspace
source /opt/ros/jazzy/setup.bash
source ~/sru_ws/install/setup.bash
```

**For real hardware:**
```bash
ros2 launch rl_nav_controller rl_nav_controller.launch.py
```

**For simulation:**
```bash
ros2 launch rl_nav_controller rl_nav_controller.launch.py sim:=true
```

**With ZED camera:**
```bash
ros2 launch rl_nav_controller rl_nav_controller.launch.py launch_zed:=true
```

### Send Navigation Goal

```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'odom'},
  pose: {position: {x: 5.0, y: 2.0, z: 0.0}}
}"
```

## System Architecture

### Data Flow

```
Depth Camera (640×400) → VAE Encoder → Navigation Policy → Velocity Commands
    ↓                        ↓                  ↓                ↓
32FC1 Image            40×64 latent        3D output      /nav_vel topic
                                         [vx, vy, ωz]

Input (60-dim):
  ├─ Depth embedding: 64 dims (channels)
  ├─ Target position: 4 dims (normalized + log distance)
  ├─ Linear velocity: 3 dims
  ├─ Angular velocity: 3 dims
  ├─ Gravity vector: 3 dims
  └─ Last action: 3 dims
```

### Key Topics

**Subscribed:**
- `/dlio/odom_node/odom` (nav_msgs/Odometry) - Robot localization
- `/zed/zed_node/depth/depth_registered` (sensor_msgs/Image) - Depth input
- `/rsl_joy` (sensor_msgs/Joy) - Joystick commands
- `/goal_pose` (geometry_msgs/PoseStamped) - Navigation goals

**Published:**
- `/path_manager/path_manager_ros/nav_vel` (geometry_msgs/Twist) - Velocity commands
- `/walle_nav/robot_base_vel` (geometry_msgs/Twist) - Base frame velocity
- `/vis/*` (visualization_msgs/Marker) - Debug markers for RViz2

## Control Modes

### 1. Autonomous Navigation
Publish goal poses to `/goal_pose` for autonomous navigation.

### 2. Smart Joystick Mode
- **Enable**: Push axis 5 down (< -0.5)
- **Control**: Move joystick to set goal direction (5 m scale)
- **Update rate**: 5 Hz with low-pass filtering

### 3. Waypoint Recording & Playback
- **Button 6**: Record current position as waypoint
- **Button 4**: Remove last waypoint
- **Button 1**: Trigger waypoint sequence (forward → reverse)
- **Button 9**: Abort current goal

### 4. Direct Teleoperation
Manual control via joystick axes (axis 1/0/2 for linear/angular motion).

## Configuration

Key parameters in [rl_nav_controller/constants.py](rl_nav_controller/constants.py):

```python
DEFAULT_CONTROL_FREQUENCY = 5.0      # Policy execution rate (Hz)
ARRIVE_GOAL_THRESHOLD = 0.75         # Goal arrival distance (m)
JOYSTICK_TIMEOUT = 15.0              # Safety timeout (s)
LOW_PASS_FILTER_COEF = [0.9, 0.5, 0.5]  # Command smoothing
SMART_JOYSTICK_SCALE = 5.0           # Smart joystick, equiv. goal distance (m)
```

## Troubleshooting

### Policy Runs Slowly
- Check GPU availability: Look for "Using device: CUDA" in console
- Verify PyTorch CUDA: `python -c "import torch; print(torch.cuda.is_available())"`
- Monitor depth rate: `ros2 topic hz /zed/zed_node/depth/depth_registered`

### Robot Doesn't Move
- Check joystick velocity ratio (axis 4): `ros2 topic echo /rsl_joy`
- Verify path manager subscribes to `/path_manager/path_manager_ros/nav_vel`
- Check for joystick timeout warnings (15 second timeout)

### Goals Not Reached
- Verify frame IDs match (`odom` for both odometry and goals)
- Adjust `ARRIVE_GOAL_THRESHOLD` in constants.py
- Check visualization markers in RViz2

## Development

### Main Components

1. **LearningModel** class: PyTorch model inference (VAE + policy)
2. **NavigationPolicyNode** class: ROS 2 integration and control logic
3. **WaypointManager** class: Waypoint recording and sequential playback
4. **VisualizationManager** class: RViz marker publishing

### Key Methods

- `depth_callback()`: Triggers policy execution at 5 Hz
- `generate_cmd_vel()`: Computes and publishes velocity commands
- `joy_callback()`: Handles joystick input and mode switching

## License

This package is licensed under the **MIT License**.

## Citation

If you use this navigation controller in your research, please cite:

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

## Acknowledgments

This navigation controller is built with:
- ROS 2 for robot middleware
- PyTorch for deep learning inference
- ZedX stereo camera for depth perception
- DLIO for robust odometry estimation
