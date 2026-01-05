"""RL Navigation Controller - Main module for autonomous robot navigation."""

import os
import time
import argparse

import numpy as np
import torch
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Joy
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R

from ament_index_python.packages import get_package_share_directory

from rl_nav_controller.utils import subtract_frame_transforms, transform_points, yaw_quat
from rl_nav_controller import constants
from rl_nav_controller.visualization import VisualizationManager
from rl_nav_controller.waypoint_manager import WaypointManager


# --------------------------------------------------------------------------------
# Learning model - RL Navigation Controller
# --------------------------------------------------------------------------------
class LearningModel:
    """Neural network model for RL-based navigation policy."""

    def __init__(self, preprocess_model_path, policy_model_path):
        """Initialize learning model with preprocessing and policy networks.

        Args:
            preprocess_model_path: Path to depth image preprocessing model
            policy_model_path: Path to navigation policy model
        """
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        if torch.cuda.is_available():
            print('\033[92m' + 'Using device: CUDA' + '\033[0m')
        else:
            print('\033[93m' + 'Using device: CPU' + '\033[0m')

        self.preprocess_model = torch.jit.load(preprocess_model_path).to(self.device)
        self.policy_model = torch.jit.load(policy_model_path).to(self.device)

        self.preprocess_model = torch.jit.optimize_for_inference(self.preprocess_model.eval())
        self.policy_model = torch.jit.optimize_for_inference(self.policy_model.eval())

        self._policy_scale = torch.tensor(
            constants.POLICY_SCALE, dtype=torch.float32
        ).to(self.device)

        # Warm up model with a fake run
        self.fake_run_once()

        print('\033[92m' + 'Learning model is ready.' + '\033[0m')

    @torch.inference_mode()
    def depth_preprocess(self, img: np.ndarray):
        img_tensor = torch.tensor(img, dtype=torch.float32).unsqueeze(0).unsqueeze(0).to(self.device)
        img_tensor = torch.nn.functional.interpolate(img_tensor, size=(40, 64), mode='bilinear', align_corners=True)
        processed_img = self.preprocess_model(img_tensor)
        return processed_img

    @torch.inference_mode()
    def normalize_target_position(self, target_pos_w, robot_pos_w, robot_orientation_w):
        target_pos_w = torch.tensor(target_pos_w, dtype=torch.float32).unsqueeze(0).to(self.device)
        robot_pos_w = torch.tensor(robot_pos_w, dtype=torch.float32).unsqueeze(0).to(self.device)
        robot_orientation_w = torch.tensor(robot_orientation_w, dtype=torch.float32).unsqueeze(0).to(self.device)

        inv_pos, inv_rot = subtract_frame_transforms(robot_pos_w, robot_orientation_w)
        target_vec_b = transform_points(target_pos_w, inv_pos, inv_rot)

        dist = target_vec_b.norm(dim=-1, keepdim=True) + 1e-6
        target_pos = target_vec_b / dist
        dist_log = torch.log(dist + 1.0)

        target_pos = torch.cat((target_pos, dist_log), dim=-1).view(-1)
        return target_pos, target_vec_b

    @torch.inference_mode()
    def predict(
        self,
        linear_vel,
        angular_vel,
        gravity_vector,
        last_action,
        target_pos_w,
        robot_pos_w,
        robot_orientation_w,
        depth_image,
        is_reset=False
    ):
        processed_embedding = self.depth_preprocess(depth_image)
        processed_embedding = processed_embedding.view(-1)

        target_pos_log, target_vec_b = self.normalize_target_position(
            target_pos_w, robot_pos_w, robot_orientation_w
        )

        input_tensor = torch.tensor(linear_vel + angular_vel + gravity_vector + last_action,
                                    dtype=torch.float32).to(self.device)
        combined_input = torch.cat((input_tensor, target_pos_log, processed_embedding), dim=0).unsqueeze(0)
        raw_action = self.policy_model(combined_input, is_reset)

        cmd_vel_tensor = torch.tanh(raw_action) * self._policy_scale
        cmd_vel = cmd_vel_tensor.squeeze(0).cpu().numpy()
        raw_action = raw_action.squeeze(0).cpu().numpy()

        return cmd_vel, raw_action, target_vec_b

    @torch.inference_mode()
    def fake_run_once(self):
        linear_vel = [0.0, 0.0, 0.0]
        angular_vel = [0.0, 0.0, 0.0]
        gravity_vector = [0.0, 0.0, -1.0]
        target_pos_w = [1.0, 0.0, 0.0]
        robot_pos_w = [0.0, 0.0, 0.0]
        robot_orientation_w = [1.0, 0.0, 0.0, 0.0]
        last_action = [0.0, 0.0, 0.0]
        depth_image = np.random.rand(600, 960)

        cmd_vel, _, _ = self.predict(
            linear_vel,
            angular_vel,
            gravity_vector,
            last_action,
            target_pos_w,
            robot_pos_w,
            robot_orientation_w,
            depth_image
        )
        print(f'Predicted cmd_vel: linear_x={cmd_vel[0]:.4f}, '
              f'linear_y={cmd_vel[1]:.4f}, angular_z={cmd_vel[2]:.4f}')


# --------------------------------------------------------------------------------
# ROS2 Node
# --------------------------------------------------------------------------------
class NavigationPolicyNode(Node):
    def __init__(
        self,
        preprocess_model_path,
        policy_model_path,
        min_depth=constants.DEFAULT_MIN_DEPTH,
        max_depth=constants.DEFAULT_MAX_DEPTH,
        control_frequency=constants.DEFAULT_CONTROL_FREQUENCY,
        use_sim=False
    ):
        """Initialize navigation policy node.

        Args:
            preprocess_model_path: Path to depth preprocessing model
            policy_model_path: Path to navigation policy model
            min_depth: Minimum valid depth value in meters
            max_depth: Maximum valid depth value in meters
            control_frequency: Control loop frequency in Hz
            use_sim: Whether running in simulation mode
        """
        super().__init__('navigation_policy_node')

        # Configuration parameters
        self.use_sim = use_sim
        self.min_depth = min_depth
        self.max_depth = max_depth
        self.control_frequency = control_frequency
        self.odom_ready = False
        self.arrive_goal_threshold = constants.ARRIVE_GOAL_THRESHOLD
        self.last_run_time = 0.0
        self.system_delay = constants.JOYSTICK_TIMEOUT

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(
            Twist, '/path_manager/path_manager_ros/nav_vel', 10
        )
        self.base_vel_publisher = self.create_publisher(
            Twist, '/walle_nav/robot_base_vel', 10
        )
        # Single publisher for all goal poses (moving goal, smart joystick, waypoints)
        self.goal_pose_publisher = self.create_publisher(
            PoseStamped, '/goal_pose', 1
        )

        # Visualization publishers
        self.twist_marker_publisher = self.create_publisher(
            Marker, 'vis/twist_cmd_marker', 10
        )
        self.goal_vector_marker_publisher = self.create_publisher(
            Marker, 'vis/goal_vector_marker', 10
        )
        self.moving_goal_marker_publisher = self.create_publisher(
            Marker, 'vis/moving_goal_marker', 10
        )
        self.recorded_waypoints_marker_pub = self.create_publisher(
            Marker, 'vis/recorded_waypoints_marker', 10
        )

        # Subscribers
        self.odom_subscriber = self.create_subscription(
            Odometry, '/dlio/odom_node/odom', self.odom_callback, 10
        )
        self.depth_subscriber = self.create_subscription(
            Image, '/zed/zed_node/depth/depth_registered', self.depth_callback, 10
        )
        self.joy_subscriber = self.create_subscription(
            Joy, '/rsl_joy', self.joy_callback, 10
        )
        self.target_position_subscriber = self.create_subscription(
            PoseStamped, '/goal_pose', self.target_position_callback, 1
        )

        # Utilities
        self.bridge = CvBridge()
        self.model = LearningModel(
            preprocess_model_path=preprocess_model_path,
            policy_model_path=policy_model_path
        )
        self.waypoint_manager = WaypointManager(self.get_logger())
        self.visualization_manager = VisualizationManager(self)

        # State variables - Robot state
        self.map_frame_id = None
        self.robot_frame_id = None
        self.robot_odom_time = 0.0
        self.robot_pos_w = None
        self.robot_orientation_w = None
        self.linear_vel_w = None
        self.angular_vel_w = None
        self.linear_vel = None
        self.angular_vel = None
        self.gravity_vector = None
        self.depth_image = None

        # State variables - Navigation
        self.target_pos_w = None
        self.last_target_pos = None
        self.is_reset_hidden_state = False
        self.last_action = self._reset_last_action()
        self.prev_cmd = np.zeros(3)  # For low-pass filtering
        self.is_abort_goal = False

        # Joystick state
        self.joy_linear_x = 0.0
        self.joy_linear_y = 0.0
        self.joy_angular_z = 0.0
        self.cmd_vel_ratio = 0.0
        self.joy_time = time.time()
        self.last_trigger_time = time.time()

        # Moving goal state
        self.moving_goal_delta = [0.0, 0.0, 0.0]

        # Smart joystick state
        self.smart_joystick_goal = [0.0, 0.0, 0.0]
        self.prev_smart_joystick_goal = [0.0, 0.0, 0.0]
        self.smart_joystick_mode_active = False
        self.smart_joystick_goal_aborted = False
        self.latest_joystick_axes = [0.0, 0.0, 0.0]

        # Timers
        self.create_timer(constants.WAYPOINT_PUBLISH_INTERVAL, self.publish_recorded_waypoints)
        self.create_timer(constants.TARGET_VECTOR_PUBLISH_INTERVAL, self.publish_target_vector)
        smart_joystick_interval = 1.0 / constants.SMART_JOYSTICK_UPDATE_FREQUENCY
        self.create_timer(smart_joystick_interval, self.update_smart_joystick_goal)

        self.get_logger().info('\033[92m' + 'Navigation policy node is ready.' + '\033[0m')

    def odom_callback(self, odom_msg: Odometry):
        """Process odometry messages and update robot state.

        Args:
            odom_msg: Odometry message from the localization system
        """
        # 1) Update “time” from odom header
        self.robot_odom_time = odom_msg.header.stamp.sec + odom_msg.header.stamp.nanosec * 1e-9

        # 2) Frame IDs (once)
        if self.map_frame_id is None:
            self.map_frame_id = odom_msg.header.frame_id
        if self.robot_frame_id is None:
            self.robot_frame_id = odom_msg.child_frame_id

        # 3) Store robot position & orientation
        self.robot_pos_w = [
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            odom_msg.pose.pose.position.z
        ]
        self.robot_orientation_w = [
            odom_msg.pose.pose.orientation.w,
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z
        ]

        # 4) Extract raw linear & angular vel from odom (world‐frame)
        self.linear_vel_w = [
            odom_msg.twist.twist.linear.x,
            odom_msg.twist.twist.linear.y,
            odom_msg.twist.twist.linear.z
        ]
        self.angular_vel_w = [
            odom_msg.twist.twist.angular.x,
            odom_msg.twist.twist.angular.y,
            odom_msg.twist.twist.angular.z
        ]

        # 5) Convert to base frame if not sim; else keep as‐is
        if self.use_sim:
            self.linear_vel = list(self.linear_vel_w)
            self.angular_vel = list(self.angular_vel_w)
        else:
            self.linear_vel = self.convert_vel_frame(self.linear_vel_w, self.robot_orientation_w)
            self.angular_vel = self.convert_vel_frame(self.angular_vel_w, self.robot_orientation_w)

        # 6) Publish “converted” base velocity
        self.publish_base_vel(self.linear_vel, self.angular_vel)

        # 7) Compute projected gravity vector in base frame
        self.gravity_vector = self.projected_gravity_vector(self.robot_orientation_w)

        # 8) Mark that odom is now available
        if not self.odom_ready:
            self.odom_ready = True

    def depth_callback(self, depth_msg: Image):
        """Process depth image and trigger policy execution.

        Args:
            depth_msg: Depth image from the camera
        """
        # 1) Don’t proceed until odometry has come in at least once
        if not self.odom_ready:
            self.get_logger().warn('\033[93m' + 'Odometry not ready, skipping depth callback.' + '\033[0m')
            return

        # 2) Convert ROS Image → cv2 / NumPy, clamp NaNs / Infs / min/max
        try:
            depth_array = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            depth_array = np.nan_to_num(depth_array, nan=0.0, posinf=self.max_depth * 2.0, neginf=0.0)
            depth_array[depth_array > self.max_depth] = 0.0
            depth_array[depth_array < self.min_depth] = 0.0
            self.depth_image = depth_array
        except Exception as e:
            self.get_logger().error(f'Error converting depth image: {e}')
            return

        # 3) Respect control frequency (Hz)
        interval = 1.0 / self.control_frequency
        if (self.robot_odom_time - self.last_run_time) < interval:
            return

        self.last_run_time = self.robot_odom_time

        # 4) Everything is ready → run policy
        self.generate_cmd_vel()

    def generate_cmd_vel(self):
        """Generate and publish velocity command from policy network."""
        # Visualize moving goal marker
        self.visualization_manager.publish_moving_goal_marker(
            self.moving_goal_delta, self.robot_pos_w, self.robot_orientation_w,
            self.robot_odom_time, self.map_frame_id, self.moving_goal_marker_publisher
        )

        # Log smart joystick mode status
        if self.smart_joystick_mode_active:
            self.get_logger().info(
                f'Smart joystick mode active - Goal: [{self.smart_joystick_goal[0]:.2f}, '
                f'{self.smart_joystick_goal[1]:.2f}, {self.smart_joystick_goal[2]:.2f}]'
            )

        # Check if arrived at goal or aborted
        is_arrived = self._check_goal_reached(self.target_pos_w, self.robot_pos_w)
        if is_arrived or self.is_abort_goal:
            twist = Twist()
            twist.linear.x = 0.0 + self.joy_linear_x
            twist.linear.y = 0.0 + self.joy_linear_y
            twist.angular.z = 0.0 + self.joy_angular_z
            self.cmd_vel_publisher.publish(twist)

            if self.is_abort_goal:
                self.waypoint_manager.re_add_aborted_waypoint(self.target_pos_w)

            # Reset goal state
            self.target_pos_w = None
            self.is_abort_goal = False
            self.last_action = self._reset_last_action()
            self.get_logger().info('Target position reset.')

        else:
            # 3) Joystick timeout logic
            if self.robot_odom_time - self.joy_time > self.system_delay:
                self.cmd_vel_ratio = 0.0
                diff = self.robot_odom_time - self.joy_time
                self.get_logger().warn(
                    '\033[93m' +
                    f'Joystick timeout, stopping the robot. Time diff: {diff:.4f}s' +
                    '\033[0m'
                )

            self.get_logger().info(f'Joystick cmd_vel ratio: {self.cmd_vel_ratio:.4f}')

            # 4) Call the learning model
            cmd, action, target_vec_b = self.model.predict(
                self.linear_vel,
                self.angular_vel,
                self.gravity_vector,
                self.last_action,
                self.target_pos_w,
                self.robot_pos_w,
                self.robot_orientation_w,
                self.depth_image,
                self.is_reset_hidden_state
            )

            if self.is_reset_hidden_state:
                self.get_logger().warn('\033[93m' + 'Resetting hidden state.' + '\033[0m')
                self.is_reset_hidden_state = False
                self.last_action = self._reset_last_action()

            self.last_action = action.tolist()

            # Apply low-pass filter and form Twist message
            twist = Twist()
            # Policy command (no joystick)
            model_cmd = np.array([
                cmd[0].item() * self.cmd_vel_ratio,
                cmd[1].item() * self.cmd_vel_ratio * constants.LATERAL_VELOCITY_SCALE,
                cmd[2].item() * self.cmd_vel_ratio
            ])
            filter_coef = np.array(constants.LOW_PASS_FILTER_COEF)
            # Low-pass filter on model command
            filt_model = filter_coef * model_cmd + (1 - filter_coef) * self.prev_cmd
            # Add joystick inputs without filtering
            twist.linear.x = float(filt_model[0]) + self.joy_linear_x
            twist.linear.y = float(filt_model[1]) + self.joy_linear_y
            twist.angular.z = float(filt_model[2]) + self.joy_angular_z
            # Update previous filtered command
            self.prev_cmd = filt_model
            self.cmd_vel_publisher.publish(twist)

            # Publish visualization markers
            self.visualization_manager.publish_twist_marker(
                twist, self.robot_odom_time, self.robot_frame_id, self.twist_marker_publisher
            )

        self.get_logger().info(
            f'Published cmd_vel: linear_x={twist.linear.x:.4f}, '
            f'linear_y={twist.linear.y:.4f}, angular_z={twist.angular.z:.4f}'
        )
        # Reset joystick deltas for next cycle
        self._reset_joystick()

    # ------------------------------
    # “Base‐frame” velocity publisher (called from odom_callback)
    # ------------------------------
    def publish_base_vel(self, linear_vel, angular_vel):
        twist = Twist()
        twist.linear.x = linear_vel[0]
        twist.linear.y = linear_vel[1]
        twist.angular.z = angular_vel[2]
        self.base_vel_publisher.publish(twist)

    # ------------------------------
    # Convert a vector in world frame → robot (base) frame
    # ------------------------------
    def convert_vel_frame(self, vel_vec, orientation_w):
        quat_xyzw = [orientation_w[1], orientation_w[2], orientation_w[3], orientation_w[0]]
        rotation = R.from_quat(quat_xyzw)
        inv_rot = rotation.inv()
        out = inv_rot.apply(np.array(vel_vec))
        return out.tolist()

    # ------------------------------
    # Compute projected gravity in robot frame
    # ------------------------------
    def projected_gravity_vector(self, robot_orientation_w):
        quat_xyzw = [robot_orientation_w[1], robot_orientation_w[2], robot_orientation_w[3], robot_orientation_w[0]]
        rotation = R.from_quat(quat_xyzw)
        inv_rot = rotation.inv()

        gravity = np.array([0.0, 0.0, -9.81])
        proj = inv_rot.apply(gravity)
        proj = proj / (np.linalg.norm(proj) + 1e-6)
        return proj.tolist()

    def joy_callback(self, joy_msg: Joy):
        """Process joystick input and update control state.

        Args:
            joy_msg: Joystick message with axes and button states
        """
        self.cmd_vel_ratio = (1.0 + joy_msg.axes[4]) * 1.0

        if joy_msg.buttons[constants.BUTTON_ABORT] == 1:
            self.is_abort_goal = True
            self.get_logger().warn('Abort goal')

        dx, dy, dz = self._moving_xyz_with_buttons(joy_msg, scale=constants.MOVING_SCALE)
        self.moving_goal_delta[0] += dx
        self.moving_goal_delta[1] += dy
        self.moving_goal_delta[2] += dz

        current_time = time.time()
        cooldown = constants.TRIGGER_BUTTON_COOLDOWN

        if not self.is_abort_goal and (current_time - self.last_trigger_time) > cooldown and \
           joy_msg.buttons[constants.BUTTON_SEND_GOAL] == 1:
            self.get_logger().warn('Trigger moving goal')
            self._publish_moving_goal()
            self.last_trigger_time = current_time

        if (current_time - self.last_trigger_time) > cooldown and \
           joy_msg.buttons[constants.BUTTON_RECORD_WAYPOINT] == 1:
            self.waypoint_manager.record_waypoint(self.robot_pos_w)
            self.last_trigger_time = current_time

        if (current_time - self.last_trigger_time) > cooldown and \
           joy_msg.buttons[constants.BUTTON_CLEAR_WAYPOINT] == 1:
            self.waypoint_manager.remove_last_waypoint()
            self.last_trigger_time = current_time

        if not self.is_abort_goal and (current_time - self.last_trigger_time) > cooldown and \
           joy_msg.buttons[constants.BUTTON_TRIGGER_WAYPOINTS] == 1:
            self.get_logger().warn('Trigger waypoints')
            if self.waypoint_manager.has_home_waypoints():
                self.waypoint_manager.start_home_waypoint_sequence()
            elif self.waypoint_manager.has_inversed_waypoints():
                self.waypoint_manager.start_inversed_waypoint_sequence()
            self.last_trigger_time = current_time

        if joy_msg.axes[constants.JOYSTICK_AXIS_SMART] < -0.5:
            # Smart joystick mode enabled
            if not self.smart_joystick_mode_active:
                # Entering smart joystick mode for the first time
                self.smart_joystick_mode_active = True
                self.smart_joystick_goal_aborted = False

            # Abort current goal only once when entering smart joystick mode
            if not self.smart_joystick_goal_aborted and self.target_pos_w is not None:
                self.is_abort_goal = True
                self.smart_joystick_goal_aborted = True
                self.get_logger().info('Smart joystick mode: aborting current navigation goal')

            # Store the latest joystick axes values
            # Goal will be updated at controlled frequency in a timer callback
            self.latest_joystick_axes = [
                joy_msg.axes[constants.JOYSTICK_AXIS_LINEAR_X],
                joy_msg.axes[constants.JOYSTICK_AXIS_LINEAR_Y],
                joy_msg.axes[constants.JOYSTICK_AXIS_LINEAR_Z]
            ]

        else:
            # Smart joystick mode disabled - use direct joystick control
            if self.smart_joystick_mode_active:
                # Exiting smart joystick mode
                self.smart_joystick_mode_active = False
                self.is_abort_goal = True
                self.get_logger().info('Exiting smart joystick mode: aborting current navigation goal')

            self._reset_smart_joystick_goal()
            # Use the joystick axes to directly control the robot
            self.joy_linear_x = joy_msg.axes[constants.JOYSTICK_AXIS_LINEAR_X] * constants.LINEAR_SCALE * 1.5
            self.joy_linear_y = joy_msg.axes[constants.JOYSTICK_AXIS_LINEAR_Y] * constants.LINEAR_SCALE
            self.joy_angular_z = joy_msg.axes[constants.JOYSTICK_AXIS_ANGULAR_Z] * constants.ANGULAR_SCALE

        if joy_msg.buttons[constants.BUTTON_RESET_HIDDEN_STATE] == 1:
            self.is_reset_hidden_state = True
            self.get_logger().warn('Force Reset hidden state')

        self.joy_time = time.time()
        
    def _generate_waypoint_using_joystick(self, linear_x, linear_y, linear_z):
        """Generate a waypoint using joystick inputs relative to robot's current position.

        The goal is always calculated as an offset from the current robot position,
        preventing unbounded accumulation. Low-pass filtering is applied for smoothness.

        Args:
            linear_x: Joystick input for forward/backward
            linear_y: Joystick input for left/right
            linear_z: Joystick input for up/down
        """
        if self.robot_pos_w is None or self.robot_orientation_w is None:
            return

        # Scale joystick inputs to goal offset in robot frame
        goal_offset_robot = torch.tensor([
            linear_x * constants.SMART_JOYSTICK_SCALE,
            linear_y * constants.SMART_JOYSTICK_SCALE,
            linear_z * constants.SMART_JOYSTICK_SCALE * constants.SMART_JOYSTICK_Z_SCALE
        ], dtype=torch.float32)

        # Transform the offset from robot frame to world frame
        robot_ori = torch.tensor(self.robot_orientation_w, dtype=torch.float32)
        robot_yaw_ori = yaw_quat(robot_ori)

        # Transform the offset (rotation only, no translation)
        goal_offset_world = transform_points(
            goal_offset_robot.unsqueeze(0),
            torch.zeros(3, dtype=torch.float32).unsqueeze(0),  # Zero translation
            robot_yaw_ori.unsqueeze(0)
        )[0]

        # Calculate target goal as robot position + offset (always relative to current position)
        target_goal_world = [
            self.robot_pos_w[0] + goal_offset_world[0].item(),
            self.robot_pos_w[1] + goal_offset_world[1].item(),
            self.robot_pos_w[2] + goal_offset_world[2].item()
        ]

        # Apply low-pass filter to smooth goal changes
        # If this is the first update or goal was reset, initialize to target
        if self.prev_smart_joystick_goal == [0.0, 0.0, 0.0]:
            self.smart_joystick_goal = target_goal_world
        else:
            alpha = constants.SMART_JOYSTICK_FILTER_ALPHA
            self.smart_joystick_goal = [
                alpha * target_goal_world[0] + (1 - alpha) * self.prev_smart_joystick_goal[0],
                alpha * target_goal_world[1] + (1 - alpha) * self.prev_smart_joystick_goal[1],
                alpha * target_goal_world[2] + (1 - alpha) * self.prev_smart_joystick_goal[2]
            ]

        # Update previous goal for next iteration
        self.prev_smart_joystick_goal = list(self.smart_joystick_goal)

    def _publish_smart_joystick_goal(self):
        """Publish the smart joystick goal (already in world frame)."""
        if self.robot_pos_w is None or self.robot_orientation_w is None:
            self.get_logger().warn('Cannot publish smart joystick goal: robot pose not available')
            return

        # Goal is already in world frame, just publish it directly
        self._publish_goal(self.smart_joystick_goal)

    def _reset_smart_joystick_goal(self):
        """Reset smart joystick goal state to origin."""
        self.smart_joystick_goal = [0.0, 0.0, 0.0]
        self.prev_smart_joystick_goal = [0.0, 0.0, 0.0]
        

    def _moving_xyz_with_buttons(self, joy_msg: Joy, scale=1.0):
        """Map discrete button presses to movement increments.

        Args:
            joy_msg: Joystick message
            scale: Scale factor for movement

        Returns:
            Tuple of (dx, dy, dz) increments
        """
        dx = scale * (bool(joy_msg.buttons[constants.BUTTON_FORWARD]) -
                      bool(joy_msg.buttons[constants.BUTTON_BACKWARD]))
        dy = scale * (bool(joy_msg.buttons[constants.BUTTON_LEFT]) -
                      bool(joy_msg.buttons[constants.BUTTON_RIGHT]))
        dz = (scale / 5.0) * (bool(joy_msg.buttons[constants.BUTTON_UP]) -
                               bool(joy_msg.buttons[constants.BUTTON_DOWN]))
        return dx, dy, dz

    def _publish_goal(self, goal_pos):
        """Publish a goal pose to the unified goal_pose topic.

        Args:
            goal_pos: Goal position [x, y, z] in world frame
        """
        goal_pose = PoseStamped()
        goal_pose.header = Header()
        goal_pose.header.stamp = Time(nanoseconds=int(self.robot_odom_time * 1e9)).to_msg()
        goal_pose.header.frame_id = self.map_frame_id
        goal_pose.pose.position.x = goal_pos[0]
        goal_pose.pose.position.y = goal_pos[1]
        goal_pose.pose.position.z = goal_pos[2]
        self.goal_pose_publisher.publish(goal_pose)

    def _publish_moving_goal(self):
        """Convert moving goal delta from robot frame to world frame and publish."""
        if self.robot_pos_w is None or self.robot_orientation_w is None:
            self.get_logger().warn('Cannot publish moving goal: robot pose/orientation not yet set.')
            return

        robot_pos = torch.tensor(self.robot_pos_w, dtype=torch.float32)
        robot_ori = torch.tensor(self.robot_orientation_w, dtype=torch.float32)
        robot_yaw_ori = yaw_quat(robot_ori)

        moving_goal_pos = torch.tensor(self.moving_goal_delta, dtype=torch.float32)
        # Transform robot-frame delta to world-frame coordinate
        moving_goal_pos_w = transform_points(
            moving_goal_pos.unsqueeze(0),
            robot_pos.unsqueeze(0),
            robot_yaw_ori.unsqueeze(0)
        )

        # Publish the world-frame goal
        self._publish_goal(moving_goal_pos_w[0].tolist())

        # Reset the moving_goal_delta
        self._reset_moving_goal()

    def publish_recorded_waypoints(self):
        """Timer callback to publish recorded waypoints sequentially."""
        if self.robot_pos_w is None:
            self.get_logger().warn('Waiting for robot position to initialize.')
            return

        # Visualize waypoints
        self.visualization_manager.publish_waypoints_marker(
            self.waypoint_manager.waypoints_visualization,
            self.map_frame_id,
            self.get_clock(),
            self.recorded_waypoints_marker_pub
        )

        # Publish home waypoints
        if self.waypoint_manager.is_home_sequence_active() and self.waypoint_manager.has_home_waypoints():
            if self.target_pos_w is None or self._check_near_goal(self.target_pos_w, self.robot_pos_w):
                next_wp = self.waypoint_manager.get_next_waypoint_home()
                self.get_logger().info(f'Publishing next waypoint: {next_wp}')
                self._publish_goal(next_wp)
            else:
                self.get_logger().info('Tracking the current waypoint ...')

            if not self.waypoint_manager.has_home_waypoints():
                self.waypoint_manager.stop_home_waypoint_sequence()

        # Publish inversed waypoints
        if self.waypoint_manager.is_inversed_sequence_active() and self.waypoint_manager.has_inversed_waypoints():
            if self.target_pos_w is None or self._check_near_goal(self.target_pos_w, self.robot_pos_w):
                next_wp = self.waypoint_manager.get_next_waypoint_inversed()
                self.get_logger().info(f'Publishing next inversed waypoint: {next_wp}')
                self._publish_goal(next_wp)
            else:
                self.get_logger().info('Tracking the current waypoint ...')

            if not self.waypoint_manager.has_inversed_waypoints():
                self.waypoint_manager.stop_inversed_waypoint_sequence()

        # Reset visualization if all waypoints published
        self.waypoint_manager.reset_visualization_if_complete()

    def _check_near_goal(self, target_pos_w, robot_pos_w):
        """Check if robot is near the goal (wider threshold than arrival).

        Args:
            target_pos_w: Target position in world frame
            robot_pos_w: Robot position in world frame

        Returns:
            True if near the goal, False otherwise
        """
        if target_pos_w is None or robot_pos_w is None:
            self.get_logger().warning('Waiting for target position.')
            return True

        dist = np.linalg.norm(np.array(target_pos_w[:2]) - np.array(robot_pos_w[:2]))
        threshold = self.arrive_goal_threshold * constants.NEAR_GOAL_THRESHOLD_MULTIPLIER
        if dist > threshold:
            return False
        else:
            self.get_logger().info('Near the current goal position.')
            return True

    def _check_goal_reached(self, target_pos_w, robot_pos_w):
        """Check if robot has arrived at the goal.

        Args:
            target_pos_w: Target position in world frame
            robot_pos_w: Robot position in world frame

        Returns:
            True if arrived, False otherwise
        """
        if target_pos_w is None or robot_pos_w is None:
            self.get_logger().warning('Waiting for target position.')
            return True

        dist = np.linalg.norm(np.array(target_pos_w[:2]) - np.array(robot_pos_w[:2]))
        if dist > self.arrive_goal_threshold:
            return False
        else:
            self.get_logger().info('Arrived at the goal position.')
            return True

    def target_position_callback(self, msg: PoseStamped):
        """Process incoming goal pose messages.

        Args:
            msg: PoseStamped message with goal position
        """
        if msg.header.frame_id != self.map_frame_id:
            self.get_logger().error(
                '\033[91m' +
                f'Target frame_id "{msg.header.frame_id}" does not match odometry frame_id "{self.map_frame_id}"' +
                '\033[0m'
            )
            return

        # If the received goal z is zero or very close to zero, use the robot's current z
        goal_z = msg.pose.position.z
        if abs(goal_z) < 1e-3:
            # Use robot's current z if available, otherwise fallback to received z
            if self.robot_pos_w is not None:
                goal_z = self.robot_pos_w[2]
            else:
                self.get_logger().warn('Robot position not available, using received z for target.')
        self.target_pos_w = [
            msg.pose.position.x,
            msg.pose.position.y,
            goal_z
        ]
        # Store as last target
        self.last_target_pos = list(self.target_pos_w)

        self.get_logger().info(
            f'Received target position: {self.target_pos_w}, '
            f'robot height: {self.robot_pos_w[2] if self.robot_pos_w else "N/A"}'
        )

    def _reset_last_action(self):
        """Reset the last action vector and filter state.

        Returns:
            Zero action vector [0, 0, 0]
        """
        self.prev_cmd = np.zeros(3)
        return [0.0, 0.0, 0.0]

    def _reset_joystick(self):
        """Reset joystick control deltas to zero."""
        self.joy_linear_x = 0.0
        self.joy_linear_y = 0.0
        self.joy_angular_z = 0.0

    def _reset_moving_goal(self):
        """Reset moving goal delta to zero."""
        self.moving_goal_delta = [0.0, 0.0, 0.0]
        self.get_logger().info('Reset moving goal delta.')

    def publish_target_vector(self):
        """Timer callback to continuously publish target vector marker."""
        if self.robot_pos_w is None or self.robot_orientation_w is None:
            return

        # Use current or last target
        tgt = self.target_pos_w if self.target_pos_w is not None else self.last_target_pos
        if tgt is None:
            return

        # Transform target position to robot frame
        _, target_vec_b = self.model.normalize_target_position(
            tgt, self.robot_pos_w, self.robot_orientation_w
        )

        # Publish visualization marker
        self.visualization_manager.publish_target_vector_marker(
            target_vec_b, self.robot_odom_time, self.robot_frame_id,
            self.goal_vector_marker_publisher
        )

    def update_smart_joystick_goal(self):
        """Timer callback to update smart joystick goal at controlled frequency."""
        if not self.smart_joystick_mode_active:
            return

        # Generate and publish waypoint using the latest joystick axes
        self._generate_waypoint_using_joystick(
            self.latest_joystick_axes[0],
            self.latest_joystick_axes[1],
            self.latest_joystick_axes[2]
        )
        self._publish_smart_joystick_goal()


# --------------------------------------------------------------------------------
# Main: initialize and spin
# --------------------------------------------------------------------------------
def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--sim', action='store_true', help='run in simulation mode')
    known_args, ros_args = parser.parse_known_args(args=args)
    rclpy.init(args=ros_args)
    use_sim = known_args.sim

    pkg_name = 'rl_nav_controller'
    pkg_path = get_package_share_directory(pkg_name)
    preprocess_model_path = os.path.join(pkg_path, 'deployment_policies', 'vae_pretrain_new_jit.pt')
    policy_model_path = os.path.join(pkg_path, 'deployment_policies', 'policy.pt')

    navigation_policy_node = NavigationPolicyNode( 
        preprocess_model_path=preprocess_model_path,
        policy_model_path=policy_model_path,
        min_depth=0.25,
        max_depth=10.0,
        control_frequency=5.0,
        use_sim=use_sim
    )

    navigation_policy_node.get_logger().info(
        f'RL Navigation launched in {"SIMULATION" if use_sim else "REAL-HARDWARE"} mode.'
    )

    try:
        rclpy.spin(navigation_policy_node)
    except KeyboardInterrupt:
        navigation_policy_node.get_logger().info('Shutting down navigation policy node.')
    finally:
        navigation_policy_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
