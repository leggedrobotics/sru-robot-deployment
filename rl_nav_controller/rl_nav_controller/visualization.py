"""Visualization utilities for RL navigation controller."""

import torch
from rclpy.time import Time
from geometry_msgs.msg import Point, Quaternion, Vector3
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

from rl_nav_controller import constants
from rl_nav_controller.utils import transform_points, yaw_quat


class VisualizationManager:
    """Manages visualization markers for the navigation controller."""

    def __init__(self, node):
        """Initialize visualization manager.

        Args:
            node: ROS2 node instance with publishers and frame IDs
        """
        self.node = node

    def publish_twist_marker(self, twist_msg, robot_odom_time, robot_frame_id, publisher):
        """Publish visualization marker for twist command.

        Args:
            twist_msg: Twist message to visualize
            robot_odom_time: Current odometry timestamp
            robot_frame_id: Robot frame ID
            publisher: Publisher for the marker
        """
        if robot_frame_id is None:
            return

        marker = Marker()
        marker.header = Header()
        marker.header.stamp = Time(nanoseconds=int(robot_odom_time * 1e9)).to_msg()
        marker.header.frame_id = robot_frame_id

        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.id = constants.TWIST_MARKER_ID

        start_point = Point(x=0.0, y=0.0, z=0.0)
        end_point = Point(
            x=twist_msg.linear.x * constants.TWIST_MARKER_SCALE,
            y=twist_msg.linear.y * constants.TWIST_MARKER_SCALE,
            z=0.0
        )
        marker.points = [start_point, end_point]

        marker.scale = Vector3(x=0.2, y=0.4, z=0.4)
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.8
        marker.pose.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        marker.frame_locked = False

        publisher.publish(marker)

    def publish_target_vector_marker(self, target_vec_b, robot_odom_time, robot_frame_id, publisher):
        """Publish visualization marker for target vector.

        Args:
            target_vec_b: Target vector in base frame (torch tensor)
            robot_odom_time: Current odometry timestamp
            robot_frame_id: Robot frame ID
            publisher: Publisher for the marker
        """
        if robot_frame_id is None:
            return

        marker = Marker()
        marker.header = Header()
        marker.header.stamp = Time(nanoseconds=int(robot_odom_time * 1e9)).to_msg()
        marker.header.frame_id = robot_frame_id

        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.id = constants.TARGET_VECTOR_MARKER_ID

        start_point = Point(x=0.0, y=0.0, z=0.0)
        end_point = Point(
            x=target_vec_b[0, 0].item(),
            y=target_vec_b[0, 1].item(),
            z=target_vec_b[0, 2].item()
        )
        marker.points = [start_point, end_point]

        marker.scale = Vector3(x=0.1, y=0.2, z=0.2)
        marker.pose.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        marker.frame_locked = False

        publisher.publish(marker)

    def publish_moving_goal_marker(self, moving_goal_delta, robot_pos_w, robot_orientation_w,
                                    robot_odom_time, map_frame_id, publisher):
        """Publish visualization marker for moving goal.

        Args:
            moving_goal_delta: Goal delta in robot frame
            robot_pos_w: Robot position in world frame
            robot_orientation_w: Robot orientation in world frame
            robot_odom_time: Current odometry timestamp
            map_frame_id: Map frame ID
            publisher: Publisher for the marker
        """
        if robot_pos_w is None or map_frame_id is None:
            return

        robot_pos = torch.tensor(robot_pos_w, dtype=torch.float32)
        robot_ori = torch.tensor(robot_orientation_w, dtype=torch.float32)
        robot_yaw_ori = yaw_quat(robot_ori)
        moving_goal_pos = torch.tensor(moving_goal_delta, dtype=torch.float32)

        moving_goal_pos_w = transform_points(
            moving_goal_pos.unsqueeze(0),
            robot_pos.unsqueeze(0),
            robot_yaw_ori.unsqueeze(0)
        )

        marker = Marker()
        marker.header = Header()
        marker.header.stamp = Time(nanoseconds=int(robot_odom_time * 1e9)).to_msg()
        marker.header.frame_id = map_frame_id

        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.id = constants.MOVING_GOAL_MARKER_ID

        start_point = Point(
            x=moving_goal_pos_w[0, 0].item(),
            y=moving_goal_pos_w[0, 1].item(),
            z=moving_goal_pos_w[0, 2].item() + 1.0
        )
        end_point = Point(
            x=moving_goal_pos_w[0, 0].item(),
            y=moving_goal_pos_w[0, 1].item(),
            z=moving_goal_pos_w[0, 2].item()
        )
        marker.points = [start_point, end_point]

        marker.scale = Vector3(x=0.3, y=0.6, z=0.6)
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        marker.pose.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        marker.frame_locked = False

        publisher.publish(marker)

    def publish_waypoints_marker(self, waypoints_visualization, map_frame_id, clock, publisher):
        """Publish visualization markers for recorded waypoints.

        Args:
            waypoints_visualization: List of waypoints to visualize
            map_frame_id: Map frame ID
            clock: ROS2 clock for timestamp
            publisher: Publisher for the marker
        """
        if map_frame_id is None:
            return

        marker = Marker()
        marker.header.stamp = clock.now().to_msg()
        marker.header.frame_id = map_frame_id
        marker.ns = "recorded_waypoints"
        marker.id = constants.WAYPOINTS_MARKER_ID
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.scale = Vector3(x=0.3, y=0.3, z=0.3)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.points = []
        for waypoint in waypoints_visualization:
            point = Point(x=waypoint[0], y=waypoint[1], z=waypoint[2])
            marker.points.append(point)

        publisher.publish(marker)
