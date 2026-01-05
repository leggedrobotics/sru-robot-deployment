"""Waypoint management for RL navigation controller."""

from collections import deque


class WaypointManager:
    """Manages waypoint recording, tracking, and playback."""

    def __init__(self, logger):
        """Initialize waypoint manager.

        Args:
            logger: ROS2 logger instance
        """
        self.logger = logger
        self.recorded_waypoints = deque()
        self.inversed_ordered_waypoints = deque()
        self.waypoints_visualization = deque()
        self.is_publish_waypoints_home = False
        self.is_publish_waypoints_inversed = False

    def record_waypoint(self, robot_pos_w):
        """Record a new waypoint at the robot's current position.

        Args:
            robot_pos_w: Robot position in world frame

        Returns:
            True if waypoint was recorded successfully
        """
        if robot_pos_w is None:
            self.logger.warn('Robot position is not available yet.')
            return False

        # Clear inversed waypoints if starting a new recording session
        if len(self.recorded_waypoints) == 0 and len(self.inversed_ordered_waypoints) > 0:
            self.inversed_ordered_waypoints.clear()
            self.waypoints_visualization.clear()
            self.logger.info('Clear the inversed waypoints, and visualization waypoints.')

        self.recorded_waypoints.append(robot_pos_w)
        self.inversed_ordered_waypoints.appendleft(robot_pos_w)
        self.waypoints_visualization.append(robot_pos_w)
        self.logger.info(
            f'Waypoint recorded: {robot_pos_w}, total waypoints: {len(self.recorded_waypoints)}'
        )
        return True

    def remove_last_waypoint(self):
        """Remove the last recorded waypoint."""
        removed_count = 0

        if len(self.recorded_waypoints) > 0:
            self.recorded_waypoints.pop()
            removed_count += 1
            self.logger.info(
                f'Removed last waypoint (home), total: {len(self.recorded_waypoints)}'
            )

        if len(self.inversed_ordered_waypoints) > 0:
            self.inversed_ordered_waypoints.popleft()
            removed_count += 1
            self.logger.info(
                f'Removed last waypoint (inversed), total: {len(self.inversed_ordered_waypoints)}'
            )

        if len(self.waypoints_visualization) > 0:
            self.waypoints_visualization.pop()
            removed_count += 1
            self.logger.info(
                f'Removed last waypoint (visualization), total: {len(self.waypoints_visualization)}'
            )

        # Sanity check
        if len(self.waypoints_visualization) < len(self.recorded_waypoints) or \
           len(self.waypoints_visualization) < len(self.inversed_ordered_waypoints):
            raise ValueError(
                'The visualization waypoints is smaller than the recorded waypoints, please check.'
            )

        if removed_count == 0:
            self.logger.warn('No waypoints to remove.')

        return removed_count > 0

    def get_next_waypoint_home(self):
        """Get next waypoint for home path.

        Returns:
            Next waypoint or None if no waypoints available
        """
        if self.recorded_waypoints:
            return self.recorded_waypoints.pop()
        return None

    def get_next_waypoint_inversed(self):
        """Get next waypoint for inversed path.

        Returns:
            Next waypoint or None if no waypoints available
        """
        if self.inversed_ordered_waypoints:
            return self.inversed_ordered_waypoints.pop()
        return None

    def start_home_waypoint_sequence(self):
        """Start publishing home waypoints."""
        if len(self.recorded_waypoints) > 0:
            self.is_publish_waypoints_home = True
            return True
        return False

    def start_inversed_waypoint_sequence(self):
        """Start publishing inversed waypoints."""
        if len(self.inversed_ordered_waypoints) > 0:
            self.is_publish_waypoints_inversed = True
            return True
        return False

    def stop_home_waypoint_sequence(self):
        """Stop publishing home waypoints."""
        self.is_publish_waypoints_home = False

    def stop_inversed_waypoint_sequence(self):
        """Stop publishing inversed waypoints."""
        self.is_publish_waypoints_inversed = False

    def reset_visualization_if_complete(self):
        """Clear visualization if all waypoints are published."""
        if not self.recorded_waypoints and not self.inversed_ordered_waypoints and self.waypoints_visualization:
            self.logger.info('All waypoints are published, reset visualization.')
            self.waypoints_visualization.clear()
            return True
        return False

    def re_add_aborted_waypoint(self, target_pos_w):
        """Re-add a waypoint that was aborted.

        Args:
            target_pos_w: Target position that was aborted
        """
        if self.is_publish_waypoints_home:
            self.recorded_waypoints.append(target_pos_w)
            self.logger.info('Goal aborted - re-adding current goal as a recorded (home) waypoint.')
        if self.is_publish_waypoints_inversed:
            self.inversed_ordered_waypoints.appendleft(target_pos_w)
            self.logger.info(
                'Goal aborted: reinserting the current goal as recorded (inversed) waypoint.'
            )

    def has_home_waypoints(self):
        """Check if home waypoints are available."""
        return len(self.recorded_waypoints) > 0

    def has_inversed_waypoints(self):
        """Check if inversed waypoints are available."""
        return len(self.inversed_ordered_waypoints) > 0

    def is_home_sequence_active(self):
        """Check if home waypoint sequence is active."""
        return self.is_publish_waypoints_home

    def is_inversed_sequence_active(self):
        """Check if inversed waypoint sequence is active."""
        return self.is_publish_waypoints_inversed
