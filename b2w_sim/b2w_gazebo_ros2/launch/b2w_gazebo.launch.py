from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    controller_params_default = PathJoinSubstitution([
        FindPackageShare("b2w_controllers"),
        "config",
        "b2w_controllers.yaml",
    ])

    declared_arguments = [
        DeclareLaunchArgument(
            "controller_params_file",
            default_value=controller_params_default,
            description="Path to the controller parameter file",
        ),
        DeclareLaunchArgument(
            "controller_environment_profile",
            default_value="default",
            description="Environment profile override for b2w_controllers",
        ),
        DeclareLaunchArgument(
            "controller_policy_path",
            default_value="",
            description="Optional absolute path to override the policy file",
        ),
        DeclareLaunchArgument(
            "enable_low_level_controller",
            default_value="false",
            description="Enable the low-level gazebo controller node",
        ),
        DeclareLaunchArgument(
            "enable_mesh_publisher",
            default_value="false",
            description="Publish Gazebo meshes as RViz markers",
        ),
        DeclareLaunchArgument(
            "enable_set_pose_bridge",
            default_value="false",
            description="Bridge the Gazebo set_pose service via ros_gz_bridge",
        ),
        DeclareLaunchArgument(
            "enable_rviz",
            default_value="false",
            description="Launch RViz with the demo configuration",
        ),
    ]

    enable_low_level_controller = LaunchConfiguration("enable_low_level_controller")
    enable_mesh_publisher = LaunchConfiguration("enable_mesh_publisher")
    enable_set_pose_bridge = LaunchConfiguration("enable_set_pose_bridge")
    enable_rviz = LaunchConfiguration("enable_rviz")

    # Include the main gazebo launch file with spawn position arguments
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("b2w_gazebo_ros2"),
                "launch",
                "gazebo.launch.py"
            ])
        )
    )
    
    static_tf_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_to_odom",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="log",
    )

    # Kinematics controller node
    kinematics_controller = Node(
        package="b2w_controllers",
        executable="b2w_controllers",
        name="b2w_controllers",
        output="screen",
        parameters=[
            LaunchConfiguration("controller_params_file"),
            {
                "environment.profile": LaunchConfiguration("controller_environment_profile"),
                "policy.path": LaunchConfiguration("controller_policy_path"),
            },
        ],
    )

    # Low-level gazebo controller node
    low_level_controller = Node(
        package="b2w_low_level_controller_gazebo",
        executable="b2w_low_level_controller_gazebo",
        name="b2w_low_level_controller_gazebo",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(enable_low_level_controller),
    )
    # mesh publisher node
    mesh_publisher = Node(
        package="b2w_sim_worlds",
        executable="publish_mesh_node",
        name="publish_mesh_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(enable_mesh_publisher),
    )

    # Service bridge for set_pose service
    service_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="set_pose_service_bridge",
        arguments=["/world/ISAACLAB_TRAIN_world/set_pose@ros_gz_interfaces/srv/SetEntityPose"],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(enable_set_pose_bridge),
    )

    # Collision monitor node
    collision_monitor = Node(
        package="b2w_collision_monitor",
        executable="b2w_collision_detector",
        name="collision_detector",
        output="screen",
        parameters=[
            {"torque_threshold": 200.0},
            {"cooldown_period_ms": 5000},
            {"enable_force_torque": True},
            {"enable_pointcloud": False},
            {"use_sim_time": True},
        ],
    )

    # Add rviz2 node
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(enable_rviz),
    )

    return LaunchDescription(declared_arguments + [
        gazebo_launch,
        kinematics_controller,
        low_level_controller,
        mesh_publisher,
        service_bridge,
        collision_monitor,
        rviz2,
        static_tf_map_to_odom,
    ])
