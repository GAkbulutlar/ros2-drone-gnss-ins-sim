import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_description = get_package_share_directory('my_robot_description')
    pkg_bringup     = get_package_share_directory('my_robot_bringup')

    urdf_path              = os.path.join(pkg_description, 'urdf', 'my_robot.urdf.xacro')
    rviz_config_path       = os.path.join(pkg_description, 'rviz', 'urdf_config.rviz')
    gazebo_config_path     = os.path.join(pkg_bringup, 'config', 'gazebo_bridge.yaml')
    controllers_config     = os.path.join(pkg_bringup, 'config', 'my_controllers.yaml')
    ekf_config             = os.path.join(pkg_bringup, 'config', 'ekf.yaml')
    ekf_imu_only_config    = os.path.join(pkg_bringup, 'config', 'ekf_imu_only.yaml')
    world_path             = os.path.join(pkg_bringup, 'worlds', 'test_world.sdf')
    gz_launch              = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')

    scripts_dir = os.path.join(pkg_bringup, 'scripts')

    # Process xacro using the Python API – avoids YAML-escaping issues entirely
    robot_description_xml = xacro.process_file(
        urdf_path,
        mappings={'controllers_config': controllers_config}
    ).toxml()

    return LaunchDescription([

        # ── Robot state publisher ─────────────────────────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description_xml,
                'use_sim_time': True,
            }]
        ),

        # ── Gazebo simulation ─────────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={'gz_args': f'{world_path} -r'}.items()
        ),

        # ── Spawn robot model ─────────────────────────────────────────────────
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description'],
        ),

        # ── Gazebo ↔ ROS bridge ───────────────────────────────────────────────
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': gazebo_config_path}],
        ),

        # ── ros2_control: spawn controllers ──────────────────────────────────
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager', '/controller_manager',
                '--controller-manager-timeout', '60',
            ],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'diff_drive_controller',
                '--controller-manager', '/controller_manager',
                '--controller-manager-timeout', '60',
            ],
        ),

        # ── EKF: fused estimate (wheels + IMU) ────────────────────────────────
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': True}],
        ),

        # ── RViz ─────────────────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
        ),

        # ── Odometry -> Path converter (trajectory lines in RViz) ────────────
        ExecuteProcess(
            cmd=['python3', os.path.join(scripts_dir, 'odom_to_path.py')],
            output='screen',
        ),
    ])
