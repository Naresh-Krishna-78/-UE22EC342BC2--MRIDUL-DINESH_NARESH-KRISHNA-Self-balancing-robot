import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- Configuration ---
    package_name = 'project55'
    urdf_file_name = 'self_balancing_robot.urdf.xacro'
    world_file_name = 'balance_world.sdf'
    use_sim_time = LaunchConfiguration('use_sim_time', default='true') # Use Gazebo's clock

    # --- Paths ---
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    urdf_path = os.path.join(pkg_share, 'urdf', urdf_file_name)
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)
    gz_sim_resource_path = os.path.join(pkg_share, 'worlds') # Let Gazebo find files in this dir

    # --- Launch Arguments ---
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # --- Robot Description Processing ---
    # Use xacro to process the URDF file
    robot_description_content = Command(['xacro ', urdf_path])

    # --- Nodes and Processes ---

    # 1. Robot State Publisher
    # Publishes TF transforms based on the robot description and joint states
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content
        }]
    )

    # 2. Gazebo Simulation (gz sim)
    # Set GAZEBO_MODEL_PATH so Gazebo finds the models associated with this package
    # Also needs the world file path
    gz_sim_process = ExecuteProcess(
         cmd=['gz', 'sim','-r', world_path,'-v','4'],
         output='screen',
         additional_env={'GZ_SIM_RESOURCE_PATH': gz_sim_resource_path} # Important for finding models/worlds
    )

    # 3. Gazebo Spawner (ros_gz_sim create)
    # Spawns the robot entity into the running Gazebo simulation
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-string', robot_description_content, # Pass processed URDF string
            '-name', 'self_balancing_robot',      # Name of the entity in Gazebo
            '-allow_renaming', 'true',
            # Initial pose (optional, slight tilt forward to encourage falling)
            # '-x', '0.0', '-y', '0.0', '-z', '0.1', '-R', '0.0', '-P', '0.05', '-Y', '0.0'
            '-x', '0.0', '-y', '0.0', '-z', '0.1' # Start slightly above ground
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 4. ROS-Gazebo Bridge (Parameter Bridge)
    # Connects ROS 2 topics with Gazebo topics
    # Crucial for sensor data (IMU), commands (cmd_vel), odometry, clock, tf
    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        output='screen',
        arguments=[
            # Clock (ROS 2 -> Gazebo Sim)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # IMU Sensor (Gazebo Sim -> ROS 2)
            '/model/self_balancing_robot/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
             # Command Velocity (ROS 2 -> Gazebo Sim)
            '/model/self_balancing_robot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
             # Odometry (Gazebo Sim -> ROS 2)
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
             # TF (Gazebo Sim -> ROS 2) - Published by DiffDrive plugin if configured
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
         # Remap Gazebo's /tf topic (Pose_V) to ROS 2's /tf (TFMessage)
        # remappings=[('/tf', '/tf_sim')] # Optional remapping if needed
    )

    # 5. Balance Controller Node (Your C++ Node)
    balance_controller_node = Node(
        package=package_name,
        executable='balance_controller',
        name='balance_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
            # Add other parameters here if you want to override defaults
            # 'kp': 20.0,
            # 'ki': 0.5,
            # 'kd': 1.0
        }]
    )

    # --- Launch Description Assembly ---
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)

    # Add nodes and processes to launch
    ld.add_action(gz_sim_process)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity_node)
    ld.add_action(ros_gz_bridge_node)
    ld.add_action(balance_controller_node)

    return ld
