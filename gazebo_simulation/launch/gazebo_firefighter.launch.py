import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directories
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_gazebo_simulation = get_package_share_directory('gazebo_simulation')
    
    # Get the world file path
    world_file_path = os.path.join(pkg_gazebo_simulation, 'worlds', 'empty.world')
    if not os.path.exists(world_file_path):
        world_file_path = 'default'  # Use Gazebo's default empty world

    # Get the URDF file path
    urdf_file_name = 'firefighter_robot_gazebo.urdf.xacro'
    urdf_file_path = os.path.join(pkg_gazebo_simulation, 'urdf', urdf_file_name)
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Declare launch arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true')
    
      # Start Gazebo server
    gzserver = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_file_path],
        output='screen'
    )
    
    # Start Gazebo client
    gzclient = ExecuteProcess(
        cmd=['ign', 'gazebo', '-g'],
        output='screen'
    )
    # Start the robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file_path]),
            'use_sim_time': use_sim_time
        }]
    )
    
    # Bridge to convert robot state messages
        # Bridge to convert robot state messages
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',  # ROS to Gazebo
            '/model/firefighter_robot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',  # Model-specific command
            '/model/firefighter_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',  # Odometry feedback
        ],
        output='screen'
    )
    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'firefighter_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        declare_use_sim_time_arg,
        gzserver,
        gzclient,
        robot_state_publisher_node,
        bridge,
        spawn_entity
    ])