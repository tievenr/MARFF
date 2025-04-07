import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
def generate_launch_description():
    # Path to the URDF file
    urdf_file_name = 'firefighter_robot.urdf.xacro'
    urdf_file_path = os.path.join(
        get_package_share_directory('rviz_simulation'),
        'urdf',
        urdf_file_name)
    
    # Rviz configuration file
    rviz_config_file = os.path.join(
        get_package_share_directory('rviz_simulation'),
        'config',
        'view_robot.rviz')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Declare launch arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true')
    
        # Start the robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file_path]),
            'use_sim_time': use_sim_time
        }]
    )
    
    # Start joint state publisher GUI
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )
    
    # Start RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Create the launch description and populate it
    ld = LaunchDescription()
    
    # Add declared launch arguments
    ld.add_action(declare_use_sim_time_arg)
    
    # Add nodes to launch description
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(rviz_node)
    
    return ld