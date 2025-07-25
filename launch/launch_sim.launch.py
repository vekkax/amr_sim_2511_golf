# library to move between files and folders in the O.S.
import os

from ament_index_python.packages import get_package_share_directory

# libraries to define the Launch file and Function
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='amr_sim_2511_golf' #<--- CHANGE ME
    world_path = os.path.join(get_package_share_directory(package_name), 'worlds','track.world')

    #rsp = IncludeLaunchDescription(
    #            PythonLaunchDescriptionSource([os.path.join(
    #                get_package_share_directory(package_name),'launch','rsp.launch.py')]),
    #                launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    #)

    rsp_acker = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','acker.launch.py')]),
                    launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'world': world_path}.items(),
             )

    # Run the spawner node from 
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'golfo'],
                        output='screen')

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joy.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )


    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','mux.yaml')
    
    twist_mux_node = Node(package='twist_mux', 
                    executable='twist_mux',
                    parameters=[twist_mux_params,{'use_sim_time': True}],
                    remappings=[('/cmd_vel_out','/cmd_vel')]
    )

    pkg_path = os.path.join(get_package_share_directory(package_name))
    default_rviz_config_path = os.path.join(pkg_path + '/config/rviz_config.rviz')
    rviz_arg = DeclareLaunchArgument(name='rvizmodel', default_value=str(default_rviz_config_path), description='Absolute path to rviz model file')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizmodel')],
    )

    launch_after_setup = RegisterEventHandler(
        OnProcessStart(
            target_action=spawn_entity,
            on_start=[
                TimerAction(period=5.0, actions=[   rviz_arg,
                                                    rviz_node,])  # wait a bit to ensure setup completes
            ]
        )
    )

    # Launch them all!
    return LaunchDescription([
        
        joystick,
        twist_mux_node,
        gazebo,
        spawn_entity,
        rsp_acker,
        launch_after_setup,

    ])