from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory
import xacro
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_name = 'amr_sim_2511_golf' #<------CHANGE ME
    pkg_path = os.path.join(get_package_share_directory(pkg_name))

   
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    xacro_file = os.path.join(pkg_path,'description_ackerman','robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time} # type: ignore

    #Get robot controller path
    robot_controllers = os.path.join(pkg_path,'config','controller.yaml')


    #controller manager node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/bicycle_steering_controller/tf_odometry", "/tf"),
            ("/bicycle_steering_controller/reference", "/cmd_vel_stamped"),
        ],
    )


    #robot state publisher
    robot_state_pub_bicycle_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    #joint state_broadcaster controller SPWANER
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    #bycicle steering controller SPAWNER
    robot_bicycle_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["bicycle_steering_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay start of joint_state_broadcaster after `robot_controller`
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_bicycle_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    


    nodes = [

        # Check if we're told to use sim time
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        robot_state_pub_bicycle_node,
        robot_bicycle_controller_spawner,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
    ]

    return LaunchDescription(nodes)

