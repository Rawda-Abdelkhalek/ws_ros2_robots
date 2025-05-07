import os
from ament_index_python.packages import (get_package_prefix, get_package_share_directory)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription)
from launch.substitutions import (Command, PathJoinSubstitution, LaunchConfiguration)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import (Node, SetParameter)
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument,SetEnvironmentVariable
from pathlib import Path


def generate_launch_description ():
        
        package_description = "my_robot_description"
        package_gazebo = "my_robot_bringup"
        
        directory_description = get_package_share_directory(package_description)
        directory_gazebo = get_package_share_directory(package_gazebo)
        
        urdf_path = os.path.join(directory_description,
                             'urdf', 'my_robot.urdf.xacro')
        
        
        launch_file_dir = os.path.join(directory_description,"launch")
        

        gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.path.join(directory_gazebo,"worlds"),
            str(Path(directory_gazebo).parent.resolve())
        ])
    )

        robot_description = ParameterValue(Command(['xacro', urdf_path]), value_type=str) 
        
        robot_state_publisher_cmd= IncludeLaunchDescription (
                PythonLaunchDescriptionSource(
                        os.path.join(launch_file_dir,"display.launch.py")
                )
        )

        gzsim_pkg = get_package_share_directory("ros_gz_sim")
        gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gzsim_pkg, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            directory_gazebo,
            'worlds',
            'empty_world.sdf'
        ])}.items(),
        )


        gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        name="my_robot_spawn",
        arguments=[
            "-name", "my_robot",
            "-allow_renaming", "true",
            "-topic", "robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.1",
        ],
        output="screen",
        )

        bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(directory_gazebo, 'config', 'gazebo_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
        )
        rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(directory_description, 'rviz', 'urdf_config.rviz')],
        )

        
        return LaunchDescription([
              gz_resource_path,
              bridge,  
              gz_spawn_entity,
              gz_sim,
              robot_state_publisher_cmd,
              rviz_node,
        ])