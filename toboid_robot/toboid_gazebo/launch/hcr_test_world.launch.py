import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro

def generate_launch_description():

  toboid_amr_path = os.path.join(
        get_package_share_directory('toboid_amr'))
  urdf = os.path.join(toboid_amr_path,
                              'urdf/toboid.urdf')

  doc = xacro.parse(open(urdf))
  xacro.process_doc(doc)
  params = {'robot_description': doc.toxml()}

  package_dir=get_package_share_directory('toboid_gazebo')
  pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
  world_file = os.path.join(package_dir,'worlds','test.world')

  return LaunchDescription([
      #launching gazebo server and our custom world in it
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world_file}.items(),
        ),
    # Adding the gzclient launch file to run gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),
    #   publishes TF for links of the robot without joints
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[params]),
    #  To publish tf for Joints only links
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            ),
#  Gazebo related stuff required to launch the robot in simulation
        #ExecuteProcess(
        #    cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        #    output='screen'),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=["-topic", "/robot_description", "-entity", "toboid_amr"])
  ])
