from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
import xacro
import launch_ros
import launch

def generate_launch_description():
    
    toboid_amr_path = os.path.join(
    get_package_share_directory('toboid_amr'))

    xacro_file = os.path.join(toboid_amr_path,
                              'urdf/toboid.urdf')
    rviz_config_path = os.path.join(toboid_amr_path, 'rviz/urdf_config.rviz')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    package_dir=get_package_share_directory('toboid_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    world_file = os.path.join(package_dir,'worlds','test.world')

    toboid_nav_path = os.path.join(
    get_package_share_directory('toboid_nav'))
    
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    # joint_state_publisher_gui_node = launch_ros.actions.Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    # )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    spawn_entity = launch_ros.actions.Node(
    	package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        output='screen',
        arguments=["-topic", "/robot_description", "-entity", "toboid"]
    )

    robot_localization_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(toboid_nav_path, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=xacro_file,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),

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
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
        robot_localization_node,
        rviz_node,
        
    ])
