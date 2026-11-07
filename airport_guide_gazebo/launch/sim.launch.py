import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_desc = get_package_share_directory('airport_guide_description')
    pkg_gz = get_package_share_directory('airport_guide_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    xacro_dosyasi = os.path.join(pkg_desc, 'urdf', 'airport_guide.urdf.xacro')
    world_dosyasi = os.path.join(pkg_gz, 'worlds', 'terminal.sdf')
    bridge_dosyasi = os.path.join(pkg_gz, 'config', 'bridge.yaml')
    rviz_dosyasi = os.path.join(pkg_gz, 'rviz', 'airport_guide.rviz')

    robot_description = ParameterValue(Command(['xacro ', xacro_dosyasi]), value_type=str)

    # 1) Gazebo: dunyayi ac ve hemen baslat (-r)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_dosyasi}'}.items(),
    )

    # 2) URDF'i /robot_description'a yayinla + sabit TF'ler
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    # 3) Robotu Gazebo'ya ekle (base_link yerden 0.12 m'de duruyor, biraz yukaridan birak)
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'airport_guide',
                   '-x', '0.0', '-y', '0.0', '-z', '0.15'],
        output='screen',
    )

    # 4) ROS 2 <-> Gazebo koprusu (topic listesi config/bridge.yaml'da)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_dosyasi, 'use_sim_time': True}],
        output='screen',
    )

    # 5) RViz (kayitli ayar dosyasi varsa onu ac, yoksa bos RViz ac)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_dosyasi] if os.path.exists(rviz_dosyasi) else [],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([gazebo, robot_state_publisher, spawn, bridge, rviz])