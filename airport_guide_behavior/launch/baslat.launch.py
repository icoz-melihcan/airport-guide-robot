"""Tek komutla her sey: Gazebo + robot + kopru + RViz, birkac saniye sonra rehber davranisi.

Kullanim:
    ros2 launch airport_guide_behavior baslat.launch.py            # A kapisi (mavi)
    ros2 launch airport_guide_behavior baslat.launch.py hedef:=B   # B kapisi (sari)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    hedef = LaunchConfiguration('hedef')
    gecikme = LaunchConfiguration('gecikme')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('airport_guide_gazebo'), 'launch', 'sim.launch.py')))

    rehber = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('airport_guide_behavior'), 'launch', 'rehber.launch.py')),
        launch_arguments={'hedef': hedef}.items())

    return LaunchDescription([
        DeclareLaunchArgument('hedef', default_value='A',
                              description='Hedef kapi: A (mavi tabela) veya B (sari tabela)'),
        DeclareLaunchArgument('gecikme', default_value='8.0',
                              description='Gazebo acildiktan kac saniye sonra robot yola ciksin'),
        sim,
        # Gazebo ve sensorler hazir olsun diye davranis dugumleri biraz gecikmeli baslar
        TimerAction(period=gecikme, actions=[rehber]),
    ])
