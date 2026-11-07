from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hedef = LaunchConfiguration('hedef')
    return LaunchDescription([
        DeclareLaunchArgument('hedef', default_value='A',
                              description='Hedef kapi: A (mavi tabela) veya B (sari tabela)'),
        Node(package='airport_guide_behavior', executable='algilama_node', name='algilama_node',
             parameters=[{'hedef': hedef, 'use_sim_time': True}], output='screen'),
        Node(package='airport_guide_behavior', executable='guide_node', name='guide_node',
             parameters=[{'hedef': hedef, 'use_sim_time': True}], output='screen'),
    ])