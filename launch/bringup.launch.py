import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    rviz_config = os.path.join(get_package_share_directory('pathplanners'),
                                   'rviz', 'visualizer.rviz')
    
    rviz2_launcher =  Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    
    ref_map_launcher = Node(
            package='pathplanners',
            executable='map_node',
        )
    
    algo_map_launcher = Node(
            package='pathplanners',
            executable='bfs',
        )
    
    static_transform_publisher = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            namespace='',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'algo_map']
    )
    ld = LaunchDescription()

    ld.add_action(rviz2_launcher)
    ld.add_action(ref_map_launcher)
    ld.add_action(static_transform_publisher)
    # ld.add_action(algo_map_launcher)
    
    
    return ld