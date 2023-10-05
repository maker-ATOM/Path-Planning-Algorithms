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

    ld = LaunchDescription()

    ld.add_action(rviz2_launcher)
    ld.add_action(ref_map_launcher)
    
    return ld