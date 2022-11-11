import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def generate_launch_description():
    pkg_name = 'fsds_ros2_bridge'
    pkg_dir = os.popen('/bin/bash -c "source /usr/share/colcon_cd/function/colcon_cd.sh && \
        colcon_cd %s && pwd"' % pkg_name).read().split('\n')[1]
    
    # start a turtlesim_node in the turtlesim1 namespace
    plot = Node(
            package='dataplot_pkg',
            namespace='',
            executable='dataplot',
            name='dataplot',
            # arguments=['--layout' , [os.path.join(pkg_dir, 'config', 'plotjuggler', 'plotlayout.xml')]]
        )

    return LaunchDescription([
        plot
    ])
