"""Launch the pose publisher demo

   ros2 launch demos pose_publisher.launch.py

   This is only intended to demonstrate the example.  Please
   edit/update as appropriate.

   This starts:
   1) The pose_publisher.py
   2) The subscriber.py
   3) RVIZ to see the visualization markers

"""

import os

from ament_index_python.packages import get_package_share_directory as pkgdir

from launch                      import LaunchDescription
from launch.actions              import Shutdown
from launch_ros.actions          import Node


#
# Generate the Launch Description
#
def generate_launch_description():

    ######################################################################
    # LOCATE FILES

    # Locate the RVIZ configuration file.
    rvizcfg = os.path.join(pkgdir('demos'), 'rviz/viewmarkers.rviz')


    ######################################################################
    # PREPARE THE LAUNCH ELEMENTS

    # Configure a node for the pose_publisher.
    node_publisher = Node(
        name       = 'pose',
        package    = 'demos',
        executable = 'pose_publisher',
        output     = 'screen',
        on_exit    = Shutdown())

    # Configure a node for the subscriber.
    node_subscriber = Node(
        name       = 'subscriber',
        package    = 'demos',
        executable = 'subscriber',
        output     = 'screen',
        on_exit    = Shutdown())

    # Configure a node for RVIZ
    node_rviz = Node(
        name       = 'rviz', 
        package    = 'rviz2',
        executable = 'rviz2',
        output     = 'screen',
        arguments  = ['-d', rvizcfg],
        on_exit    = Shutdown())


    ######################################################################
    # RETURN THE ELEMENTS IN ONE LIST

    return LaunchDescription([
        # Start the boolean pulisher and the subscriber.
        node_publisher,
        node_subscriber,
        node_rviz,
    ])
