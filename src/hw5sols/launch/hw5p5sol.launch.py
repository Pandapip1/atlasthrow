"""Launch HW5 P5

   ros2 launch hw6sols hw5p5sol.launch.py

This should start
  1) The robot_state_publisher to broadcast the robot model
  2) The HW5 P5 code to test the kinematic chain

"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory as pkgdir

from launch                            import LaunchDescription
from launch.actions                    import DeclareLaunchArgument
from launch.actions                    import OpaqueFunction
from launch.actions                    import Shutdown
from launch.substitutions              import LaunchConfiguration
from launch_ros.actions                import Node


#
# Generate the Launch Description
#
def generate_launch_description():

    ######################################################################
    # LOCATE FILES

    # Locate the URDF file.
    urdf = os.path.join(pkgdir('hw5code'), 'urdf/kintester.urdf')

    # Load the robot's URDF file (XML).
    with open(urdf, 'r') as file:
        robot_description = file.read()


    ######################################################################
    # PREPARE THE LAUNCH ELEMENTS

    # Configure a node for the robot_state_publisher.
    node_robot_state_publisher = Node(
        name       = 'robot_state_publisher', 
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output     = 'screen',
        parameters = [{'robot_description': robot_description}])
    
    # Configure a node for the kinematic chain testing
    node_testing = Node(
        name       = 'kintest', 
        package    = 'hw5sols',
        executable = 'hw5p5sol',
        output     = 'screen',
        on_exit    = Shutdown())


    ######################################################################
    # RETURN THE ELEMENTS IN ONE LIST
    return LaunchDescription([
        # Start the robot_state_publisher, and the testing.
        node_robot_state_publisher,
        node_testing,
    ])
