'''
hw3p4.py

   This is a skeleton for HW3 Problem 4.  PLEASE EDIT (SEARCH FOR FIXME)!

   It creates a trajectory generation node to command the joint
   movements.

   Node:        /trajectory
   Publish:     /joint_states           sensor_msgs/JointState
'''

import rclpy
import numpy as np
import tf2_ros

from math               import pi, sin, cos, acos, atan2, sqrt, fmod

from asyncio            import Future
from rclpy.node         import Node
from geometry_msgs.msg  import PoseStamped, TwistStamped
from geometry_msgs.msg  import TransformStamped
from sensor_msgs.msg    import JointState
from std_msgs.msg       import Header

from utils.TransformHelpers     import *


#
#   Trajectory Class
#
#   Pulling the computations out, to make it easier to read?
#

class Trajectory():
    def __init__(self):
        pass

    def jointnames(self):
        return [
            'theta_1',
            'theta_2',
            'theta_3',
            'theta_4',
        ]

    def evaluate(self, t, dt):
        q = np.array([
            np.pi / 3 * np.sin(2 * t),
            np.pi / 4 * np.sin(1 * t),
            np.pi / 2 * np.sin(3 * t),
            np.pi / 9 * np.sin(6 * t),
        ])
        qdot = np.array([
            2 * np.pi / 3 * np.cos(2 * t),
            1 * np.pi / 4 * np.cos(1 * t),
            3 * np.pi / 2 * np.cos(3 * t),
            6 * np.pi / 9 * np.cos(6 * t),
        ])

        return(q, qdot)


#
#   Trajectory Generator Node Class
#
#   This inherits all the standard ROS node stuff, but adds an
#   update() method to be called regularly by an internal timer and a
#   shutdown method to stop the timer.
#
#   Take the node name and the update frequency as arguments.
#
class TrajectoryNode(Node):
    # Initialization.
    def __init__(self, name, future):
        # Initialize the node and store the future object (to end).
        super().__init__(name)
        self.future = future

        ##############################################################
        # INITIALIZE YOUR TRAJECTORY DATA!

        # We pull this into the above object.
        self.trajectory = Trajectory()
        self.jointnames = self.trajectory.jointnames()


        ##############################################################
        # Setup the logistics of the node:
        # Add a publisher to send the joint commands.
        self.pubjoint = self.create_publisher(JointState, '/joint_states', 10)

        # Wait for a connection to happen.  This isn't necessary, but
        # means we don't start until the rest of the system is ready.
        self.get_logger().info("Waiting for a /joint_states subscriber...")
        while(not self.count_subscribers('/joint_states')):
            pass

        # Set up the timer to update at 100Hz, with (t=0) occuring in
        # the first update cycle (dt) from now.
        self.dt    = 0.01                       # 100Hz.
        self.t     = -self.dt                   # Seconds since start
        self.now   = self.get_clock().now()     # ROS time since 1970
        self.timer = self.create_timer(self.dt, self.update)
        self.get_logger().info("Running with dt of %f seconds (%fHz)" %
                               (self.dt, 1/self.dt))

    # Shutdown
    def shutdown(self):
        # Destroy the timer, then shut down the node.
        self.timer.destroy()
        self.destroy_node()


    # Update - send a new joint command every time step.
    def update(self):
        # Increment time.  We do so explicitly to avoid system jitter.
        self.t   = self.t   + self.dt
        self.now = self.now + rclpy.time.Duration(seconds=self.dt)

        ##############################################################
        # COMPUTE THE TRAJECTORY AT THIS TIME INSTANCE.

        # Compute the joint positions and velocities in the trajectory.
        (q, qdot) = self.trajectory.evaluate(self.t, self.dt)


        ##############################################################
        # Finish by publishing the data (here joint commands).
        header=Header(stamp=self.now.to_msg(), frame_id='world')
        self.pubjoint.publish(JointState(
            header=header,
            name=self.jointnames,
            position=q.tolist(),
            velocity=qdot.tolist()))


#
#  Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Create a future object to signal when the trajectory ends.
    future = Future()

    # Initialize the trajectory generator node.
    trajectory = TrajectoryNode('trajectory', future)

    # Spin, meaning keep running (taking care of the timer callbacks
    # and message passing), until interrupted or the trajectory is
    # complete (as signaled by the future object).
    rclpy.spin_until_future_complete(trajectory, future)

    # Report the reason for shutting down.
    if future.done():
        trajectory.get_logger().info("Stopping: " + future.result())
    else:
        trajectory.get_logger().info("Stopping: Interrupted")

    # Shutdown the node and ROS.
    trajectory.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
