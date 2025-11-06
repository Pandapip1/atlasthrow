'''
hw5p1.py

   This is a skeleton for HW5 Problem 1.  PLEASE EDIT (SEARCH FOR FIXME)!

   Repeatedly and smoothly move the 3DOF.

   Node:        /trajectory
   Publish:     /joint_states           sensor_msgs/JointState
'''

import rclpy
import numpy as np
import tf2_ros

from math               import pi, sin, cos, acos, atan2, sqrt, fmod, exp

from asyncio            import Future
from rclpy.node         import Node
from geometry_msgs.msg  import PoseStamped, TwistStamped
from geometry_msgs.msg  import TransformStamped
from sensor_msgs.msg    import JointState
from std_msgs.msg       import Header

# Grab the Utilities
from utils.TransformHelpers     import *
from utils.TrajectoryUtils      import *


#
#   Trajectory Generator Node Class
#
#   This inherits all the standard ROS node stuff, but adds an
#   update() method to be called regularly by an internal timer and a
#   shutdown method to stop the timer.
#
#   Arguments are the node name and a future object (to force a shutdown).
#
class TrajectoryNode(Node):
    # Initialization.
    def __init__(self, name, future):
        # Initialize the node and store the future object (to end).
        super().__init__(name)
        self.future = future

        ##############################################################
        # INITIALIZE YOUR TRAJECTORY DATA!

        # Define the list of joint names MATCHING THE JOINT NAMES IN THE URDF!
        self.jointnames = ['theta1', 'theta2', 'theta3']

        # Define the three joint positions.
        self.qF = [
            np.radians(np.array([   0,  60, -120])),
            np.radians(np.array([ -90, 135,  -90])),
            np.radians(np.array([-180,   0,    0]))
        ]

        # Precompute velocity data
        timestep = 2
        vA = np.zeros(3)
        vC = np.zeros(3)
        # https://www.symbolab.com/solver/matrix-calculator/rref%5Cbegin%7Bpmatrix%7D1%26-1%261%5E%7B2%7D%26-1%5E%7B3%7D%260%260%260%260%26a%5C%5C%20%20%20%200%20%260%260%260%261%261%261%5E%7B2%7D%261%5E%7B3%7D%26b%5C%5C%20%20%20%20%201%260%260%260%260%260%260%260%26c%5C%5C%20%20%20%200%260%260%260%261%260%260%260%26c%5C%5C%20%20%20%200%261%26-2%5Ccdot1%263%5Ccdot1%5E%7B2%7D%260%260%260%260%26d%5C%5C%20%20%20%200%260%260%260%260%261%262%5Ccdot1%263%5Ccdot1%5E%7B2%7D%26e%5C%5C%20%20%20%200%261%260%260%260%26-1%260%260%260%5C%5C%20%20%20%200%260%262%260%260%260%26-2%260%260%5Cend%7Bpmatrix%7D?or=input
        vB = (3/4 * (self.qF[2] - self.qF[0])) / timestep

        # Assign final list
        self.vF = [vA, vB, vC]


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

        # Stop everything after 8 seconds - makes the graphing nicer.
        if self.t > 12:
            self.future.set_result("Trajectory has ended")
            return

        # First modulo the time by 4 seconds
        t = fmod(self.t, 4.0)

        # Compute the joint values
        currentLeg = round((self.t // 2) % 3)
        nextLeg = (currentLeg + 1) % 3
        (qd, qddot) = spline(t % 2, 2., self.qF[currentLeg], self.qF[nextLeg], self.vF[currentLeg], self.vF[nextLeg])

        ##############################################################
        # Finish by publishing the data:
        #  qd and qddot = Joint Desired   as  /joint_states  to view/plot
        header=Header(stamp=self.now.to_msg(), frame_id='world')
        self.pubjoint.publish(JointState(
            header=header,
            name=self.jointnames,
            position=qd.tolist(),
            velocity=qddot.tolist()))


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
