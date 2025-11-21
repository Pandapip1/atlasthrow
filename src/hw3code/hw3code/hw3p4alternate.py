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

FIXME!  CAUTION: COPY THIS FILE TO HW3P4.PY IF YOU PREFER THIS ALTERNATE FORM!!

class Trajectory():
    # Initialization.
    def __init__(self):

        FIXME!  For now remove this line.  In the future, edit here!

        # Initialize any variables that you want to store between cycles!
        pass

    # Declare the joint names.
    def jointnames(self):

        FIXME!  For now remove this line.  In the future, change the names!

        # Return a list of joint names MATCHING THE JOINT NAMES IN THE URDF!
        return ['pan', 'tilt']

    # Evaluate at the given time and time step since last call.
    def evaluate(self, t, dt):

        FIXME!  Adjust these computations (and remove this line)

        # Compute the joint values.  For now, we do this individually
        # before building up numpy arrays.
        theta_pan  = -1.0 * t
        omega_pan  = -1.0

        theta_tilt =  2.0 * t
        omega_tilt =  2.0

        # We build up numpy arrays, consistent with future computations.
        q    = np.array([theta_pan, theta_tilt])
        qdot = np.array([omega_pan, omega_tilt])

        # Return as NumPy arrays
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
