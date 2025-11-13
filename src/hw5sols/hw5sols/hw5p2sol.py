'''
hw5p2sol.py

   This is the solution code for HW5 Problem 2.

   Repeatedly and smoothly move the 3DOF, with resultant task data.

   Node:        /trajectory
   Publish:     /joint_states           sensor_msgs/JointState
                /pose                   geometry_msgs.msg/PoseStamped
                /twist                  geometry_msgs.msg/TwistStamped
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

# Also import the 3DOF kinematics.
from hw4sols.hw4p1sol           import fkin, Jac


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
        self.qA = np.radians(np.array([   0,  60, -120]))
        self.qB = np.radians(np.array([ -90, 135,  -90]))
        self.qC = np.radians(np.array([-180,   0,    0]))

        # Set the pan velocity, requested at B.  Both compute the same:
        vpan    = 1.5 * pi/4
        _, vpan = goto(2, 4, 0, -pi)

        self.zero  = np.zeros(3)
        self.qdotB = np.array([vpan, 0.0, 0.0])

        self.get_logger().info(f'The pan velocity at B is {vpan} rad/sec')


        ##############################################################
        # Setup the logistics of the node:
        # Add publishers to send the joint and task commands.  Also
        # add a TF broadcaster, so the desired pose appears in RVIZ.
        self.pubjoint = self.create_publisher(JointState, '/joint_states', 10)
        self.pubpose  = self.create_publisher(PoseStamped, '/pose', 10)
        self.pubtwist = self.create_publisher(TwistStamped, '/twist', 10)
        self.tfbroad  = tf2_ros.TransformBroadcaster(self)

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

        # # Stop everything after 12 seconds - makes the graphing nicer.
        # if self.t > 12:
        #     self.future.set_result("Trajectory has ended")
        #     return

        # First modulo the time by 6 seconds
        t = fmod(self.t, 6.0)

        # Compute the joint values
        if False:
            # Case (a)
            if   (t < 2.0): (qd, qddot) = goto(t    , 2.0, self.qA, self.qB)
            elif (t < 4.0): (qd, qddot) = goto(t-2.0, 2.0, self.qB, self.qC)
            else:           (qd, qddot) = goto(t-4.0, 2.0, self.qC, self.qA)

        else:
            # Case (b)
            if   (t < 2.0): (qd, qddot) = spline(t    , 2.0, self.qA, self.qB, self.zero, self.qdotB)
            elif (t < 4.0): (qd, qddot) = spline(t-2.0, 2.0, self.qB, self.qC, self.qdotB, self.zero)
            else:           (qd, qddot) = goto(  t-4.0, 2.0, self.qC, self.qA)

        # Calculate the matching tip movements (task space) using the
        # forward kinematics.
        pd = fkin(qd)
        Rd = Reye()             # Ignore any orientation
        vd = Jac(qd) @ qddot
        wd = vzero()            # Ignore any angular velocity

        # With the joint trajectory, the resulting joint command (qc)
        # is the same as the joint desired (qd)!
        # NOTE: Do not confuse qc (command) with qC (third target).
        qc    = qd
        qcdot = qddot


        ##############################################################
        # Finish by publishing the data (joint and task commands).
        #  qc and qcdot = Joint Commands  as  /joint_states  to view/plot
        #  pd and Rd    = Task pos/orient as  /pose & TF     to view/plot
        #  vd and wd    = Task velocities as  /twist         to      plot
        header=Header(stamp=self.now.to_msg(), frame_id='world')
        self.pubjoint.publish(JointState(
            header=header,
            name=self.jointnames,
            position=qc.tolist(),
            velocity=qcdot.tolist()))
        self.pubpose.publish(PoseStamped(
            header=header,
            pose=Pose_from_Rp(Rd,pd)))
        self.pubtwist.publish(TwistStamped(
            header=header,
            twist=Twist_from_vw(vd,wd)))
        self.tfbroad.sendTransform(TransformStamped(
            header=header,
            child_frame_id='desired',
            transform=Transform_from_Rp(Rd,pd)))


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
