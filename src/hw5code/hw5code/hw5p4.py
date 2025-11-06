'''
hw5p4.py

   This is a skeleton for HW5 Problem 4.  PLEASE EDIT (SEARCH FOR FIXME)!

   This creates a purely rotational movement.

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


#
#   Gimbal Kinematics
#
def fkin(q):
    return Rotz(q[0]) @ Rotx(q[1]) @ Roty(q[2])

def Jac(q):
    J1 = nz()
    J2 = Rotz(q[0]) @ nx()
    J3 = Rotz(q[0]) @ Rotx(q[1]) @ ny()
    return np.hstack((J1.reshape((3,1)), J2.reshape((3,1)), J3.reshape((3,1))))


def Rot(v, alpha):
    # Normalize axis
    v = v / np.linalg.norm(v)
    # Apply rotations to rotate us to X
    theta_z = np.atan2(v[1], v[0])
    v_1 = Rotz(theta_z) @ v
    theta_y = np.atan2(v[2], v_1[0])
    # Rotate so that the X axis points towards v, then rotate around X, then reverse the rotations
    return Rotz(-theta_z) @ Roty(-theta_y) @ Rotx(alpha) @ Roty(theta_y) @ Rotz(theta_z)

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
        self.jointnames = ['pan', 'tilt', 'roll']

        # Initialize the stored joint command position and task error.
        self.q_c = np.zeros(3)
        self.e  = vzero()

        # Pick the convergence bandwidth.
        self.lam = 20


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

        if self.t > 15:
            self.future.set_result("Trajectory has ended")
            return

        ##############################################################
        # COMPUTE THE TRAJECTORY AT THIS TIME INSTANCE.

        # # Stop everything after 15s - makes the graphing nicer.
        # if self.t > 15.0:
        #     self.future.set_result("Trajectory has ended")
        #     return

        # Choose the alpha/beta angles based on the phase.
        if self.t <= 2.0:
            # Part A (t<=2):
            (alpha, alphadot) = goto(self.t, 2, 0, -np.pi / 2)
            (beta,  betadot)  = (0.0, 0.0)
        else:
            # Part B (t>2):
            (alpha, alphadot) = (-np.pi / 2, 0.0)
            (beta,  betadot)  = (self.t - 3 + np.e**(2-self.t), 1-np.e**(2-self.t))

        # Set up the desired rotation and angular velocity.
        axis = np.linalg.inv(Roty(alpha)) @ np.array([0.05, 0.05, 0])
        axis = axis / np.linalg.norm(axis)
        R_d = Roty(alpha) @ Rot(axis, beta)
        omega_d = np.array([0, alphadot, 0]) + axis * betadot

        # Grab the last joint command position and task error.
        last_q_c = self.q_c
        last_e  = self.e

        # Compute the inverse kinematics
        # FIXME: Inverse Kinematics for rotation
        lamdba = 20
        qdot_c = np.linalg.inv(Jac(last_q_c)) @ (omega_d + lamdba * eR(R_d, fkin(last_q_c)))

        # Integrate the joint position.
        q_c = last_q_c + self.dt * qdot_c

        # Compute the error (to be used next cycle).
        # FIXME: Compute the resultant task error.
        e = 0

        # Save the joint command position and task error.
        self.q_c = q_c
        self.e = e

        # Ignore any translation (position and linear velocity).
        p_d = pzero()
        v_d = vzero()


        ##############################################################
        # Finish by publishing the data (joint and task commands).
        #  qc and qcdot = Joint Commands  as  /joint_states  to view/plot
        #  pd and Rd    = Task pos/orient as  /pose & TF     to view/plot
        #  vd and wd    = Task velocities as  /twist         to      plot
        header=Header(stamp=self.now.to_msg(), frame_id='world')
        self.pubjoint.publish(JointState(
            header=header,
            name=self.jointnames,
            position=q_c.tolist(),
            velocity=qdot_c.tolist()))
        self.pubpose.publish(PoseStamped(
            header=header,
            pose=Pose_from_Rp(R_d, p_d)))
        self.pubtwist.publish(TwistStamped(
            header=header,
            twist=Twist_from_vw(v_d, omega_d)))
        self.tfbroad.sendTransform(TransformStamped(
            header=header,
            child_frame_id='desired',
            transform=Transform_from_Rp(R_d, p_d)))


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
