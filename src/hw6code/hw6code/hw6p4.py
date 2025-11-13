'''
hw6p4sol.py

   This is a skeleton for HW6 Problem 4.  PLEASE EDIT (SEARCH FOR FIXME)!

   This uses the inverse kinematics from Problem 3, but adds a more
   complex trajectory.

   Node:        /trajectory
   Publish:     /joint_states           sensor_msgs/JointState
                /pose                   geometry_msgs.msg/PoseStamped
                /twist                  geometry_msgs.msg/TwistStamped
'''

import rclpy
import numpy as np
import scipy as sp
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

# Grab the general fkin from HW5 P5.
from hw5code.KinematicChain     import KinematicChain

import sys

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
        self.jointnames=['theta1','theta2','theta3','theta4','theta5','theta6']

        self.chain = KinematicChain(self, 'world', 'tip', self.jointnames)

        self.q0 = np.radians(np.array([0, 90, -90, 0, 0, 0]))
        ptip, Rtip, Jv, Jw = self.chain.fkin(self.q0)
        self.p0 = ptip
        self.R0 = Rtip
        self.targets = [
            np.array([-0.3, 0.5, 0.15]),
            np.array([0.3, 0.5, 0.15])
        ]
        self.targets_R = [
            np.eye(3),
            np.array([
                [0, 1, 0],
                [0, 0, -1],
                [-1, 0, 0]
            ]).T
        ]
        self.thru_target = np.array([0, 0.5, 0.5])
        self.last_target_index = -1
        self.target_index = 0
        self.target_initial_t = 0
        self.target_final_t = 3

        ##############################################################
        # Inverse kinematics setup
        self.qlast = self.q0
        self.qdotlast = np.zeros(6)


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

        if self.t > 8.0:
            self.future.set_result("Trajectory has ended")
            return

        ##############################################################
        # COMPUTE THE TRAJECTORY AT THIS TIME INSTANCE.
        ptip, Rtip, Jv, Jw = self.chain.fkin(self.qlast)

        if self.t >= self.target_final_t:
            self.last_target_index = self.target_index
            self.target_index = (self.target_index + 1) % len(self.targets)
            self.target_initial_t = self.t
            self.target_final_t = self.t + 2.5

        target_thru_t = (self.target_initial_t + self.target_final_t) / 2.

        prev_target = self.targets[self.last_target_index] if self.last_target_index != -1 else self.p0

        if self.t < target_thru_t:
            pd, vd = spline(
                self.t - self.target_initial_t,
                target_thru_t - self.target_initial_t,
                prev_target,
                self.thru_target,
                np.zeros(3),
                (self.targets[self.target_index] - prev_target) / (self.target_final_t - self.target_initial_t)
            )
        else:
            pd, vd = spline(
                self.t - target_thru_t,
                self.target_final_t - target_thru_t,
                self.thru_target,
                self.targets[self.target_index],
                (self.targets[self.target_index] - prev_target) / (self.target_final_t - self.target_initial_t),
                np.zeros(3),
            )
        
        # Rotation is independent: calculate angle between current and previous,
        # axis, and current angular velocity and spline it. Completely ignore the
        # intermediate
        Rrel_traj = Rtip.T @ self.targets_R[self.target_index]
        interm, _ = spline(
            self.t - self.target_initial_t,
            self.target_final_t - self.target_initial_t,
            0,
            1,
            0,
            0
        )
        Rd = Rtip @ sp.linalg.expm(interm * sp.linalg.logm(Rrel_traj))

        ##############################################################
        # Inverse kinematics

        lam = 0

        Jv_inv = Jv.T @ np.linalg.inv(Jv @ Jv.T + (lam**2) * np.eye(Jv.shape[0]))

        # Stay inside the null space of the velocity jacobian, ensures angle changes don't cause position changes
        Nv = np.eye(Jv.shape[1]) - Jv_inv @ Jv
        Jw_Nv = Jw @ Nv
        Jw_inv = Nv @ Jw_Nv.T @ np.linalg.inv(Jw_Nv @ Jw_Nv.T + (lam**2) * np.eye(Jw_Nv.shape[0]))

        vc = (pd - ptip) / self.dt

        Rrel = Rd @ Rtip.T
        wc = np.array([Rrel[2, 1] - Rrel[1, 2], Rrel[0, 2] - Rrel[2, 0], Rrel[1, 0] - Rrel[0, 1]]) / self.dt

        qcdot = Jv_inv @ vc + Jw_inv @ wc
        qc = self.qlast + qcdot * self.dt

        self.qlast = qc
        self.qdotlast = qcdot

        ##############################################################
        # Computed angular velocity
        _, _, _, Jwnew = self.chain.fkin(qc)
        wd = Jwnew * qcdot

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
        sys.stdout.flush()


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
