'''
hw6p3sol.py

   This is the solution code for HW6 Problem 3.

   This combines position and orientation movements.  It moves (a)
   from the initial position to the starting point, then (b) up/down
   while rotating the tip (cube).

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

# Grab the general fkin from HW5 P5.
from hw5sols.KinematicChainSol  import KinematicChain


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
        self.jointnames=['theta1','theta2','theta3','theta4','theta5','theta6']

        # Set up the kinematic chain object.
        self.chain = KinematicChain(self, 'world', 'tip', self.jointnames)

        # Define the matching initial joint/task positions.
        self.q0 = np.radians(np.array([0, 90, -90, 0, 0, 0]))
        #self.p0 = np.array([0.0, 0.55, 1.0])
        #self.R0 = Reye()
        (self.p0, self.R0, _, _) = self.chain.fkin(self.q0)

        # Define the other points.
        self.plow  = np.array([0.0, 0.5, 0.3])
        self.phigh = np.array([0.0, 0.5, 0.9])

        # Initialize the stored joint command position and task errors.
        self.qc = self.q0.copy()
        self.ep = vzero()
        self.eR = vzero()

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

        ##############################################################
        # COMPUTE THE TRAJECTORY AT THIS TIME INSTANCE.

        # # Stop everything after 11s - makes the graphing nicer.
        # if self.t > 11.0:
        #     self.future.set_result("Trajectory has ended")
        #     return

        # Decide which phase we are in:
        if self.t < 3.0:
            # Approach movement:
            (s0, s0dot) = goto(self.t, 3.0, 0.0, 1.0)

            pd = self.p0 + (self.plow - self.p0) * s0
            vd =           (self.plow - self.p0) * s0dot

            Rd = Rotz(-pi/2 * s0)
            wd = nz() * (-pi/2 * s0dot)

        else:
            # Pre-compute the path variables.  To show different
            # options, we compute the position path variable using
            # sinusoids and the orientation variable via splines.
            t  = (self.t - 3.0)
            t1 = (self.t - 3.0) % 8.0
    
            sp    =      - cos(pi/2 * t)
            spdot = pi/2 * sin(pi/2 * t)

            if t1 < 4.0:  (sR, sRdot) = goto(t1,     4.0, -1.0,  1.0)
            else:         (sR, sRdot) = goto(t1-4.0, 4.0,  1.0, -1.0)

            # Use the path variables to compute the trajectory.
            pd = 0.5*(self.phigh+self.plow) + 0.5*(self.phigh-self.plow) * sp
            vd =                            + 0.5*(self.phigh-self.plow) * spdot

            Rd = Rotz(pi/2 * sR)
            wd = nz() * (pi/2 * sRdot)

        
        # Grab the last joint command position and task errors.
        qclast = self.qc
        eplast = self.ep
        eRlast = self.eR

        # Compute the old forward kinematics to get the Jacobians.
        (_, _, Jv, Jw) = self.chain.fkin(qclast)

        # Compute the reference velocities (with errors of last cycle).
        vr = vd + self.lam * eplast
        wr = wd + self.lam * eRlast

        # Compute the inverse kinematics.
        J     = np.vstack((Jv, Jw))
        xrdot = np.concatenate((vr, wr))
        qcdot = np.linalg.inv(J) @ xrdot

        # Integrate the joint position.
        qc = qclast + self.dt * qcdot

        # Compute the new forward kinematics for equivalent task commands.
        (pc, Rc, _, _) = self.chain.fkin(qc)

        # Save the joint command position and task errors.
        self.qc = qc
        self.ep = ep(pd, pc)
        self.eR = eR(Rd, Rc)


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
