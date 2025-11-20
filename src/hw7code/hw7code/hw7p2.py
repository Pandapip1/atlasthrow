'''hw7p2.py

   This is a placeholder and skeleton code for HW7 Problem 2.

   Insert your HW7 P1 code and edit for this problem!

'''

import rclpy
import numpy as np
import sys
import tf2_ros

from math               import pi, sin, cos, acos, atan2, sqrt, fmod, exp

from asyncio            import Future
from rclpy.node         import Node
from geometry_msgs.msg  import PoseStamped, TwistStamped
from geometry_msgs.msg  import TransformStamped
from sensor_msgs.msg    import JointState
from std_msgs.msg       import Header, Float64

# Grab the Utilities
from utils.TransformHelpers     import *
from utils.TrajectoryUtils      import *

# Grab the general fkin from HW5 P5.
from hw5sols.KinematicChainSol  import KinematicChain
from hw7code.repulsion import repulsion


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
        self.variation = "C"

        # Define the list of joint names MATCHING THE JOINT NAMES IN THE URDF!
        self.jointnames=['theta1','theta2','thetaX','theta3','theta4','theta5','theta6']

        # Set up the kinematic chain object.
        self.chain = KinematicChain(self, 'world', 'tip', self.jointnames)
        self.elbowchain=KinematicChain(self,"world","elbow",self.jointnames[0:4])
        self.wristchain=KinematicChain(self,"world","wrist",self.jointnames[0:5])

        # Define the matching initial joint/task positions.
        self.q0 = np.radians(np.array([0, 90, 0, -90, 0, 0, 0]))
        #self.p0 = np.array([0.0, 0.55, 1.0])
        #self.R0 = Reye()
        (self.p0, self.R0, _, _) = self.chain.fkin(self.q0)

        # Define the other points.
        self.pleft  = np.array([ 0.3, 0.5, 0.15])
        self.pright = np.array([-0.3, 0.5, 0.15])
        self.Rleft  = Rotx(-np.pi/2) @ Roty(-np.pi/2)
        self.Rleft  = Rotz( np.pi/2) @ Rotx(-np.pi/2)
        self.Rright = Reye()

        # Initialize the stored joint command position and task errors.
        self.qc = self.q0.copy()
        self.ep = vzero()
        self.eR = vzero()
        if self.variation == "A" or self.variation == "C":
            self.ec = np.array([0.])
        elif self.variation == "B":
            self.ec = np.array([0.])
        else:
            raise Exception("oops wrong variation")

        # Pick the convergence bandwidth.
        self.lam = 20
        self.lam_secondary = 10

        self.ranges = np.array([
            [-np.pi, np.pi / 2],
            [-np.pi, np.pi / 2],
            [0, np.pi],
            [-np.pi, 0],
            [-np.pi / 2, np.pi / 2],
            [-np.pi / 2, np.pi / 2],
            [-np.pi / 2, np.pi / 2],
        ])

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

        # # Stop everything after 8s - makes the graphing nicer.
        # if self.t > 8.0:
        #     self.future.set_result("Trajectory has ended")
        #     return

        # Decide which phase we are in:
        if self.t < 3.0:
            # Approach movement:
            (s0, s0dot) = goto(self.t, 3.0, 0.0, 1.0)

            pd = self.p0 + (self.pright - self.p0) * s0
            vd =           (self.pright - self.p0) * s0dot

            Rd = Reye()
            wd = np.zeros(3)

        else:
            # Cyclic (sinusoidal) movements, after the first 3s.
            s    =            cos(pi/2.5 * (self.t-3))
            sdot = - pi/2.5 * sin(pi/2.5 * (self.t-3))

            # Use the path variables to compute the position trajectory.
            pd = np.array([-0.3*s    , 0.5, 0.5-0.35*s**2  ])
            vd = np.array([-0.3*sdot , 0.0,    -0.70*s*sdot])

            # Choose one of the following methods to compute orientation.
            if False:
                alpha    = - pi/4 * (s-1)
                alphadot = - pi/4 * sdot

                Rd = Rotx(-alpha) @ Roty(-alpha)
                wd = (- nx() - Rotx(-alpha) @ ny()) * alphadot

            elif False:
                alpha    = - pi/4 * (s-1)
                alphadot = - pi/4 * sdot
                
                Rd = Rotz(alpha) @ Rotx(-alpha)
                wd = (nz() - Rotz(alpha) @ nx()) * alphadot

            else:
                alpha    = - pi/3 * (s-1)
                alphadot = - pi/3 * sdot

                nleft = np.array([1, 1, -1]) / sqrt(3)
                Rd    = Rotn(nleft, -alpha)
                wd    = - nleft * alphadot
        if self.variation == "A" or self.variation == "C":
            cd = np.array([0.])
        elif self.variation == "B":
            cd = np.array([0.])
        else:
            raise Exception("oops wrong variation")
        
        # Grab the last joint command position and task errors.
        qclast = self.qc
        eplast = self.ep
        eRlast = self.eR
        eclast = self.ec

        # Compute the old forward kinematics to get the Jacobians.
        (_, _, Jv, Jw) = self.chain.fkin(qclast)
        if self.variation == "A" or self.variation == "C":
            Jc = np.array([[
                0., 0., 0., 0., 0., 0., 0.
            ]])
        elif self.variation == "B":
            Jc = np.array([[
                0.5, 0., 1., 0., 0., 0., 0.
            ]])
        else:
            raise Exception("oops wrong variation")

        # Compute the reference velocities (with errors of last cycle).
        vr = vd + self.lam * eplast
        wr = wd + self.lam * eRlast
        cr = cd + self.lam * eclast

        # Compute the commanded gradient of our secondary
        if self.variation == "A" or self.variation == "B":
            g = np.array([0., 0., 0., 0., 0., 0., 0.])
        elif self.variation == "C":
            g = (qclast - (self.ranges[:, 0] + self.ranges[:, 1]) * 0.5) / (self.ranges[:, 1] - self.ranges[:, 0])
        else:
            raise Exception("oops wrong variation")

        # Compute the inverse kinematics.
        J     = np.vstack((Jv, Jw, Jc))
        J_inv = np.linalg.pinv(J)
        xrdot = np.concatenate((vr, wr, cr))
        N     = np.eye(qclast.size) - J_inv @ J
        qcdot = J_inv @ xrdot - self.lam_secondary * (N @ g)

        # Integrate the joint position.
        qc = qclast + self.dt * qcdot

        # Compute the new forward kinematics for equivalent task commands.
        (pc, Rc, _, _) = self.chain.fkin(qc)
        cc = Jc @ qc

        # Save the joint command position and task errors.
        self.qc = qc
        self.ep = ep(pd, pc)
        self.eR = eR(Rd, Rc)
        self.ec = ep(cd, cc)

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
