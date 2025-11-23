'''
atlasquat.py

   This is the code to make Atlas squat.

   This uses the inverse kinematics from Problem 3, but adds a more
   complex trajectory.

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
        self.centerLink = "pelvis"
        self.leftFootLink = "l_foot"
        self.rightFootLink = "r_foot"
        self.leftHandLink = "l_foot"
        self.rightHandLink = "r_foot"

        # Set up the kinematic chain object.
        self.leftFootChain = KinematicChain(self, self.centerLink, self.leftFootLink, order_urdf=True)
        self.rightFootChain = KinematicChain(self, self.centerLink, self.rightFootLink, order_urdf=True)
        self.leftHandChain = KinematicChain(self, self.centerLink, self.leftHandLink, order_urdf=True)
        self.rightHandChain = KinematicChain(self, self.centerLink, self.rightHandLink, order_urdf=True)

        self.jointnames = self.rightHandChain.jointnames

        # Define the matching initial joint/task positions.
        self.q0 = np.zeros(len(self.jointnames))

        (pL0, RL0, _, _) = self.leftFootChain.fkin(self.q0)
        (pR0, RR0, _, _) = self.rightFootChain.fkin(self.q0)

        squat_height = 0.25

        # Define the other points.
        self.leftFootUp = pL0.copy()
        self.leftFootDown = pL0.copy()
        self.leftFootDown[2] -= squat_height
        self.rightFootUp = pR0.copy()
        self.rightFootDown = pR0.copy()
        self.rightFootDown[2] -= squat_height

        # Initialize the stored joint command position and task errors.
        self.qc = self.q0.copy()
        self.ep = np.zeros(6)
        self.eR = np.zeros(6)

        # Pick the convergence bandwidth.
        self.lam = 20

        # Other constants
        self.period = 6.0 # how long it takes for atlas to do one squat (up->down->up)

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

        # Grab the last joint command position and task errors.
        qclast = self.qc
        eplast = self.ep
        eRlast = self.eR

        # # First task: keep velocity of "center of mass" 0
        # pdcom = np.array() # x, y position of current com
        # vdcom = np.zeros(2) # desired x, y velocity of com is 0

        # (_, _, Jhead, _) = self.headChain.fkin(qclast)

        # Jcom = Jhead / 2 # Jacobian for first task
        # Jcom = Jcom[:-1, :] # disregard z coordinate of "com"

        # Second task: moving feet up and down
        t1 = self.t % self.period
        if t1 < self.period / 2:
            # going up->down
            (s, sdot) = goto(t1, self.period/2, 0.0, 1.0)

            pdLeftFoot = self.leftFootDown + (self.leftFootUp - self.leftFootDown) * s
            vdLeftFoot = (self.leftFootUp - self.leftFootDown) * sdot
            pdRightFoot = self.rightFootDown + (self.rightFootUp - self.rightFootDown) * s
            vdRightFoot = (self.rightFootUp - self.rightFootDown) * sdot
        else:
            # going down->up
            (s, sdot) = goto(t1, self.period/2, 0.0, 1.0)

            pdLeftFoot = self.leftFootUp + (self.leftFootDown - self.leftFootUp) * s
            vdLeftFoot = (self.leftFootDown - self.leftFootUp) * sdot
            pdRightFoot = self.rightFootUp + (self.rightFootDown - self.rightFootUp) * s
            vdRightFoot = (self.rightFootDown - self.rightFootUp) * sdot
        
        RdL = Reye()
        RdR = Reye()
        pdfeet = np.concatenate((pdLeftFoot, pdRightFoot)) # target x, y, z coordinates of left + right feet
        Rdfeet = np.vstack((RdL, RdR))
        vdfeet = np.concatenate((vdLeftFoot, vdRightFoot)) # target x,
        wdfeet = np.concatenate((vzero(), vzero()))

        (_, _, JvleftFoot, JwleftFoot) = self.leftFootChain.fkin(qclast)
        (_, _, JvrightFoot, JwRightFoot) = self.rightFootChain.fkin(qclast)

        """
        pdfeet = np.vstack((pdLeftFoot, pdRightFoot)) # target x, y, z coordinates of left + right feet
        vdfeet = np.vstack((vdLeftFoot, vdRightFoot)) # target x, y, z velocities of left + right feet

        Rdfeet = np.vstack((Reye(), Reye()))
        wdfeet = np.vstack((vzero(), vzero()))

        (_, _, JvleftFoot, JwleftFoot) = self.leftFootChain.fkin(qclast)
        (_, _, JvrightFoot, JwRightFoot) = self.rightFootChain.fkin(qclast)
        
        Jvfeet = np.vstack((JvleftFoot, JvrightFoot)) 
        Jwfeet = np.vstack((JwleftFoot, JwRightFoot)) 
        Jfeet = np.vstack((Jvfeet, Jwfeet)) # Jacobian for feet task
        """
        Jfeet = np.vstack((JvleftFoot, JwleftFoot, JvrightFoot, JwRightFoot))
        # Compute the reference velocities (with errors of last cycle).
        vrfeet = vdfeet + self.lam * eplast
        wrfeet = wdfeet + self.lam * eRlast

        # Compute the inverse kinematics.
        xrdotfeet = np.concatenate((vrfeet, wrfeet))
        qcdot = np.linalg.inv(Jfeet) @ xrdotfeet

        # Integrate the joint position.
        qc = qclast + self.dt * qcdot

        (pcleftFoot, RcleftFoot, _, _) = self.leftFootChain(qc)
        (pcrightFoot, RcrightFoot, _, _) = self.rightFootChain(qc)
        epL = ep(pdLeftFoot, pcleftFoot)
        epR = ep(pdRightFoot, pcrightFoot)
        eRL = eR(RdL, RcleftFoot)
        eRR = eR(RdR, RcrightFoot)

        self.ep = np.concatenate((epL, epR))
        self.eR = np.concatenate((eRL, eRR))
        """
        pc = np.vstack((pcleftFoot, pcrightFoot))
        Rc = np.vstack((RcleftFoot, RcrightFoot))
        """
        # Save the joint command position and task errors.
        self.qc = qc
        #self.ep = ep(pdfeet, pc)
        #self.eR = eR6(Rdfeet, Rc)


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
            pose=Pose_from_Rp(Rdfeet,pdfeet)))
        self.pubtwist.publish(TwistStamped(
            header=header,
            twist=Twist_from_vw(vdfeet,wdfeet)))
        self.tfbroad.sendTransform(TransformStamped(
            header=header,
            child_frame_id='desired',
            transform=Transform_from_Rp(Rdfeet,pdfeet)))


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
