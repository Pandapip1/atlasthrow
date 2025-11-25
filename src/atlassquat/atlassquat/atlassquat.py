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
from advkinematicchain.AdvancedKinematicChain import AdvancedKinematicChain
from advkinematicchain.LinkPoseConstraint import LinkPoseConstraint
from advkinematicchain.LockPlaneConstraint import LockPlaneConstraint
from advkinematicchain.COMPlaneConstraint import COMPlaneConstraint

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
        self.leftHandLink = "l_hand"
        self.rightHandLink = "r_hand"

        # Set up the kinematic chain object.
        q0 = {
            # Slight bias to correct side of singularity for squat purposes
            "l_leg_hpy": -0.2,
            "r_leg_hpy": -0.2,
            "l_leg_kny": 0.2,
            "r_leg_kny": 0.2,
        }
        self.chain = AdvancedKinematicChain(self, q0, {})
        self.leftFootConstraint = LinkPoseConstraint("leftFootPosition", self.chain, self.leftFootLink, self.centerLink, np.zeros(3), np.eye(3), np.zeros(3), np.zeros(3))
        self.rightFootConstraint = LinkPoseConstraint("rightFootPosition", self.chain, self.rightFootLink, self.centerLink, np.zeros(3), np.eye(3), np.zeros(3), np.zeros(3))
        self.footingConstraint = LockPlaneConstraint("footLock", self.chain, self.centerLink, self.leftFootLink, self.rightFootLink, np.array([0., 0., 1.]))
        self.balancingConstraint = COMPlaneConstraint("balancing", self.chain, self.leftFootLink, self.rightFootLink, self.leftFootLink, np.array([0., 0., 1.]))
        self.chain.add_constraint(self.leftFootConstraint)
        self.chain.add_constraint(self.rightFootConstraint)
        self.chain.add_constraint(self.footingConstraint)
        self.chain.add_constraint(self.balancingConstraint)

        self.jointnames = self.chain.joint_names

        # Define the matching initial joint/task positions.
        self.q0 = np.zeros(len(self.jointnames))

        (pL0, RL0, _, _) = self.chain.relative_fkin(self.q0, self.centerLink, self.leftFootLink)
        (pR0, RR0, _, _) = self.chain.relative_fkin(self.q0, self.centerLink, self.rightFootLink)

        squat_height = 0.25

        # Define the other points.
        self.leftFootUp = pL0.copy()
        self.leftFootUp[2] += squat_height
        self.leftFootDown = pL0.copy()
        
        self.rightFootUp = pR0.copy()
        self.rightFootUp[2] += squat_height
        self.rightFootDown = pR0.copy()

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

        # Second task: moving feet up and down
        t1 = self.t % self.period
        offset = np.array([0., 0., 0.])
        if t1 < self.period / 2:
            # going up->down
            (s, sdot) = goto(t1, self.period/2, 0., 1.)

            pdLeftFoot = self.leftFootDown + (self.leftFootUp - self.leftFootDown) * s + offset
            vdLeftFoot = (self.leftFootUp - self.leftFootDown) * sdot
            pdRightFoot = self.rightFootDown + (self.rightFootUp - self.rightFootDown) * s + offset
            vdRightFoot = (self.rightFootUp - self.rightFootDown) * sdot
        else:
            # going down->up
            (s, sdot) = goto(t1 - self.period/2, self.period/2, 1., 0.)

            pdLeftFoot = self.leftFootDown + (self.leftFootUp - self.leftFootDown) * s + offset
            vdLeftFoot = (self.leftFootUp - self.leftFootDown) * sdot
            pdRightFoot = self.rightFootDown + (self.rightFootUp - self.rightFootDown) * s + offset
            vdRightFoot = (self.rightFootUp - self.rightFootDown) * sdot
        
        RdL = Reye()
        RdR = Reye()
        pdfeet = np.concatenate((pdLeftFoot, pdRightFoot)) # target x, y, z coordinates of left + right feet
        Rdfeet = np.vstack((RdL, RdR))
        vdfeet = np.concatenate((vdLeftFoot, vdRightFoot)) # target x,
        wdfeet = np.concatenate((vzero(), vzero()))

        self.leftFootConstraint.setDesiredPosition(pdLeftFoot)
        self.leftFootConstraint.setDesiredVelocity(vdLeftFoot)
        self.rightFootConstraint.setDesiredPosition(pdRightFoot)
        self.rightFootConstraint.setDesiredVelocity(vdRightFoot)

        qc, qcdot = self.chain.ikin(self.dt)

        ##############################################################
        # Finish by publishing the data (joint and task commands).
        #  qc and qcdot = Joint Commands  as  /joint_states  to view/plot
        #  pd and Rd    = Task pos/orient as  /pose & TF     to view/plot
        #  vd and wd    = Task velocities as  /twist         to      plot
        header=Header(stamp=self.now.to_msg(), frame_id='world')
        self.pubjoint.publish(JointState(
            header=header,
            name=self.chain.joint_names,
            position=qc.tolist(),
            velocity=qcdot.tolist()))
        self.pubpose.publish(PoseStamped(
            header=header,
            pose=Pose_from_Rp(RdL, pdLeftFoot)))
        self.pubtwist.publish(TwistStamped(
            header=header,
            twist=Twist_from_vw(vdfeet, wdfeet)))
        self.tfbroad.sendTransform(TransformStamped(
            header=header,
            child_frame_id='desired',
            transform=Transform_from_Rp(RdL,pdLeftFoot)))


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
