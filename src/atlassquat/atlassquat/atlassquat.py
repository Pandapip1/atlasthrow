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
import sys

from math               import pi, sin, cos, acos, atan2, sqrt, fmod, exp

from asyncio            import Future
from rclpy.node         import Node
from geometry_msgs.msg  import PoseStamped, TwistStamped
from geometry_msgs.msg  import TransformStamped, Point
from visualization_msgs.msg import Marker
from sensor_msgs.msg    import JointState
from std_msgs.msg       import Header

# Grab the Utilities
from utils.TransformHelpers     import *
from utils.TrajectoryUtils      import *

# Grab the general fkin from HW5 P5.
from advkinematicchain.AdvancedKinematicChain import AdvancedKinematicChain
from advkinematicchain.LinkPoseConstraint import LinkPoseConstraint
from advkinematicchain.LinkPositionConstraint import LinkPositionConstraint
from advkinematicchain.LockPlaneConstraint import LockPlaneConstraint
from advkinematicchain.COMPlaneConstraint import COMPlaneConstraint
from advkinematicchain.JointSetConstraint import JointSetConstraint
from advkinematicchain.JJRelativeProjectionConstraint import JJRelativeProjectionConstraint

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
            "back_bky": -0.5,
            "r_arm_ely": -0.2,
        }

        self.chain = AdvancedKinematicChain(self, q0, {}, gamma=0.15)
        self.leftFootConstraint = LinkPoseConstraint("leftFootPosition", self.chain, self.leftFootLink, self.centerLink, np.zeros(3), np.eye(3), np.zeros(3), np.zeros(3))
        self.rightFootConstraint = LinkPoseConstraint("rightFootPosition", self.chain, self.rightFootLink, self.centerLink, np.zeros(3), np.eye(3), np.zeros(3), np.zeros(3))
        self.torsoAngleConstraint = JointSetConstraint("torsoAngleConstraint", self.chain, [ "back_bkz" ])
        self.footingConstraint1 = LockPlaneConstraint("footLock", self.chain, self.centerLink, self.leftFootLink, self.rightFootLink, np.array([0., 0., 1.]))
        self.footingConstraint2 = JJRelativeProjectionConstraint("footLock2", self.chain, self.leftFootLink, self.rightFootLink, np.array([0., 0., 1.]))
        self.balancingConstraint = COMPlaneConstraint("balancing", self.chain, self.leftFootLink, self.rightFootLink, self.leftFootLink, np.array([0., 0., 1.]))
        self.fullBodyConstraint = JointSetConstraint("fullBodyConstraint", self.chain, self.chain.joint_names)

        self.leftHandConstraint = LinkPositionConstraint("leftHandPosition", self.chain, self.leftHandLink, self.centerLink, np.zeros(3), np.eye(3), np.zeros(3), np.zeros(3))
        self.rightHandConstraint = LinkPositionConstraint("rightHandPosition", self.chain, self.rightHandLink, self.centerLink, np.zeros(3), np.eye(3), np.zeros(3), np.zeros(3))

        # Balancing
        self.chain.add_constraint(self.footingConstraint1)
        self.chain.add_constraint(self.footingConstraint2)
        self.chain.add_constraint(self.balancingConstraint)

        # self.chain.add_constraint(self.leftFootConstraint)
        # self.chain.add_constraint(self.rightFootConstraint)
        # self.chain.add_constraint(self.torsoAngleConstraint)
        # self.chain.add_constraint(self.leftHandConstraint)
        # self.chain.add_constraint(self.rightHandConstraint)
        self.chain.add_constraint(self.fullBodyConstraint)

        self.jointnames = self.chain.joint_names

        # Define the matching initial joint/task positions.
        self.q0 = np.array(self.chain.qc)

        (pL0, RL0, _, _) = self.chain.relative_fkin(self.q0, self.centerLink, self.leftFootLink)
        (pR0, RR0, _, _) = self.chain.relative_fkin(self.q0, self.centerLink, self.rightFootLink)
        
        (pRH0, _, _, _) = self.chain.relative_fkin(self.q0, self.centerLink, self.rightHandLink)
        self.pRH0 = pRH0

        (pLH0, _, _, _) = self.chain.relative_fkin(self.q0, self.centerLink, self.leftHandLink)
        self.pLH0 = pLH0

        squat_height = 0.15

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
        self.periodThrow = 6.0

        self.balltime = 1.2 # How long the throw takes
        self.throw_offset = 1.3 # how far away from target to throw just dummy
        self.release_height = 0.5 # dummy can tune later


        #Target and ball code
        self.target_radius = 0.20   # 20 cm sphere
        self.ball_radius = 0.20   # 20 cm sphere
        self.xy_bounds = [0.6, 2]  # x, y limits can change just dummy values
        self.z_bounds = [0.6, 2]  # z limit can change just dummy values

        self.spawn_new_target()
        self.spawn_ball()

        self.gravity = -1.25
        self.compute_throw_end(self.target_position)

        ##############################################################
        # Setup the logistics of the node:
        # Add publishers to send the joint and task commands.  Also
        # add a TF broadcaster, so the desired pose appears in RVIZ.
        self.pubjoint = self.create_publisher(JointState, '/joint_states', 10)
        self.pubpose  = self.create_publisher(PoseStamped, '/pose', 10)
        self.pubtwist = self.create_publisher(TwistStamped, '/twist', 10)
        self.tfbroad  = tf2_ros.TransformBroadcaster(self)
        self.pubballpos = self.create_publisher(Point, "/ball_position", 10)
        self.pubballmarker = self.create_publisher(Marker, "/ball_marker", 10)
        self.pubtarget = self.create_publisher(Marker, "/target_marker", 10)

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
    
    # Compute the final thro position and velocity
    def compute_throw_end(self, target_pos):
        xp, yp, zp = target_pos

        self.throw_direction = atan2(yp, xp)
        self.get_logger().info(f"throw_direction: {self.throw_direction}")

        xhand = xp - self.throw_offset * cos(self.throw_direction)
        yhand = yp - self.throw_offset * sin(self.throw_direction)
        zhand = self.release_height

        xvel = (xp - xhand) / self.balltime
        yvel = (yp - yhand) / self.balltime
        zvel = (zp - zhand + 0.5 * self.gravity * self.balltime**2) / self.balltime

        self.pRHThrow = np.array([xhand, yhand, zhand]) # final position after throw
        self.vRHThrow = np.array([xvel , yvel , zvel]) # final velocity after throw 
        self.get_logger().info(f"pRHThrow: {self.pRHThrow}")
        self.get_logger().info(f"vRHThrow: {self.vRHThrow}")

        self.qInitThrow = self.chain.qc # joint configuration at start of throw
        (self.qFinThrow, self.qdotFinThrow) = self.chain.relative_ikin(self.leftFootConstraint, self.rightHandLink, pd=self.pRHThrow, vd=self.vRHThrow) # joint configuration at end of throw

    # Spawn target at random
    def spawn_new_target(self):
        self.target_position = np.array([
            np.random.uniform(*self.xy_bounds),
            np.random.uniform(*self.xy_bounds),
            np.random.uniform(*self.z_bounds),
        ])
        self.get_logger().info(f"New target spawned at {self.target_position}")
        self.target_hit = False

        self.target_marker = Marker()
        self.target_marker.header.frame_id = "world"
        self.target_marker.type = Marker.SPHERE
        self.target_marker.action = Marker.ADD

        self.target_marker.scale.x = 2 * self.target_radius
        self.target_marker.scale.y = 2 * self.target_radius
        self.target_marker.scale.z = 2 * self.target_radius

        self.target_marker.color.r = 1.0
        self.target_marker.color.g = 0.1
        self.target_marker.color.b = 0.1
        self.target_marker.color.a = 1.0
    
    # Spawn ball in hand
    def spawn_ball(self):
        self.ball_marker = Marker()
        self.ball_marker.header.frame_id = "world"
        self.ball_marker.type = Marker.SPHERE
        self.ball_marker.action = Marker.ADD
        self.ball_marker.scale.x = self.ball_radius
        self.ball_marker.scale.y = self.ball_radius
        self.ball_marker.scale.z = self.ball_radius
        self.ball_marker.color.r = 0.1
        self.ball_marker.color.g = 0.8
        self.ball_marker.color.b = 0.1
        self.ball_marker.color.a = 1.0

        self.ball_position = self.pRH0.copy()
        self.ball_velocity = np.zeros(3)
        self.ball_released = False
    
    # CHECK COLLISION
    def ball_collision(self, ball_point):
        ball_pos = np.array([ball_point.x, ball_point.y, ball_point.z])
        dist = np.linalg.norm(ball_pos - self.target_position)

        if dist <= self.target_radius:
            if not self.target_hit:
                self.target_hit = True
                self.get_logger().info("TARGET HIT! Respawning...")
                self.spawn_new_target()
                self.spawn_ball()
                self.compute_throw_end(self.target_position)

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
        t2 = self.t % self.periodThrow

        tI = 0.
        tThrow = self.periodThrow / 2.
        tReturn = self.periodThrow

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

        if t2 < tThrow:
            (pdRightHand, vdRightHand) = spline(t2 - tI, tThrow - tI, self.pRH0, self.pRHThrow, np.zeros(3), np.array(self.vRHThrow))

            (qcThrow, qcdotThrow) = spline(t2 - tI, tThrow, self.qInitThrow, self.qFinThrow, np.zeros(len(self.qdotFinThrow)), self.qdotFinThrow)
        else:
            (pdRightHand, vdRightHand) = spline(t2 - tThrow, tReturn - tThrow, self.pRHThrow, self.pRH0, np.array(self.vRHThrow), np.zeros(3))

            (qcThrow, qcdotThrow) = spline(t2 - tThrow, tReturn - tThrow, self.qFinThrow, self.qInitThrow,  self.qdotFinThrow, np.zeros(len(self.qdotFinThrow)))

        RdL = Reye()
        RdR = Reye()
        pdfeet = np.concatenate((pdLeftFoot, pdRightFoot)) # target x, y, z coordinates of left + right feet
        Rdfeet = np.vstack((RdL, RdR))
        vdfeet = np.concatenate((vdLeftFoot, vdRightFoot)) # target x,
        wdfeet = np.concatenate((vzero(), vzero()))

        self.fullBodyConstraint.setJointPositions(qcThrow)
        self.fullBodyConstraint.setJointVelocitys(qcdotThrow)

        self.leftFootConstraint.setDesiredPosition(pdLeftFoot)
        self.leftFootConstraint.setDesiredVelocity(vdLeftFoot)
        self.rightFootConstraint.setDesiredPosition(pdRightFoot)
        self.rightFootConstraint.setDesiredVelocity(vdRightFoot)

        self.rightHandConstraint.setDesiredPosition(pdRightHand) # uncomment if joint spline doesn't work
        self.rightHandConstraint.setDesiredVelocity(vdRightHand)

            

        #self.leftHandConstraint.setDesiredPosition(pdLeftHand)
        #self.leftHandConstraint.setDesiredVelocity(vdLeftHand)

        # self.torsoAngleConstraint.setJointPosition(wzdTorso)

        # self.rightElbowAngleConstraint.setJointPosition(0)

        qc, qcdot = self.chain.ikin(self.dt)

        # Release ball
        readyToThrow = t2 >= tThrow

        if readyToThrow and not self.ball_released:
            (_, _, Jv, _) = self.chain.relative_fkin(qc, self.centerLink, self.rightHandLink) 
            self.ball_velocity = Jv @ qcdot

            self.ball_released = True
            self.get_logger().info(f"BALL RELEASED with velocity {self.ball_velocity}")
            self.get_logger().info(f"RH at position {pdRightHand}")

        if not self.ball_released:
            (pRH, _, _, _) = self.chain.relative_fkin(qc, self.centerLink, self.rightHandLink) 
            self.ball_position = pRH.copy()
        else:
            self.ball_position += self.ball_velocity * self.dt
            self.ball_velocity[2] += self.gravity * self.dt

        ##############################################################
        # Finish by publishing the data (joint and task commands).
        #  qc and qcdot = Joint Commands  as  /joint_states  to view/plot
        #  pd and Rd    = Task pos/orient as  /pose & TF     to view/plot
        #  vd and wd    = Task velocities as  /twist         to      plot
        header=Header(stamp=self.now.to_msg(), frame_id='world')

        ball_point = Point()
        ball_point.x = float(self.ball_position[0])
        ball_point.y = float(self.ball_position[1])
        ball_point.z = float(self.ball_position[2])
        self.pubballpos.publish(ball_point)

        self.ball_marker.header.stamp = self.get_clock().now().to_msg()
        self.ball_marker.pose.position = ball_point
        self.pubballmarker.publish(self.ball_marker)

        self.ball_collision(ball_point)

        self.target_marker.header.stamp = self.get_clock().now().to_msg()
        self.target_marker.pose.position.x = float(self.target_position[0])
        self.target_marker.pose.position.y = float(self.target_position[1])
        self.target_marker.pose.position.z = float(self.target_position[2])
        self.pubtarget.publish(self.target_marker)

        self.pubjoint.publish(JointState(
            header=header,
            name=self.chain.joint_names,
            position=qc.tolist(),
            velocity=qcdot.tolist()))
        self.pubpose.publish(PoseStamped(
            header=header,
            pose=Pose_from_Rp(RdL, pdRightHand)))
        self.pubtwist.publish(TwistStamped(
            header=header,
            twist=Twist_from_vw(vdfeet, wdfeet)))
        self.tfbroad.sendTransform(TransformStamped(
            header=header,
            child_frame_id='desired',
            transform=Transform_from_Rp(RdL,pdRightHand)))

        sys.stdout.flush()
        sys.stderr.flush()

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
