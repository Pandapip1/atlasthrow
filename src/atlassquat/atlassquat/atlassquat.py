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

from math               import pi, sin, cos, acos, atan2, sqrt, fmod, exp, floor

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
        self.rightShoulderLink = "pelvis"
        self.worldFrameLink = "l_foot"
        self.throwJointOverrides = {
            "back_bky": 0.,
            "back_bkx": 0.,
            "back_bkz": 0.,
            "l_leg_hpz": 0.,
            "l_leg_hpy": 0.,
            "l_leg_hpx": 0.,
            "l_leg_aky": 0.,
            "l_leg_akx": 0.,
            "l_leg_kny": 0.
        }

        # Set up the kinematic chain object.
        q0 = {
            # Slight bias to correct side of singularity for squat purposes
            "l_leg_hpy": -0.3,
            "r_leg_hpy": -0.3,
            "l_leg_kny": 0.6,
            "r_leg_kny": 0.6,
            "l_leg_aky": -0.3,
            "r_leg_aky": -0.3,
            # "r_arm_ely": -0.2,

            "r_leg_hpx": -0.2,
            "r_leg_akx": 0.2,
            "l_leg_hpx": 0.2,
            "l_leg_akx": -0.2,
        }

        self.chain = AdvancedKinematicChain(self, q0, {}, gamma=0.1)
        self.balancingConstraint = COMPlaneConstraint("balancing", self.chain, self.leftFootLink, self.rightFootLink, self.leftFootLink, np.array([0., 0., 1.]), priority=1, lam=0.1)
        self.footingConstraint1 = LockPlaneConstraint("footLock", self.chain, self.centerLink, self.leftFootLink, self.rightFootLink, np.array([0., 0., 1.]), priority=1, lam=0.1)
        self.footingConstraint2 = JJRelativeProjectionConstraint("footLock2", self.chain, self.leftFootLink, self.rightFootLink, np.array([0., 0., 1.]), priority=1, lam=0.1)

        self.rightHandConstraint = LinkPositionConstraint("rightHandPosition", self.chain, self.rightHandLink, self.worldFrameLink, joint_effectiveness=self.throwJointOverrides, lam=0.7)

        # Balancing
        self.chain.add_constraint(self.balancingConstraint)
        self.chain.add_constraint(self.footingConstraint1)
        self.chain.add_constraint(self.footingConstraint2)

        self.chain.add_constraint(self.rightHandConstraint)

        self.jointnames = self.chain.joint_names

        # Define the matching initial joint/task positions.
        self.q0 = np.array(self.chain.qc)
        
        (pRH0, _, _, _) = self.chain.relative_fkin(self.q0, self.worldFrameLink, self.rightHandLink)
        self.pRH0 = pRH0

        (pLH0, _, _, _) = self.chain.relative_fkin(self.q0, self.worldFrameLink, self.leftHandLink)
        self.pLH0 = pLH0

        (pRF0, RRF0, _, _) = self.chain.relative_fkin(self.q0, self.worldFrameLink, self.rightFootLink)
        self.pRF0 = pRF0
        self.RRF0 = RRF0

        # Initialize the stored joint command position and task errors.
        self.qc = self.q0.copy()
        self.qcdot = self.q0.copy()
        self.ep = np.zeros(6)
        self.eR = np.zeros(6)

        # Pick the convergence bandwidth.
        self.lam = 20

        # Other constants
        self.periodThrow = 2.

        self.balltime = 2.0 # How long the throw takes
        self.throw_offset = 0.5 # how far away from target to throw just dummy
        self.release_height = 0 # dummy can tune later


        #Target and ball code
        self.target_radius = 0.26  # 26 cm sphere
        self.ball_radius = 0.12    # 12 cm sphere
        self.x_bounds = [1, 3]  # x, y limits can change just dummy values
        self.y_bounds = [-2.5, -0.5]
        self.z_bounds = [1, 1.4]  # z limit can change just dummy values

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
        self.dt    = 1e-2                      # 100Hz.
        self.t     = -self.dt                   # Seconds since start
        self.now   = self.get_clock().now()     # ROS time since 1970
        self.timer = self.create_timer(self.dt, self.update)
        self.get_logger().info("Running with dt of %f seconds (%fHz)" %
                               (self.dt, 1/self.dt))
        self.tI = 0.
    
        
    
    # Compute the final thro position and velocity
    def compute_throw_end(self, target_pos):
        (pShoulder, _, _, _) = self.chain.relative_fkin(self.qc, self.worldFrameLink, 'utorso')
        self.pShoulder = pShoulder

        xp, yp, zp = target_pos

        self.throw_direction = atan2(yp - pShoulder[1], xp - pShoulder[0])
        self.get_logger().info(f"throw_direction: {self.throw_direction}")

        xhand = pShoulder[0] + self.throw_offset * np.cos(self.throw_direction) - 0.5
        yhand = pShoulder[1] + self.throw_offset * np.sin(self.throw_direction) - 0.5
        zhand = pShoulder[2] + 0.2

        print(f"pShoulder: {pShoulder}, xhand: {xhand}, yhand: {yhand}")

        xvel = (xp - xhand) / self.balltime
        yvel = (yp - yhand) / self.balltime
        zvel = (zp - zhand - 0.5 * self.gravity * self.balltime**2) / self.balltime

        self.pRHThrow = np.array([xhand, yhand, zhand]) # final position after throw
        self.vRHThrow = np.array([xvel , yvel , zvel]) # final velocity after throw 
        self.get_logger().info(f"pRHThrow: {self.pRHThrow}")
        self.get_logger().info(f"vRHThrow: {self.vRHThrow}")

    # Spawn target at random
    def spawn_new_target(self):
        self.target_position = np.array([
            np.random.uniform(*self.x_bounds),
            np.random.uniform(*self.y_bounds),
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
        self.ball_marker.scale.x = 2 * self.ball_radius
        self.ball_marker.scale.y = 2 * self.ball_radius
        self.ball_marker.scale.z = 2 * self.ball_radius
        self.ball_marker.color.r = 0.1
        self.ball_marker.color.g = 0.8
        self.ball_marker.color.b = 0.1
        self.ball_marker.color.a = 1.0

        self.ball_position = self.pRH0.copy()
        self.ball_velocity = np.zeros(3)
        self.ball_released = False
        self.readyToThrow = False
        self.timesincerelease = 0.0
    
    # CHECK COLLISION
    def ball_collision(self, ball_point):
        ball_pos = np.array([ball_point.x, ball_point.y, ball_point.z])
        dist = np.linalg.norm(ball_pos - self.target_position)

        if dist <= self.target_radius + self.ball_radius:
            if not self.target_hit or self.timesincerelease > self.balltime + 1:
                self.target_hit = True
                self.get_logger().info("TARGET HIT! Respawning...")
                self.spawn_new_target()
                self.spawn_ball()
                self.compute_throw_end(self.target_position)
                self.tI = self.t
        
        if not self.target_hit and self.tI + self.periodThrow + self.balltime + 1 < self.t:
            self.target_hit = True
            self.get_logger().info("TARGET MISSSED! Respawning...")
            self.spawn_new_target()
            self.spawn_ball()
            self.compute_throw_end(self.target_position)
            self.tI = self.t
            
            

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
        tI = self.tI
        tThrow = tI + self.periodThrow

        self.throw_direction_rad = self.throw_direction

        if self.t < tThrow:
            (pdRightHand, vdRightHand) = spline(self.dt, tThrow - self.t, self.rightHandConstraint.getDesiredPosition(), self.pRHThrow, self.rightHandConstraint.getDesiredVelocity(), np.array(self.vRHThrow))
        elif self.t < tThrow + self.balltime:
            (pdRightHand, vdRightHand) = spline(self.dt, tThrow - self.balltime - self.t, self.rightHandConstraint.getDesiredPosition(), self.pRH0, self.rightHandConstraint.getDesiredVelocity(), np.zeros(3))
        else:
            (pdRightHand, vdRightHand) = self.pRH0, np.zeros(3)
        
        self.rightHandConstraint.setDesiredPosition(pdRightHand)
        self.rightHandConstraint.setDesiredVelocity(vdRightHand)
        
        qc, qcdot = self.chain.ikin(self.dt)
        self.qc = qc
        self.qcdot = qcdot

        # Release ball
        self.readyToThrow = self.t >= tThrow

        if self.readyToThrow  and not self.ball_released:
            (pRH, _, Jv, _) = self.chain.relative_fkin(qc, self.worldFrameLink, self.rightHandLink) 
            self.ball_velocity = Jv @ qcdot

            self.ball_released = True
            # self.ball_position = self.pRHThrow
            # self.ball_velocity = vdRightHand

            self.get_logger().info(f"BALL RELEASED with velocity {self.ball_velocity}")
            self.get_logger().info(f"BALL RELEASED at position {pRH}")

        if not self.ball_released:
            (pRH, _, _, _) = self.chain.relative_fkin(qc, self.worldFrameLink, self.rightHandLink) 
            self.ball_position = pRH.copy()
        else:
            self.ball_position += self.ball_velocity * self.dt
            self.ball_velocity[2] += self.gravity * self.dt
            self.timesincerelease += self.dt

        ##############################################################
        # Finish by publishing the data (joint and task commands).
        #  qc and qcdot = Joint Commands  as  /joint_states  to view/plot
        #  pd and Rd    = Task pos/orient as  /pose & TF     to view/plot
        #  vd and wd    = Task velocities as  /twist         to      plot

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
        
        header=Header(stamp=self.now.to_msg(), frame_id='world')
        self.pubjoint.publish(JointState(
            header=header,
            name=self.chain.joint_names,
            position=qc.tolist(),
            velocity=qcdot.tolist()))
        self.pubpose.publish(PoseStamped(
            header=header,
            pose=Pose_from_Rp(np.eye(3), self.rightHandConstraint.getDesiredPosition())))
        self.pubtwist.publish(TwistStamped(
            header=header,
            twist=Twist_from_vw(self.rightHandConstraint.getDesiredVelocity(), np.zeros(3))))

        (ppelvis, Rpelvis, _, _) = self.chain.relative_fkin(qc, self.worldFrameLink, self.centerLink)

        header=Header(stamp=self.now.to_msg(), frame_id='world')
        self.tfbroad.sendTransform(TransformStamped(
            header=header,
            child_frame_id='pelvis',
            transform=Transform_from_Rp(Rpelvis,ppelvis)))

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
