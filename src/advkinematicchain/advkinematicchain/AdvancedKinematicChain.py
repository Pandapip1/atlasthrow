'''
KinematicChainSol.py

   This is the solution code for Kinematic Chains (HW5 Problem 5).

   chain = KinematicChain(node, basefame, tipframe, expectedjointnames)

      Initialize the kinematic chain, reading from the URDF message on
      the topic '/robot_description', sent by the robot_state_publisher.
      Determine the kinematic steps walking from the baseframe to the
      tipframe.  This expects the active joints to match the given names.

   (ptip, Rtip, Jv, Jw) = chain.fkin(q)

      Compute the forward kinematics and report the results.


   Node:        As called
   Subscribe:   /robot_description      std_msgs/String
'''

import enum
import rclpy
import numpy as np

from abc                        import ABC, abstractmethod
from rclpy.node                 import Node
from rclpy.qos                  import QoSProfile, DurabilityPolicy
from std_msgs.msg               import String
from urdf_parser_py.urdf        import Robot
import warnings

# Grab the Utilities
from utils.TransformHelpers     import *

######################################################################
#################  HELPER CODE - FEEL FREE TO SKIP  ##################
######################################################################
#
#   Helper Functions: Report info and trigger an error
#
#   This uses the given Node's logger to post information (print) or
#   trigger an error.
#
def info(node, string):
    node.get_logger().info("AdvancedKinematicChain: " + string)

def error(node, string):
    node.get_logger().error("AdvancedKinematicChain: " + string)
    raise Exception(string)


#
#   Helper Function: Read URDF's HTML from /robot_description
#
#   This uses the given Node to read the URDF's HTML desctiption from
#   the standard /robot_description topic.
#
def read_HTML(node):
    # Report the action.
    info(node, "Waiting for the URDF to be published...")

    # Define the variable to receive the html and a callback function
    # to place the message content there.
    html = None
    def callback(msg):
        nonlocal html
        html = msg.data

    # Create a temporary subscriber to receive the URDF.  We use the
    # TRANSIENT_LOCAL durability, so that we see the last message
    # already published (if any).
    topic   = '/robot_description'
    quality = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
    sub     = node.create_subscription(String, topic, callback, quality)

    # Wait for the message.
    while html is None:
        rclpy.spin_once(node)

    # Destroy the subscriber and return the html!
    node.destroy_subscription(sub)
    return html

######################################################################
############################  MAIN CODE  #############################
######################################################################
#
#   Single URDF Step
#
#   This captures a single step from one frame to the next.  It be of type:
#
#     FIXED     Just a fixed T-matrix shift, nothing moving, not an active DOF.
#     REVOLUTE  A fixed T-matrix shift, followed by a rotation about an axis.
#     LINEAR    A fixed T-matrix shift, followed by a transation along an axis.
#
#   It contains several pieces of permanent data (coming from the URDF):
#
#     name      String showing the name
#     type      One of the above
#     Tshift    Fixed shift: Transform of this frame w.r.t. previous frame
#     nlocal    Joint axis (if applicable) in this local frame
#
#   We also add information how this relates to the active joints:
#
#     dof       If an active dof (not FIXED), the dof number
#

# Define the joint types.
class JointType(enum.Enum):
    FIXED    = 0
    REVOLUTE = 1
    LINEAR   = 2

# Define a single step in the URDF (kinematic chain).
class URDFStep():
    def __init__(self, name, joint_type, reverse, Tshift, nlocal):
        nlocal_norm = nlocal / np.linalg.norm(nlocal) if nlocal is not None and not np.isclose(np.linalg.norm(nlocal), 0.) else None
        # Store the permanent/fixed/URDF data.
        self.name       = name        # Joint name
        self.joint_type = joint_type  # Joint type (per above enumeration)
        self.reverse    = reverse     # Whether the joint is reversed (i.e. child to parent)
        self.Tshift     = Tshift      # Transform w.r.t. previous frame
        self.nlocal     = nlocal_norm # Joint axis in this local frame

    def __repr__(self):
        return f"URDFStep(name={self.name}, joint_type={self.joint_type}, reverse={self.reverse})"

    def __eq__(self, other):
        if not isinstance(other, URDFStep): return False
        t_eq = (self.Tshift is None and other.Tshift is None) or (self.Tshift is not None and other.Tshift is not None and np.allclose(self.Tshift, other.Tshift))
        n_eq = (self.nlocal is None and other.nlocal is None) or (self.nlocal is not None and other.nlocal is not None and np.allclose(self.nlocal, other.nlocal))
        return self.name == other.name and self.joint_type == other.joint_type and self.reverse == other.reverse and t_eq and n_eq
    
    def getReverse(self):
        return URDFStep(
            name = self.name,
            joint_type = self.joint_type,
            reverse = not self.reverse,
            Tshift = self.Tshift,
            nlocal = self.nlocal,
        )

class IKinConstraint(ABC):
    def __init__(self, name, chain):
        self.name = name
        self.chain = chain

    @abstractmethod
    def getRowTargets(self, dt) -> np.array:
        """
        Gets the desired values for this constraint

        TIP: use self.chain.qc to get current joint positions, qcdot to get current joint velocities
        """
        pass

    @abstractmethod
    def getVelocityCoeffs(self, dt) -> np.array:
        """
        Gets the joint velocity coefficients for this constraint

        TIP: use self.chain.joint_names to get the ordering
        """
        pass

#
#   Kinematic Chain Object
#
#   This stores the information provided by the URDF in the form of
#   kinematic chain (list of steps).  In particular, see the fkin()
#   function, as it walks up the chain to determine the transforms.
#

# Define the full kinematic chain
class AdvancedKinematicChain():
    # Initialization - load the URDF and set up the chain.
    def __init__(self, node, q0, q0dot, gamma = 0.0):
        self.node = node

        # Initialize constraints array
        self.constraints = []
        self.gamma = gamma # For damped pseudoinverse

        # Read the URDF's HTML description.
        self.robot = Robot.from_xml_string(read_HTML(node))

        # Get joint ordering
        joints = [
            joint for joint in self.robot.joints
            if joint.type not in ['fixed']
        ]
        self.joint_names = [ joint.name for joint in joints ]
        self.joint_lower_limits = np.array([ joint.limit.lower for joint in joints ])
        self.joint_upper_limits = np.array([ joint.limit.upper for joint in joints ])

        self.qc = np.array([
            q0[joint_name] if joint_name in q0 else 0.
            for joint_name in self.joint_names
        ])
        self.qcdot = np.array([
            q0dot[joint_name] if joint_name in q0dot else 0.
            for joint_name in self.joint_names
        ])

        # Traverse the joints
        self.link_traversal = {}
        for initial_link in self.robot.links:
            self.link_traversal[initial_link.name] = {
                initial_link.name: []
            }
            found_connection = True
            while found_connection:
                found_connection = False
                for joint in self.robot.joints:
                    joint_type = \
                        JointType.REVOLUTE if joint.type in [ 'revolute', 'continuous' ] else \
                        JointType.LINEAR if joint.type in [ 'prismatic' ] else \
                        JointType.FIXED if joint.type in [ 'fixed' ] else \
                        error(self.node, f"Unknown joint type: {joint.type}")
                    nlocal = n_from_URDF_axis(joint.axis) if joint_type is not JointType.FIXED else None
                    Tshift = (Teye() if joint.origin is None else T_from_URDF_origin(joint.origin))
                    add_fwd = joint.parent in self.link_traversal[initial_link.name].keys()
                    add_rev = joint.child in self.link_traversal[initial_link.name].keys()
                    add = add_fwd != add_rev
                    og_step = URDFStep(
                        name=joint.name,
                        joint_type=joint_type,
                        Tshift=Tshift,
                        nlocal=nlocal,
                        reverse=False
                    )
                    if add:
                        step = og_step if add_fwd else og_step.getReverse()
                        first = joint.child if add_fwd else joint.parent
                        second = joint.parent if add_fwd else joint.child
                        self.link_traversal[initial_link.name][first] = self.link_traversal[initial_link.name][second] + [ step ]
                        found_connection = True

    def relative_fkin(self, q, initial_link, final_link):
        # Check the number of joints
        if (len(q) != len(self.joint_names)):
            error(self.node, "Given %d joint angles, expected %d" %
                  (len(q), len(self.joint_names)))
        if initial_link not in self.link_traversal:
            error(self.node, f"Could not find initial link {initial_link} in URDF")
        if final_link not in self.link_traversal[initial_link]:
            error(self.node, f"Could not find path to final link {final_link} from initial link {initial_link}")
        
        chain = self.link_traversal[initial_link][final_link]

        ### INITIALIZE ###
        # We will build up three lists.  For each DOF (non-fixed, active
        # step) collect the type, position (pi), axis (ni) w.r.t. the base.
        _type = []
        p     = []
        n     = []
        idxs  = []

        # Initialize the T matrix to walk up the chain, w.r.t. the base frame!
        T = Teye()

        ### PHASE 1: WALK UP THE CHAIN ###
        # We walk the chain, one URDF step at a time, adjusting T as we
        # go.  Each step could be a fixed or active URDF joint.
        for step in chain:
            try:
                idx = self.joint_names.index(step.name)
            except ValueError:
                if step.joint_type != JointType.FIXED:
                    error(self.node, f"{step.name} somehow not in chain?!")
                else:
                    idx = None

            # Take action based on the joint type: Move the transform T
            # up the kinematic chain (remaining w.r.t. the base frame).
            Tshift = step.Tshift if step.Tshift is not None else Teye()
            Ttrans = \
                T_from_Rp(Rotn(step.nlocal, q[idx]), pzero()) if step.joint_type == JointType.REVOLUTE else \
                T_from_Rp(Reye(), step.nlocal * q[idx]) if step.joint_type == JointType.LINEAR else \
                Teye() if step.joint_type == JointType.FIXED else \
                error(self.node, f"Unknown joint type: {step.joint_type}")
            if not step.reverse:
                T = T @ Tshift @ Ttrans
            else:
                T = T @ np.linalg.inv(Ttrans) @ np.linalg.inv(Tshift)

            # For active joints (our DOFs), store the type, positon (pi),
            # and axis (ni) info, w.r.t. the base frame.
            if step.joint_type != JointType.FIXED:
                _type.append(step.joint_type)
                p.append(p_from_T(T))
                n.append(R_from_T(T) @ step.nlocal)
                idxs.append(idx)

        # Collect the tip information.
        ptip = p_from_T(T)
        Rtip = R_from_T(T)

        ### PHASE 2: USE ABOVE INFOMATION TO BUILD THE JACOBIAN ###
        # Collect the Jacobian for each active joint.
        J_len = len(self.joint_names)
        Jv = np.zeros((3, J_len))
        Jw = np.zeros((3, J_len))
        for i in range(len(idxs)):
            idx = idxs[i]
            # Fill in the appropriate Jacobian column based on the
            # type.  The Jacobian (like the data) is w.r.t. the base.
            if _type[i] is JointType.REVOLUTE:
                # Revolute is a rotation:
                Jv[:, idx] = cross(n[i], ptip - p[i])
                Jw[:, idx] = n[i]
            elif _type[i] is JointType.LINEAR:
                # Linear is a translation:
                Jv[:, idx] = n[i]
                Jw[:, idx] = np.zeros(3)

        # Return the info
        return (ptip, Rtip, Jv, Jw)
    
    def relative_ikin(self, initial_link, final_link, final_link_sec=None, pd = None, pd_sec=None, Rd = None, vd = None, wd = None, q_init=None, movable_joints=None, gamma=3e-3, threshold_p = 1e-4, threshold_R = 1e-4):
        chain = self
        if pd is None and Rd is None:
            raise ValueError("One of pd or Rd must be defined")
        
        if movable_joints is None:
            movable_joints = self.joint_names

        e_p = np.inf
        e_R = np.inf
        c = gamma
        
        if q_init is None:
            q_init = np.zeros(len(self.joint_names))
        
        qc = q_init

        inv_mask = [self.joint_names.index(joint) for joint in (set(self.joint_names) - set(movable_joints)) ]
        mask = [self.joint_names.index(joint) for joint in movable_joints ]

        # Newton-Raphson to figure out inverse kinematics
        pclast = pd
        if pd is not None:
            while np.linalg.norm(e_p) >= threshold_p:
                (pc, _, Jv, _) = chain.relative_fkin(qc, initial_link, final_link)

                Jv[:, inv_mask] = 0

                e_p = ep(pclast, pc)
                e_pa = ep(pd, pc)
                qc = qc + c * np.linalg.pinv(Jv) @ e_pa
                pclast = pc

                if pd_sec is not None:
                    (pc_sec, _, Jv_sec, _) = chain.relative_fkin(qc, initial_link, final_link_sec)  

                    Jv_sec[:, inv_mask] = 0

                    e_p_sec = ep(pclast, pc_sec) #???
                    qc = qc + (np.eye(Jv.shape[0]) - np.linalg.pinv(Jv) @ Jv) @ np.linalg.pinv(Jv_sec) @ e_p_sec

        Rclast = Rd
        if Rd is not None:
            while np.linalg.norm(e_R) >= threshold_R:
                (_, Rc, _, Jw) = chain.relative_fkin(qc, initial_link, final_link)

                Jw[:, inv_mask] = 0

                e_R = eR(Rclast, Rc)
                e_Ra = eR(Rd, Rc)
                qc = qc + c * np.linalg.pinv(Jw) @ e_Ra
                Rclast = Rc

        qcdot = np.zeros(len(self.joint_names))
            
        if(vd is not None):
            (_, _, Jv, _) = chain.relative_fkin(qc, initial_link, final_link) 
            Jv[:, inv_mask] = 0
            qcdot = np.linalg.pinv(Jv) @ vd
        elif(wd is not None):
            ...
        
        # qc = qc % (2 * np.pi) # TODO: Handle prismatic

        (pa, Ra, Jva, Jwa) = self.relative_fkin(qc, initial_link, final_link)
        perror = pa - pd
        verror = Jva @ qcdot - vd

        if np.linalg.norm(perror) >= 0.1:
            warnings.warn(f"position off by {perror}")
        if np.linalg.norm(verror) >= 0.1:
            warnings.warn(f"velocity off by {verror}")

        return qc[mask], qcdot[mask] # qcdot will be none if vd and wd are None

    def get_jointnames(self, initial_link, final_link):
        names = []

        for link in self.link_traversal[initial_link][final_link]:
            names.append(link.name)
        
        return names
    
    def add_constraint(self, constraint):
        self.constraints.append(constraint)

    def remove_constraint(self, constraint):
        self.constraints.remove(constraint)

    def clear_constraints(self):
        self.constraints.clear()
    
    def ikin(self, dt):
        N = np.eye(len(self.qc))
        qcdot = np.zeros(len(self.qc))
        for constraint in self.constraints:
            desired = constraint.getRowTargets(dt)
            J = constraint.getVelocityCoeffs(dt)
            J_inv = J.T @ np.linalg.pinv(J @ J.T + np.diag(self.gamma**2 * np.ones(J.shape[0])))
            qcdot += N @ J_inv @ desired
            N -= J_inv @ J

        self.qcdot = qcdot
        self.qc += dt * qcdot
        return self.qc, self.qcdot
