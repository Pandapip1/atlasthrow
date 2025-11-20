'''repulsion.py

   This provides the repulsion torque used in HW7 Problem 2.
'''

import numpy as np


#
#   Repulsion Joint Torques
#
#   This computes the equivalent joint torques that mimic a repulsion
#   force between the forearm and the top edge of the wall.  It uses
#   the kinematic chains to the elbow (4 joints) and wrist (5 joints)
#   to get the forearm segment.
#
def repulsion(q, elbowchain, wristchain):
    # Compute the wrist and elbow points.
    (pelbow, _, _, _)   = elbowchain.fkin(q[0:4])  # 4 joints
    (pwrist, _, Jv, Jw) = wristchain.fkin(q[0:5])  # 5 joints

    # Expand the Jacobians to all joints (zeroing the irrelevant last 2).
    Jv = np.hstack((Jv, np.zeros((3,2))))
    Jw = np.hstack((Jw, np.zeros((3,2))))

    # Determine the wall (obstacle) "line"
    pw = np.array([0, 0, 0.3])
    dw = np.array([0, 1, 0])

    # Determine the forearm "line"
    pa = pwrist
    da = pelbow - pwrist

    # Solve for the closest point on the forearm.
    a = (pw - pa) @ np.linalg.pinv(np.vstack((-dw, np.cross(dw, da), da)))
    parm = pa + max(0, min(1, a[2])) * da

    # Solve for the matching wall point.
    pwall = pw + dw * np.inner(dw, parm-pw) / np.inner(dw, dw)

    # Compute the distance and repulsion force
    d = np.linalg.norm(parm-pwall)
    F = (parm-pwall) / d**2

    # Map the repulsion force acting at parm to the equivalent force
    # and torque actiing at the wrist point.
    Fwrist = F
    Twrist = np.cross(parm-pwrist, F)

    # Convert the force/torque to joint torques (J^T).
    tau = np.vstack((Jv, Jw)).T @ np.concatenate((Fwrist, Twrist))

    # Return the joint torques.
    return tau
