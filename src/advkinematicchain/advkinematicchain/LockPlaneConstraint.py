import numpy as np
import scipy as sp

from advkinematicchain.AdvancedKinematicChain import IKinConstraint

class LockPlaneConstraint(IKinConstraint):
    def __init__(self, name, chain, target_link_ref, target_link_1, target_link_2, axis, lam_v = 1.0, lam_w = 1.0):
        super().__init__(name, chain)
        self.target_link_ref = target_link_ref
        self.target_link_1 = target_link_1
        self.target_link_2 = target_link_2
        self.axis = axis / np.linalg.norm(axis)
        self.lam_v = lam_v
        self.lam_w = lam_w

    def getConstraintType(self):
        return ConstraintType.ROW

    def getRowTargets(self, dt):
        if dt == 0:
            return np.zeros(6)

        qc = self.chain.qc

        p1, R1, _, _ = self.chain.relative_fkin(qc, self.target_link_ref, self.target_link_1)
        p2, R2, _, _ = self.chain.relative_fkin(qc, self.target_link_ref, self.target_link_2)

        dp = p2 - p1
        v_target = -self.lam_v * np.dot(dp, self.axis) * self.axis / dt

        R_err = R1.T @ R2
        W = sp.linalg.logm(R_err)
        w_target = -self.lam_w * np.array([W[2,1], W[0,2], W[1,0]]) / dt

        return np.concatenate([v_target, w_target])

    def getPositionCoeffs(self, dt):
        return np.zeros((6, len(self.chain.joint_names)))

    def getVelocityCoeffs(self, dt):
        qc = self.chain.qc
        _, _, Jv1, Jw1 = self.chain.relative_fkin(qc, self.target_link_ref, self.target_link_1)
        _, _, Jv2, Jw2 = self.chain.relative_fkin(qc, self.target_link_ref, self.target_link_2)

        Jv = np.outer(self.axis, self.axis) @ (Jv2 - Jv1)

        Jw = Jw2 - Jw1

        return np.vstack([Jv, Jw])
