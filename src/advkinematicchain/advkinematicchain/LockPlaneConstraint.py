import numpy as np
import scipy as sp

from advkinematicchain.AdvancedKinematicChain import IKinConstraint

class LockPlaneConstraint(IKinConstraint):
    def __init__(self, name, chain, target_link_ref, target_link_1, target_link_2, axis, lam = 1.0):
        super().__init__(name, chain)
        self.target_link_ref = target_link_ref
        self.target_link_1 = target_link_1
        self.target_link_2 = target_link_2
        self.axis = axis / np.linalg.norm(axis)
        self.lam = lam

    def getRowTargets(self, dt):
        if dt == 0:
            return np.zeros(6)

        qc = self.chain.qc

        _, R1, _, _ = self.chain.relative_fkin(qc, self.target_link_ref, self.target_link_1)
        _, R2, _, _ = self.chain.relative_fkin(qc, self.target_link_ref, self.target_link_2)

        R_err = R1.T @ R2
        W = sp.linalg.logm(R_err)
        w_target = -self.lam * np.array([W[2,1], W[0,2], W[1,0]]) / dt

        return w_target

    def getVelocityCoeffs(self, dt):
        qc = self.chain.qc
        _, _, _, Jw1 = self.chain.relative_fkin(qc, self.target_link_ref, self.target_link_1)
        _, _, _, Jw2 = self.chain.relative_fkin(qc, self.target_link_ref, self.target_link_2)

        Jw = Jw2 - Jw1

        return Jw
