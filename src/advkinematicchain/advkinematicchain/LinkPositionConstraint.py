import numpy as np
import scipy as sp

from advkinematicchain.AdvancedKinematicChain import IKinConstraint

class LinkPositionConstraint(IKinConstraint):
    def __init__(self, name, chain, target_link, reference_link, lam=1.0, priority=None, joint_effectiveness=None):
        super().__init__(name, chain, priority)
        self.target_link = target_link
        self.reference_link = reference_link
        p0, _, Jv, _ = self.chain.relative_fkin(self.chain.qc, self.reference_link, self.target_link)
        self.pd = p0
        self.vd = Jv @ self.chain.qcdot
        self.lam = lam
        self.joint_effectiveness = joint_effectiveness or {}

    def _joint_weight_matrix(self):
        joint_names = self.chain.joint_names
        n = len(joint_names)
        W = np.eye(n)

        for name, weight in self.joint_effectiveness.items():
            if name not in joint_names:
                # Unknown joint name; skip or raise if needed
                continue
            idx = joint_names.index(name)
            W[idx, idx] = weight

        return W

    def getRowTargets(self, dt):
        if dt == 0:
            return np.zeros(6)
        qc = self.chain.qc
        pc, Rc, _, _ = self.chain.relative_fkin(qc, self.reference_link, self.target_link)

        dp = self.pd - pc
        vd = self.vd + self.lam * dp / dt

        return np.concatenate([vd])

    def getVelocityCoeffs(self, dt):
        qc = self.chain.qc
        _, _, Jv, Jw = self.chain.relative_fkin(qc, self.reference_link, self.target_link)

        W = self._joint_weight_matrix()
        Jv_scaled = Jv @ W

        # print({ self.chain.joint_names[i]: np.linalg.norm(Jv_scaled[:, i]) for i in range(len(self.chain.joint_names)) })

        return np.vstack([Jv_scaled])

    def getDesiredPosition(self):
        return self.pd

    def getDesiredVelocity(self):
        return self.vd

    def setDesiredPosition(self, pd):
        old = self.pd
        self.pd = pd
        return old

    def setDesiredVelocity(self, vd):
        old = self.vd
        self.vd = vd
        return old
