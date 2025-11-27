import numpy as np
import scipy as sp

from advkinematicchain.AdvancedKinematicChain import IKinConstraint

class JJRelativeProjectionConstraint(IKinConstraint):
    def __init__(self, name, chain, link_base, link_aux, axis, lam = 1.0):
        super().__init__(name, chain)
        self.link_base = link_base
        self.link_aux = link_aux
        self.axis = axis / np.linalg.norm(axis)
        d0, _, _, _ = self.chain.relative_fkin(self.chain.qc, self.link_base, self.link_aux)
        self.dd = np.dot(d0, self.axis)
        self.vd = 0
        self.lam = lam

    def getRowTargets(self, dt):
        qc = self.chain.qc

        dp, _, _, _ = self.chain.relative_fkin(qc, self.link_base, self.link_aux)
        v_target = self.vd + self.lam * (np.dot(dp, self.axis) - self.dd)

        return np.array(v_target * self.axis)

    def getPositionCoeffs(self, dt):
        return np.zeros((3, len(self.chain.joint_names)))

    def getVelocityCoeffs(self, dt):
        qc = self.chain.qc
        _, _, Jv1, _ = self.chain.relative_fkin(qc, self.link_base, self.link_aux)
        _, _, Jv2, _ = self.chain.relative_fkin(qc, self.link_aux, self.link_base)

        return Jv1 - Jv2

    def getDistance(self):
        return self.dd
    
    def setDistance(self, dd):
        old = self.dd
        self.dd = dd
        return old

    def getVelocity(self):
        return self.vd
    
    def setVelocity(self, vd):
        old = self.vd
        self.vd = vd
        return old
