import numpy as np
import scipy as sp

from advkinematicchain.AdvancedKinematicChain import IKinConstraint

class LinkPositionConstraint(IKinConstraint):
    def __init__(self, name, chain, target_link, reference_link, lam = 1.0):
        super().__init__(name, chain)
        self.target_link = target_link
        self.reference_link = reference_link
        p0, R0, Jv, Jw = self.chain.relative_fkin(self.chain.qc, self.reference_link, self.target_link)
        self.pd = p0
        self.Rd = R0
        self.vd = Jv @ self.chain.qcdot
        self.wd = Jv @ self.chain.qcdot
        self.lam = lam

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
        return np.vstack([Jv])
    
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
