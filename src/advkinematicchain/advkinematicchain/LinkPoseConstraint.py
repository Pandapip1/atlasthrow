import numpy as np
import scipy as sp

from advkinematicchain.AdvancedKinematicChain import IKinConstraint

class LinkPoseConstraint(IKinConstraint):
    def __init__(self, name, chain, target_link, reference_link, p0, R0, v0, w0, lam_v = 1.0, lam_w = 1.0):
        super().__init__(name, chain)
        self.target_link = target_link
        self.reference_link = reference_link
        self.pd = p0
        self.Rd = R0
        self.vd = v0
        self.wd = w0
        self.lam_v = lam_v
        self.lam_w = lam_w

    def getRowTargets(self, dt):
        if dt == 0:
            return np.zeros(6)
        qc = self.chain.qc
        pc, Rc, _, _ = self.chain.relative_fkin(qc, self.reference_link, self.target_link)

        dp = self.pd - pc
        vd = self.vd + self.lam_v * dp / dt

        R_err = Rc.T @ self.Rd
        W = sp.linalg.logm(R_err)
        wd = self.wd + self.lam_w * np.array([W[2,1], W[0,2], W[1,0]]) / dt

        return np.concatenate([vd, wd])

    def getPositionCoeffs(self, dt):
        return np.zeros((6, len(self.chain.joint_names)))

    def getVelocityCoeffs(self, dt):
        qc = self.chain.qc
        _, _, Jv, Jw = self.chain.relative_fkin(qc, self.reference_link, self.target_link)
        return np.vstack([Jv, Jw])
    
    def getDesiredPosition(self):
        return self.pd
    
    def getDesiredVelocity(self):
        return self.vd
    
    def getDesiredRotation(self):
        return self.Rd
    
    def getDesiredAngularVelocity(self):
        return self.wd
    
    def setDesiredPosition(self, pd):
        old = self.pd
        self.pd = pd
        return old
    
    def setDesiredVelocity(self, vd):
        old = self.vd
        self.vd = vd
        return old
    
    def setDesiredRotation(self, Rd):
        old = self.Rd
        self.Rd = Rd
        return old
    
    def setDesiredAngularVelocity(self, wd):
        old = self.wd
        self.wd = wd
        return old
