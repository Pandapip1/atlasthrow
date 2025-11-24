import numpy as np
import scipy as sp

from advkinematicchain.AdvancedKinematicChain import IKinConstraint, ConstraintType

class LinkPoseConstraint(IKinConstraint):
    def __init__(self, name, chain, target_link, reference_link, p0, R0):
        super().__init__(name, chain)
        self.target_link = target_link
        self.reference_link = reference_link
        self.pd = p0
        self.Rd = R0

    def getConstraintType(self):
        return ConstraintType.ROW

    def getRowTargets(self, dt):
        qc = self.chain.qc
        pc, Rc, _, _ = self.chain.relative_fkin(qc, self.reference_link, self.target_link)

        dp = self.pd - pc
        vd = dp / dt

        R_err = self.Rd @ Rc.T
        W = sp.linalg.logm(R_err)
        wd = -np.array([W[2,1], W[0,2], W[1,0]]) / dt

        return np.concatenate([vd, wd])

    def getPositionCoeffs(self, dt):
        n = len(self.chain.joint_names)
        return np.vstack([np.zeros(n), np.zeros(n), np.zeros(n), np.zeros(n), np.zeros(n), np.zeros(n)])

    def getVelocityCoeffs(self, dt):
        qc = self.chain.qc
        _, _, Jv, Jw = self.chain.relative_fkin(qc, self.reference_link, self.target_link)
        return np.vstack([Jv, Jw])
    
    def getDesiredPosition(self):
        return self.pd
    
    def getDesiredRotation(self):
        return self.Rd
    
    def setDesiredPosition(self, pd):
        old = self.pd
        self.pd = pd
        return old
    
    def setDesiredRotation(self, Rd):
        old = self.Rd
        self.Rd = Rd
        return old

    # Unused gradient descent stuff
    def getPositionGradient(self):
        raise NotImplementedError()

    def getVelocityGradient(self):
        raise NotImplementedError()

    def getGain(self, joint_positions=None, joint_velocities=None):
        raise NotImplementedError()
