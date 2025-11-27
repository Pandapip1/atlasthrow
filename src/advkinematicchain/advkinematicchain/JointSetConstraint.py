import numpy as np
import scipy as sp

from advkinematicchain.AdvancedKinematicChain import IKinConstraint

class JointSetConstraint(IKinConstraint):
    def __init__(self, name, chain, link, q0):
        super().__init__(name, chain)
        self.link = link
        self.qc = q0

    def getRowTargets(self, dt):
        return np.array([ self.qc ])

    def getPositionCoeffs(self, dt):
        if dt == 0:
            return np.zeros(1)

        return np.where(np.arange(len(self.chain.joint_names)) == self.chain.joint_names.index(self.link), 1, 0).reshape((1, len(self.chain.joint_names)))

    def getVelocityCoeffs(self, dt):
        return np.zeros((1, len(self.chain.joint_names)))
    
    def getJointPosition(self):
        return self.qc
    
    def setJointPosition(self, qc):
        old = self.qc
        self.qc = qc
        return old
