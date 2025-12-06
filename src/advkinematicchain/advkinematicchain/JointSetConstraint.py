import numpy as np
import scipy as sp

from advkinematicchain.AdvancedKinematicChain import IKinConstraint

class JointSetConstraint(IKinConstraint):
    def __init__(self, name, chain, joints, lam=1.0):
        super().__init__(name, chain)
        self.joints = joints
        self.qc = np.array([
            self.chain.qc[self.chain.joint_names.index(joint)]
            for joint in joints
        ])
        self.qcdot = np.zeros(len(joints))
        self.lam = 1.0

    def getRowTargets(self, dt):
        qlast = np.array([
            self.chain.qc[self.chain.joint_names.index(joint)]
            for joint in self.joints
        ])

        return (self.qc - qlast) * self.lam + self.qcdot

    def getVelocityCoeffs(self, dt):
        qc = self.chain.qc

        return np.vstack([
            np.where(np.arange(len(self.chain.joint_names)) == self.chain.joint_names.index(joint), 1, 0).reshape((1, len(self.chain.joint_names)))
            for joint in self.joints
        ])
    
    def getJointPositions(self):
        return self.qc

    def setJointPositions(self, qc):
        old = self.qc
        self.qc = qc
        return old
    
    def getJointVelocitys(self):
        return self.qc

    def setJointVelocitys(self, qcdot):
        old = self.qcdot
        self.qcdot = qcdot
        return old
