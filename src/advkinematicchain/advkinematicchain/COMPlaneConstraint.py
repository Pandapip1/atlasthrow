import numpy as np
import scipy as sp

from advkinematicchain.AdvancedKinematicChain import IKinConstraint

class COMPlaneConstraint(IKinConstraint):
    def __init__(self, name, chain, link1, link2, reference_link, dir_vec, lam=0.5):
        super().__init__(name, chain)
        self.link1 = link1
        self.link2 = link2
        self.reference_link = reference_link
        self.dir_vec = dir_vec / np.linalg.norm(dir_vec)
        self.lam = lam

        self._cache_qc = None
        self._cache_fk = {}
        self._cache_com = None
        self._cache_Jcom = None
    
    def _getCOMData(self, robot, qc):
        if self._cache_qc is None or not np.array_equal(self._cache_qc, qc):
            self._cache_fk.clear()
            self._cache_qc = qc

            com = np.zeros(3)
            Jcom = np.zeros((3, len(self.chain.joint_names)))
            total_mass = 0.

            for link in robot.links:
                if link.inertial is None:
                    continue

                if link.name not in self._cache_fk:
                    self._cache_fk[link.name] = self.chain.relative_fkin(qc, self.reference_link, link.name)
                p, R, Jv, _ = self._cache_fk[link.name]

                inertial_data = link.inertial
                total_mass += inertial_data.mass
                com += (p + R @ inertial_data.origin.xyz) * inertial_data.mass
                Jcom += Jv * inertial_data.mass

            self._cache_com = com / total_mass
            self._cache_Jcom = Jcom / total_mass

        return self._cache_com, self._cache_Jcom

    def getRowTargets(self, dt):
        if dt == 0:
            return np.zeros(1)

        qc = self.chain.qc

        com, _ = self._getCOMData(self.chain.robot, qc)

        p1, _, _, _ = self.chain.relative_fkin(qc, self.reference_link, self.link1)
        p2, _, _, _ = self.chain.relative_fkin(qc, self.reference_link, self.link2)
        normal = np.cross(p2 - p1, self.dir_vec)
        normal = normal / np.linalg.norm(normal)

        dist = np.dot(com - p1, normal)
        v_target = -self.lam * dist / dt

        return np.array([v_target])

    def getVelocityCoeffs(self, dt):
        com, Jcom = self._getCOMData(self.chain.robot, self.chain.qc)

        p1, _, _, _ = self.chain.relative_fkin(self.chain.qc, self.reference_link, self.link1)
        p2, _, _, _ = self.chain.relative_fkin(self.chain.qc, self.reference_link, self.link2)
        normal = np.cross(p2 - p1, self.dir_vec)
        normal = normal / np.linalg.norm(normal)

        return (normal @ Jcom).reshape((1, len(self.chain.joint_names)))
