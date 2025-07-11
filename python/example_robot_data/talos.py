import numpy as np
import pinocchio as pin

from .utils import RobotLoader, readParamsFromSrdf


class TalosLoader(RobotLoader):
    path = "talos_data"
    urdf_filename = "talos_reduced.urdf"
    srdf_filename = "talos.srdf"
    free_flyer = True
    has_rotor_parameters = True


class TalosBoxLoader(TalosLoader):
    urdf_filename = "talos_reduced_box.urdf"


class TalosFullLoader(TalosLoader):
    urdf_filename = "talos_full_v2.urdf"


class TalosFullBoxLoader(TalosLoader):
    urdf_filename = "talos_full_v2_box.urdf"


class TalosArmLoader(TalosLoader):
    urdf_filename = "talos_left_arm.urdf"
    free_flyer = False


class TalosLegsLoader(TalosLoader):
    def __init__(self, verbose=False):
        super().__init__(verbose=verbose)
        legMaxId = 14
        m1 = self.robot.model
        m2 = pin.Model()
        for j, M, name, parent, Y in zip(
            m1.joints, m1.jointPlacements, m1.names, m1.parents, m1.inertias
        ):
            if j.id < legMaxId:
                jid = m2.addJoint(parent, getattr(pin, j.shortname())(), M, name)
                idx_q, idx_v = m2.joints[jid].idx_q, m2.joints[jid].idx_v
                m2.upperPositionLimit[idx_q : idx_q + j.nq] = m1.upperPositionLimit[
                    j.idx_q : j.idx_q + j.nq
                ]
                m2.lowerPositionLimit[idx_q : idx_q + j.nq] = m1.lowerPositionLimit[
                    j.idx_q : j.idx_q + j.nq
                ]
                m2.velocityLimit[idx_v : idx_v + j.nv] = m1.velocityLimit[
                    j.idx_v : j.idx_v + j.nv
                ]
                m2.effortLimit[idx_v : idx_v + j.nv] = m1.effortLimit[
                    j.idx_v : j.idx_v + j.nv
                ]
                assert jid == j.id
                m2.appendBodyToJoint(jid, Y, pin.SE3.Identity())

        upperPos = m2.upperPositionLimit
        upperPos[:7] = 1
        m2.upperPositionLimit = upperPos
        lowerPos = m2.lowerPositionLimit
        lowerPos[:7] = -1
        m2.lowerPositionLimit = lowerPos
        effort = m2.effortLimit
        effort[:6] = np.inf
        m2.effortLimit = effort

        # q2 = self.robot.q0[:19]
        for f in m1.frames:
            if tuple(int(i) for i in pin.__version__.split(".")) >= (3, 0, 0):
                if f.parentJoint < legMaxId:
                    m2.addFrame(f)
            elif f.parent < legMaxId:
                m2.addFrame(f)

        g2 = pin.GeometryModel()
        for g in self.robot.visual_model.geometryObjects:
            if g.parentJoint < 14:
                g2.addGeometryObject(g)

        self.robot.model = m2
        self.robot.data = m2.createData()
        self.robot.visual_model = g2
        # self.robot.q0=q2
        self.robot.visual_data = pin.GeometryData(g2)

        # Load SRDF file
        self.robot.q0 = readParamsFromSrdf(
            self.robot.model,
            self.srdf_path,
            self.verbose,
            self.has_rotor_parameters,
            self.ref_posture,
        )

        assert (m2.armature[:6] == 0.0).all()
        # Add the free-flyer joint limits to the new model
        self.addFreeFlyerJointLimits()
