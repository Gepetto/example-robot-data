import sys
import warnings
from os.path import dirname, exists, join

import numpy as np
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper

pin.switchToNumpyArray()


def getModelPath(subpath, printmsg=False):
    paths = [
        join(dirname(dirname(dirname(dirname(__file__)))), 'robots'),
        join(dirname(dirname(dirname(__file__))), 'robots')
    ]
    try:
        from .path import EXAMPLE_ROBOT_DATA_MODEL_DIR
        paths.append(EXAMPLE_ROBOT_DATA_MODEL_DIR)
    except ImportError:
        pass
    paths += [join(p, '../../../share/example-robot-data/robots') for p in sys.path]
    for path in paths:
        if exists(join(path, subpath.strip('/'))):
            if printmsg:
                print("using %s as modelPath" % path)
            return path
    raise IOError('%s not found' % subpath)


def readParamsFromSrdf(model, SRDF_PATH, verbose=False, has_rotor_parameters=True, referencePose='half_sitting'):
    if has_rotor_parameters:
        pin.loadRotorParameters(model, SRDF_PATH, verbose)
    model.armature = np.multiply(model.rotorInertia.flat, np.square(model.rotorGearRatio.flat))
    pin.loadReferenceConfigurations(model, SRDF_PATH, verbose)
    q0 = pin.neutral(model)
    if referencePose is not None:
        q0 = model.referenceConfigurations[referencePose].copy()
    return q0


def addFreeFlyerJointLimits(model):
    ub = model.upperPositionLimit
    ub[:7] = 1
    model.upperPositionLimit = ub
    lb = model.lowerPositionLimit
    lb[:7] = -1
    model.lowerPositionLimit = lb


def robot_loader(path,
                 urdf_filename,
                 urdf_subpath=None,
                 srdf_filename=None,
                 verbose=False,
                 has_rotor_parameters=False,
                 ref_posture='half_sitting',
                 free_flyer=False):
    """Helper function to load a robot."""
    urdf_path = join(urdf_subpath, urdf_filename)
    model_path = getModelPath(urdf_path)
    robot = RobotWrapper.BuildFromURDF(join(model_path, urdf_path), [join(model_path, '../..')],
                                       pin.JointModelFreeFlyer() if free_flyer else None)

    if srdf_filename is not None:
        robot.q0 = readParamsFromSrdf(robot.model, join(model_path, path, 'srdf', srdf_filename), verbose,
                                      has_rotor_parameters, ref_posture)
    if free_flyer:
        addFreeFlyerJointLimits(robot.model)

    return robot


def loadANYmal(withArm=None):
    if withArm is None:
        URDF_FILENAME = "anymal.urdf"
        SRDF_FILENAME = "anymal.srdf"
        REF_POSTURE = "standing"
    elif withArm == "kinova":
        URDF_FILENAME = "anymal-kinova.urdf"
        SRDF_FILENAME = "anymal-kinova.srdf"
        REF_POSTURE = "standing_with_arm_up"

    URDF_SUBPATH = "anymal_b_simple_description/robots"

    return robot_loader('anymal_b_simple_description',
                        URDF_FILENAME,
                        URDF_SUBPATH,
                        SRDF_FILENAME,
                        ref_posture=REF_POSTURE,
                        free_flyer=True)


def loadTalosArm():
    URDF_FILENAME = "talos_left_arm.urdf"
    SRDF_FILENAME = "talos.srdf"
    URDF_SUBPATH = "talos_data/robots"

    return robot_loader('talos_data', URDF_FILENAME, URDF_SUBPATH, SRDF_FILENAME)


def loadTalos(legs=False):
    URDF_FILENAME = "talos_reduced.urdf"
    SRDF_FILENAME = "talos.srdf"
    URDF_SUBPATH = "talos_data/robots"

    robot = robot_loader('talos_data', URDF_FILENAME, URDF_SUBPATH, SRDF_FILENAME, free_flyer=True)

    assert (robot.model.armature[:6] == 0.).all()

    if legs:
        legMaxId = 14
        m1 = robot.model
        m2 = pin.Model()
        for j, M, name, parent, Y in zip(m1.joints, m1.jointPlacements, m1.names, m1.parents, m1.inertias):
            if j.id < legMaxId:
                jid = m2.addJoint(parent, getattr(pin, j.shortname())(), M, name)
                upperPos = m2.upperPositionLimit
                lowerPos = m2.lowerPositionLimit
                effort = m2.effortLimit
                upperPos[m2.joints[jid].idx_q:m2.joints[jid].idx_q + j.nq] = m1.upperPositionLimit[j.idx_q:j.idx_q +
                                                                                                   j.nq]
                lowerPos[m2.joints[jid].idx_q:m2.joints[jid].idx_q + j.nq] = m1.lowerPositionLimit[j.idx_q:j.idx_q +
                                                                                                   j.nq]
                effort[m2.joints[jid].idx_v:m2.joints[jid].idx_v + j.nv] = m1.effortLimit[j.idx_v:j.idx_v + j.nv]
                m2.upperPositionLimit = upperPos
                m2.lowerPositionLimit = lowerPos
                m2.effortLimit = effort
                assert (jid == j.id)
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

        # q2 = robot.q0[:19]
        for f in m1.frames:
            if f.parent < legMaxId:
                m2.addFrame(f)

        g2 = pin.GeometryModel()
        for g in robot.visual_model.geometryObjects:
            if g.parentJoint < 14:
                g2.addGeometryObject(g)

        robot.model = m2
        robot.data = m2.createData()
        robot.visual_model = g2
        # robot.q0=q2
        robot.visual_data = pin.GeometryData(g2)

        # Load SRDF file
        robot.q0 = robot.q0[:robot.model.nq]
        model_path = getModelPath(join(URDF_SUBPATH, URDF_FILENAME))
        robot.q0 = readParamsFromSrdf(robot.model, join(model_path, 'talos_data/srdf', SRDF_FILENAME), False)

        assert ((m2.armature[:6] == 0.).all())
        # Add the free-flyer joint limits to the new model
        addFreeFlyerJointLimits(robot.model)

    return robot


def loadTalosLegs():
    warnings.warn("`loadTalosLegs()` is deprecated. Please use `loadTalos(legs=True)`", DeprecationWarning, 2)
    return loadTalos(legs=True)


def loadHyQ():
    URDF_FILENAME = "hyq_no_sensors.urdf"
    SRDF_FILENAME = "hyq.srdf"
    URDF_SUBPATH = "hyq_description/robots"

    return robot_loader('hyq_description',
                        URDF_FILENAME,
                        URDF_SUBPATH,
                        SRDF_FILENAME,
                        ref_posture="standing",
                        free_flyer=True)


def loadSolo(solo=True):
    if solo:
        URDF_FILENAME = "solo.urdf"
    else:
        URDF_FILENAME = "solo12.urdf"
    SRDF_FILENAME = "solo.srdf"
    URDF_SUBPATH = "solo_description/robots"

    return robot_loader('solo_description',
                        URDF_FILENAME,
                        URDF_SUBPATH,
                        SRDF_FILENAME,
                        ref_posture="standing",
                        free_flyer=True)


def loadKinova():
    URDF_FILENAME = "kinova.urdf"
    SRDF_FILENAME = "kinova.srdf"
    URDF_SUBPATH = "kinova_description/robots"

    return robot_loader('kinova_description', URDF_FILENAME, URDF_SUBPATH, SRDF_FILENAME, ref_posture="arm_up")


def loadTiago():
    URDF_FILENAME = "tiago.urdf"
    # SRDF_FILENAME = "tiago.srdf"
    URDF_SUBPATH = "tiago_description/robots"
    return robot_loader('tiago_description', URDF_FILENAME, URDF_SUBPATH)


def loadTiagoNoHand():
    URDF_FILENAME = "tiago_no_hand.urdf"
    # SRDF_FILENAME = "tiago.srdf"
    URDF_SUBPATH = "tiago_description/robots"
    return robot_loader('tiago_description', URDF_FILENAME, URDF_SUBPATH)


def loadICub(reduced=True):
    if reduced:
        URDF_FILENAME = "icub_reduced.urdf"
    else:
        URDF_FILENAME = "icub.urdf"
    SRDF_FILENAME = "icub.srdf"
    URDF_SUBPATH = "icub_description/robots"

    return robot_loader('icub_description', URDF_FILENAME, URDF_SUBPATH, SRDF_FILENAME, free_flyer=True)


def loadPanda():
    URDF_FILENAME = "panda.urdf"
    URDF_SUBPATH = "panda_description/urdf"

    return robot_loader('panda_description', URDF_FILENAME, URDF_SUBPATH)


def loadUR(robot=5, limited=False, gripper=False):
    assert (not (gripper and (robot == 10 or limited)))
    URDF_FILENAME = "ur%i%s_%s.urdf" % (robot, "_joint_limited" if limited else '', 'gripper' if gripper else 'robot')
    URDF_SUBPATH = "ur_description/urdf"
    if robot == 5 or robot == 3 and gripper:
        SRDF_FILENAME = "ur%i%s.srdf" % (robot, '_gripper' if gripper else '')
    else:
        SRDF_FILENAME = None

    return robot_loader('ur_description', URDF_FILENAME, URDF_SUBPATH, SRDF_FILENAME, ref_posture=None)


def loadHector():
    URDF_FILENAME = "quadrotor_base.urdf"
    URDF_SUBPATH = "hector_description/robots"

    return robot_loader('hector_description', URDF_FILENAME, URDF_SUBPATH, free_flyer=True)


def loadDoublePendulum():
    URDF_FILENAME = "double_pendulum.urdf"
    URDF_SUBPATH = "double_pendulum_description/urdf"

    return robot_loader('double_pendulum_description', URDF_FILENAME, URDF_SUBPATH)


def loadRomeo():
    URDF_FILENAME = "romeo.urdf"
    URDF_SUBPATH = "romeo_description/urdf"

    return robot_loader('romeo_description', URDF_FILENAME, URDF_SUBPATH, free_flyer=True)


def loadIris():
    URDF_FILENAME = "iris_simple.urdf"
    URDF_SUBPATH = "iris_description/robots"

    return robot_loader('iris_description', URDF_FILENAME, URDF_SUBPATH, free_flyer=True)
