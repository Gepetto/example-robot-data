import sys
from os.path import dirname, exists, join

import numpy as np
import pinocchio
from pinocchio.robot_wrapper import RobotWrapper

pinocchio.switchToNumpyArray()


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


def readParamsFromSrdf(robot, SRDF_PATH, verbose, has_rotor_parameters=True, referencePose='half_sitting'):
    rmodel = robot.model

    if has_rotor_parameters:
        pinocchio.loadRotorParameters(rmodel, SRDF_PATH, verbose)
    rmodel.armature = np.multiply(rmodel.rotorInertia.flat, np.square(rmodel.rotorGearRatio.flat))
    pinocchio.loadReferenceConfigurations(rmodel, SRDF_PATH, verbose)
    if referencePose is not None:
        robot.q0.flat[:] = rmodel.referenceConfigurations[referencePose].copy()
    return


def addFreeFlyerJointLimits(robot):
    rmodel = robot.model

    ub = rmodel.upperPositionLimit
    ub[:7] = 1
    rmodel.upperPositionLimit = ub
    lb = rmodel.lowerPositionLimit
    lb[:7] = -1
    rmodel.lowerPositionLimit = lb


def loadANYmal(withArm=None):
    if withArm is None:
        URDF_FILENAME = "anymal.urdf"
        SRDF_FILENAME = "anymal.srdf"
        REF_POSTURE = "standing"
    elif withArm == "kinova":
        URDF_FILENAME = "anymal-kinova.urdf"
        SRDF_FILENAME = "anymal-kinova.srdf"
        REF_POSTURE = "standing_with_arm_up"
    URDF_SUBPATH = "/anymal_b_simple_description/robots/" + URDF_FILENAME
    SRDF_SUBPATH = "/anymal_b_simple_description/srdf/" + SRDF_FILENAME
    modelPath = getModelPath(URDF_SUBPATH)
    # Load URDF file
    robot = RobotWrapper.BuildFromURDF(modelPath + URDF_SUBPATH, [modelPath], pinocchio.JointModelFreeFlyer())
    # Load SRDF file
    readParamsFromSrdf(robot, modelPath + SRDF_SUBPATH, False, False, referencePose=REF_POSTURE)
    # Add the free-flyer joint limits
    addFreeFlyerJointLimits(robot)
    return robot


def loadTalosArm():
    URDF_FILENAME = "talos_left_arm.urdf"
    URDF_SUBPATH = "/talos_data/robots/" + URDF_FILENAME
    SRDF_FILENAME = "talos.srdf"
    SRDF_SUBPATH = "/talos_data/srdf/" + SRDF_FILENAME
    modelPath = getModelPath(URDF_SUBPATH)
    # Load URDF file
    robot = RobotWrapper.BuildFromURDF(modelPath + URDF_SUBPATH, [modelPath])

    # Load SRDF file
    readParamsFromSrdf(robot, modelPath + SRDF_SUBPATH, False)
    return robot


def loadTalos():
    URDF_FILENAME = "talos_reduced.urdf"
    SRDF_FILENAME = "talos.srdf"
    SRDF_SUBPATH = "/talos_data/srdf/" + SRDF_FILENAME
    URDF_SUBPATH = "/talos_data/robots/" + URDF_FILENAME
    modelPath = getModelPath(URDF_SUBPATH)
    # Load URDF file
    robot = RobotWrapper.BuildFromURDF(modelPath + URDF_SUBPATH, [modelPath], pinocchio.JointModelFreeFlyer())
    # Load SRDF file
    readParamsFromSrdf(robot, modelPath + SRDF_SUBPATH, False)
    assert ((robot.model.armature[:6] == 0.).all())
    # Add the free-flyer joint limits
    addFreeFlyerJointLimits(robot)
    return robot


def loadTalosLegs():
    robot = loadTalos()
    URDF_FILENAME = "talos_reduced.urdf"
    SRDF_FILENAME = "talos.srdf"
    SRDF_SUBPATH = "/talos_data/srdf/" + SRDF_FILENAME
    URDF_SUBPATH = "/talos_data/robots/" + URDF_FILENAME
    modelPath = getModelPath(URDF_SUBPATH)

    legMaxId = 14
    m1 = robot.model
    m2 = pinocchio.Model()
    for j, M, name, parent, Y in zip(m1.joints, m1.jointPlacements, m1.names, m1.parents, m1.inertias):
        if j.id < legMaxId:
            jid = m2.addJoint(parent, getattr(pinocchio, j.shortname())(), M, name)
            upperPos = m2.upperPositionLimit
            lowerPos = m2.lowerPositionLimit
            effort = m2.effortLimit
            upperPos[m2.joints[jid].idx_q:m2.joints[jid].idx_q + j.nq] = m1.upperPositionLimit[j.idx_q:j.idx_q + j.nq]
            lowerPos[m2.joints[jid].idx_q:m2.joints[jid].idx_q + j.nq] = m1.lowerPositionLimit[j.idx_q:j.idx_q + j.nq]
            effort[m2.joints[jid].idx_v:m2.joints[jid].idx_v + j.nv] = m1.effortLimit[j.idx_v:j.idx_v + j.nv]
            m2.upperPositionLimit = upperPos
            m2.lowerPositionLimit = lowerPos
            m2.effortLimit = effort
            assert (jid == j.id)
            m2.appendBodyToJoint(jid, Y, pinocchio.SE3.Identity())

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

    g2 = pinocchio.GeometryModel()
    for g in robot.visual_model.geometryObjects:
        if g.parentJoint < 14:
            g2.addGeometryObject(g)

    robot.model = m2
    robot.data = m2.createData()
    robot.visual_model = g2
    # robot.q0=q2
    robot.visual_data = pinocchio.GeometryData(g2)

    # Load SRDF file
    robot.q0 = robot.q0[:robot.model.nq]
    readParamsFromSrdf(robot, modelPath + SRDF_SUBPATH, False)

    assert ((m2.armature[:6] == 0.).all())
    # Add the free-flyer joint limits
    addFreeFlyerJointLimits(robot)
    return robot


def loadHyQ():
    URDF_FILENAME = "hyq_no_sensors.urdf"
    SRDF_FILENAME = "hyq.srdf"
    SRDF_SUBPATH = "/hyq_description/srdf/" + SRDF_FILENAME
    URDF_SUBPATH = "/hyq_description/robots/" + URDF_FILENAME
    modelPath = getModelPath(URDF_SUBPATH)
    # Load URDF file
    robot = RobotWrapper.BuildFromURDF(modelPath + URDF_SUBPATH, [modelPath], pinocchio.JointModelFreeFlyer())
    # Load SRDF file
    readParamsFromSrdf(robot, modelPath + SRDF_SUBPATH, False, False, "standing")
    # Add the free-flyer joint limits
    addFreeFlyerJointLimits(robot)
    return robot


def loadSolo(solo=True):
    if solo:
        URDF_FILENAME = "solo.urdf"
    else:
        URDF_FILENAME = "solo12.urdf"
    SRDF_FILENAME = "solo.srdf"
    SRDF_SUBPATH = "/solo_description/srdf/" + SRDF_FILENAME
    URDF_SUBPATH = "/solo_description/robots/" + URDF_FILENAME
    modelPath = getModelPath(URDF_SUBPATH)
    # Load URDF file
    robot = RobotWrapper.BuildFromURDF(modelPath + URDF_SUBPATH, [modelPath], pinocchio.JointModelFreeFlyer())
    # Load SRDF file
    readParamsFromSrdf(robot, modelPath + SRDF_SUBPATH, False, False, "standing")
    # Add the free-flyer joint limits
    addFreeFlyerJointLimits(robot)
    return robot


def loadKinova():
    URDF_FILENAME = "kinova.urdf"
    SRDF_FILENAME = "kinova.srdf"
    SRDF_SUBPATH = "/kinova_description/srdf/" + SRDF_FILENAME
    URDF_SUBPATH = "/kinova_description/robots/" + URDF_FILENAME
    modelPath = getModelPath(URDF_SUBPATH)
    # Load URDF file
    robot = RobotWrapper.BuildFromURDF(modelPath + URDF_SUBPATH, [modelPath])
    # Load SRDF file
    readParamsFromSrdf(robot, modelPath + SRDF_SUBPATH, False, False, "arm_up")
    return robot


def loadTiago():
    URDF_FILENAME = "tiago.urdf"
    #    SRDF_FILENAME = "tiago.srdf"
    #    SRDF_SUBPATH = "/tiago_description/srdf/" + SRDF_FILENAME
    URDF_SUBPATH = "/tiago_description/robots/" + URDF_FILENAME
    modelPath = getModelPath(URDF_SUBPATH)
    # Load URDF file
    robot = RobotWrapper.BuildFromURDF(modelPath + URDF_SUBPATH, [modelPath])
    # Load SRDF file
    #    readParamsFromSrdf(robot, modelPath+SRDF_SUBPATH, False)
    return robot


def loadTiagoNoHand():
    URDF_FILENAME = "tiago_no_hand.urdf"
    #    SRDF_FILENAME = "tiago.srdf"
    #    SRDF_SUBPATH = "/tiago_description/srdf/" + SRDF_FILENAME
    URDF_SUBPATH = "/tiago_description/robots/" + URDF_FILENAME
    modelPath = getModelPath(URDF_SUBPATH)
    # Load URDF file
    robot = RobotWrapper.BuildFromURDF(modelPath + URDF_SUBPATH, [modelPath])
    # Load SRDF file
    #    readParamsFromSrdf(robot, modelPath+SRDF_SUBPATH, False)
    return robot


def loadICub(reduced=True):
    if reduced:
        URDF_FILENAME = "icub_reduced.urdf"
    else:
        URDF_FILENAME = "icub.urdf"
    SRDF_FILENAME = "icub.srdf"
    SRDF_SUBPATH = "/icub_description/srdf/" + SRDF_FILENAME
    URDF_SUBPATH = "/icub_description/robots/" + URDF_FILENAME
    modelPath = getModelPath(URDF_SUBPATH)
    # Load URDF file
    robot = RobotWrapper.BuildFromURDF(modelPath + URDF_SUBPATH, [modelPath], pinocchio.JointModelFreeFlyer())
    # Load SRDF file
    readParamsFromSrdf(robot, modelPath + SRDF_SUBPATH, False, False)
    # Add the free-flyer joint limits
    addFreeFlyerJointLimits(robot)
    return robot


def loadUR(robot=5, limited=False, gripper=False):
    assert (not (gripper and (robot == 10 or limited)))
    URDF_FILENAME = "ur%i%s_%s.urdf" % (robot, "_joint_limited" if limited else '', 'gripper' if gripper else 'robot')
    URDF_SUBPATH = "/ur_description/urdf/" + URDF_FILENAME
    modelPath = getModelPath(URDF_SUBPATH)
    model = RobotWrapper.BuildFromURDF(modelPath + URDF_SUBPATH, [modelPath])
    if robot == 5 or robot == 3 and gripper:
        SRDF_FILENAME = "ur%i%s.srdf" % (robot, '_gripper' if gripper else '')
        SRDF_SUBPATH = "/ur_description/srdf/" + SRDF_FILENAME
        readParamsFromSrdf(model, modelPath + SRDF_SUBPATH, False, False, None)
    return model


def loadHector():
    URDF_FILENAME = "quadrotor_base.urdf"
    URDF_SUBPATH = "/hector_description/robots/" + URDF_FILENAME
    modelPath = getModelPath(URDF_SUBPATH)
    robot = RobotWrapper.BuildFromURDF(modelPath + URDF_SUBPATH, [modelPath], pinocchio.JointModelFreeFlyer())
    return robot


def loadDoublePendulum():
    URDF_FILENAME = "double_pendulum.urdf"
    URDF_SUBPATH = "/double_pendulum_description/urdf/" + URDF_FILENAME
    modelPath = getModelPath(URDF_SUBPATH)
    robot = RobotWrapper.BuildFromURDF(modelPath + URDF_SUBPATH, [modelPath])
    return robot


def loadRomeo():
    URDF_FILENAME = "romeo.urdf"
    URDF_SUBPATH = "/romeo_description/urdf/" + URDF_FILENAME
    modelPath = getModelPath(URDF_SUBPATH)
    return RobotWrapper.BuildFromURDF(modelPath + URDF_SUBPATH, [modelPath], pinocchio.JointModelFreeFlyer())

def loadIris():
    URDF_FILENAME = "iris_simple.urdf"
    URDF_SUBPATH = "/iris_description/robots/" + URDF_FILENAME
    modelPath = getModelPath(URDF_SUBPATH)
    robot = RobotWrapper.BuildFromURDF(modelPath + URDF_SUBPATH, [modelPath], pinocchio.JointModelFreeFlyer())
    return robot