from os.path import exists, join

import numpy as np
import pinocchio
from pinocchio.robot_wrapper import RobotWrapper


def getModelPath(subpath):
    for path in ['..', '../..', '/opt/openrobots/share/example-robot-data']:
        if exists(join(path, subpath.strip('/'))):
            print("using %s as modelPath" % path)
            return path
    raise IOError('%s not found' % (subpath))


def readParamsFromSrdf(robot, SRDF_PATH, verbose):
    rmodel = robot.model

    pinocchio.loadRotorParameters(rmodel, SRDF_PATH, verbose)
    rmodel.armature = np.multiply(rmodel.rotorInertia.flat, np.square(rmodel.rotorGearRatio.flat))
    pinocchio.loadReferenceConfigurations(rmodel, SRDF_PATH, verbose)
    robot.q0.flat[:] = rmodel.referenceConfigurations["half_sitting"].copy()
    return


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
    readParamsFromSrdf(robot, modelPath + SRDF_SUBPATH, False)
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
    readParamsFromSrdf(robot, modelPath + SRDF_SUBPATH, False)
    return robot
