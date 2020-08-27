import sys
import warnings
from os.path import dirname, exists, join

import numpy as np

import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper

pin.switchToNumpyArray()


def getModelPath(subpath, printmsg=False):
    source = dirname(dirname(dirname(__file__)))  # top level source directory
    paths = [
        join(dirname(dirname(dirname(source))), 'robots'),  # function called from "make release" in build/ dir
        join(dirname(source), 'robots'),  # function called from a build/ dir inside top level source
        join(source, 'robots')  # function called from top level source dir
    ]
    try:
        from .path import EXAMPLE_ROBOT_DATA_MODEL_DIR, EXAMPLE_ROBOT_DATA_SOURCE_DIR
        paths.append(EXAMPLE_ROBOT_DATA_MODEL_DIR)  # function called from installed project
        paths.append(EXAMPLE_ROBOT_DATA_SOURCE_DIR)  # function called from off-tree build dir
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
                 srdf_filename=None,
                 urdf_subpath='robots',
                 verbose=False,
                 has_rotor_parameters=False,
                 ref_posture='half_sitting',
                 free_flyer=False):
    """Helper function to load a robot."""
    urdf_path = join(path, urdf_subpath, urdf_filename)
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

    return robot_loader('anymal_b_simple_description',
                        URDF_FILENAME,
                        SRDF_FILENAME,
                        ref_posture=REF_POSTURE,
                        free_flyer=True)


def loadTalos(legs=False, arm=False, full=False):
    if arm:
        URDF_FILENAME = "talos_left_arm.urdf"
    elif full:
        URDF_FILENAME = "talos_full_v2.urdf"
    else:
        URDF_FILENAME = "talos_reduced.urdf"
    SRDF_FILENAME = "talos.srdf"

    robot = robot_loader('talos_data', URDF_FILENAME, SRDF_FILENAME, free_flyer=not arm)

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
        model_path = getModelPath(join('talos_data/robots', URDF_FILENAME))
        robot.q0 = readParamsFromSrdf(robot.model, join(model_path, 'talos_data/srdf', SRDF_FILENAME), False)

        assert (m2.armature[:6] == 0.).all()
        # Add the free-flyer joint limits to the new model
        addFreeFlyerJointLimits(robot.model)

    return robot


def loadTalosLegs():
    warnings.warn("`loadTalosLegs()` is deprecated. Please use `loadTalos(legs=True)`", DeprecationWarning, 2)
    return loadTalos(legs=True)


def loadTalosArm():
    warnings.warn("`loadTalosArm()` is deprecated. Please use `loadTalos(arm=True)`", DeprecationWarning, 2)
    return loadTalos(arm=True)


def loadHyQ():
    return robot_loader('hyq_description', "hyq_no_sensors.urdf", "hyq.srdf", ref_posture="standing", free_flyer=True)


def loadSolo(solo=True):
    return robot_loader('solo_description',
                        "solo.urdf" if solo else "solo12.urdf",
                        "solo.srdf",
                        ref_posture="standing",
                        free_flyer=True)


def loadKinova():
    return robot_loader('kinova_description', "kinova.urdf", "kinova.srdf", ref_posture="arm_up")


def loadTiago(hand=True):
    return robot_loader('tiago_description', "tiago.urdf" if hand else "tiago_no_hand.urdf")


def loadTiagoNoHand():
    warnings.warn("`loadTiagoNoHand()` is deprecated. Please use `loadTiago(hand=False)`", DeprecationWarning, 2)
    return loadTiago(hand=False)


def loadICub(reduced=True):
    return robot_loader('icub_description',
                        "icub_reduced.urdf" if reduced else "icub.urdf",
                        "icub.srdf",
                        free_flyer=True)


def loadPanda():
    return robot_loader('panda_description', "panda.urdf", urdf_subpath='urdf')


def loadUR(robot=5, limited=False, gripper=False):
    assert not (gripper and (robot == 10 or limited))
    URDF_FILENAME = "ur%i%s_%s.urdf" % (robot, "_joint_limited" if limited else '', 'gripper' if gripper else 'robot')
    if robot == 5 or robot == 3 and gripper:
        SRDF_FILENAME = "ur%i%s.srdf" % (robot, '_gripper' if gripper else '')
    else:
        SRDF_FILENAME = None

    return robot_loader('ur_description', URDF_FILENAME, SRDF_FILENAME, urdf_subpath='urdf', ref_posture=None)


def loadHector():
    return robot_loader('hector_description', "quadroto_base.urdf", free_flyer=True)


def loadDoublePendulum():
    return robot_loader('double_pendulum_description', "double_pendulum.urdf", urdf_subpath='urdf')


def loadRomeo():
    return robot_loader('romeo_description', "romeo.urdf", urdf_subpath='urdf', free_flyer=True)


def loadIris():
    return robot_loader('iris_description', "iris_simple.urdf", free_flyer=True)


ROBOTS = {
    'anymal': (loadANYmal, {}),
    'anymal_kinova': (loadANYmal, {
        'withArm': 'kinova'
    }),
    'double_pendulum': (loadDoublePendulum, {}),
    'hector': (loadHector, {}),
    'hyq': (loadHyQ, {}),
    'icub': (loadICub, {
        'reduced': False
    }),
    'icub_reduced': (loadICub, {
        'reduced': True
    }),
    'iris': (loadIris, {}),
    'kinova': (loadKinova, {}),
    'panda': (loadPanda, {}),
    'romeo': (loadRomeo, {}),
    'solo': (loadSolo, {}),
    'solo12': (loadSolo, {
        'solo': False
    }),
    'talos': (loadTalos, {}),
    'talos_arm': (loadTalos, {
        'arm': True
    }),
    'talos_legs': (loadTalos, {
        'legs': True
    }),
    'talos_full': (loadTalos, {
        'full': True
    }),
    'tiago': (loadTiago, {}),
    'tiago_no_hand': (loadTiago, {
        'hand': False
    }),
    'ur5': (loadUR, {}),
    'ur5_gripper': (loadUR, {
        'gripper': True
    }),
    'ur5_limited': (loadUR, {
        'limited': True
    }),
}


def load(name, display=False):
    """Load a robot by its name, and optionnaly display it in a viewer."""
    loader, kwargs = ROBOTS[name]
    robot = loader(**kwargs)
    if display:
        robot.initViewer(loadModel=True)
        robot.display(robot.q0)
    return robot
