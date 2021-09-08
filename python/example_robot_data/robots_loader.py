import sys
import warnings
from os.path import dirname, exists, join

import numpy as np
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper

pin.switchToNumpyArray()


def _depr_msg(deprecated, key):
    return "`%s` is deprecated. Please use `load('%s')`" % (deprecated, key)


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


class RobotLoader(object):
    path = ''
    urdf_filename = ''
    srdf_filename = ''
    sdf_filename = ''
    urdf_subpath = 'robots'
    srdf_subpath = 'srdf'
    sdf_subpath = ''
    ref_posture = 'half_sitting'
    has_rotor_parameters = False
    free_flyer = False
    verbose = False

    def __init__(self):
        if self.urdf_filename:
            if self.sdf_filename:
                raise AttributeError("Please choose between URDF *or* SDF")
            df_path = join(self.path, self.urdf_subpath, self.urdf_filename)
            builder = RobotWrapper.BuildFromURDF
        else:
            df_path = join(self.path, self.sdf_subpath, self.sdf_filename)
            try:
                builder = RobotWrapper.BuildFromSDF
            except AttributeError:
                raise ImportError("Building SDF models require pinocchio >= 2.9.2")
        self.model_path = getModelPath(df_path, self.verbose)
        self.df_path = join(self.model_path, df_path)
        self.robot = builder(self.df_path, [join(self.model_path, '../..')],
                             pin.JointModelFreeFlyer() if self.free_flyer else None)

        if self.srdf_filename:
            self.srdf_path = join(self.model_path, self.path, self.srdf_subpath, self.srdf_filename)
            self.robot.q0 = readParamsFromSrdf(self.robot.model, self.srdf_path, self.verbose,
                                               self.has_rotor_parameters, self.ref_posture)

            if pin.WITH_HPP_FCL and pin.WITH_HPP_FCL_BINDINGS:
                # Add all collision pairs
                self.robot.collision_model.addAllCollisionPairs()

                # Remove collision pairs per SRDF
                pin.removeCollisionPairs(self.robot.model, self.robot.collision_model, self.srdf_path, False)

                # Recreate collision data since the collision pairs changed
                self.robot.collision_data = self.robot.collision_model.createData()
        else:
            self.srdf_path = None
            self.robot.q0 = pin.neutral(self.robot.model)

        if self.free_flyer:
            self.addFreeFlyerJointLimits()

    def addFreeFlyerJointLimits(self):
        ub = self.robot.model.upperPositionLimit
        ub[:7] = 1
        self.robot.model.upperPositionLimit = ub
        lb = self.robot.model.lowerPositionLimit
        lb[:7] = -1
        self.robot.model.lowerPositionLimit = lb

    @property
    def q0(self):
        warnings.warn("`q0` is deprecated. Please use `robot.q0`", FutureWarning, 2)
        return self.robot.q0


class A1Loader(RobotLoader):
    path = 'a1_description'
    urdf_filename = "a1.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "a1.srdf"
    ref_posture = "standing"
    free_flyer = True


class ANYmalLoader(RobotLoader):
    path = 'anymal_b_simple_description'
    urdf_filename = "anymal.urdf"
    srdf_filename = "anymal.srdf"
    ref_posture = "standing"
    free_flyer = True


class ANYmalKinovaLoader(ANYmalLoader):
    urdf_filename = "anymal-kinova.urdf"
    srdf_filename = "anymal-kinova.srdf"
    ref_posture = "standing_with_arm_up"


class BaxterLoader(RobotLoader):
    path = "baxter_description"
    urdf_filename = "baxter.urdf"
    urdf_subpath = "urdf"


def loadANYmal(withArm=None):
    if withArm:
        warnings.warn(_depr_msg('loadANYmal(kinova)', 'anymal_kinova'), FutureWarning, 2)
        loader = ANYmalKinovaLoader
    else:
        warnings.warn(_depr_msg('loadANYmal()', 'anymal'), FutureWarning, 2)
        loader = ANYmalLoader
    return loader().robot


class TalosLoader(RobotLoader):
    path = 'talos_data'
    urdf_filename = "talos_reduced.urdf"
    srdf_filename = "talos.srdf"
    free_flyer = True


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
    def __init__(self):
        super(TalosLegsLoader, self).__init__()
        legMaxId = 14
        m1 = self.robot.model
        m2 = pin.Model()
        for j, M, name, parent, Y in zip(m1.joints, m1.jointPlacements, m1.names, m1.parents, m1.inertias):
            if j.id < legMaxId:
                jid = m2.addJoint(parent, getattr(pin, j.shortname())(), M, name)
                idx_q, idx_v = m2.joints[jid].idx_q, m2.joints[jid].idx_v
                m2.upperPositionLimit[idx_q:idx_q + j.nq] = m1.upperPositionLimit[j.idx_q:j.idx_q + j.nq]
                m2.lowerPositionLimit[idx_q:idx_q + j.nq] = m1.lowerPositionLimit[j.idx_q:j.idx_q + j.nq]
                m2.velocityLimit[idx_v:idx_v + j.nv] = m1.velocityLimit[j.idx_v:j.idx_v + j.nv]
                m2.effortLimit[idx_v:idx_v + j.nv] = m1.effortLimit[j.idx_v:j.idx_v + j.nv]
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
            if f.parent < legMaxId:
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
        self.robot.q0 = readParamsFromSrdf(self.robot.model, self.srdf_path, self.verbose, self.has_rotor_parameters,
                                           self.ref_posture)

        assert (m2.armature[:6] == 0.).all()
        # Add the free-flyer joint limits to the new model
        self.addFreeFlyerJointLimits()


def loadTalos(legs=False, arm=False, full=False, box=False):
    if legs:
        warnings.warn(_depr_msg('loadTalos(legs)', 'talos_legs'), FutureWarning, 2)
        loader = TalosLegsLoader
    elif arm:
        warnings.warn(_depr_msg('loadTalos(arm)', 'talos_arm'), FutureWarning, 2)
        loader = TalosArmLoader
    elif full:
        if box:
            warnings.warn(_depr_msg('loadTalos(full, box)', 'talos_full_box'), FutureWarning, 2)
            loader = TalosFullBoxLoader
        else:
            warnings.warn(_depr_msg('loadTalos(full)', 'talos_full'), FutureWarning, 2)
            loader = TalosFullLoader
    else:
        if box:
            warnings.warn(_depr_msg('loadTalos(box)', 'talos_box'), FutureWarning, 2)
            loader = TalosBoxLoader
        else:
            warnings.warn(_depr_msg('loadTalos()', 'talos'), FutureWarning, 2)
            loader = TalosLoader
    return loader().robot


def loadTalosLegs():
    warnings.warn(_depr_msg('loadTalosLegs()', 'talos_legs'), FutureWarning, 2)
    return loadTalos(legs=True)


def loadTalosArm():
    warnings.warn(_depr_msg('loadTalosArm()', 'talos_arm'), FutureWarning, 2)
    return loadTalos(arm=True)


class HyQLoader(RobotLoader):
    path = "hyq_description"
    urdf_filename = "hyq_no_sensors.urdf"
    srdf_filename = "hyq.srdf"
    ref_posture = "standing"
    free_flyer = True


def loadHyQ():
    warnings.warn(_depr_msg('loadHyQ()', 'hyq'), FutureWarning, 2)
    return HyQLoader().robot


class BoltLoader(RobotLoader):
    path = 'bolt_description'
    urdf_filename = "bolt.urdf"
    srdf_filename = "bolt.srdf"
    ref_posture = "standing"
    free_flyer = True


class Solo8Loader(RobotLoader):
    path = 'solo_description'
    urdf_filename = "solo.urdf"
    srdf_filename = "solo.srdf"
    ref_posture = "standing"
    free_flyer = True


class SoloLoader(Solo8Loader):
    def __init__(self, *args, **kwargs):
        warnings.warn('"solo" is deprecated, please try to load "solo8"')
        return super(SoloLoader, self).__init__(*args, **kwargs)


class Solo12Loader(Solo8Loader):
    urdf_filename = "solo12.urdf"


def loadSolo(solo=True):
    warnings.warn(_depr_msg('loadSolo()', 'solo8'), FutureWarning, 2)
    loader = Solo8Loader if solo else Solo12Loader
    return loader().robot


class FingerEduLoader(RobotLoader):
    path = 'finger_edu_description'
    urdf_filename = "finger_edu.urdf"
    srdf_filename = "finger_edu.srdf"
    ref_posture = "hanging"
    free_flyer = False


class KinovaLoader(RobotLoader):
    path = "kinova_description"
    urdf_filename = "kinova.urdf"
    srdf_filename = "kinova.srdf"
    ref_posture = "arm_up"


def loadKinova():
    warnings.warn(_depr_msg('loadKinova()', 'kinova'), FutureWarning, 2)
    return KinovaLoader().robot


class TiagoLoader(RobotLoader):
    path = "tiago_description"
    urdf_filename = "tiago.urdf"


class TiagoDualLoader(TiagoLoader):
    urdf_filename = "tiago_dual.urdf"


class TiagoNoHandLoader(TiagoLoader):
    urdf_filename = "tiago_no_hand.urdf"


def loadTiago(hand=True):
    if hand:
        warnings.warn(_depr_msg('loadTiago()', 'tiago'), FutureWarning, 2)
        loader = TiagoLoader
    else:
        warnings.warn(_depr_msg('loadTiago(hand=False)', 'tiago_no_hand'), FutureWarning, 2)
        loader = TiagoNoHandLoader
    return loader().robot


def loadTiagoNoHand():
    warnings.warn(_depr_msg('loadTiagoNoHand()', 'tiago_no_hand'), FutureWarning, 2)
    return loadTiago(hand=False)


class ICubLoader(RobotLoader):
    path = "icub_description"
    urdf_filename = "icub.urdf"
    srdf_filename = "icub.srdf"
    free_flyer = True


class ICubReducedLoader(ICubLoader):
    urdf_filename = "icub_reduced.urdf"


def loadICub(reduced=True):
    if reduced:
        warnings.warn(_depr_msg('loadICub()', 'icub_reduced'), FutureWarning, 2)
        loader = ICubReducedLoader
    else:
        warnings.warn(_depr_msg('loadICub(reduced=False)', 'icub'), FutureWarning, 2)
        loader = ICubLoader
    return loader().robot


class PandaLoader(RobotLoader):
    path = "panda_description"
    urdf_filename = "panda.urdf"
    urdf_subpath = "urdf"


def loadPanda():
    warnings.warn(_depr_msg('loadPanda()', 'panda'), FutureWarning, 2)
    return PandaLoader().robot


class UR3Loader(RobotLoader):
    path = "ur_description"
    urdf_filename = "ur3_robot.urdf"
    urdf_subpath = "urdf"
    ref_posture = None


class UR3GripperLoader(UR3Loader):
    urdf_filename = "ur3_gripper.urdf"
    srdf_filename = "ur3_gripper.srdf"


class UR3LimitedLoader(UR3Loader):
    urdf_filename = "ur3_joint_limited_robot.urdf"


class UR5Loader(UR3Loader):
    urdf_filename = "ur5_robot.urdf"
    srdf_filename = "ur5.srdf"


class UR5GripperLoader(UR5Loader):
    urdf_filename = "ur5_gripper.urdf"
    srdf_filename = "ur5_gripper.srdf"


class UR5LimitedLoader(UR5Loader):
    urdf_filename = "ur5_joint_limited_robot.urdf"


class UR10Loader(UR3Loader):
    urdf_filename = "ur10_robot.urdf"


class UR10LimitedLoader(UR10Loader):
    urdf_filename = "ur10_joint_limited_robot.urdf"


def loadUR(robot=5, limited=False, gripper=False):
    if robot == 3:
        if limited:
            warnings.warn(_depr_msg('loadUr(3, limited)', 'ur3_limited'), FutureWarning, 2)
            loader = UR3LimitedLoader
        elif gripper:
            warnings.warn(_depr_msg('loadUr(3, gripper)', 'ur3_gripper'), FutureWarning, 2)
            loader = UR3GripperLoader
        else:
            warnings.warn(_depr_msg('loadUr(3)', 'ur3'), FutureWarning, 2)
            loader = UR3Loader
    elif robot == 5:
        if limited:
            warnings.warn(_depr_msg('loadUr(limited)', 'ur5_limited'), FutureWarning, 2)
            loader = UR5LimitedLoader
        elif gripper:
            warnings.warn(_depr_msg('loadUr(gripper)', 'ur5_gripper'), FutureWarning, 2)
            loader = UR5GripperLoader
        else:
            warnings.warn(_depr_msg('loadUr()', 'ur5'), FutureWarning, 2)
            loader = UR5Loader
    elif robot == 10:
        if limited:
            warnings.warn(_depr_msg('loadUr(10, limited)', 'ur10_limited'), FutureWarning, 2)
            loader = UR10LimitedLoader
        else:
            warnings.warn(_depr_msg('loadUr(10)', 'ur10'), FutureWarning, 2)
            loader = UR10Loader
    return loader().robot


class HectorLoader(RobotLoader):
    path = "hector_description"
    urdf_filename = "quadrotor_base.urdf"
    free_flyer = True


def loadHector():
    warnings.warn(_depr_msg('loadHector()', 'hector'), FutureWarning, 2)
    return HectorLoader().robot


class DoublePendulumLoader(RobotLoader):
    path = "double_pendulum_description"
    urdf_filename = "double_pendulum.urdf"
    urdf_subpath = "urdf"


def loadDoublePendulum():
    warnings.warn(_depr_msg('loadDoublePendulum()', 'double_pendulum'), FutureWarning, 2)
    return DoublePendulumLoader().robot


class RomeoLoader(RobotLoader):
    path = "romeo_description"
    urdf_filename = "romeo.urdf"
    urdf_subpath = "urdf"
    free_flyer = True


def loadRomeo():
    warnings.warn(_depr_msg('loadRomeo()', 'romeo'), FutureWarning, 2)
    return RomeoLoader().robot


class SimpleHumanoidLoader(RobotLoader):
    path = 'simple_humanoid_description'
    urdf_subpath = 'urdf'
    urdf_filename = 'simple_humanoid.urdf'
    srdf_filename = 'simple_humanoid.srdf'
    free_flyer = True


class SimpleHumanoidClassicalLoader(SimpleHumanoidLoader):
    urdf_filename = 'simple_humanoid_classical.urdf'
    srdf_filename = 'simple_humanoid_classical.srdf'


class IrisLoader(RobotLoader):
    path = "iris_description"
    urdf_filename = "iris_simple.urdf"
    free_flyer = True


def loadIris():
    warnings.warn(_depr_msg('loadIris()', 'iris'), FutureWarning, 2)
    return IrisLoader().robot


ROBOTS = {
    'a1': A1Loader,
    'anymal': ANYmalLoader,
    'anymal_kinova': ANYmalKinovaLoader,
    'baxter': BaxterLoader,
    'double_pendulum': DoublePendulumLoader,
    'hector': HectorLoader,
    'hyq': HyQLoader,
    'icub': ICubLoader,
    'icub_reduced': ICubReducedLoader,
    'iris': IrisLoader,
    'kinova': KinovaLoader,
    'panda': PandaLoader,
    'romeo': RomeoLoader,
    'simple_humanoid': SimpleHumanoidLoader,
    'simple_humanoid_classical': SimpleHumanoidClassicalLoader,
    'bolt': BoltLoader,
    'solo': SoloLoader,
    'solo8': Solo8Loader,
    'solo12': Solo12Loader,
    'finger_edu': FingerEduLoader,
    'talos': TalosLoader,
    'talos_box': TalosBoxLoader,
    'talos_arm': TalosArmLoader,
    'talos_legs': TalosLegsLoader,
    'talos_full': TalosFullLoader,
    'talos_full_box': TalosFullBoxLoader,
    'tiago': TiagoLoader,
    'tiago_dual': TiagoDualLoader,
    'tiago_no_hand': TiagoNoHandLoader,
    'ur3': UR5Loader,
    'ur3_gripper': UR3GripperLoader,
    'ur3_limited': UR3LimitedLoader,
    'ur5': UR5Loader,
    'ur5_gripper': UR5GripperLoader,
    'ur5_limited': UR5LimitedLoader,
    'ur10': UR10Loader,
    'ur10_limited': UR10LimitedLoader,
}


def loader(name, display=False, rootNodeName=''):
    """Load a robot by its name, and optionnaly display it in a viewer."""
    if name not in ROBOTS:
        robots = ", ".join(sorted(ROBOTS.keys()))
        raise ValueError("Robot '%s' not found. Possible values are %s" % (name, robots))
    inst = ROBOTS[name]()
    if display:
        if rootNodeName:
            inst.robot.initViewer()
            inst.robot.viz.loadViewerModel(rootNodeName=rootNodeName)
        else:
            inst.robot.initViewer(loadModel=True)
        inst.robot.display(inst.robot.q0)
    return inst


def load(name, display=False, rootNodeName=''):
    """Load a robot by its name, and optionnaly display it in a viewer."""
    return loader(name, display, rootNodeName).robot


def load_full(name, display=False, rootNodeName=''):
    """Load a robot by its name, optionnaly display it in a viewer, and provide its q0 and paths."""
    inst = loader(name, display, rootNodeName)
    return inst.robot, inst.robot.q0, inst.df_path, inst.srdf_path
