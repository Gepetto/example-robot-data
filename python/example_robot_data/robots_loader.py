import typing

import pinocchio as pin

try:
    from .path import (  # noqa: F401
        EXAMPLE_ROBOT_DATA_MODEL_DIR,
        EXAMPLE_ROBOT_DATA_SOURCE_DIR,
    )
except ImportError:
    pass

from .human import HumanLoader
from .panda import PandaLoader, PandaLoaderCollision
from .talos import (
    TalosArmLoader,
    TalosBoxLoader,
    TalosFullBoxLoader,
    TalosFullLoader,
    TalosLegsLoader,
    TalosLoader,
)
from .utils import RobotLoader, getModelPath, readParamsFromSrdf  # noqa: F401


class B1Loader(RobotLoader):
    path = "b1_description"
    urdf_filename = "b1.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "b1.srdf"
    ref_posture = "standing"
    free_flyer = True


class Go1Loader(RobotLoader):
    path = "go1_description"
    urdf_filename = "go1.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "go1.srdf"
    ref_posture = "standing"
    free_flyer = True


class FalconBravo7NoEndEffectorLoader(RobotLoader):
    path = "falcon_description"
    urdf_filename = "falcon_bravo7_no_ee.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "falcon_bravo7_no_ee.srdf"
    ref_posture = "standing"
    free_flyer = True


class BluevoltaBravo7NoEndEffectorLoader(RobotLoader):
    path = "bluevolta_description"
    urdf_filename = "bluevolta_bravo7_no_ee.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "bluevolta_bravo7_no_ee.srdf"
    ref_posture = "standing"
    free_flyer = True


class FalconBravo7GripperLoader(RobotLoader):
    path = "falcon_description"
    urdf_filename = "falcon_bravo7_gripper.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "falcon_bravo7_gripper.srdf"
    ref_posture = "standing"
    free_flyer = True


class BluevoltaBravo7GripperLoader(RobotLoader):
    path = "bluevolta_description"
    urdf_filename = "bluevolta_bravo7_gripper.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "bluevolta_bravo7_gripper.srdf"
    ref_posture = "standing"
    free_flyer = True


class Bravo7NoEndEffectorLoader(RobotLoader):
    path = "bravo7_description"
    urdf_filename = "bravo7_no_ee.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "bravo7_no_ee.srdf"
    ref_posture = "standing"
    free_flyer = False


class Bravo7GripperLoader(RobotLoader):
    path = "bravo7_description"
    urdf_filename = "bravo7_gripper.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "bravo7_gripper.srdf"
    ref_posture = "standing"
    free_flyer = False


class Go2Loader(RobotLoader):
    path = "go2_description"
    urdf_filename = "go2.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "go2.srdf"
    ref_posture = "standing"
    free_flyer = True


class A1Loader(RobotLoader):
    path = "a1_description"
    urdf_filename = "a1.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "a1.srdf"
    ref_posture = "standing"
    free_flyer = True


class Z1Loader(RobotLoader):
    path = "z1_description"
    urdf_filename = "z1.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "z1.srdf"
    ref_posture = "arm_up"


class B1Z1Loader(B1Loader):
    urdf_filename = "b1-z1.urdf"
    srdf_filename = "b1-z1.srdf"
    ref_posture = "standing_with_arm_home"


class ANYmalLoader(RobotLoader):
    path = "anymal_b_simple_description"
    urdf_filename = "anymal.urdf"
    srdf_filename = "anymal.srdf"
    ref_posture = "standing"
    free_flyer = True


class ANYmalCLoader(RobotLoader):
    path = "anymal_c_simple_description"
    urdf_subpath = "urdf"
    urdf_filename = "anymal.urdf"
    srdf_filename = "anymal.srdf"
    ref_posture = "standing"
    free_flyer = True


class LaikagoLoader(RobotLoader):
    path = "laikago_description"
    urdf_subpath = "urdf"
    urdf_filename = "laikago.urdf"
    free_flyer = True


class ANYmalKinovaLoader(ANYmalLoader):
    urdf_filename = "anymal-kinova.urdf"
    srdf_filename = "anymal-kinova.srdf"
    ref_posture = "standing_with_arm_up"


class BaxterLoader(RobotLoader):
    path = "baxter_description"
    urdf_filename = "baxter.urdf"
    srdf_filename = "baxter_manipulation.srdf"
    urdf_subpath = "urdf"
    srdf_subpath = "srdf"
    ref_posture = "neutral"


class CassieLoader(RobotLoader):
    path = "cassie_description"
    if tuple(int(i) for i in pin.__version__.split(".")) > (2, 9, 1):
        sdf_filename = "cassie.sdf"
    else:
        sdf_filename = "cassie_v2.sdf"
    sdf_subpath = "robots"
    srdf_filename = "cassie_v2.srdf"
    ref_posture = "standing"
    free_flyer = True
    sdf_root_link_name = "pelvis"
    sdf_parent_guidance: typing.ClassVar = [
        "left-roll-op",
        "left-yaw-op",
        "left-pitch-op",
        "left-knee-op",
        "left-tarsus-spring-joint",
        "left-foot-op",
        "right-roll-op",
        "right-yaw-op",
        "right-pitch-op",
        "right-knee-op",
        "right-tarsus-spring-joint",
        "right-foot-op",
    ]


class AsrTwoDofLoader(RobotLoader):
    path = "asr_twodof_description"
    urdf_filename = "TwoDofs.urdf"
    urdf_subpath = "urdf"


class HyQLoader(RobotLoader):
    path = "hyq_description"
    urdf_filename = "hyq_no_sensors.urdf"
    srdf_filename = "hyq.srdf"
    ref_posture = "standing"
    free_flyer = True


class BoltLoader(RobotLoader):
    path = "bolt_description"
    urdf_filename = "bolt.urdf"
    srdf_filename = "bolt.srdf"
    ref_posture = "standing"
    free_flyer = True


class BorinotLoader(RobotLoader):
    path = "borinot_description"
    urdf_subpath = "urdf"
    srdf_subpath = "srdf"
    urdf_filename = "borinot_flying_arm_2.urdf"
    srdf_filename = "borinot_flying_arm_2.srdf"
    ref_posture = "home"
    free_flyer = True


class Solo8Loader(RobotLoader):
    path = "solo_description"
    urdf_filename = "solo.urdf"
    srdf_filename = "solo.srdf"
    ref_posture = "standing"
    free_flyer = True


class Solo12Loader(Solo8Loader):
    urdf_filename = "solo12.urdf"


class FingerEduLoader(RobotLoader):
    path = "finger_edu_description"
    urdf_filename = "finger_edu.urdf"
    srdf_filename = "finger_edu.srdf"
    ref_posture = "hanging"
    free_flyer = False


class KinovaLoader(RobotLoader):
    path = "kinova_description"
    urdf_filename = "kinova.urdf"
    srdf_filename = "kinova.srdf"
    ref_posture = "arm_up"


class TiagoLoader(RobotLoader):
    path = "tiago_description"
    urdf_filename = "tiago.urdf"


class TiagoDualLoader(TiagoLoader):
    urdf_filename = "tiago_dual.urdf"


class TiagoNoHandLoader(TiagoLoader):
    urdf_filename = "tiago_no_hand.urdf"


class TiagoProLoader(RobotLoader):
    path = "tiago_pro_description"
    urdf_filename = "tiago_pro.urdf"


class ICubLoader(RobotLoader):
    path = "icub_description"
    urdf_filename = "icub.urdf"
    srdf_filename = "icub.srdf"
    free_flyer = True


class ICubReducedLoader(ICubLoader):
    urdf_filename = "icub_reduced.urdf"


class AlexNubHandsLoader(RobotLoader):
    path = "alex_description"
    urdf_filename = "alex_nub_hands.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "alex_nub_hands.srdf"
    ref_posture = "default"


class AlexPsyonicHandsLoader(RobotLoader):
    path = "alex_description"
    urdf_filename = "alex_psyonic_hands.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "alex_psyonic_hands.srdf"
    ref_posture = "default"


class AlexSakeHandsLoader(RobotLoader):
    path = "alex_description"
    urdf_filename = "alex_sake_hands.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "alex_sake_hands.srdf"
    ref_posture = "default"


class AllegroRightHandLoader(RobotLoader):
    path = "allegro_hand_description"
    urdf_filename = "allegro_right_hand.urdf"
    urdf_subpath = "urdf"


class AllegroLeftHandLoader(RobotLoader):
    path = "allegro_hand_description"
    urdf_filename = "allegro_left_hand.urdf"
    urdf_subpath = "urdf"


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


class HectorLoader(RobotLoader):
    path = "hector_description"
    urdf_filename = "quadrotor_base.urdf"
    free_flyer = True


class HextiltLoader(RobotLoader):
    path = "hextilt_description"
    urdf_subpath = "urdf"
    srdf_subpath = "srdf"
    urdf_filename = "hextilt_flying_arm_5.urdf"
    srdf_filename = "hextilt_flying_arm_5.srdf"
    ref_posture = "home"
    free_flyer = True


class DoublePendulumLoader(RobotLoader):
    path = "double_pendulum_description"
    urdf_filename = "double_pendulum.urdf"
    urdf_subpath = "urdf"


class DoublePendulumContinuousLoader(DoublePendulumLoader):
    urdf_filename = "double_pendulum_continuous.urdf"


class DoublePendulumSimpleLoader(DoublePendulumLoader):
    urdf_filename = "double_pendulum_simple.urdf"


class QuadrupedLoader(RobotLoader):
    path = "quadruped_description"
    urdf_subpath = "urdf"
    urdf_filename = "quadruped.urdf"
    free_flyer = True


class RomeoLoader(RobotLoader):
    path = "romeo_description"
    urdf_filename = "romeo.urdf"
    urdf_subpath = "urdf"
    free_flyer = True


class SimpleHumanoidLoader(RobotLoader):
    path = "simple_humanoid_description"
    urdf_subpath = "urdf"
    urdf_filename = "simple_humanoid.urdf"
    srdf_filename = "simple_humanoid.srdf"
    free_flyer = True


class SimpleHumanoidClassicalLoader(SimpleHumanoidLoader):
    urdf_filename = "simple_humanoid_classical.urdf"
    srdf_filename = "simple_humanoid_classical.srdf"


class IrisLoader(RobotLoader):
    path = "iris_description"
    urdf_filename = "iris_simple.urdf"
    free_flyer = True


class PR2Loader(RobotLoader):
    path = "pr2_description"
    urdf_filename = "pr2.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "pr2.srdf"
    free_flyer = True
    ref_posture = "tuck_left_arm"


ROBOTS = {
    "b1": B1Loader,
    "bravo7_gripper": Bravo7GripperLoader,
    "bravo7_no_ee": Bravo7NoEndEffectorLoader,
    "falcon_bravo7_no_ee": FalconBravo7NoEndEffectorLoader,
    "falcon_bravo7_gripper": FalconBravo7GripperLoader,
    "bluevolta_bravo7_no_ee": BluevoltaBravo7NoEndEffectorLoader,
    "bluevolta_bravo7_gripper": BluevoltaBravo7GripperLoader,
    "go1": Go1Loader,
    "go2": Go2Loader,
    "a1": A1Loader,
    "z1": Z1Loader,
    "b1_z1": B1Z1Loader,
    "anymal": ANYmalLoader,
    "anymal_c": ANYmalCLoader,
    "anymal_kinova": ANYmalKinovaLoader,
    "asr_twodof": AsrTwoDofLoader,
    "baxter": BaxterLoader,
    "cassie": CassieLoader,
    "double_pendulum": DoublePendulumLoader,
    "double_pendulum_continuous": DoublePendulumContinuousLoader,
    "double_pendulum_simple": DoublePendulumSimpleLoader,
    "hector": HectorLoader,
    "hextilt": HextiltLoader,
    "human": HumanLoader,
    "hyq": HyQLoader,
    "icub": ICubLoader,
    "icub_reduced": ICubReducedLoader,
    "iris": IrisLoader,
    "kinova": KinovaLoader,
    "laikago": LaikagoLoader,
    "panda": PandaLoader,
    "panda_collision": PandaLoaderCollision,
    "alex_nub_hands": AlexNubHandsLoader,
    "alex_psyonic_hands": AlexPsyonicHandsLoader,
    "alex_sake_hands": AlexSakeHandsLoader,
    "allegro_right_hand": AllegroRightHandLoader,
    "allegro_left_hand": AllegroLeftHandLoader,
    "quadruped": QuadrupedLoader,
    "romeo": RomeoLoader,
    "simple_humanoid": SimpleHumanoidLoader,
    "simple_humanoid_classical": SimpleHumanoidClassicalLoader,
    "bolt": BoltLoader,
    "borinot": BorinotLoader,
    "solo8": Solo8Loader,
    "solo12": Solo12Loader,
    "finger_edu": FingerEduLoader,
    "pr2": PR2Loader,
    "talos": TalosLoader,
    "talos_box": TalosBoxLoader,
    "talos_arm": TalosArmLoader,
    "talos_legs": TalosLegsLoader,
    "talos_full": TalosFullLoader,
    "talos_full_box": TalosFullBoxLoader,
    "tiago": TiagoLoader,
    "tiago_dual": TiagoDualLoader,
    "tiago_no_hand": TiagoNoHandLoader,
    "tiago_pro": TiagoProLoader,
    "ur3": UR5Loader,
    "ur3_gripper": UR3GripperLoader,
    "ur3_limited": UR3LimitedLoader,
    "ur5": UR5Loader,
    "ur5_gripper": UR5GripperLoader,
    "ur5_limited": UR5LimitedLoader,
    "ur10": UR10Loader,
    "ur10_limited": UR10LimitedLoader,
}


def loader(name, display=False, rootNodeName="", verbose=False):
    """Load a robot by its name, and optionally display it in a viewer."""
    if name not in ROBOTS:
        robots = ", ".join(sorted(ROBOTS.keys()))
        raise ValueError(f"Robot '{name}' not found. Possible values are {robots}")
    inst = ROBOTS[name](verbose=verbose)
    if display:
        if rootNodeName:
            inst.robot.initViewer()
            inst.robot.viz.loadViewerModel(rootNodeName=rootNodeName)
        else:
            inst.robot.initViewer(loadModel=True)
        inst.robot.display(inst.robot.q0)
    return inst


def load(name, display=False, rootNodeName="", verbose=False):
    """Load a robot by its name, and optionnaly display it in a viewer."""
    return loader(name, display, rootNodeName, verbose).robot


def load_full(name, display=False, rootNodeName="", verbose=False):
    """Load a robot by its name, optionnaly display it in a viewer,
    and provide its q0 and paths."""
    inst = loader(name, display, rootNodeName, verbose)
    return inst.robot, inst.robot.q0, inst.df_path, inst.srdf_path
