import os
import typing
import xml.etree.ElementTree as ET
from types import MethodType

import numpy as np
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


def _robot_get_constraints(self, fallback_infer=True):
    srdf_path = getattr(self, "srdf_path", None)
    if srdf_path is None:
        urdf_path = getattr(self, "urdf", None)
        if urdf_path is not None:
            base_dir = os.path.dirname(urdf_path)
            srdf_candidate = os.path.join(
                os.path.dirname(base_dir),
                "srdf",
                os.path.splitext(os.path.basename(urdf_path))[0] + ".srdf",
            )
            if os.path.exists(srdf_candidate):
                srdf_path = srdf_candidate
                self.srdf_path = srdf_path
    if srdf_path is None:
        return []
    return constraints_from_srdf(
        self.model,
        srdf_path,
        fallback_infer=fallback_infer,
        q=self.q0,
    )


class LoopConstraintDescription:
    def __init__(
        self,
        name,
        joint1_id,
        joint1_placement,
        joint2_id,
        joint2_placement,
        mask,
        frame1=None,
        frame2=None,
    ):
        if len(mask) != 6:
            raise ValueError(
                f"Constraint mask must contain 6 values, got {len(mask)}: {mask}"
            )
        self.name = name
        self.joint1_id = joint1_id
        self.joint1_placement = joint1_placement
        self.joint2_id = joint2_id
        self.joint2_placement = joint2_placement
        self.mask = [bool(v) for v in mask]
        self.frame1 = frame1
        self.frame2 = frame2
        self.nc = self.size()

    def size(self):
        return sum(1 for v in self.mask if v)


def _legacy_mask(mask_key):
    if mask_key == "3d":
        return [True, False, True, False, False, False]
    if mask_key == "6d":
        return [True, False, True, False, True, False]
    raise ValueError(f"Unsupported legacy constraint type '{mask_key}'.")


def _parse_constraint_mask(mask_txt):
    axis_map = {
        "x": 0,
        "tx": 0,
        "y": 1,
        "ty": 1,
        "z": 2,
        "tz": 2,
        "roll": 3,
        "rx": 3,
        "r": 3,
        "pitch": 4,
        "ry": 4,
        "p": 4,
        "yaw": 5,
        "rz": 5,
    }

    tokens = [
        token.strip().lower()
        for token in str(mask_txt).replace(",", " ").split()
        if token.strip()
    ]
    if len(tokens) == 6 and all(
        token in {"0", "1", "false", "true"} for token in tokens
    ):
        return [token in {"1", "true"} for token in tokens]

    mask = [False] * 6
    for token in tokens:
        if token not in axis_map:
            raise ValueError(
                f"Unsupported mask token '{token}'. "
                "Use x y z roll pitch yaw, tx ty tz rx ry rz, "
                "or a 6-entry boolean mask."
            )
        mask[axis_map[token]] = True
    if not any(mask):
        raise ValueError("Constraint mask cannot be empty.")
    return mask


def _frame_placement_in_parent(model, data, q, frame_id):
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)

    joint_id = model.frames[frame_id].parentJoint
    parent_id = model.parents[joint_id]
    return data.oMi[parent_id].inverse() * data.oMf[frame_id]


def _frame_motion_derivatives(model, q, frame_id, eps=1e-7, probe=0.5):
    """Numerically differentiate a frame with respect to its supporting joint."""
    joint_id = model.frames[frame_id].parentJoint
    joint = model.joints[joint_id]
    derivatives = []
    data = model.createData()

    for velocity_id in range(joint.idx_v, joint.idx_v + joint.nv):
        tangent = np.zeros(model.nv)
        tangent[velocity_id] = eps

        # The second center avoids missing a coordinate at a singular sample,
        # e.g. the z derivative of a planar revolute joint at angle zero.
        probe_tangent = np.zeros(model.nv)
        probe_tangent[velocity_id] = probe
        centers = (q, pin.integrate(model, q, probe_tangent))
        for center in centers:
            minus = _frame_placement_in_parent(
                model, data, pin.integrate(model, center, -tangent), frame_id
            )
            plus = _frame_placement_in_parent(
                model, data, pin.integrate(model, center, tangent), frame_id
            )
            linear = (plus.translation - minus.translation) / (2.0 * eps)
            angular = pin.log3(plus.rotation @ minus.rotation.T) / (2.0 * eps)
            derivatives.append(np.concatenate((linear, angular)))

    if not derivatives:
        return np.empty((6, 0))
    return np.column_stack(derivatives)


def _numerical_constraint_mask(model, q, frame1_id, frame2_id):
    derivatives = np.column_stack(
        (
            _frame_motion_derivatives(model, q, frame1_id),
            _frame_motion_derivatives(model, q, frame2_id),
        )
    )
    if derivatives.shape[1] == 0:
        raise ValueError(
            "Cannot infer a constraint mask from frames whose parent joints "
            "have no degrees of freedom. Specify the mask explicitly."
        )

    magnitudes = np.max(np.abs(derivatives), axis=1)
    mask = np.zeros(6, dtype=bool)
    for rows in (slice(0, 3), slice(3, 6)):
        scale = np.max(magnitudes[rows])
        if scale > 1e-9:
            mask[rows] = magnitudes[rows] > max(1e-9, 1e-6 * scale)

    if not np.any(mask):
        raise ValueError(
            "The numerical frame derivatives are zero; specify the constraint "
            "mask explicitly."
        )
    return mask.tolist()


def _mask_from_loop_tag(tag, model, q, frame1_id, frame2_id):
    mask_txt = tag.attrib.get("mask")
    if mask_txt is not None:
        return _parse_constraint_mask(mask_txt)

    legacy_type = tag.attrib.get("type")
    if legacy_type is not None:
        return _legacy_mask(legacy_type.lower())

    return _numerical_constraint_mask(model, q, frame1_id, frame2_id)


def constraints_from_srdf(model, srdf_path, fallback_infer=True, q=None):
    if q is None:
        q = pin.neutral(model)
    root = ET.parse(srdf_path).getroot()
    constraints = []

    for tag in root.findall(".//loop_constraint"):
        frame1 = tag.attrib["frame1"]
        frame2 = tag.attrib["frame2"]
        id1 = model.getFrameId(frame1)
        id2 = model.getFrameId(frame2)
        constraints.append(
            LoopConstraintDescription(
                name=f"{frame1}C{frame2}",
                joint1_id=model.frames[id1].parentJoint,
                joint1_placement=model.frames[id1].placement,
                joint2_id=model.frames[id2].parentJoint,
                joint2_placement=model.frames[id2].placement,
                mask=_mask_from_loop_tag(tag, model, q, id1, id2),
                frame1=frame1,
                frame2=frame2,
            )
        )

    if constraints or not fallback_infer:
        return constraints

    names = [fr.name for fr in model.frames]
    groups = {}
    for name in names:
        lname = name.lower()
        if "closedloop" not in lname:
            continue
        if name.endswith("A"):
            groups.setdefault(name[:-1], {})["A"] = name
        elif name.endswith("B"):
            groups.setdefault(name[:-1], {})["B"] = name

    for _, sides in groups.items():
        if "A" not in sides or "B" not in sides:
            continue
        frame1, frame2 = sides["B"], sides["A"]
        id1 = model.getFrameId(frame1)
        id2 = model.getFrameId(frame2)
        constraints.append(
            LoopConstraintDescription(
                name=f"{frame1}C{frame2}",
                joint1_id=model.frames[id1].parentJoint,
                joint1_placement=model.frames[id1].placement,
                joint2_id=model.frames[id2].parentJoint,
                joint2_placement=model.frames[id2].placement,
                mask=_numerical_constraint_mask(model, q, id1, id2),
                frame1=frame1,
                frame2=frame2,
            )
        )

    return constraints


class CentauroLoader(RobotLoader):
    path = "centauro_description"
    urdf_filename = "centauro.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "centauro.srdf"
    ref_posture = "homing_balanced"
    free_flyer = True


class B1Loader(RobotLoader):
    path = "b1_description"
    urdf_filename = "b1.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "b1.srdf"
    ref_posture = "standing"
    free_flyer = True


class B1ClosedLoopLoader(RobotLoader):
    path = "b1_closed_loop_description"
    urdf_filename = "b1_closed_loop.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "b1_closed_loop.srdf"
    ref_posture = "standing"
    free_flyer = True


class B1LegLoader(RobotLoader):
    path = "b1_leg_FL"
    urdf_filename = "b1.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "b1.srdf"
    ref_posture = "standing"
    free_flyer = False


class B1Leg3DLoader(RobotLoader):
    path = "b1_leg_FL"
    urdf_filename = "b1_3d.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "b1_3d.srdf"
    ref_posture = "standing"
    free_flyer = False


class B1Leg6DLoader(RobotLoader):
    path = "b1_leg_FL"
    urdf_filename = "b1_6d.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "b1_6d.srdf"
    ref_posture = "standing"
    free_flyer = False


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


class KangarooLoader(RobotLoader):
    path = "kangaroo_description"
    urdf_filename = "kangaroo.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "kangaroo.srdf"
    ref_posture = "standing"
    free_flyer = True


class KangarooLegsKinLoader(RobotLoader):
    path = "kangaroo_description"
    urdf_filename = "kangaroo_loop.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "kangaroo_loop.srdf"
    ref_posture = "standing"
    free_flyer = True


class KangarooKneeKinLoader(RobotLoader):
    path = "kangaroo_description"
    urdf_filename = "kangaroo_knee_kin.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "kangaroo_knee_kin.srdf"
    ref_posture = "standing"
    free_flyer = True


class KangarooKnee1AnkleKinLoader(RobotLoader):
    path = "kangaroo_description"
    urdf_filename = "kangaroo_knee_1ankle_kin.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "kangaroo_knee_1ankle_kin.srdf"
    ref_posture = "standing"
    free_flyer = True


class KangarooKnee2AnkleKinLoader(RobotLoader):
    path = "kangaroo_description"
    urdf_filename = "kangaroo_knee_2ankle_kin.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "kangaroo_knee_2ankle_kin.srdf"
    ref_posture = "standing"
    free_flyer = True


class A1Loader(RobotLoader):
    path = "a1_description"
    urdf_filename = "a1.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "a1.srdf"
    ref_posture = "standing"
    free_flyer = True


class G1Loader(RobotLoader):
    path = "g1_description"
    urdf_subpath = "urdf"
    urdf_filename = "g1_29dof_rev_1_0.urdf"
    ref_posture = "standing"
    free_flyer = True


class G1WithHandsLoader(G1Loader):
    path = "g1_description"
    urdf_subpath = "urdf"
    urdf_filename = "g1_29dof_with_hand_rev_1_0.urdf"
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


class AlexanderLoader(RobotLoader):
    path = "alexander_description"
    urdf_filename = "alexander_v1.lowerBodyOnly.urdf"
    urdf_subpath = "urdf"
    free_flyer = True
    ref_posture = "default"
    srdf_filename = "alexander.srdf"


class AlexNubHandsLoader(RobotLoader):
    path = "alex_description"
    urdf_filename = "alex_nub_hands.urdf"
    urdf_subpath = "urdf"
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


class SO100Loader(RobotLoader):
    path = "so_arm_description"
    urdf_filename = "so100.urdf"
    urdf_subpath = "urdf"
    free_flyer = True


class SO101Loader(RobotLoader):
    path = "so_arm_description"
    urdf_filename = "so101.urdf"
    urdf_subpath = "urdf"
    free_flyer = True


class PR2Loader(RobotLoader):
    path = "pr2_description"
    urdf_filename = "pr2.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "pr2.srdf"
    free_flyer = True
    ref_posture = "tuck_left_arm"


class xArm7Loader(RobotLoader):
    path = "xarm_description"
    urdf_subpath = "urdf"
    urdf_filename = "xarm7.urdf"
    srdf_filename = "xarm7.srdf"
    ref_posture = "home"


ROBOTS = {
    "kangaroo": KangarooLoader,
    "kangaroo_legs_kin": KangarooLegsKinLoader,
    "kangaroo_knee_kin": KangarooKneeKinLoader,
    "kangaroo_knee_1ankle_kin": KangarooKnee1AnkleKinLoader,
    "kangaroo_knee_2ankle_kin": KangarooKnee2AnkleKinLoader,
    "centauro": CentauroLoader,
    "b1": B1Loader,
    "b1_closed_loop": B1ClosedLoopLoader,
    "b1_leg": B1LegLoader,
    "b1_leg_3D": B1Leg3DLoader,
    "b1_leg_6D": B1Leg6DLoader,
    "bravo7_gripper": Bravo7GripperLoader,
    "bravo7_no_ee": Bravo7NoEndEffectorLoader,
    "falcon_bravo7_no_ee": FalconBravo7NoEndEffectorLoader,
    "falcon_bravo7_gripper": FalconBravo7GripperLoader,
    "bluevolta_bravo7_no_ee": BluevoltaBravo7NoEndEffectorLoader,
    "bluevolta_bravo7_gripper": BluevoltaBravo7GripperLoader,
    "go1": Go1Loader,
    "go2": Go2Loader,
    "a1": A1Loader,
    "g1": G1Loader,
    "g1_with_hands": G1WithHandsLoader,
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
    "alexander": AlexanderLoader,
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
    "so100": SO100Loader,
    "so101": SO101Loader,
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
    "xarm7": xArm7Loader,
}


def loader(name, display=False, rootNodeName="", verbose=False):
    """Load a robot by its name, and optionally display it in a viewer."""
    if name not in ROBOTS:
        robots = ", ".join(sorted(ROBOTS.keys()))
        raise ValueError(f"Robot '{name}' not found. Possible values are {robots}")
    inst = ROBOTS[name](verbose=verbose)
    inst.robot.get_constraints = MethodType(_robot_get_constraints, inst.robot)
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
