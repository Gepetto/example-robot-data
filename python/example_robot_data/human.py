import typing

import numpy as np
import pinocchio as pin

from .utils import RobotLoader


class HumanLoader(RobotLoader):
    path = "human_description"
    urdf_filename = "human.urdf"
    free_flyer = True
    ref_posture = "anatomical"
    # Enforced, unchangeable free-flyer orientation (90° about X, and Y↔Z swap)
    freeflyer_ori: np.ndarray = np.array(
        [
            [1, 0, 0],
            [0, 0, -1],
            [0, 1, 0],
        ]
    )

    def __init__(
        self,
        height: typing.Optional[float] = None,
        weight: typing.Optional[float] = None,
        gender: str = "m",
        verbose: bool = False,
    ):
        """
        Initialize the robot loader with optional scaling and configuration parameters. The scaling of the model is based on Dumas 2007, except for the
        abdomen and thorax segments, which are scaled based de Leva 1996.
        Parameters:
            height (float, optional): The height to scale the robot model to. If None, uses the default height in the urdf of 1.80m.
            weight (float, optional): The weight to scale the robot model to. If None, uses the default weight in the urdf 75kg.
            gender (str, optional): The gender specification for scaling, default is 'm'.
            verbose (bool, optional): If True, enables verbose output. Default is False.
        Behavior:
            - Calls the base loader's initializer.
            - If height or weight is provided, scales the robot model accordingly.
            - The free-flyer is present, it enforces its orientation and reapplies joint limits.
        """

        # call base loader
        super().__init__(verbose=verbose)

        if height is not None or weight is not None:
            self._scale_pin_model(
                self.robot.model, self.robot.visual_model, height, weight, gender
            )

        # automatically apply the enforced free-flyer orientation
        if self.free_flyer:
            # get joint index
            j_id = self.robot.model.getJointId("root_joint")
            # assign enforced rotation
            self.robot.model.jointPlacements[j_id].rotation = self.freeflyer_ori
            # re-apply limits
            self.addFreeFlyerJointLimits()

    @staticmethod
    def get_dict_inertial_param(
        height: float, weight: float, gender: str = "m"
    ) -> dict:
        inertial_segment_names = [
            "middle_pelvis",
            "left_upperleg",
            "left_lowerleg",
            "left_foot",
            "middle_abdomen",
            "middle_thorax",
            "middle_head",
            "left_upperarm",
            "left_lowerarm",
            "left_hand",
            "right_upperarm",
            "right_lowerarm",
            "right_hand",
            "right_upperleg",
            "right_lowerleg",
            "right_foot",
        ]

        sgt_lengths = {}

        dicts = []

        # Ratios from Dumas 2007 and De Leva 1996 for abdomen and thorax
        for name in inertial_segment_names:
            dict_sgmt = {}
            if name == "middle_pelvis":
                length = (0.0634 if gender == "f" else 0.0505) * height
                sgt_lengths["middle_pelvis_0"] = length
                sgt_lengths["middle_pelvis_0_width"] = (
                    0.1478 if gender == "f" else 0.1265
                ) * height
                dict_sgmt["mass"] = np.round(
                    (0.146 if gender == "f" else 0.142) * weight, 2
                )
                dict_sgmt["com"] = np.round(
                    (
                        np.array([-0.009, -0.232, 0.002])
                        if gender == "f"
                        else np.array([0.028, -0.28, -0.006])
                    )
                    * length,
                    3,
                )
                dict_sgmt["inertia"] = np.round(
                    (
                        np.array(
                            [
                                (length * 0.91) ** 2,
                                -((length * 0.34) ** 2),
                                -((length * 0.01) ** 2),
                                (length * 1) ** 2,
                                -((length * 0.01) ** 2),
                                (length * 0.79) ** 2,
                            ]
                        )
                        if gender == "f"
                        else np.array(
                            [
                                (length * 1.01) ** 2,
                                -((length * 0.25) ** 2),
                                -((length * 0.12) ** 2),
                                (length * 1.06) ** 2,
                                -((length * 0.08) ** 2),
                                (length * 0.95) ** 2,
                            ]
                        )
                    )
                    * dict_sgmt["mass"],
                    5,
                )
                dicts.append(dict_sgmt)
            elif name == "left_upperleg":
                length = (0.2354 if gender == "f" else 0.2441) * height
                sgt_lengths["left_upperleg_0"] = length
                dict_sgmt["mass"] = np.round(
                    (0.146 if gender == "f" else 0.123) * weight, 2
                )
                dict_sgmt["com"] = np.round(
                    (
                        np.array([-0.077, -0.377, 0.009])
                        if gender == "f"
                        else np.array([-0.041, -0.429, 0.033])
                    )
                    * length,
                    3,
                )
                dict_sgmt["inertia"] = np.round(
                    (
                        np.array(
                            [
                                (length * 0.31) ** 2,
                                (length * 0.07) ** 2,
                                -((length * 0.02) ** 2),
                                (length * 0.19) ** 2,
                                -((length * 0.07) ** 2),
                                (length * 0.32) ** 2,
                            ]
                        )
                        if gender == "f"
                        else np.array(
                            [
                                (length * 0.29) ** 2,
                                (length * 0.07) ** 2,
                                -((length * 0.02) ** 2),
                                (length * 0.15) ** 2,
                                -((length * 0.07) ** 2),
                                (length * 0.3) ** 2,
                            ]
                        )
                    )
                    * dict_sgmt["mass"],
                    5,
                )
                dicts.append(dict_sgmt)
            elif name == "left_lowerleg":
                length = (0.2410 if gender == "f" else 0.2446) * height
                sgt_lengths["left_lowerleg_0"] = length
                dict_sgmt["mass"] = np.round(
                    (0.045 if gender == "f" else 0.048) * weight, 2
                )
                dict_sgmt["com"] = np.round(
                    (
                        np.array([-0.049, -0.404, 0.031])
                        if gender == "f"
                        else np.array([-0.048, -0.41, 0.007])
                    )
                    * length,
                    3,
                )
                dict_sgmt["inertia"] = np.round(
                    (
                        np.array(
                            [
                                (length * 0.28) ** 2,
                                (length * 0.02) ** 2,
                                (length * 0.01) ** 2,
                                (length * 0.1) ** 2,
                                (length * 0.06) ** 2,
                                (length * 0.28) ** 2,
                            ]
                        )
                        if gender == "f"
                        else np.array(
                            [
                                (length * 0.28) ** 2,
                                (length * 0.04) ** 2,
                                (length * 0.02) ** 2,
                                (length * 0.1) ** 2,
                                (length * 0.05) ** 2,
                                (length * 0.28) ** 2,
                            ]
                        )
                    )
                    * dict_sgmt["mass"],
                    5,
                )
                dicts.append(dict_sgmt)
            elif name == "left_foot":
                length = (0.1447 if gender == "f" else 0.1497) * height
                sgt_lengths["left_foot_0"] = length
                dict_sgmt["mass"] = np.round(
                    (0.01 if gender == "f" else 0.012) * weight, 2
                )
                dict_sgmt["com"] = np.round(
                    (
                        np.array([0.27, -0.218, 0.039])
                        if gender == "f"
                        else np.array([0.382, -0.151, 0.026])
                    )
                    * length,
                    3,
                )
                dict_sgmt["inertia"] = np.round(
                    (
                        np.array(
                            [
                                (length * 0.17) ** 2,
                                -((length * 0.10) ** 2),
                                (length * 0.06) ** 2,
                                (length * 0.36) ** 2,
                                -((length * 0.04) ** 2),
                                (length * 0.35) ** 2,
                            ]
                        )
                        if gender == "f"
                        else np.array(
                            [
                                (length * 0.17) ** 2,
                                (length * 0.13) ** 2,
                                -((length * 0.08) ** 2),
                                (length * 0.37) ** 2,
                                (length * 0) ** 2,
                                (length * 0.36) ** 2,
                            ]
                        )
                    )
                    * dict_sgmt["mass"],
                    5,
                )
                dicts.append(dict_sgmt)
            elif name == "middle_abdomen":
                length = (0.1183 if gender == "f" else 0.1237) * height
                sgt_lengths["middle_abdomen_0"] = length
                dict_sgmt["mass"] = np.round(
                    (0.1465 if gender == "f" else 0.1633) * weight, 2
                )
                dict_sgmt["com"] = np.round(
                    (
                        np.array([-0.016, -0.4512, -0.006])
                        if gender == "f"
                        else np.array([-0.036, 0.4502, -0.002])
                    )
                    * length,
                    3,
                )
                dict_sgmt["inertia"] = np.round(
                    (
                        np.array(
                            [
                                (length * 0.29) ** 2,
                                (length * 0.22) ** 2,
                                (length * 0.05) ** 2,
                                (length * 0.27) ** 2,
                                -((length * 0.05) ** 2),
                                (length * 0.29) ** 2,
                            ]
                        )
                        if gender == "f"
                        else np.array(
                            [
                                (length * 0.27) ** 2,
                                (length * 0.18) ** 2,
                                -((length * 0.02) ** 2),
                                (length * 0.25) ** 2,
                                -((length * 0.04) ** 2),
                                (length * 0.28) ** 2,
                            ]
                        )
                    )
                    * dict_sgmt["mass"],
                    5,
                )
                dicts.append(dict_sgmt)
            elif name == "middle_thorax":
                length = (0.1314 if gender == "f" else 0.1390) * height
                sgt_lengths["middle_thorax_0"] = length
                dict_sgmt["mass"] = np.round(
                    (0.1545 if gender == "f" else 0.1596) * weight, 2
                )
                dict_sgmt["com"] = np.round(
                    (
                        np.array([-0.016, 0.5050, -0.006])
                        if gender == "f"
                        else np.array([-0.036, 0.5066, -0.002])
                    )
                    * length,
                    3,
                )
                dict_sgmt["inertia"] = np.round(
                    (
                        np.array(
                            [
                                (length * 0.29) ** 2,
                                (length * 0.22) ** 2,
                                (length * 0.05) ** 2,
                                (length * 0.27) ** 2,
                                -((length * 0.05) ** 2),
                                (length * 0.29) ** 2,
                            ]
                        )
                        if gender == "f"
                        else np.array(
                            [
                                (length * 0.27) ** 2,
                                (length * 0.18) ** 2,
                                -((length * 0.02) ** 2),
                                (length * 0.25) ** 2,
                                -((length * 0.04) ** 2),
                                (length * 0.28) ** 2,
                            ]
                        )
                    )
                    * dict_sgmt["mass"],
                    5,
                )
                dicts.append(dict_sgmt)
            elif name == "middle_head":
                length = (0.1308 if gender == "f" else 0.1310) * height
                sgt_lengths["middle_head_0"] = length
                dict_sgmt["mass"] = np.round(
                    0.067 * weight, 2
                )  # same for male and female
                dict_sgmt["com"] = np.round(
                    (
                        np.array([-0.07, 0.597, 0])
                        if gender == "f"
                        else np.array([-0.062, 0.555, 0.001])
                    )
                    * length,
                    3,
                )
                dict_sgmt["inertia"] = np.round(
                    (
                        np.array(
                            [
                                (length * 0.32) ** 2,
                                -((length * 0.06) ** 2),
                                (length * 0.01) ** 2,
                                (length * 0.27) ** 2,
                                -((length * 0.01) ** 2),
                                (length * 0.34) ** 2,
                            ]
                        )
                        if gender == "f"
                        else np.array(
                            [
                                (length * 0.31) ** 2,
                                -((length * 0.09) ** 2),
                                -((length * 0.02) ** 2),
                                (length * 0.25) ** 2,
                                (length * 0.03) ** 2,
                                (length * 0.33) ** 2,
                            ]
                        )
                    )
                    * dict_sgmt["mass"],
                    5,
                )
                dicts.append(dict_sgmt)
            elif name == "left_upperarm":
                length = (0.1510 if gender == "f" else 0.1531) * height
                sgt_lengths["left_upperarm_0"] = length
                dict_sgmt["mass"] = np.round(
                    (0.022 if gender == "f" else 0.024) * weight, 2
                )
                dict_sgmt["com"] = np.round(
                    (
                        np.array([-0.073, -0.454, -0.028])
                        if gender == "f"
                        else np.array([0.017, -0.452, -0.026])
                    )
                    * length,
                    3,
                )
                dict_sgmt["inertia"] = np.round(
                    (
                        np.array(
                            [
                                (length * 0.33) ** 2,
                                (length * 0.03) ** 2,
                                -((length * 0.05) ** 2),
                                (length * 0.17) ** 2,
                                -((length * 0.14) ** 2),
                                (length * 0.33) ** 2,
                            ]
                        )
                        if gender == "f"
                        else np.array(
                            [
                                (length * 0.31) ** 2,
                                (length * 0.06) ** 2,
                                (length * 0.05) ** 2,
                                (length * 0.14) ** 2,
                                (length * 0.02) ** 2,
                                (length * 0.32) ** 2,
                            ]
                        )
                    )
                    * dict_sgmt["mass"],
                    5,
                )
                dicts.append(dict_sgmt)
            elif name == "left_lowerarm":
                length = (0.1534 if gender == "f" else 0.1593) * height
                sgt_lengths["left_lowerarm_0"] = length
                dict_sgmt["mass"] = np.round(
                    (0.013 if gender == "f" else 0.017) * weight, 2
                )
                dict_sgmt["com"] = np.round(
                    (
                        np.array([0.021, -0.411, 0.019])
                        if gender == "f"
                        else np.array([0.01, -0.417, 0.014])
                    )
                    * length,
                    3,
                )
                dict_sgmt["inertia"] = np.round(
                    (
                        np.array(
                            [
                                (length * 0.26) ** 2,
                                (length * 0.1) ** 2,
                                (length * 0.04) ** 2,
                                (length * 0.14) ** 2,
                                -((length * 0.13) ** 2),
                                (length * 0.25) ** 2,
                            ]
                        )
                        if gender == "f"
                        else np.array(
                            [
                                (length * 0.28) ** 2,
                                (length * 0.03) ** 2,
                                (length * 0.02) ** 2,
                                (length * 0.11) ** 2,
                                -((length * 0.08) ** 2),
                                (length * 0.27) ** 2,
                            ]
                        )
                    )
                    * dict_sgmt["mass"],
                    5,
                )
                dicts.append(dict_sgmt)
            elif name == "left_hand":
                length = (0.0989 if gender == "f" else 0.1014) * height
                sgt_lengths["left_hand_0"] = length
                dict_sgmt["mass"] = np.round(
                    (0.005 if gender == "f" else 0.006) * weight, 2
                )
                dict_sgmt["com"] = np.round(
                    (
                        np.array([0.077, -0.768, 0.048])
                        if gender == "f"
                        else np.array([0.082, -0.839, 0.074])
                    )
                    * length,
                    3,
                )
                dict_sgmt["inertia"] = np.round(
                    (
                        np.array(
                            [
                                (length * 0.63) ** 2,
                                (length * 0.29) ** 2,
                                (length * 0.23) ** 2,
                                (length * 0.43) ** 2,
                                -((length * 0.28) ** 2),
                                (length * 0.58) ** 2,
                            ]
                        )
                        if gender == "f"
                        else np.array(
                            [
                                (length * 0.61) ** 2,
                                (length * 0.22) ** 2,
                                (length * 0.15) ** 2,
                                (length * 0.38) ** 2,
                                -((length * 0.2) ** 2),
                                (length * 0.56) ** 2,
                            ]
                        )
                    )
                    * dict_sgmt["mass"],
                    5,
                )
                dicts.append(dict_sgmt)
            elif name == "right_upperarm":
                length = (0.1510 if gender == "f" else 0.1531) * height
                sgt_lengths["right_upperarm_0"] = length
                dict_sgmt["mass"] = np.round(
                    (0.022 if gender == "f" else 0.024) * weight, 2
                )
                dict_sgmt["com"] = np.round(
                    (
                        np.array([-0.073, -0.454, -0.028])
                        if gender == "f"
                        else np.array([0.017, -0.452, -0.026])
                    )
                    * length,
                    3,
                )
                dict_sgmt["inertia"] = np.round(
                    (
                        np.array(
                            [
                                (length * 0.33) ** 2,
                                (length * 0.03) ** 2,
                                -((length * 0.05) ** 2),
                                (length * 0.17) ** 2,
                                -((length * 0.14) ** 2),
                                (length * 0.33) ** 2,
                            ]
                        )
                        if gender == "f"
                        else np.array(
                            [
                                (length * 0.31) ** 2,
                                (length * 0.06) ** 2,
                                (length * 0.05) ** 2,
                                (length * 0.14) ** 2,
                                (length * 0.02) ** 2,
                                (length * 0.32) ** 2,
                            ]
                        )
                    )
                    * dict_sgmt["mass"],
                    5,
                )
                dicts.append(dict_sgmt)
            elif name == "right_lowerarm":
                length = (0.1534 if gender == "f" else 0.1593) * height
                sgt_lengths["right_lowerarm_0"] = length
                dict_sgmt["mass"] = np.round(
                    (0.013 if gender == "f" else 0.017) * weight, 2
                )
                dict_sgmt["com"] = np.round(
                    (
                        np.array([0.021, -0.411, 0.019])
                        if gender == "f"
                        else np.array([0.01, -0.417, 0.014])
                    )
                    * length,
                    3,
                )
                dict_sgmt["inertia"] = np.round(
                    (
                        np.array(
                            [
                                (length * 0.26) ** 2,
                                (length * 0.1) ** 2,
                                (length * 0.04) ** 2,
                                (length * 0.14) ** 2,
                                -((length * 0.13) ** 2),
                                (length * 0.25) ** 2,
                            ]
                        )
                        if gender == "f"
                        else np.array(
                            [
                                (length * 0.28) ** 2,
                                (length * 0.03) ** 2,
                                (length * 0.02) ** 2,
                                (length * 0.11) ** 2,
                                -((length * 0.08) ** 2),
                                (length * 0.27) ** 2,
                            ]
                        )
                    )
                    * dict_sgmt["mass"],
                    5,
                )
                dicts.append(dict_sgmt)
            elif name == "right_hand":
                length = (0.0989 if gender == "f" else 0.1014) * height
                sgt_lengths["right_hand_0"] = length
                dict_sgmt["mass"] = np.round(
                    (0.005 if gender == "f" else 0.006) * weight, 2
                )
                dict_sgmt["com"] = np.round(
                    (
                        np.array([0.077, -0.768, 0.048])
                        if gender == "f"
                        else np.array([0.082, -0.839, 0.074])
                    )
                    * length,
                    3,
                )
                dict_sgmt["inertia"] = np.round(
                    (
                        np.array(
                            [
                                (length * 0.63) ** 2,
                                (length * 0.29) ** 2,
                                (length * 0.23) ** 2,
                                (length * 0.43) ** 2,
                                -((length * 0.28) ** 2),
                                (length * 0.58) ** 2,
                            ]
                        )
                        if gender == "f"
                        else np.array(
                            [
                                (length * 0.61) ** 2,
                                (length * 0.22) ** 2,
                                (length * 0.15) ** 2,
                                (length * 0.38) ** 2,
                                -((length * 0.2) ** 2),
                                (length * 0.56) ** 2,
                            ]
                        )
                    )
                    * dict_sgmt["mass"],
                    5,
                )
                dicts.append(dict_sgmt)
            elif name == "right_upperleg":
                length = (0.2354 if gender == "f" else 0.2441) * height
                sgt_lengths["right_upperleg_0"] = length
                dict_sgmt["mass"] = np.round(
                    (0.146 if gender == "f" else 0.123) * weight, 2
                )
                dict_sgmt["com"] = np.round(
                    (
                        np.array([-0.077, -0.377, 0.009])
                        if gender == "f"
                        else np.array([-0.041, -0.429, 0.033])
                    )
                    * length,
                    3,
                )
                dict_sgmt["inertia"] = np.round(
                    (
                        np.array(
                            [
                                (length * 0.31) ** 2,
                                -((length * 0.07) ** 2),
                                -((length * 0.02) ** 2),
                                (length * 0.19) ** 2,
                                -((length * 0.07) ** 2),
                                (length * 0.32) ** 2,
                            ]
                        )
                        if gender == "f"
                        else np.array(
                            [
                                (length * 0.29) ** 2,
                                -((length * 0.07) ** 2),
                                -((length * 0.02) ** 2),
                                (length * 0.15) ** 2,
                                -((length * 0.07) ** 2),
                                (length * 0.3) ** 2,
                            ]
                        )
                    )
                    * dict_sgmt["mass"],
                    5,
                )
                dicts.append(dict_sgmt)
            elif name == "right_lowerleg":
                length = (0.2410 if gender == "f" else 0.2446) * height
                sgt_lengths["right_lowerleg_0"] = length
                dict_sgmt["mass"] = np.round(
                    (0.045 if gender == "f" else 0.048) * weight, 2
                )
                dict_sgmt["com"] = np.round(
                    (
                        np.array([-0.049, -0.404, 0.031])
                        if gender == "f"
                        else np.array([-0.048, -0.41, 0.007])
                    )
                    * length,
                    3,
                )
                dict_sgmt["inertia"] = np.round(
                    (
                        np.array(
                            [
                                (length * 0.28) ** 2,
                                (length * 0.02) ** 2,
                                (length * 0.01) ** 2,
                                (length * 0.1) ** 2,
                                (length * 0.06) ** 2,
                                (length * 0.28) ** 2,
                            ]
                        )
                        if gender == "f"
                        else np.array(
                            [
                                (length * 0.28) ** 2,
                                (length * 0.04) ** 2,
                                (length * 0.02) ** 2,
                                (length * 0.1) ** 2,
                                (length * 0.05) ** 2,
                                (length * 0.28) ** 2,
                            ]
                        )
                    )
                    * dict_sgmt["mass"],
                    5,
                )
                dicts.append(dict_sgmt)
            elif name == "right_foot":
                length = (0.1447 if gender == "f" else 0.1497) * height
                sgt_lengths["right_foot_0"] = length
                dict_sgmt["mass"] = np.round(
                    (0.01 if gender == "f" else 0.012) * weight, 2
                )
                dict_sgmt["com"] = np.round(
                    (
                        np.array([0.27, -0.218, 0.039])
                        if gender == "f"
                        else np.array([0.382, -0.151, 0.026])
                    )
                    * length,
                    3,
                )
                dict_sgmt["inertia"] = np.round(
                    (
                        np.array(
                            [
                                (length * 0.17) ** 2,
                                -((length * 0.10) ** 2),
                                (length * 0.06) ** 2,
                                (length * 0.36) ** 2,
                                -((length * 0.04) ** 2),
                                (length * 0.35) ** 2,
                            ]
                        )
                        if gender == "f"
                        else np.array(
                            [
                                (length * 0.17) ** 2,
                                (length * 0.13) ** 2,
                                -((length * 0.08) ** 2),
                                (length * 0.37) ** 2,
                                (length * 0) ** 2,
                                (length * 0.36) ** 2,
                            ]
                        )
                    )
                    * dict_sgmt["mass"],
                    5,
                )
                dicts.append(dict_sgmt)

        return dict(zip(inertial_segment_names, dicts)), sgt_lengths

    @staticmethod
    def get_dict_joint_placements(height: float, gender: str = "m") -> dict:
        joint_names = [
            "middle_thoracic_Z",
            "middle_cervical_Z",
            "left_clavicle_joint_X",
            "right_clavicle_joint_X",
            "left_shoulder_Z",
            "left_elbow_Z",
            "left_wrist_Z",
            "right_shoulder_Z",
            "right_elbow_Z",
            "right_wrist_Z",
            "left_hip_Z",
            "left_knee_Z",
            "left_ankle_Z",
            "right_hip_Z",
            "right_knee_Z",
            "right_ankle_Z",
        ]

        lengths_names = [
            "L_abdomen",
            "L_thorax_cerv",
            "L_thorax_supr",
            "L_upperarm",
            "L_lowerarm",
            "L_upperleg",
            "L_lowerleg",
        ]
        ratios = np.array(
            [
                0.1183
                if gender == "f"
                else 0.1237,  # L_abdomen MPT  from XYP to OMPH (De Leva 1996)
                0.1314
                if gender == "f"
                else 0.1390,  # L_thorax UPT from CERV to XYPH (De Leva 1996)
                0.0821 if gender == "f" else 0.0980,  # from SUPR to XYPH (De Leva 1996)
                0.1510 if gender == "f" else 0.1531,  # L_upperarm (Dumas 2007)
                0.1534 if gender == "f" else 0.1593,  # L_lowerarm (Dumas 2007)
                0.2354 if gender == "f" else 0.2441,  # L_upperleg (Dumas 2007)
                0.2410 if gender == "f" else 0.2446,  # L_lowerleg (Dumas 2007)
            ]
        )
        # Ratios from De Leva : HJC from Seidel
        # SJC Schnorenberg-style AC/H = 0.132 (F) or 0.0129 (M)
        SJ_thorax_X_r = 0.0043 if gender == "f" else 0.0046
        SJ_thorax_Y_r = -0.0449 if gender == "f" else -0.0416
        SJ_thorax_Z_r = 0.1108 if gender == "f" else 0.1164
        HJ_pelvis_X_r = 0.0138 if gender == "f" else 0.0126
        HJ_pelvis_Y_r = -0.0570 if gender == "f" else -0.0558
        HJ_pelvis_Z_r = 0.0548 if gender == "f" else 0.0457

        lengths = np.round(ratios * height, 3)  # mm accuracy
        dict_lengths = dict(zip(lengths_names, lengths))

        joint_placements = []
        for j in joint_names:
            if j == "middle_thoracic_Z":
                joint_placements.append(np.array([0, dict_lengths["L_abdomen"], 0]))
            elif j == "middle_cervical_Z":
                joint_placements.append(np.array([0, dict_lengths["L_thorax_cerv"], 0]))
            elif j == "left_clavicle_joint_X":
                joint_placements.append(np.array([0, dict_lengths["L_thorax_supr"], 0]))
            elif j == "right_clavicle_joint_X":
                joint_placements.append(np.array([0, dict_lengths["L_thorax_supr"], 0]))
            elif j == "left_shoulder_Z":
                joint_placements.append(
                    np.round(
                        height
                        * np.array([SJ_thorax_X_r, SJ_thorax_Y_r, -SJ_thorax_Z_r]),
                        3,
                    )
                )
            elif j == "left_elbow_Z":
                joint_placements.append(np.array([0, -dict_lengths["L_upperarm"], 0]))
            elif j == "left_wrist_Z":
                joint_placements.append(np.array([0, -dict_lengths["L_lowerarm"], 0]))
            elif j == "right_shoulder_Z":
                joint_placements.append(
                    np.round(
                        height
                        * np.array([SJ_thorax_X_r, SJ_thorax_Y_r, SJ_thorax_Z_r]),
                        3,
                    )
                )
            elif j == "right_elbow_Z":
                joint_placements.append(np.array([0, -dict_lengths["L_upperarm"], 0]))
            elif j == "right_wrist_Z":
                joint_placements.append(np.array([0, -dict_lengths["L_lowerarm"], 0]))
            elif j == "left_hip_Z":
                joint_placements.append(
                    np.round(
                        height
                        * np.array([HJ_pelvis_X_r, HJ_pelvis_Y_r, -HJ_pelvis_Z_r]),
                        3,
                    )
                )
            elif j == "left_knee_Z":
                joint_placements.append(np.array([0, -dict_lengths["L_upperleg"], 0]))
            elif j == "left_ankle_Z":
                joint_placements.append(np.array([0, -dict_lengths["L_lowerleg"], 0]))
            elif j == "right_hip_Z":
                joint_placements.append(
                    np.round(
                        height
                        * np.array([HJ_pelvis_X_r, HJ_pelvis_Y_r, HJ_pelvis_Z_r]),
                        3,
                    )
                )
            elif j == "right_knee_Z":
                joint_placements.append(np.array([0, -dict_lengths["L_upperleg"], 0]))
            elif j == "right_ankle_Z":
                joint_placements.append(np.array([0, -dict_lengths["L_lowerleg"], 0]))

        return dict(zip(joint_names, joint_placements))

    @staticmethod
    def get_dict_meshes_scale(
        height: float, sgt_lengths: dict, gender: str = "m"
    ) -> dict:
        meshes_names = [
            "middle_pelvis_0",
            "left_upperleg_0",
            "left_lowerleg_0",
            "left_lowerleg_1",
            "left_foot_0",
            "middle_abdomen_0",
            "middle_abdomen_1",
            "middle_head_0",
            "middle_head_1",
            "left_upperarm_0",
            "left_upperarm_1",
            "left_lowerarm_0",
            "left_lowerarm_1",
            "left_hand_0",
            "right_upperarm_0",
            "right_upperarm_1",
            "right_lowerarm_0",
            "right_lowerarm_1",
            "right_hand_0",
            "right_upperleg_0",
            "right_lowerleg_0",
            "right_lowerleg_1",
            "right_foot_0",
        ]

        scales = []

        for name in meshes_names:
            if (
                name == "middle_pelvis_0"
            ):  # pelvis mesh size is : X = 37.050m, Y = 32.551m, Z = 39.252m
                scale = np.round(
                    np.array(
                        [
                            1.09 * sgt_lengths["middle_pelvis_0_width"] / 39.252,
                            sgt_lengths["middle_pelvis_0_width"] / 39.252,
                            sgt_lengths["middle_pelvis_0_width"] / 39.252,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif (
                name == "left_upperleg_0"
            ):  # upperleg mesh size is : X = 20.977m, Y = 68.524m, Z = 20.977m
                scale = np.round(
                    np.array(
                        [
                            0.96 * sgt_lengths["left_upperleg_0"] / 68.524,
                            sgt_lengths["left_upperleg_0"] / 68.524,
                            1.08 * sgt_lengths["left_upperleg_0"] / 68.524,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif (
                name == "left_lowerleg_0"
            ):  # lowerleg mesh size is : X = 17.986m, Y = 67m, Z = 17.977m
                scale = np.round(
                    np.array(
                        [
                            0.96 * sgt_lengths["left_lowerleg_0"] / 67,
                            sgt_lengths["left_lowerleg_0"] / 67,
                            1.07 * sgt_lengths["left_lowerleg_0"] / 67,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif name == "left_lowerleg_1":  # knee mesh is a sphere of 12.5m
                scale = np.round(
                    np.array(
                        [
                            0.7
                            * (0.053 * height if gender == "f" else 0.056 * height)
                            / 12.5,
                            0.7
                            * (0.053 * height if gender == "f" else 0.056 * height)
                            / 12.5,
                            0.7
                            * (0.053 * height if gender == "f" else 0.056 * height)
                            / 12.5,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif (
                name == "left_foot_0"
            ):  # foot mesh size is : X = 40.5m, Y = 16.5m, Z = 17.999m
                scale = np.round(
                    np.array(
                        [
                            sgt_lengths["left_foot_0"] / 40.5,
                            sgt_lengths["left_foot_0"] / 40.5,
                            sgt_lengths["left_foot_0"] / 40.5,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif name == "middle_abdomen_0":  # abdomen meshis a sphere of radius 30 m
                scale = np.round(
                    np.array(
                        [
                            sgt_lengths["middle_abdomen_0"] / 30,
                            sgt_lengths["middle_abdomen_0"] / 30,
                            sgt_lengths["middle_abdomen_0"] / 30,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif (
                name == "middle_abdomen_1"
            ):  # torso mesh size is : X = 36.991m, Y = 35m, Z = 40.029m
                scale = np.round(
                    np.array(
                        [
                            sgt_lengths["middle_thorax_0"] / 35,
                            sgt_lengths["middle_thorax_0"] / 35,
                            sgt_lengths["middle_thorax_0"] / 35,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif (
                name == "middle_head_0"
            ):  # head mesh size is : X = 32.049m, Y = 42m, Z = 32.049m
                scale = np.round(
                    np.array(
                        [
                            sgt_lengths["middle_head_0"] / 42,
                            sgt_lengths["middle_head_0"] / 42,
                            sgt_lengths["middle_head_0"] / 42,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif name == "middle_head_1":  # neck mesh
                scale = np.round(
                    np.array(
                        [
                            (0.078 * height if gender == "f" else 0.086 * height)
                            / 31.0,
                            (0.078 * height if gender == "f" else 0.086 * height)
                            / 31.0,
                            (0.078 * height if gender == "f" else 0.086 * height)
                            / 31.0,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif name == "left_upperarm_0":  # shoulder mesh is a shpere of radius 17.5
                scale = np.round(
                    np.array(
                        [
                            (0.037 * height if gender == "f" else 0.038 * height)
                            / 17.5,
                            (0.037 * height if gender == "f" else 0.038 * height)
                            / 17.5,
                            (0.037 * height if gender == "f" else 0.038 * height)
                            / 17.5,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif (
                name == "left_upperarm_1"
            ):  # upperarm mesh is of size : X = 17.158m, Y = 46m, Z = 17.183m
                scale = np.round(
                    np.array(
                        [
                            1.05 * sgt_lengths["left_upperarm_0"] / 46,
                            sgt_lengths["left_upperarm_0"] / 46,
                            1.16 * sgt_lengths["left_upperarm_0"] / 46,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif (
                name == "left_lowerarm_0"
            ):  # lowerarm mesh is of size : X = 15.547m, Y = 45m, Z = 15.541m
                scale = np.round(
                    np.array(
                        [
                            sgt_lengths["left_lowerarm_0"] / 45,
                            sgt_lengths["left_lowerarm_0"] / 45,
                            sgt_lengths["left_lowerarm_0"] / 45,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif name == "left_lowerarm_1":  # elbow mesh is a sphere of radius 12.5
                scale = np.round(
                    np.array(
                        [
                            (0.037 * height if gender == "f" else 0.038 * height)
                            / 12.5,
                            (0.037 * height if gender == "f" else 0.038 * height)
                            / 12.5,
                            (0.037 * height if gender == "f" else 0.038 * height)
                            / 12.5,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif (
                name == "left_hand_0"
            ):  # hand mesh is of size : X = 12.055m, Y = 30m, Z = 16.646m
                scale = np.round(
                    np.array(
                        [
                            sgt_lengths["left_hand_0"] / 30,
                            sgt_lengths["left_hand_0"] / 30,
                            sgt_lengths["left_hand_0"] / 30,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif name == "right_upperarm_0":  # shoulder mesh
                scale = np.round(
                    np.array(
                        [
                            (0.037 * height if gender == "f" else 0.038 * height)
                            / 17.5,
                            (0.037 * height if gender == "f" else 0.038 * height)
                            / 17.5,
                            (0.037 * height if gender == "f" else 0.038 * height)
                            / 17.5,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif name == "right_upperarm_1":  # upperarm mesh
                scale = np.round(
                    np.array(
                        [
                            1.05 * sgt_lengths["right_upperarm_0"] / 46,
                            sgt_lengths["right_upperarm_0"] / 46,
                            1.16 * sgt_lengths["right_upperarm_0"] / 46,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif name == "right_lowerarm_0":  # lowerarm mesh
                scale = np.round(
                    np.array(
                        [
                            sgt_lengths["right_lowerarm_0"] / 45,
                            sgt_lengths["right_lowerarm_0"] / 45,
                            sgt_lengths["right_lowerarm_0"] / 45,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif name == "right_lowerarm_1":  # elbow mesh
                scale = np.round(
                    np.array(
                        [
                            (0.037 * height if gender == "f" else 0.038 * height)
                            / 12.5,
                            (0.037 * height if gender == "f" else 0.038 * height)
                            / 12.5,
                            (0.037 * height if gender == "f" else 0.038 * height)
                            / 12.5,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif name == "right_hand_0":  # hand mesh
                scale = np.round(
                    np.array(
                        [
                            sgt_lengths["right_hand_0"] / 30,
                            sgt_lengths["right_hand_0"] / 30,
                            sgt_lengths["right_hand_0"] / 30,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif name == "right_upperleg_0":  # upperleg mesh
                scale = np.round(
                    np.array(
                        [
                            0.96 * sgt_lengths["right_upperleg_0"] / 68.524,
                            sgt_lengths["right_upperleg_0"] / 68.524,
                            1.08 * sgt_lengths["right_upperleg_0"] / 68.524,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif name == "right_lowerleg_0":  # lowerleg mesh
                scale = np.round(
                    np.array(
                        [
                            0.96 * sgt_lengths["right_lowerleg_0"] / 67,
                            sgt_lengths["right_lowerleg_0"] / 67,
                            1.07 * sgt_lengths["right_lowerleg_0"] / 67,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif name == "right_lowerleg_1":  # knee mesh
                scale = np.round(
                    np.array(
                        [
                            0.7
                            * (0.053 * height if gender == "f" else 0.056 * height)
                            / 12.5,
                            0.7
                            * (0.053 * height if gender == "f" else 0.056 * height)
                            / 12.5,
                            0.7
                            * (0.053 * height if gender == "f" else 0.056 * height)
                            / 12.5,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif name == "right_foot_0":  # foot mesh
                scale = np.round(
                    np.array(
                        [
                            sgt_lengths["right_foot_0"] / 40.5,
                            sgt_lengths["right_foot_0"] / 40.5,
                            sgt_lengths["right_foot_0"] / 40.5,
                        ]
                    ),
                    4,
                )
                scales.append(scale)

        return dict(zip(meshes_names, scales))

    def _scale_pin_model(
        self,
        model: pin.Model,
        visual_model: pin.GeometryModel,
        height: float,
        weight: float,
        gender: str = "m",
    ) -> None:
        """
        Overwrite model.inertias, model.jointPlacements, and model.geometryObjects.meshScale
        based on anthropometry.
        """
        # compute parameters
        joints = self.get_dict_joint_placements(height, gender)
        inert, sgt_lengths = self.get_dict_inertial_param(height, weight, gender)
        scales = self.get_dict_meshes_scale(height, sgt_lengths, gender)

        # apply inertias
        for seg, P in inert.items():
            b_id = model.frames[model.getFrameId(seg)].parentJoint

            Ixx, Ixy, Ixz, Iyy, Iyz, Izz = P["inertia"]
            I_mat = np.array([[Ixx, Ixy, Ixz], [Ixy, Iyy, Iyz], [Ixz, Iyz, Izz]])
            model.inertias[b_id] = pin.Inertia(P["mass"], P["com"], I_mat)

        # apply joint placements
        for j, pos in joints.items():
            j_id = model.getJointId(j)
            M = model.jointPlacements[j_id]
            M.translation = pos
            model.jointPlacements[j_id] = M

        # scale visuals
        assert len(visual_model.geometryObjects.tolist()) == len(scales)
        for geom_obj in visual_model.geometryObjects:
            geom_obj.meshScale = scales[geom_obj.name]
