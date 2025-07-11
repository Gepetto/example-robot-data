import sys
import typing
from os.path import dirname, exists, join

import numpy as np
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper

try:
    from .path import EXAMPLE_ROBOT_DATA_MODEL_DIR, EXAMPLE_ROBOT_DATA_SOURCE_DIR
except ImportError:
    pass


def getModelPath(subpath, verbose=False):
    source = dirname(dirname(dirname(__file__)))  # top level source directory
    paths = [
        # function called from "make release" in build/ dir
        join(dirname(dirname(dirname(source))), "robots"),
        # function called from a build/ dir inside top level source
        join(dirname(source), "robots"),
        # function called from top level source dir
        join(source, "robots"),
    ]
    try:
        EXAMPLE_ROBOT_DATA_MODEL_DIR

        # function called from installed project
        paths.append(EXAMPLE_ROBOT_DATA_MODEL_DIR)
        # function called from off-tree build dir
        paths.append(EXAMPLE_ROBOT_DATA_SOURCE_DIR)
    except NameError:
        pass
    paths += [join(p, "../../../share/example-robot-data/robots") for p in sys.path]
    for path in paths:
        if exists(join(path, subpath.strip("/"))):
            if verbose:
                print(f"using {path} as modelPath")
            return path
    raise OSError(f"{subpath} not found")


def readParamsFromSrdf(
    model,
    SRDF_PATH,
    verbose=False,
    has_rotor_parameters=True,
    referencePose="half_sitting",
):
    if has_rotor_parameters:
        pin.loadRotorParameters(model, SRDF_PATH, verbose)
    model.armature = np.multiply(
        model.rotorInertia.flat, np.square(model.rotorGearRatio.flat)
    )
    pin.loadReferenceConfigurations(model, SRDF_PATH, verbose)
    q0 = pin.neutral(model)
    if referencePose is not None:
        q0 = model.referenceConfigurations[referencePose].copy()
    q0 = pin.normalize(model, q0)
    return q0


class RobotLoader:
    path = ""
    urdf_filename = ""
    srdf_filename = ""
    sdf_filename = ""
    sdf_root_link_name = ""
    sdf_parent_guidance: typing.ClassVar = []
    urdf_subpath = "robots"
    srdf_subpath = "srdf"
    sdf_subpath = ""
    ref_posture = "half_sitting"
    has_rotor_parameters = False
    free_flyer = False
    model_path = None

    def __init__(self, verbose=False):
        self.verbose = verbose
        if self.urdf_filename:
            if self.sdf_filename:
                raise AttributeError("Please choose between URDF *or* SDF")
            df_path = join(self.path, self.urdf_subpath, self.urdf_filename)
            builder = RobotWrapper.BuildFromURDF
            if self.model_path is None:
                self.model_path = getModelPath(df_path, self.verbose)
            self.df_path = join(self.model_path, df_path)
            self.robot = builder(
                self.df_path,
                [join(self.model_path, "../..")],
                pin.JointModelFreeFlyer() if self.free_flyer else None,
            )
        else:
            df_path = join(self.path, self.sdf_subpath, self.sdf_filename)
            try:
                builder = RobotWrapper.BuildFromSDF
                if self.model_path is None:
                    self.model_path = getModelPath(df_path, self.verbose)
                self.df_path = join(self.model_path, df_path)
                if tuple(int(i) for i in pin.__version__.split(".")) > (2, 9, 1):
                    self.robot = builder(
                        self.df_path,
                        package_dirs=[join(self.model_path, "../..")],
                        root_joint=(
                            pin.JointModelFreeFlyer() if self.free_flyer else None
                        ),
                        root_link_name=self.sdf_root_link_name,
                        parent_guidance=self.sdf_parent_guidance,
                    )
                else:
                    self.robot = builder(
                        self.df_path,
                        package_dirs=[join(self.model_path, "../..")],
                        root_joint=(
                            pin.JointModelFreeFlyer() if self.free_flyer else None
                        ),
                    )
            except AttributeError:
                raise ImportError("Building SDF models require pinocchio >= 3.0.0")

        if self.srdf_filename:
            self.srdf_path = join(
                self.model_path, self.path, self.srdf_subpath, self.srdf_filename
            )
            self.robot.q0 = readParamsFromSrdf(
                self.robot.model,
                self.srdf_path,
                self.verbose,
                self.has_rotor_parameters,
                self.ref_posture,
            )

            if pin.WITH_HPP_FCL and pin.WITH_HPP_FCL_BINDINGS:
                # Add all collision pairs
                self.robot.collision_model.addAllCollisionPairs()

                # Remove collision pairs per SRDF
                pin.removeCollisionPairs(
                    self.robot.model, self.robot.collision_model, self.srdf_path, False
                )

                # Recreate collision data since the collision pairs changed
                self.robot.collision_data = self.robot.collision_model.createData()
        else:
            self.srdf_path = None
            self.robot.q0 = pin.neutral(self.robot.model)
        root = getModelPath(self.path)
        self.robot.urdf = join(root, self.path, self.urdf_subpath, self.urdf_filename)

        if self.free_flyer:
            self.addFreeFlyerJointLimits()

    def addFreeFlyerJointLimits(self):
        ub = self.robot.model.upperPositionLimit
        ub[:7] = 1
        self.robot.model.upperPositionLimit = ub
        lb = self.robot.model.lowerPositionLimit
        lb[:7] = -1
        self.robot.model.lowerPositionLimit = lb

    def generate_capsule_name(self, base_name: str, existing_names: list) -> str:
        """Generates a unique capsule name for a geometry object.

        Args:
            base_name (str): The base name of the geometry object.
            existing_names (list): List of names already assigned to capsules.

        Returns:
            str: Unique capsule name.
        """
        i = 0
        while f"{base_name}_capsule_{i}" in existing_names:
            i += 1
        return f"{base_name}_capsule_{i}"
