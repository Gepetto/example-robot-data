from os.path import join

import hppfcl
import numpy as np
import pinocchio as pin

from .utils import RobotLoader, getModelPath


class PandaLoader(RobotLoader):
    path = "panda_description"
    urdf_filename = "panda.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "panda.srdf"
    ref_posture = "default"


class PandaLoaderCollision(PandaLoader):
    urdf_filename = "panda_collision.urdf"

    def __init__(self, verbose=False):
        super().__init__(verbose=verbose)

        cmodel = self.robot.collision_model.copy()
        list_names_capsules = []
        # Iterate through geometry objects in the collision model
        for geom_object in cmodel.geometryObjects:
            geometry = geom_object.geometry
            # Remove superfluous suffix from the name
            base_name = "_".join(geom_object.name.split("_")[:-1])

            # Convert cylinders to capsules
            if isinstance(geometry, hppfcl.Cylinder):
                name = self.generate_capsule_name(base_name, list_names_capsules)
                list_names_capsules.append(name)
                capsule = pin.GeometryObject(
                    name=name,
                    parent_frame=int(geom_object.parentFrame),
                    parent_joint=int(geom_object.parentJoint),
                    collision_geometry=hppfcl.Capsule(
                        geometry.radius, geometry.halfLength
                    ),
                    placement=geom_object.placement,
                )
                capsule.meshColor = np.array([249, 136, 126, 125]) / 255  # Red color
                self.robot.collision_model.addGeometryObject(capsule)
                self.robot.collision_model.removeGeometryObject(geom_object.name)

            # Remove spheres associated with links
            elif isinstance(geometry, hppfcl.Sphere) and "link" in geom_object.name:
                self.robot.collision_model.removeGeometryObject(geom_object.name)

        # Recreate collision data since the collision pairs changed
        self.robot.collision_data = self.robot.collision_model.createData()

        self.srdf_path = None
        self.robot.q0 = pin.neutral(self.robot.model)
        root = getModelPath(self.path)
        self.robot.urdf = join(root, self.path, self.urdf_subpath, self.urdf_filename)
