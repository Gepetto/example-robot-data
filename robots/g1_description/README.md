# Unitree G1 Description

upstream: https://github.com/unitreerobotics/unitree_ros/tree/master/robots/g1_description
license: BSD 3-Clause License

Modifications:
- limited the loader registry to just the core `G1Loader`/`G1WithHandsLoader`, added `ref_posture="standing"` and re-enabled `free_flyer` so Pinocchio/Crocoddyl builds have sensible defaults.
- pruned the mesh directory down to only the files actually referenced by those two URDFs and removed the unused `inspire_hand` folder.
- dropped the `<mujoco>` blocks that only applied to MuJoCo so the URDFs stay focused on URDF/Pinocchio usage.
