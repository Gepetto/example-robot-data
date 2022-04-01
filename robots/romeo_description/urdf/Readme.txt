urdf files have been checkout from Aldebaran repository
https://github.com/ros-aldebaran/romeo_robot/tree/master/romeo_description
v 0.0.9

It has not been possible for now to synchronize with Aldebaran repository for the following reasons :
- we need a romeo_small version of the robot (mostly without fingers), change between romeo and romeo_small could be checked with a simple diff since they are build from the same model
- Aldebaran and sot convention differ so it has been needed to change the model :
  - Aldebaran model defines a joint named "base_joint", this name is already used inside sot and could not be redefined. This "base_joint" has been renamed "waist".
  - Aldebaran model considers trunkYaw from torso to body and we need trunkYaw from body to torso as it is represented below, so the model has been changed accordingly

Romeo Aldebaran
base_link --- base_joint ---> torso --- trunkYaw --> body

Romeo sot
base_link -- waist --> body -- trunkYaw --> torso




tips:
If you want to check the model, you can use urdf_to_graphiz to get an image of the urdf tree.
