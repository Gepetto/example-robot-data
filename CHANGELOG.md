# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

## [Unreleased]

- added CHANGELOG.md in https://github.com/Gepetto/example-robot-data/pull/193

## [4.0.9] - 2023-11-29

### What's Changed
* Supported pinocchio with cppadcg installation by @cmastalli in https://github.com/Gepetto/example-robot-data/pull/180
* [pre-commit.ci] pre-commit autoupdate by @pre-commit-ci in https://github.com/Gepetto/example-robot-data/pull/181
* Verbose by @nim65s in https://github.com/Gepetto/example-robot-data/pull/183
* [pre-commit.ci] pre-commit autoupdate by @pre-commit-ci in https://github.com/Gepetto/example-robot-data/pull/184
* [pre-commit.ci] pre-commit autoupdate by @pre-commit-ci in https://github.com/Gepetto/example-robot-data/pull/187
* [pre-commit.ci] pre-commit autoupdate by @pre-commit-ci in https://github.com/Gepetto/example-robot-data/pull/188
* CMake: require >= 3.10 by @nim65s in https://github.com/Gepetto/example-robot-data/pull/185
* Update jrlcmakemodules to support Python 3.12 by @jorisv in https://github.com/Gepetto/example-robot-data/pull/190
* [pre-commit.ci] pre-commit autoupdate by @pre-commit-ci in https://github.com/Gepetto/example-robot-data/pull/189
* [pre-commit.ci] pre-commit autoupdate by @pre-commit-ci in https://github.com/Gepetto/example-robot-data/pull/191
* Sync submodule cmake by @jcarpent in https://github.com/Gepetto/example-robot-data/pull/192

### New Contributors
* @jorisv made their first contribution in https://github.com/Gepetto/example-robot-data/pull/190

## [4.0.8] - 2023-07-18

- updated copyright and contributors
- updated pre-commit
- updated cmake for eigenpy 3.1.0
- updated submodule

## [4.0.7] - 2023-05-18

- use CMake for `EXAMPLE_ROBOT_DATA_MODEL_DIR`

## [4.0.6] - 2023-05-13

- [go1][b1] Added fake inertia in base for pybullet
- updated for eigenpy v3
- cmake format
- cmake: fetch submodule if not available
- reduce some meshes
- move import of paths
- pre-commit update
- sync submodule

## [4.0.5] - 2023-02-16

### What's Changed
* Wrong rotation in allegro visual urdf description by @lmontaut in https://github.com/Gepetto/example-robot-data/pull/157
* go1: fix inertia of fake camera links for pybullet by @nim65s in https://github.com/Gepetto/example-robot-data/pull/158

## [4.0.4] - 2023-02-10

### What's Changed
* [pre-commit.ci] pre-commit autoupdate by @pre-commit-ci in https://github.com/Gepetto/example-robot-data/pull/141
* [pre-commit.ci] pre-commit autoupdate by @pre-commit-ci in https://github.com/Gepetto/example-robot-data/pull/142
* Remove Lamp nodes that are referencing non-existing library_lights by @petrikvladimir in https://github.com/Gepetto/example-robot-data/pull/143
* Fix Kinova mesh materials and remove Anymal mounting block by @wxmerkt in https://github.com/Gepetto/example-robot-data/pull/145
* Add ANYmal C by @wxmerkt in https://github.com/Gepetto/example-robot-data/pull/144
* Fix Panda inertials, add SRDF by @wxmerkt in https://github.com/Gepetto/example-robot-data/pull/147
* [pre-commit.ci] pre-commit autoupdate by @pre-commit-ci in https://github.com/Gepetto/example-robot-data/pull/148
* [pre-commit.ci] pre-commit autoupdate by @pre-commit-ci in https://github.com/Gepetto/example-robot-data/pull/150
* [panda_description] Update meshes from franka_ros to get textures by @wxmerkt in https://github.com/Gepetto/example-robot-data/pull/149
* Fix tests for pybullet by @nim65s in https://github.com/Gepetto/example-robot-data/pull/151
* Add Unitree B1 & Go1 by @Sergim96 in https://github.com/Gepetto/example-robot-data/pull/152
* [pre-commit.ci] pre-commit autoupdate by @pre-commit-ci in https://github.com/Gepetto/example-robot-data/pull/153
* [pre-commit.ci] pre-commit autoupdate by @pre-commit-ci in https://github.com/Gepetto/example-robot-data/pull/154
* Allegro hand: adding missing inertia information by @lmontaut in https://github.com/Gepetto/example-robot-data/pull/156

### New Contributors
* @petrikvladimir made their first contribution in https://github.com/Gepetto/example-robot-data/pull/143
* @Sergim96 made their first contribution in https://github.com/Gepetto/example-robot-data/pull/152

## [4.0.3] - 2022-11-09

### What's Changed
* [pre-commit.ci] pre-commit autoupdate by @pre-commit-ci in https://github.com/Gepetto/example-robot-data/pull/138
* [pre-commit.ci] pre-commit autoupdate by @pre-commit-ci in https://github.com/Gepetto/example-robot-data/pull/139
* Adding allegro hands (left and right) by @lmontaut in https://github.com/Gepetto/example-robot-data/pull/140

### New Contributors
* @lmontaut made their first contribution in https://github.com/Gepetto/example-robot-data/pull/140

## [4.0.2] - 2022-09-01

- Tests: check cassie with pinocchio 2.9.1 / 2.9.2
- Update CMake for eigenpy 2.7.11
- Modernize CMake if available
- update tooling

## [4.0.1] - 2022-04-11

- fix compatiblity for dependencies

## [4.0.0] - 2022-04-01

- Add laikago
- Add simplified double pendulum
- update simple humanoid
- update SDF loader & Cassie for pinocchio 2.9.2 (private)
- :warning: remove deprecated loaders :warning:
- setup tooling:
    - black
    - flake8
    - pre-commit + CI
- reformat for black / pre-commit

## [3.13.1] - 2022-03-01

- added Articulated soft robot
- added Continuous double pendulum
- cassie/srdf: update q0 config for pelvis orientation
- talos: add rotor parameters
- finger_edu: fix inertia
- a1: fix inertia
- allow customization of model path
- updated python format

## [3.12.0] - 2021-11-12

- cassie: update q0
- talos: use rotor parameters
- python loader: allow customization of model path

## [3.11.0] - 2021-09-17

- add a1_description (Unitree A1 robot)
- add cassie
- add SDF support
- fix velocity limits in Talos legs
- document upstreams & license

## [3.10.0] - 2021-06-17

- Add finger_edu & bolt robots
- Add all collision pairs and remove disabled pairs from SRDF
- Define missing materials to fix warnings
- Rename "Solo" to "Solo8"

## [3.9.1] - 2021-05-28

- baxter got its missing hands back

## [3.9.0] - 2021-05-26

- add Baxter robot
- add Tiago Dual robot
- remove duplication of q0
- define q0 as neutral if the robot doesn't have a SRDF
- unit test for all UR robots
- fix inertia of all UR robots
- fix Anymal-Kinova SRDF collision matrix

## [3.8.0] - 2021-03-10

- add shortcut for load_full
- report warnings to end users
- fix anymal SRDF
- define unknown materials to workaround URDF parsers warnings
- add inertias infos to avoid unit mass on pybullet
- add a pybullet mass test
- add simple_humanoid_description

## [3.7.0] - 2020-11-24

- fix double negative in module API
- anymal: remove lights
- use RobotLoader and deprecate `load<robot>()` functions
- allow loading multiple robots, by specifying `rootNodeName`

## [3.6.1] - 2020-09-25

Changes since v3.6.0:
- fix typo in Hector
- fix collision pairs in hyq
- fix collision model in talos

## [3.6.0] - 2020-09-09

- add full Talos

## [3.5.0] - 2020-08-14

This new release fixes major flows in the scripts and provides full support of Windows.

## [3.4.2] - 2020-06-23

- allow user to define `PYTHON_SITELIB`

## [3.4.0] - 2020-06-20

- added panda, an arm robot
- fixed visual loading : using ROS_PACKAGE_PATH is no longer necessary :tada:

## [3.3.0] - 2020-04-29

- update solo12
- [README] document ROS_PACKAGE_PATH use
- [CMake] pinocchio required only for tests, fix #22
- :warning:  Fix package paths in URDFs :warning:

## [3.2.0] - 2020-04-02

Changes since v3.1.2:
- delete duplicate file
- Add the IRIS quadcopter model
- [CMake] Export

## [3.1.2] - 2020-02-07

Changes since v3.1.1:
- Migrate universal robot srdf files from hpp-universal-robots.
- Include ur srdf into loadUR
- Added effort limits for reduced models
- Fix compatibility with numpy matrix

## [3.1.1] - 2019-11-27

Changes since v3.1.0:
- add path.py.in, similar to path.hpp.in
- romeo_description: remove unused files
- anymal_b_simple_description: Add missing inertia, fix for simulator
- hyq_description: Add inertia information for simulator
- [Python] switch to numpy arrays

## [3.1.0] - 2019-11-25

Changes since v3.0.0:
- add path.hpp
- install package.xml
- update & clean CMake

## [3.0.0] - 2019-11-25

Changes since v2.5.1:
- :warning: moved robot descriptions in robots/ subdirectory
- :warning: moved headers in include/example-robot-data
- added description of double-pendulum & hector
- Fix ANYmal-Kinova URDF
- Fix the package.xml

## [2.5.1] - 2019-11-08

Changes since v2.5.0:
- move universal robot urdf with gripper from [hpp-universal-robot](https://github.com/humanoid-path-planner/hpp-universal-robot)

This was required for https://github.com/nmansard/supaero2020

## [2.5.0] - 2019-11-05

Changes since v2.4.0:
- add romeo description

## [2.4.0] - 2019-10-29

Changes since v2.3.0:
- Added the Kinova arm + ANYmal with kinova
- renamed the reference postures
- Add package.xml to allow build in catkin workspace
- Update CMake

## [2.3.0] - 2019-10-04

Changes since v2.2.0:
- add ANYmal

## [2.2.0] - 2019-09-24

Changes since v2.1.0:
- fix loading of robots without rotor parameters
- add ur_description

## [2.1.0] - 2019-09-19

- added solo

## [2.0.0] - 2019-08-29

Added tiago & icub
Added optionnal python helpers, which depend on pinocchio

## [1.0.0] - 2019-08-29

Initial release

[Unreleased]: https://github.com/gepetto/example-robot-data/compare/v4.0.9...HEAD
[4.0.9]: https://github.com/gepetto/example-robot-data/compare/v4.0.8...v4.0.9
[4.0.8]: https://github.com/gepetto/example-robot-data/compare/v4.0.7...v4.0.8
[4.0.7]: https://github.com/gepetto/example-robot-data/compare/v4.0.6...v4.0.7
[4.0.6]: https://github.com/gepetto/example-robot-data/compare/v4.0.5...v4.0.6
[4.0.5]: https://github.com/gepetto/example-robot-data/compare/v4.0.4...v4.0.5
[4.0.4]: https://github.com/gepetto/example-robot-data/compare/v4.0.3...v4.0.4
[4.0.3]: https://github.com/gepetto/example-robot-data/compare/v4.0.2...v4.0.3
[4.0.2]: https://github.com/gepetto/example-robot-data/compare/v4.0.1...v4.0.2
[4.0.1]: https://github.com/gepetto/example-robot-data/compare/v4.0.0...v4.0.1
[4.0.0]: https://github.com/gepetto/example-robot-data/compare/v3.13.1...v4.0.0
[3.13.1]: https://github.com/gepetto/example-robot-data/compare/v3.12.0...v3.13.1
[3.12.0]: https://github.com/gepetto/example-robot-data/compare/v3.11.0...v3.12.0
[3.11.0]: https://github.com/gepetto/example-robot-data/compare/v3.10.0...v3.11.0
[3.10.0]: https://github.com/gepetto/example-robot-data/compare/v3.9.1...v3.10.0
[3.9.1]: https://github.com/gepetto/example-robot-data/compare/v3.9.0...v3.9.1
[3.9.0]: https://github.com/gepetto/example-robot-data/compare/v3.8.0...v3.9.0
[3.8.0]: https://github.com/gepetto/example-robot-data/compare/v3.7.0...v3.8.0
[3.7.0]: https://github.com/gepetto/example-robot-data/compare/v3.6.1...v3.7.0
[3.6.1]: https://github.com/gepetto/example-robot-data/compare/v3.6.0...v3.6.1
[3.6.0]: https://github.com/gepetto/example-robot-data/compare/v3.5.0...v3.6.0
[3.5.0]: https://github.com/gepetto/example-robot-data/compare/v3.4.2...v3.5.0
[3.4.2]: https://github.com/gepetto/example-robot-data/compare/v3.4.0...v3.4.2
[3.4.0]: https://github.com/gepetto/example-robot-data/compare/v3.3.0...v3.4.0
[3.3.0]: https://github.com/gepetto/example-robot-data/compare/v3.2.0...v3.3.0
[3.2.0]: https://github.com/gepetto/example-robot-data/compare/v3.1.2...v3.2.0
[3.1.2]: https://github.com/gepetto/example-robot-data/compare/v3.1.1...v3.1.2
[3.1.1]: https://github.com/gepetto/example-robot-data/compare/v3.1.0...v3.1.1
[3.1.0]: https://github.com/gepetto/example-robot-data/compare/v3.0.0...v3.1.0
[3.0.0]: https://github.com/gepetto/example-robot-data/compare/v2.5.1...v3.0.0
[2.5.1]: https://github.com/gepetto/example-robot-data/compare/v2.5.0...v2.5.1
[2.5.0]: https://github.com/gepetto/example-robot-data/compare/v2.4.0...v2.5.0
[2.4.0]: https://github.com/gepetto/example-robot-data/compare/v2.3.0...v2.4.0
[2.3.0]: https://github.com/gepetto/example-robot-data/compare/v2.2.0...v2.3.0
[2.2.0]: https://github.com/gepetto/example-robot-data/compare/v2.1.0...v2.2.0
[2.1.0]: https://github.com/gepetto/example-robot-data/compare/v2.0.0...v2.1.0
[2.0.0]: https://github.com/gepetto/example-robot-data/compare/v1.0.0...v2.0.0
[1.0.0]: https://github.com/gepetto/example-robot-data/releases/tag/v1.0.0
