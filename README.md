Example robot URDFs
===============================================

## <img align="center" height="20" src="https://i.imgur.com/vAYeCzC.png"/> Introduction

This repository includes a set of robot descriptions that are aimed to be used in benchmarking. These source files do not intend to substitute original their repositories.


**Authors:**  [Carlos Mastalli](https://cmastalli.github.io/) <br />
**With additional support from the Gepetto team at LAAS-CNRS.**

[![pipeline status](https://gepgitlab.laas.fr/gepetto/example-robot-data/badges/master/build.svg)](https://gepgitlab.laas.fr/Gepetto/example-robot-data/commits/master)


## <img align="center" height="20" src="https://i.imgur.com/x1morBF.png"/> Installation
You can install this package throught robotpkg. robotpkg is a package manager tailored for robotics softwares. It greatly simplifies the release of new versions along with the management of their dependencies. You just need to add the robotpkg apt repository to your sources.list and then use `sudo apt install robotpkg-example-robot-data` or `sudo apt install robotpkg-py27-example-robot-data` if you need the Python loaders:

### Add robotpkg apt repository
If you have never added robotpkg as a softwares repository, please follow first the instructions from 1 to 3. Otherwise, go directly to instruction 4. Those instructions are similar to the installation procedures presented in [http://robotpkg.openrobots.org/debian.html](http://robotpkg.openrobots.org/debian.html).

1. Add robotpkg as source repository to apt:

		sudo tee /etc/apt/sources.list.d/robotpkg.list <<EOF
		deb [arch=amd64] http://robotpkg.openrobots.org/wip/packages/debian/pub $(lsb_release -sc) robotpkg
		deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -sc) robotpkg
		EOF

2. Register the authentication certificate of robotpkg:

		curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -

3. You need to run at least once apt update to fetch the package descriptions:

		sudo apt-get update

4. The installation of example-robot-data:

		sudo apt install robotpkg-example-robot-data


## Show the robot

(you will need pinocchio and its python bindings)

`python -m example_robot_data [hyq,talos,talos_arm,talos_legs,icub,solo,solo12,tiago,tiago_no_hand]`
