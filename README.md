# Example robot URDFs

[![pipeline status](https://gitlab.laas.fr/gepetto/example-robot-data/badges/master/pipeline.svg)](https://gitlab.laas.fr/gepetto/example-robot-data/-/commits/master)

This repository includes a set of robot descriptions that are aimed to be used in benchmarking, unit-tests, teachings,
tutorials or show-cases. These source files do not intend to substitute their original repositories.

## :penguin: Installation

### :package: From Debian / Ubuntu packages, with [robotpkg](http://robotpkg.openrobots.org)

1. If you have never added robotpkg's software repository, [do it now](http://robotpkg.openrobots.org/debian.html):
   ```bash
   sudo tee /etc/apt/sources.list.d/robotpkg.list <<EOF
   deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -sc) robotpkg
   EOF

   curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -
   sudo apt update
   ```

2. installation of example-robot-data and its python utils:
   ```bash
   sudo apt install robotpkg-py3\*-example-robot-data
   ```

### :turtle: With ROS

Just clone it (with `--recursive`) into a catkin workspace.

### :file_folder: From source

Clone it (with `--recursive`), create a `build` directory inside, and:
```bash
cmake .. && make && make install
```

## :gear: Configuration

Unless you got this package from catkin, you will need to set `ROS_PACKAGE_PATH` to your `$CMAKE_INSTALL_PREFIX/share`
(eg. `/usr/local/share` by default, or `/opt/openrobots/share` with robotpkg).

## :robot: Show a robot with [gepetto-gui](https://github.com/gepetto/gepetto-viewer-corba)

`python -m example_robot_data <robot>`

Where `<robot>` can be:

- `anymal`
- `anymal_kinova`
- `hector`
- `hyq`
- `iris`
- `solo`
- `solo12`
- `talos`
- `talos_arm`
- `talos_legs`
- `tiago`
- `tiago_no_hand`
- `icub`
- `ur5`

## :copyright: Credits

### :writing_hand: Written by

- [Carlos Mastalli](https://cmastalli.github.io/), The University of Edinburgh :uk:
- [Guilhem Saurel](https://github.com/nim65s), LAAS-CNRS :fr:

### :construction_worker: With contributions from

- [Justin Carpentier](https://jcarpent.github.io/), INRIA :fr:
- [Pierre Fernbach](https://pfernbach.github.io/), LAAS-CNRS :fr:
- [Florent Lamiraux](https://gepettoweb.laas.fr/index.php/Members/FlorentLamiraux), LAAS-CNRS :fr:
- [Wolfgang Merkt](http://www.wolfgangmerkt.com/research/), University of Oxford :uk:
- [Josep Martí Saumell](https://www.iri.upc.edu/staff/jmarti), IRI: CSIC-UPC :es:
