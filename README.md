## Overview

This package is a c++ library for monte-carlo simulation of highway scenarios, mainly developed for the paper [High-Level Decision Making for Automated Highway Driving via Behavior Cloning](https://ieeexplore.ieee.org/document/9761743).
The python-api is designed for the [interactive highway simulation](https://github.com/einsteinguang/interactive_highway_simulation).


## Installation

This library relies mainly on [Catkin](https://catkin-tools.readthedocs.io/en/latest/index.html) for building and is targeted towards Linux.

At least **C++14** is required.

### Dependencies
Besides [Catkin](https://catkin-tools.readthedocs.io/en/latest/index.html), the dependencies are
* `Boost` (from 1.58)
* [`mrt_cmake_modules`](https://github.com/KIT-MRT/mrt_cmake_modules), a CMake helper library
* `boost-python, python2 or python3` (for python_api)

For Ubuntu, the steps are the following:
* [Set up ROS](http://wiki.ros.org/ROS/Installation), and install at least `rospack`, `catkin` and `mrt_cmake_modules` (e.g. `ros-melodic-rospack`, `ros-melodic-catkin`, `ros-melodic-mrt-cmake-modules`):
```
sudo apt-get install ros-melodic-rospack ros-melodic-catkin ros-melodic-mrt-cmake-modules
```

* Install the dependencies above:
```bash
sudo apt-get install libboost-dev libeigen3-dev libpugixml-dev libpython-dev libboost-python-dev python-catkin-tools
```

**On 16.04 and below**, `mrt_cmake_modules` is not available in ROS and you have to clone it into your workspace (`git clone https://github.com/KIT-MRT/mrt_cmake_modules.git`).

### Building
As usual with Catkin, after you have sourced the ros installation, you have to create a workspace and clone all required packages there. Then you can build.
```shell
source /opt/ros/$ROS_DISTRO/setup.bash
mkdir catkin_ws && cd catkin_ws && mkdir src
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo # build in release mode (or whatever you prefer)
cd src
git clone https://THIS_REPO
cd ..
catkin build
```

### Python3

The python bindings are implemented and tested with python 3. In general it should work with pythion 2.7 and above as well, but I didn't test it yet.


## Citation
```commandline
@ARTICLE{9761743,
  author={Wang, Lingguang and Fernandez, Carlos and Stiller, Christoph},
  journal={IEEE Transactions on Intelligent Vehicles}, 
  title={High-Level Decision Making for Automated Highway Driving via Behavior Cloning}, 
  year={2022},
  pages={1-1},
  doi={10.1109/TIV.2022.3169207}}
```