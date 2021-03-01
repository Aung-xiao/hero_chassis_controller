# hero_chassis_controller

## Overview

This is the research result of DynamicX team member PiyuanSu.

**Keywords:** RoboMaster, ROS, ros_control,PID


### License

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: PeiYuanSu<br />
Affiliation: [Dynamicx]()<br />
Maintainer: PyuanSu, supeiyuan@gmail.com**

The hero_chassis_controller package has been tested under [ROS] Indigo, Melodic and Noetic on respectively  18.04 and
20.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.


#### Dependencies


- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [rm_description](https://github.com/gdut-dynamic-x/rm_description)
- controller_interface
- forward_command_controller
- hardware_interface
- pluginlib
- control_toolbox
- realtime_tools
- control_msgs

Install dependencies:

    sudo rosdep install --from-paths src

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package
using

	cd catkin_workspace/src
	git clone git@github.com:Aung-xiao/hero_chassis_controller.git
    # git clone https://github.com/Aung-xiao/hero_chassis_controller.git
	cd ../
	catkin build # Actually nothing to build


## Usage


Run the controller with:

	roslaunch hero_chassis_coroller effort_velocity_controller.launch

## Config files

Config file config

* **controllers.yaml**  Params of front_left_velocity_controller back_left_velocity_controller front_right_velocity_controller back_right_velocity_controller and joint_state_controller.



## Launch files

* **fort_velocity_controller.launch:** Hero chassis  simulation and velocity controller

## Bugs & Feature Requests

Please report bugs and request features using
the [Issue Tracker](https://github.com/Aung-xiao/hero_chassis_controller/issues)
[ROS]: http://www.ros.org