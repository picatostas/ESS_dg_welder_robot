# Welder Robot

This project is for controlling the welder robot used to weld the aluminum blade grids of the Multi-Grid neutron detector.
Using [OpenCV](https://opencv.org/) and [ROS](https://www.ros.org/) middleware. Implemented in Python and C++.

A description of the contents of each directory in the repository can be found in [documentation/DIRECTORIES.md](documentation/DIRECTORIES.md).

## Getting started

### Prerequisites

* Ubuntu 18.04
* ROS Melodic Morenia

### Building

Using the [catkin](http://wiki.ros.org/catkin) utility.

Copy or make a symbolic link to all the contents from ``dg_welder_robot/ros_nodes/`` of this repository into ``${YOUR_WORKSPACE}/src/``, then run ``catkin_make`` and ``source /devel/setup.bash`` from the root directory of your workspace.

For those nodes running on the computer, copy/link the contents in ``ros_nodes/ubuntu_server/src `` for those running in RPi, the same but ``ros_nodes/raspberry/src ``

How to link the files

`ln -s dg_welder_robot/ros_nodes/ubuntu_server/src ``${YOUR_WORKSPACE}/ `

`ln -s dg_welder_robot/ros_nodes/raspberry/src ``${YOUR_WORKSPACE}/ `

## Running the welder robot

All of the following ROS nodes are located in ``ros_nodes/``, and ``roscore`` must be run prior to starting any of the nodes.

### Line detection

**For operations**
`> rosrun line_detection line_detection_batch_query.py` 

**For tests**
`> rosrun line_detection line_detection_batch_stream.py`

### CNC interface

`> roslaunch cnc_interface smoothie.launch`

Launch files: `smoothie.launch` is for a custom build of shapeoko T using Smoothieboard as a controller. For other axis systems create the `.launch` file. 

*If connected, the CNC will start homing after launching cnc_interface.*

### Dummy welder
	
`> rosrun dummy_welder dummy_welder.py`

The detection regions of interest (ROI) are set in `welder_fsm.py` file in `dummy_welder/`. A ROI is here a coordinate from where the camera is able to spot a bunch of grids---for the prototype welder robot it's specifically 6 ROI for 16 grids and 3 grids per ROI (skipping the overlapped ones).

See description of the finite state machine (FSM) as a draw.io diagram ``'FSM welder-laser.xml'`` in the parent directory.

### Laser ctrl

`> rosrun laser_ctrl laser_ctrl.py`

### Welder node GUI
	
`> rqt --standalone welder_node_gui`

### Spinnaker SDK camera driver

`> roslaunch spinnaker_sdk_camera_driver acquisition.launch`

## Author(s)

* Pablo Costas Franco

## License

See [LICENSE](LICENSE) in parent directory