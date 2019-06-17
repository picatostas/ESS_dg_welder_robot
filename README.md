# Welder Robot

This project is for controlling the welder robot used to weld the aluminum blade grids of the Multi-Grid neutron detector.
Using [OpenCV](https://opencv.org/) and [ROS](https://www.ros.org/) middleware. Implemented in Python and C++.

A description of the contents of each directory in the repository can be found in [documentation/DIRECTORIES.md](documentation/DIRECTORIES.md).

## Getting started

### Prerequisites

* Ubuntu 18.04
* ROS Melodic Morenia

### Building

catkin

## Running the welder robot

All of the following ROS nodes are located in ros_nodes/

### Line detection

**For operations**
`> rosrun line_detection line_detection_batch_query.py` 

**For tests**
`> rosrun line_detection line_detection_batch_stream.py`

### CNC interface

`> roslaunch cnc_interface smoothie.launch`

Launch files: `smoothie.launch` is for a custom build of shapeoko T using Smoothieboard as a controller. For other axis systems create the `.launch` file. 

*Be careful when launching cnc_interface as the CNC will start homing.*

### Dummy welder
	
`> rosrun dummy_welder dummy_welder.py`

The detection ROI are set in `welder_fsm.py` file in `/dummy_welder`. A ROI is a coordinate from where the camera is able to spot a bunch of grids---in the case of the prototype, 6 ROI for 16 grids, 3 grids per ROI, skipping the overlapped ones.

See description of the FSM used as a draw.io diagram 'FSM welder-laser.xml' in the parent directory

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