# dg_welder_robot

This code is for controlling the welder robot use to weld the aluminum blades of Multi Grid neutron detectors.
Using OpenCV and ROS middleware.

ROS version used : Melodic.

Ubuntu 18.04

Developed by Pablo Costas Franco

## Description of nodes:



### Line_detection
	

Detects grid lines
	
#### Suscribed topics: 

-       Camera_frames

-       Camera info

-       Detection_query

 
#### Published topics:

-       Blade & ref locations

-       Image with lines

 

### cnc_interface
	

Sends GCODE through serial to the CNC

#### Suscribed topics: 	

-       XYZ coordinates cmd

-       CNC stop
	
#### Published topics:
-       CNC movement status & position

### Dummy welder
	

Core of the system, generates trajectories and has 2 FSM to control everything

#### Suscribed topics:

- Blade & ref location

- CNC status

- Laser status

- Welder cmd
	
#### Published topics:

- Detection query

- Welder status

- Laser cmd

- Cnc cmd & stop

- Welder progress

### Laser_ctrl
	

Controls the activation of the laser, checking that the status is the one that has to be in any moment
	
#### Suscribed topics:

- Laser cmd

- Camera frames

- Camera info

#### Published topics:

- Camera frames with crosshair

### Welder_node_gui
	

GUI to control welder. Reads from the Qt Gui file and runs the GUI within rqt ROS gui framework
	
#### Suscribed topics:

- Welder status, time & progress

- Camera frames with crosshair

- Laser cmd

#### Published topics:	

- Welder cmd

 

### Spinnaker_sdk_camera_driver
	

Purpose: Publish FLIR camera in ros topics
	
#### Suscribed topics:

- None
	
#### Published topics:	

- Camera_frames

- Camera info



## How to run each node:



### Line_detection
	

`rosrun line_detection line_detection_batch_query.py` for operation

`rosrun line_detection line_detection_batch_stream.py` for tests

### cnc_interface

Using a modified version of ROS-GRBL from openautomation, which basically deals with some exceptions of the 
serial driver.

https://github.com/openautomation/ROS-GRBL


`roslaunch cnc_interface smoothie.launch`

Launch files: smoothie.launch is for a custom build of shapeoko T using Smoothieboard as a controller.

For other Axis systems create the .launch file 

 
Be careful when launching cnc_interface as the CNC will automatically do the homing.

### Dummy welder
	

`rosrun dummy_welder dummy_welder.py`


The detection ROI are set in welder_fsm.py file in dummy_welder, a ROI is a coordinate from where the camera is able to spot a bunch of grids, in the case of the prototype, 6 roi for 16 grids, 3 grids per roi, skipping the overlapped ones.


### Laser_ctrl	

`rosrun laser_ctrl laser_ctrl.py`

### Welder_node_gui
	

`rqt –standalone welder_node_gui`

The GUI has been design with QtCreator. .ui file will be found in the Node files.


Once everything is running in the GUI should appear the video stream, and when pressing start everything should work.

### Spinnaker_sdk_camera_driver
	
Node written in C++, by neufieldrobotics

https://github.com/neufieldrobotics/spinnaker_sdk_camera_driver

`roslaunch spinnaker_sdk_camera_driver acquisition.launch`


## List of things that needs to be done


### Line_detection
	
Take in account projected locations for the ROI

### cnc_interface


### Dummy welder

- Calculate projection matrix and get real coordinates from the image pixels location.

- Scale the code to process 48 grids instead of 16.


### Laser_ctrl

Change dependency on led_ctrl.py which was running on raspberryPi previous to use FLIR Camera.


### Welder_node_gui
	
Use Qt signals for updating the GUI based on ROS callbacks, as now is crashy.

### Spinnaker_sdk_camera_driver
	

Implement option for changing camera orientation and add parameter in .launch file.
