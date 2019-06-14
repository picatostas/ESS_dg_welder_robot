# Directories

The repository contains the following directories

```bash
> tree -d -L 2
.
├── camera_calibration
│   ├── flir
│   └── raspicam
├── documentation
└── ros_nodes
    ├── cnc_interface
    ├── dummy_welder
    ├── laser_ctrl
    ├── led_ctrl
    ├── line_detection
    ├── raspicam_node
    ├── spinnaker_camera_driver
    ├── welder-gui
    └── welder_node_gui

14 directories
```

## Camera calibration

Directory            	| Function
-------------        	| -------------
flir						| calibration files for the FLIR camera
raspicam					| calibration files for the raspicam camera

## ROS nodes

Directory            	| Function
-------------        	| -------------
cnc interface				| controls the CNC by sending GCODE through serial
dummy welder				| core application, generates trajectories and has 2 finite state machines 
laser ctrl				| controls the activation of the laser
led ctrl					| 
line detection			| detects grid lines
raspicam node				| 
spinnaker sdk camera driver	| publishes FLIR camera in ROS topics
welder gui				| 
welder node gui			| GUI to control welder

## Documentation

Detailed documentation

# Detailed node information

## Line detection

#### Suscribed topics: 
- Camera frames
- Camera info
- Detection query

#### Published topics:
- Blade and ref locations
- Image with lines

## CNC interface

Using a modified version of ROS-GRBL from openautomation, which deals with some exceptions of the serial driver.

[https://github.com/openautomation/ROS-GRBL](https://github.com/openautomation/ROS-GRBL)

#### Suscribed topics: 	
- XYZ coordinates cmd
- CNC stop
	
#### Published topics:
- CNC movement status & position

## Dummy welder

#### Suscribed topics:
- Blade and ref location
- CNC status
- Laser status
- Welder cmd
	
#### Published topics:
- Detection query
- Welder status
- Laser cmd
- CNC cmd and stop
- Welder progress

## Laser ctrl
	
#### Suscribed topics:
- Laser cmd
- Camera frames
- Camera info

#### Published topics:
- Camera frames with crosshair

## Welder node GUI
	
Designed with QtCreator. Reads from the Qt GUI file and runs the GUI within rqt ROS gui framework. `.ui` file will be found in the Node files.
	
#### Suscribed topics:
- Welder status, time, and progress
- Camera frames with crosshair
- Laser cmd

#### Published topics:	
- Welder cmd
 
## Spinnaker SDK camera driver

Node written in C++ by neufieldrobotics. [Source](https://github.com/neufieldrobotics/spinnaker_sdk_camera_driver).
	
#### Suscribed topics:
- 
	
#### Published topics:	
- Camera frames
- Camera info