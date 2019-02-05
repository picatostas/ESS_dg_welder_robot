#!/usr/bin/env python
import time
from fsm import fsm, fsm_trans
from welder_class import welder
import numpy as np

welder_states = {"IDLE"       : "0",
    			 "WAIT_CENTER": "1",
    			  "WAIT_TRAJ" : "2",
    			  "WAIT_MARK" : "3",
    			  "DO_TRAJ"   : "4",
    			  "WAIT_POINT": "5",
    			  "WAIT_LASER": "6"}

robot = welder()

# guard functions
def is_start():

	return robot.start

def is_not_start():

	return not robot.start

def is_centered():

	return robot.cnc_pos[:2] == robot.center_points[robot.detect_idx][:2]

def is_not_centered():

	return robot.cnc_pos[:2] != robot.center_points[robot.detect_idx][:2]

def is_marker():

	return robot.cnc_pos[:2] == robot.grid_refs[0][:2]

def is_not_marker():

	return robot.cnc_pos[:2] != robot.grid_refs[0][:2]

def grids_not_received():
	return ( robot.detect_idx != robot.GRIDS_TO_PROCESS and robot.traj_received)

def traj_received():
	return robot.detect_idx == robot.GRIDS_TO_PROCESS

def traj_not_received():

	return not robot.traj_received


def is_traj_completed():

	return robot.traj_completed

def is_not_traj_completed():


	return (not robot.traj_completed)

def is_cnc_last():

	cnc = []
	rob = []
	for i in range(2):
		cnc.append(( '%.2f' %    float(robot.cnc_pos[i])) )
		rob.append(( '%.2f' % float(robot.last_point[i])) )
	return cnc == rob

def is_cnc_not_last():

	cnc = []
	rob = []
	for i in range (2):
		cnc.append(( '%.2f' %    float(robot.cnc_pos[i])) )
		rob.append(( '%.2f' % float(robot.last_point[i])) )
	return cnc != rob

def is_laser_sync():
	return  robot.laser_cmd == robot.laser_status

def is_not_laser_sync():
	return  robot.laser_cmd != robot.laser_status

# transition callbacks


def center():

	print("ROBOT_FSM: Awaiting centering")
	robot.start = False
	robot.start_time = time.time()
	robot.move_to(robot.center_points[robot.detect_idx])

def detection_query():

	if robot.detect_time_en is True:
		robot.detect_time = time.time()
		robot.detect_time_en = False
	print("ROBOT_FSM: Robot centered, awaiting coordinates")
	char = 'q'
	robot.line_query_pub.publish(char)

def place_on_marker():

	print("ROBOT_FSM: Awaiting reference marking")
	robot.move_to(robot.grid_refs[0])

##############################################
# guide for traj indexing robot.traj[grid][blade_number][points][coordinates]
#|                               Traj                          |
#--------------------------------------------------------------  
#|                 Grid               |
# ------------------------------------ x 16
#|       blade       |................|
# -------------------  x 16
#|  start  |   end   |................|
#|  x , y  |  x , y  |................|
#
##############################################

def calculate_point():
	# first Point in a grid 
	if len(robot.traj[0]) == robot.LINES_PER_BLADE and len(robot.traj[0][0]) == 2:
		robot.last_point    = robot.traj[0][0].pop(0)
		robot.next_is_blade = False
		print("GO TO: First point in grid & blade")
	# rest of the points
	else:  
		#new blade, go to closest point
		if len(robot.traj[0][0]) == 2:
			robot.next_is_blade = False
			dist1 = length(robot.last_point, robot.traj[0][0][0])
			dist2 = length(robot.last_point, robot.traj[0][0][1])
			if dist1 < dist2:
				robot.last_point = robot.traj[0][0].pop(0)
				print("GO TO: First point in next blade")
			else:
				robot.last_point = robot.traj[0][0].pop(1)
				print("GO TO: Last  point in next blade")
		# already in blade
		else:
			robot.next_is_blade  = True
			robot.last_point     = robot.traj[0][0].pop(0)
			print("GO TO: Last point in current blade")
	

	for coord in robot.last_point:
		coord = '%.2f' % float(coord)

	print("ROBOT_FSM: Point -> %s"% robot.last_point[:2])

def send_point():
	print("ROBOT_FSM: Laser synced, moving CNC")
	robot.laser_sync = True
	robot.move_to(robot.last_point)

def await_for_traj():
	print("ROBOT_FSM: Awaiting centering")
	robot.move_to(robot.center_points[robot.detect_idx])
	robot.traj_received = False

def update_traj():

	print("ROBOT_FSM: Point reached")
	print("Overall time: %.2f s , welding time: %.2f s"%((time.time() - robot.start_time),(time.time() - robot.welding_time)))
	## End of blade
	if len(robot.traj[0][0]) == 0:
		robot.traj[0].pop(0)
		print("ROBOT_FSM: Blade finished, %d blades remaining" %len(robot.traj[0]))
	## End of grid
	if len(robot.traj[0]) == 0:
		robot.traj.pop(0)
		robot.weld_idx += 1	
		print("ROBOT_FSM:  Grid finished, %d grids  remaining" %len(robot.traj))
	## End of welding half of batch
	if len(robot.traj) == 0:
		print("ROBOT_FSM: Welding completed")
		robot.traj_completed = True
		robot.traj_received  = False
		robot.laser_cmd      = False
		robot.laser_sync     = True
		robot.next_is_blade  = False
		robot.detect_idx	 = 0
	else:
		print("ROBOT_FSM: Welding in progress")
		robot.traj_completed = False

def marker_reached():
	robot.welding_time = time.time()
	print("ROBOT_FSM: Marker Reached, Starting Traj")

welder_trans_tt = [fsm_trans(        welder_states["IDLE"],          is_not_start,        welder_states["IDLE"],            None),
				   fsm_trans(        welder_states["IDLE"],              is_start, welder_states["WAIT_CENTER"], 		  center),
				   fsm_trans( welder_states["WAIT_CENTER"],       is_not_centered, welder_states["WAIT_CENTER"], 		    None),
			       fsm_trans( welder_states["WAIT_CENTER"],           is_centered,   welder_states["WAIT_TRAJ"], detection_query),
			       fsm_trans(   welder_states["WAIT_TRAJ"],    grids_not_received, welder_states["WAIT_CENTER"],  await_for_traj),
			       fsm_trans(   welder_states["WAIT_TRAJ"],     traj_not_received,   welder_states["WAIT_TRAJ"],        	None),
			       fsm_trans(   welder_states["WAIT_TRAJ"],         traj_received,   welder_states["WAIT_MARK"], place_on_marker),
			       fsm_trans(   welder_states["WAIT_MARK"],         is_not_marker,   welder_states["WAIT_MARK"],        	None),
			       fsm_trans(   welder_states["WAIT_MARK"],             is_marker,     welder_states["DO_TRAJ"],  marker_reached),
			       fsm_trans(     welder_states["DO_TRAJ"], is_not_traj_completed,  welder_states["WAIT_LASER"], calculate_point),
			       fsm_trans(  welder_states["WAIT_LASER"],     is_not_laser_sync,  welder_states["WAIT_LASER"],     		None),
			       fsm_trans(  welder_states["WAIT_LASER"],         is_laser_sync,  welder_states["WAIT_POINT"],      send_point),
			       fsm_trans(  welder_states["WAIT_POINT"],       is_cnc_not_last,  welder_states["WAIT_POINT"],            None),
			       fsm_trans(  welder_states["WAIT_POINT"],           is_cnc_last,     welder_states["DO_TRAJ"],     update_traj),
			       fsm_trans(     welder_states["DO_TRAJ"],     is_traj_completed,        welder_states["IDLE"],            None)]	


def length(p0, p1):

	x1 = float(p1[0])
	y1 = float(p1[1])

	return np.sqrt(np.power(y1-p0[1],2) + np.power(x1-p0[0],2))



def welder_fsm_new():
	return fsm(welder_trans_tt,welder_states["IDLE"])