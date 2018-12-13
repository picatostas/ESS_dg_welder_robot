#!/usr/bin/env python

from fsm import fsm, fsm_trans
from welder_class import welder
import numpy as np

welder_states = {"IDLE"       : "0",
    			 "WAIT_CENTER": "1",
    			  "WAIT_TRAJ" : "2",
    			  "WAIT_MARK" : "3",
    			  "DO_TRAJ"   : "4",
    			  "WAIT_POINT": "5"}

robot = welder()

# guard functions
def is_start():

	return robot.start

def is_not_start():

	return not robot.start

def is_centered():

	return robot.cnc_pos[:2] == robot.center_point[:2]

def is_not_centered():

	return robot.cnc_pos[:2] != robot.center_point[:2]

def is_marker():

	return robot.cnc_pos[:2] == robot.grid_ref[:2]

def is_not_marker():

	return robot.cnc_pos[:2] != robot.grid_ref[:2]

def traj_received():

	return robot.traj_received

def traj_not_received():

	return not robot.traj_received


def is_traj_completed():

	return robot.traj_completed

def is_not_traj_completed():

	return not robot.traj_completed

def is_cnc_last():

	cnc = []
	rob = []
	for i in range(2):
		cnc.append(( '%.2f' % robot.cnc_pos[0]) )
		rob.append(( '%.2f' % robot.last_point[0]) )
	
	return cnc == rob

def is_cnc_not_last():

	cnc = []
	rob = []
	for i in range (2):
		cnc.append(( '%.2f' % robot.cnc_pos[0]) )
		rob.append(( '%.2f' % robot.last_point[0]) )
	
	return cnc != rob


# transition callbacks

def center():

	print("ROBOT_FSM : Robot started, awaiting centering")
	robot.start = False
	robot.move_to(robot.center_point,0)

def detection_query():

	print("ROBOT_FSM : Robot centered, awating coordinates")
	char = 'q'
	robot.line_query_pub.publish(char)

def place_on_marker():

	print("ROBOT_FSM : Robot centered, awaiting reference marking")
	robot.move_to(robot.grid_ref,0)

##############################################
# guide for traj indexing robot.traj[blade_number][points][coordinates]
#|                 traj               |
# ------------------------------------
#|       blade       |................|
# -------------------  x 16
#|  start  |   end   |................|
#|  x , y  |  x , y  |................|
#
##############################################

def calculate_point():

	robot.is_blade = False
	if len(robot.traj) == 16 and len(robot.traj[0]) == 2:
		robot.is_blade = False
		print("ROBOT_FSM : Trajectory initiated")
		robot.last_point = robot.traj[0].pop(0)
	else:
		robot.is_blade = False
		print("ROBOT_FSM : Next point to process")
		if len(robot.traj[0]) > 1:
			dist1 = length(robot.last_point, robot.traj[0][0])
			dist2 = length(robot.last_point, robot.traj[0][1])
			#print("dist to point 1 of next blade :" + str(dist1))
			#print("dist to point 2 of next blade :" + str(dist2))
			print("ROBOT_FSM: Welding in grid")
			robot.is_blade = True
			if dist1 < dist2:
				robot.last_point = robot.traj[0].pop(0)
			else:
				robot.last_point = robot.traj[0].pop(1)
		else:
			robot.is_blade = False
			robot.last_point = robot.traj[0].pop(0)
	#print("Point type : " + str(type(robot.last_point)))
	for coord in robot.last_point:
		coord = '%.2f' % coord
		#print(coord)		
	print("ROBOT_FSM: Point -> %s"% robot.last_point[:2])
	robot.move_to(robot.last_point,0)

def update_traj():

	print("ROBOT_FSM: Point reached")
	if len(robot.traj[0]) == 0:
		robot.traj.pop(0)
		print("ROBOT_FSM: blade finished, %d blades remaining" %len(robot.traj))
	if len(robot.traj) == 0:
		print("ROBOT_FSM : Trajectory completed")
		robot.move_to(robot.center_point,0)
		#robot.start = False
		robot.traj_completed = True
	else:
		print("ROBOT_FSM : Trajectory on progress")
		robot.traj_completed = False


welder_trans_tt = [fsm_trans(        welder_states["IDLE"],          is_not_start,        welder_states["IDLE"],            None),
				   fsm_trans(        welder_states["IDLE"],              is_start, welder_states["WAIT_CENTER"], 		  center),
				   fsm_trans( welder_states["WAIT_CENTER"],       is_not_centered, welder_states["WAIT_CENTER"], 		    None),
			       fsm_trans( welder_states["WAIT_CENTER"],           is_centered,   welder_states["WAIT_TRAJ"], detection_query),
			       fsm_trans(   welder_states["WAIT_TRAJ"],     traj_not_received,   welder_states["WAIT_TRAJ"],        	None),
			       fsm_trans(   welder_states["WAIT_TRAJ"],         traj_received,   welder_states["WAIT_MARK"], place_on_marker),
			       fsm_trans(   welder_states["WAIT_MARK"],         is_not_marker,   welder_states["WAIT_MARK"],        	None),
			       fsm_trans(   welder_states["WAIT_MARK"],             is_marker,     welder_states["DO_TRAJ"],        	None),
			       fsm_trans(     welder_states["DO_TRAJ"], is_not_traj_completed,  welder_states["WAIT_POINT"], calculate_point),
			       fsm_trans(  welder_states["WAIT_POINT"],       is_cnc_not_last,  welder_states["WAIT_POINT"],            None),
			       fsm_trans(  welder_states["WAIT_POINT"],           is_cnc_last,     welder_states["DO_TRAJ"],     update_traj),
			       fsm_trans(     welder_states["DO_TRAJ"],     is_traj_completed,        welder_states["IDLE"],            None)]	


def length(p0, p1):

	x1 = float(p1[0])
	y1 = float(p1[1])

	return np.sqrt(np.power(y1-p0[1],2) + np.power(x1-p0[0],2))



def welder_fsm_new():
	return fsm(welder_trans_tt,welder_states["IDLE"])