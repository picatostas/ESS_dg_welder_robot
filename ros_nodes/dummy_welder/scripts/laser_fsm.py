from fsm import fsm, fsm_trans
from welder_class import welder
from welder_fsm import *


laser_states = {"LASER_OFF" : "0",
	            "LASER_ON"  : "1"}


def next_is_blade():

	return robot.is_blade

def next_is_not_blade():

	return not robot.is_blade

def turn_on():
	print("LASER_FSM: Turning on the laser")
	robot.laser_pub.publish('f')
	robot.laser_status = True


def turn_off():
	print("LASER_FSM: Turning off the laser")
	robot.laser_pub.publish('s')
	robot.laser_status = False

laser_trans_tt = [fsm_trans(laser_states["LASER_OFF"],     next_is_blade,  laser_states["LASER_ON"],  turn_on),
			      fsm_trans( laser_states["LASER_ON"], next_is_not_blade, laser_states["LASER_OFF"], turn_off)]

def laser_fsm_new():
	return fsm(laser_trans_tt,laser_states["LASER_OFF"])
