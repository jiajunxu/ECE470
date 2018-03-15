import numpy as np 
import vrep
from scipy.linalg import expm
from helper import get_s, screw2twist, get_quat, screws, M
import time

joint_handles = []

##########################################################
# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
    raise Exception('Failed connecting to remote API server')

# check the error code returned by the vrep functions
# method = 1: get. method = 0: set
def check_error(result, name, method):
	if result != vrep.simx_return_ok:
		if method:
			exception = 'could not get ' + name
			raise Exception(exception)
		else:
			exception = 'could not set ' + name
			raise Exception(exception)

# get the UR3 joint handle
def get_handle(joint_number):
	joint_name = 'UR3_joint' + str(joint_number+1)
	result, joint_handle = vrep.simxGetObjectHandle(clientID, joint_name, vrep.simx_opmode_blocking)
	check_error(result, joint_name, 1)

	return joint_handle

# get the object handle + error checking
def get_obj_handle(handle_name, mode):
	result, handle = vrep.simxGetObjectHandle(clientID, 'ReferenceFrame', mode)
	check_error(result, handle_name, 1)
	return handle

# set the UR3 joint handle + error checking
def set_joint_value(joint_number, theta, mode):
	result, vrep.simxSetJointTargetPosition(clientID, joint_handles[joint_number], theta, mode)
	check_error(result, joint_number, 0)

def set_obj_postition(handle, pos, mode):
	result = vrep.simxSetObjectPosition(clientID, handle, -1, end[0:3, 3], mode)
	check_error(result, handle, 0)

def set_obj_quaternion(handle, quat, mode):
	result = vrep.simxSetObjectQuaternion(clientID, handle, -1, quat, mode)
	check_error(result, handle, 0)
	
# get joint handles
for i in range(0, 6):
	h = get_handle(i)
	joint_handles.append(h)

# target position
pos = 
# target orientation
quat = 

ref_frame0_handle = get_obj_handle('ReferenceFrame0', vrep.simx_opmode_blocking)
set_obj_position(ref_fram0_handle, pos, vrep.simx_opmode_oneshot)
set_obj_orientation(ref_frame0_handle, quat, vrep.simx_opmode_oneshot)


#############################################################
# Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

# Wait two seconds
time.sleep(2)

