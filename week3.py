import numpy as np 
import vrep
from scipy.linalg import expm
from helper import get_s, screw2twist, get_quat, screws, M, inverse_kinematics
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
			exception = 'could not get ' + str(name)
			raise Exception(exception)
		else:
			exception = 'could not set ' + str(name)
			raise Exception(exception)

# get the UR3 joint handle
def get_handle(joint_number):
	joint_name = 'UR3_joint' + str(joint_number+1)
	result, joint_handle = vrep.simxGetObjectHandle(clientID, joint_name, vrep.simx_opmode_blocking)
	check_error(result, joint_name, 1)

	return joint_handle

# get the object handle + error checking
def get_obj_handle(handle_name, mode):
	result, handle = vrep.simxGetObjectHandle(clientID, handle_name, mode)
	check_error(result, handle_name, 1)
	return handle

# set the UR3 joint handle + error checking
def set_joint_value(joint_number, theta, mode):
	result = vrep.simxSetJointTargetPosition(clientID, joint_handles[joint_number], theta, mode)
	# check_error(result, joint_number, 0)

def set_obj_position(handle, pos, mode):
	result = vrep.simxSetObjectPosition(clientID, handle, -1, pos, mode)
	# check_error(result, handle, 0)

def set_obj_quaternion(handle, quat, mode):
	result = vrep.simxSetObjectQuaternion(clientID, handle, -1, quat, mode)
	# check_error(result, handle, 0)
	
# get joint handles
for i in range(0, 6):
	h = get_handle(i)
	joint_handles.append(h)

# target position
pos = np.array([[ 0.2127047],   [0.45317947],  [0.33899255]])
# target orientation
R = np.array([[0.14644661, -0.85355339, -0.5],
 [0.85355339, -0.14644661, 0.5],
 [-0.5, -0.5, 0.70710678]])
quat = get_quat(R)
T_2in0 = np.zeros((4,4))
T_2in0[0:3, 0:3] = R
T_2in0[0:3, 3] = pos.reshape((3,))
T_2in0[3,3] = 1

# thetas = inverse_kinematics(T_2in0)
thetas = np.array([[-np.pi/4],[np.pi/4],[np.pi/4],[0],[np.pi/4],[np.pi/4]])
ref_frame0_handle = get_obj_handle('ReferenceFrame0', vrep.simx_opmode_blocking)


#############################################################
# Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)
print("start simulating")

# Wait two seconds
time.sleep(2)

print("setting frame0 position and orientation")
# vrep.simxSetObjectPosition(clientID, ref_frame0_handle, -1, pos.reshape(3,1), vrep.simx_opmode_oneshot)
set_obj_position(ref_frame0_handle, pos.reshape((3,)), vrep.simx_opmode_oneshot)
set_obj_quaternion(ref_frame0_handle, quat, vrep.simx_opmode_oneshot)
# vrep.simxSetObjectQuaternion(clientID, ref_frame0_handle, -1, quat, vrep.simx_opmode_oneshot)


time.sleep(2)

print("setting joint positions")
for i in range(0, 6):
	set_joint_value(i, thetas[i][0], vrep.simx_opmode_oneshot)


time.sleep(2)

# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)

# Close the connection to V-REP
vrep.simxFinish(clientID)
