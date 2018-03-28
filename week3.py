import numpy as np 
import vrep
from scipy.linalg import expm
from helper import get_s, screw2twist, get_quat, screws, M, inverse_kinematics, quat2R
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
		print("return code: ", result)
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
	check_error(result, handle, 0)

def set_obj_quaternion(handle, quat, mode):
	result = vrep.simxSetObjectQuaternion(clientID, handle, -1, quat, mode)
	check_error(result, handle, 0)
	
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

thetas = inverse_kinematics(T_2in0)
# thetas = np.array([[-np.pi/4],[np.pi/4],[np.pi/4],[0],[np.pi/4],[np.pi/4]])
ref_frame0_handle = get_obj_handle('ReferenceFrame0', vrep.simx_opmode_blocking)
connection_handle = get_obj_handle('UR3_connection', vrep.simx_opmode_blocking)

hand_handle = get_obj_handle("MicoHand", vrep.simx_opmode_blocking)

fingers_handles = []
fingers_handles.append(get_obj_handle("MicoHand_fingers12_motor1", vrep.simx_opmode_blocking))
fingers_handles.append(get_obj_handle("MicoHand_fingers12_motor2", vrep.simx_opmode_blocking))
# closing_joint_handles = []
# closing_joint_handles.append(get_obj_handle('Barrett_openCloseJoint', vrep.simx_opmode_blocking))
# closing_joint_handles.append(get_obj_handle('Barrett_openCloseJoint0', vrep.simx_opmode_blocking))

cup_handle = get_obj_handle('Cup', vrep.simx_opmode_blocking)


#############################################################
# Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)
print("start simulating")

result, connector_pos = vrep.simxGetObjectPosition(clientID, connection_handle, -1, vrep.simx_opmode_oneshot_wait)
check_error(result, "connector pos", 1)
print("connector postition: \n", connector_pos)

result, connector_quat = vrep.simxGetObjectQuaternion(clientID, connection_handle, -1, vrep.simx_opmode_oneshot_wait)
check_error(result, "connector quat", 1)
print("connector_quat: \n", connector_quat)
# get cup position
result, cup_pos = vrep.simxGetObjectPosition(clientID, cup_handle, -1, vrep.simx_opmode_oneshot_wait)
check_error(result, "cup position", 1)
print("cup postition: ", cup_pos)
result, cup_quat = vrep.simxGetObjectQuaternion(clientID, cup_handle, -1, vrep.simx_opmode_oneshot_wait)
check_error(result, "cup quat", 1)
print("cup quat: ", cup_quat)

target_pos =[-0.1939985603094101, 8.508563041687012e-05, 0.6511231660842896]
target_pos_2 = [0.06673718988895416, -0.3410583734512329, 0.5191779136657715]
cup_pos_1 = [-0.375, 0.06, 0.5717]
target_pos_1 = [-0.19399848580360413, 0.08519570529460907, 0.5534545183181763]
target_quat = [1.519918441772461e-06, -0.7075284719467163, -7.152557373046875e-07, 0.7066848874092102]
target_quat_1 = [-0.0002670586109161377, -0.7076961994171143, 8.082389831542969e-05, 0.7065169215202332]
target_quat_2 = [0.24275845289230347, -0.6646961569786072, 0.29831135272979736, 0.6405135989189148]
T_2in0[0:3, 3] = np.array(target_pos_1).reshape((3,))
R = quat2R(target_quat_1)
T_2in0[0:3, 0:3] = R
thetas = inverse_kinematics(T_2in0)


T_2in0_1 = np.zeros((4,4))
T_2in0_1[3,3] = 1
T_2in0_1[0:3, 0:3] = quat2R(target_quat)
T_2in0_1[0:3, 3] = target_pos
thetas2 = inverse_kinematics(T_2in0_1)

T_2in0_2 = np.zeros((4,4))
T_2in0_2[3,3] = 1
T_2in0_2[0:3, 0:3] = quat2R(target_quat_2)
T_2in0_2[0:3, 3] = target_pos_2
thetas3 = inverse_kinematics(T_2in0_2)

vrep.simxSetJointTargetVelocity(clientID, fingers_handles[0], 0.02, vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetVelocity(clientID, fingers_handles[1], 0.02, vrep.simx_opmode_oneshot)


# Wait two seconds
time.sleep(5)

print("setting frame0 position and orientation")
# vrep.simxSetObjectPosition(clientID, ref_frame0_handle, -1, pos.reshape(3,1), vrep.simx_opmode_oneshot)
set_obj_position(ref_frame0_handle, target_pos_1, vrep.simx_opmode_oneshot_wait)
set_obj_quaternion(ref_frame0_handle, target_quat_1, vrep.simx_opmode_oneshot_wait)
# vrep.simxSetObjectQuaternion(clientID, ref_frame0_handle, -1, quat, vrep.simx_opmode_oneshot)

print("setting joint positions")
for i in range(0, 6):
	set_joint_value(i, thetas[i][0], vrep.simx_opmode_oneshot)

time.sleep(2)
vrep.simxSetJointTargetVelocity(clientID, fingers_handles[0], -0.02, vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetVelocity(clientID, fingers_handles[1], -0.02, vrep.simx_opmode_oneshot)

set_obj_position(ref_frame0_handle, target_pos, vrep.simx_opmode_oneshot_wait)
set_obj_quaternion(ref_frame0_handle, target_quat, vrep.simx_opmode_oneshot_wait)


result, hand_pos = vrep.simxGetObjectPosition(clientID, hand_handle, -1, vrep.simx_opmode_oneshot_wait)
set_obj_position(cup_handle, cup_pos_1, vrep.simx_opmode_blocking)
time.sleep(3)
# thetas = inverse_kinematics(T_2in0_1)
for i in range(0, 6):
	set_joint_value(i, thetas2[i][0], vrep.simx_opmode_oneshot)

time.sleep(2)

set_obj_position(ref_frame0_handle, target_pos_2, vrep.simx_opmode_oneshot_wait)
set_obj_quaternion(ref_frame0_handle, target_quat_2, vrep.simx_opmode_oneshot_wait)

time.sleep(2)
for i in range(0, 6):
	set_joint_value(i, thetas3[i][0], vrep.simx_opmode_oneshot)

time.sleep(2)

vrep.simxSetJointTargetVelocity(clientID, fingers_handles[0], 0.02, vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetVelocity(clientID, fingers_handles[1], 0.02, vrep.simx_opmode_oneshot)

time.sleep(2)

set_obj_position(ref_frame0_handle, [1,1,1], vrep.simx_opmode_oneshot_wait)


time.sleep(10)

# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)

# Close the connection to V-REP
vrep.simxFinish(clientID)
