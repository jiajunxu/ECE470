import numpy as np 
import vrep
from scipy.linalg import expm
from helper import get_s, screw2twist, get_quat, screws, M, inverse_kinematics, quat2R, p_robot, r_robot, d_robot, get_robot_config, collision_given_theta, plan_path, collision_robot
import time
import lib as lib
import sys
import signal


joint_handles = []
dummy_handles = []
obstacle_handle = []
fingers_handles = []
dummy_diameter = d_robot
p_obstacles = np.array([[],[],[]])
r_obstacles = np.array([[0.06]*11])
r_obstacles = np.concatenate((r_obstacles, np.array([[0.05]*8])), axis=1)
r_obstacles = np.concatenate((r_obstacles, np.array([[0.06]*6])), axis=1)
r_obstacles = np.concatenate((r_obstacles, np.array([[0.05]*8])), axis=1)
r_obstacles = np.concatenate((r_obstacles, np.array([[0.06]*8])), axis=1)
##########################################################
# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
    raise Exception('Failed connecting to remote API server')

def signal_handler(signal, frame):
	for i in range(len(dummy_handles)):
		vrep.simxRemoveObject(clientID, dummy_handles[i], vrep.simx_opmode_oneshot_wait)
	# Stop simulation
	vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

	# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
	vrep.simxGetPingTime(clientID)

	# Close the connection to V-REP
	vrep.simxFinish(clientID)
	sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# check the error code returned by the vrep functions
# method = 1: get. method = 0: set
def check_error(result, name, method):
	if result != vrep.simx_return_ok:
		print("return code: ", result)
		if method:
			exception = 'could not get ' + str(name)
		else:
			exception = 'could not set ' + str(name)
		
		for i in range(len(dummy_handles)):
			vrep.simxRemoveObject(clientID, dummy_handles[i], vrep.simx_opmode_oneshot_wait)
		# Stop simulation
		vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

		# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
		vrep.simxGetPingTime(clientID)

		# Close the connection to V-REP
		vrep.simxFinish(clientID)
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

def get_dummy_handle():
	dummy = "Dummy"
	# dummy_handles.append(get_obj_handle(dummy, vrep.simx_opmode_blocking))
	for i in range(0,p_robot.shape[1]):
		print(i)
		# dummy_handles.append(get_obj_handle(dummy + str(i), vrep.simx_opmode_blocking))
		return_code, handle = vrep.simxCreateDummy(clientID, dummy_diameter[0][i] , None,vrep.simx_opmode_blocking)
		check_error(return_code, "dummy", 1)
		dummy_handles.append(handle)
# set the UR3 joint handle + error checking
def set_joint_value(joint_number, theta, mode):
	result = vrep.simxSetJointTargetPosition(clientID, joint_handles[joint_number], theta, mode)
	check_error(result, "joint "+str(joint_number), 0)

def set_obj_position(handle, pos, mode):
	result = vrep.simxSetObjectPosition(clientID, handle, -1, pos, mode)
	check_error(result, handle, 0)

def set_obj_quaternion(handle, quat, mode):
	result = vrep.simxSetObjectQuaternion(clientID, handle, -1, quat, mode)
	check_error(result, handle, 0)

def get_joint_value(joint_number):
	result, value = vrep.simxGetJointPosition(clientID, joint_handles[joint_number], vrep.simx_opmode_oneshot_wait)
	check_error(result, "joint "+ str(joint_number), 1)
	return value

def get_obj_position(handle, obj_name):
	result, value = vrep.simxGetObjectPosition(clientID, handle, -1, vrep.simx_opmode_oneshot_wait)
	check_error(result, obj_name, 1)
	return value

def get_obj_quat(handle, obj_name):
	result, value = vrep.simxGetObjectQuaternion(clientID, handle, -1, vrep.simx_opmode_oneshot_wait)
	check_error(result, obj_name, 1)
	return value

def get_obstacles_handles():
	dummy = "Dummy"
	obstacle_handle.append(get_obj_handle(dummy, vrep.simx_opmode_blocking))
	for i in range(37-1):
		name = dummy + str(i)
		obstacle_handle.append(get_obj_handle(name, vrep.simx_opmode_blocking))


# get joint handles
for i in range(0, 6):
	h = get_handle(i)
	joint_handles.append(h)
# get the cup handle
cup_handle = get_obj_handle('Cup', vrep.simx_opmode_blocking)
cup_0_handle = get_obj_handle('Cup0', vrep.simx_opmode_blocking)
# get_dummy_handle()
ref_handle = get_obj_handle("ReferenceFrame", vrep.simx_opmode_blocking)
ref_0_handle = get_obj_handle("ReferenceFrame0", vrep.simx_opmode_blocking)
ref_1_handle = get_obj_handle("ReferenceFrame1", vrep.simx_opmode_blocking)
ref_2_handle = get_obj_handle("ReferenceFrame2", vrep.simx_opmode_blocking)
get_obstacles_handles()
fingers_handles.append(get_obj_handle("MicoHand_fingers12_motor1", vrep.simx_opmode_blocking))
fingers_handles.append(get_obj_handle("MicoHand_fingers12_motor2", vrep.simx_opmode_blocking))


vrep.simxSetJointTargetVelocity(clientID, joint_handles[0], 0.005, vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetVelocity(clientID, joint_handles[1], 0.005, vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetVelocity(clientID, joint_handles[0], 0.005, vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetVelocity(clientID, joint_handles[1], 0.005, vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetVelocity(clientID, joint_handles[0], 0.005, vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetVelocity(clientID, joint_handles[1], 0.005, vrep.simx_opmode_oneshot)



# for i in range(len(dummy_handles)):
# 	print(i)
# 	p_j = p_robot[0:3, i].reshape((3,1))
# 	set_obj_position(dummy_handles[i], p_j, vrep.simx_opmode_oneshot_wait)
# time.sleep(200)
#############################################################
#############################################################
#############################################################
# Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)
print("start simulating")

# get cup position and orientation
# result, cup_0_pos = vrep.simxGetObjectPosition(clientID, cup_0_handle, -1, vrep.simx_opmode_oneshot_wait)
# check_error(result, "cup position", 1)
cup_0_pos = get_obj_position(cup_0_handle, "cup 0 pos")
cup_pos = get_obj_position(cup_handle, "cup pos")
cup_quat = get_obj_quat(cup_0_handle, "cup quat")
ref_pos = get_obj_position(ref_handle, "reference frame")
ref_quat = get_obj_quat(ref_handle, "reference frame")
ref_0_pos = get_obj_position(ref_0_handle, "reference frame 0")
ref_0_quat = get_obj_quat(ref_0_handle, "reference frame 0")
ref_1_pos = get_obj_position(ref_1_handle, "reference frame 1")
ref_2_pos = get_obj_position(ref_2_handle, "reference frame 2")




# get the obstacles' positions
for i in range(len(obstacle_handle)):
	pos = get_obj_position(obstacle_handle[i], "ob pos")
	pos = np.array(pos).reshape((3,1))
	p_obstacles = np.concatenate((p_obstacles, pos), axis=1) 

p_obstacles = np.concatenate((p_obstacles, np.array(cup_pos).reshape((3,1))), axis=1)
r_obstacles = np.concatenate((r_obstacles, np.array([[0.1]])),axis=1)
p_obstacles = np.concatenate((p_obstacles, np.array(cup_0_pos).reshape((3,1))), axis=1)
r_obstacles = np.concatenate((r_obstacles, np.array([[0.1]])),axis=1)
# collision_robot(p_robot, r_robot, p_obstacles, r_obstacles)



########################################################
##########################################################
##########################################################
## get the goal pose 1
T_1 = np.zeros((4,4))
R = quat2R(cup_quat)
# R = np.array([[0,-1,0],[0,0,-1],[1,0,0]])
R = np.array([[0,0,-1],[0,1,0],[1,0,0]])
T_1[0:3,0:3] = R
T_1[0:3,3] = np.array(cup_0_pos).reshape((3,))
T_1[0][3] += 0.1
T_1[3,3] = 1
thetas1 = inverse_kinematics(T_1)

# thetas1 = np.array([[np.pi/2],[0],[0],[0],[0],[0]])
current_thetas = np.zeros((6,1))
for i in range(6):
	current_thetas[i][0] = get_joint_value(i)

path = plan_path(current_thetas, thetas1, p_obstacles, r_obstacles)

for i in range(path.shape[1]):
	theta = path[:,i].reshape((6,1))
	for j in range(6):
		set_joint_value(j, theta[j], vrep.simx_opmode_oneshot_wait)
	time.sleep(1.2)

vrep.simxSetJointTargetVelocity(clientID, fingers_handles[0], 0.02, vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetVelocity(clientID, fingers_handles[1], 0.02, vrep.simx_opmode_oneshot)

time.sleep(1)

thetas_old = path[:,path.shape[1]-1].reshape((6,1))
T_1[0][3] -= 0.1
thetas = inverse_kinematics(T_1)
while np.max(np.abs(thetas-thetas_old)) > np.pi/2:
	thetas = inverse_kinematics(T_1)
for i in range(4):
	ts = thetas_old + ((thetas-thetas_old)/4.0) * (i+1)
	for j in range(6):
		set_joint_value(j, ts[j], vrep.simx_opmode_oneshot_wait)
	time.sleep(1.2)


vrep.simxSetJointTargetVelocity(clientID, fingers_handles[0], -0.02, vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetVelocity(clientID, fingers_handles[1], -0.02, vrep.simx_opmode_oneshot)


p_obstacles[:,-1] = np.array([[1000],[1000],[1000]]).reshape((3,))

###########################################
##########################################
#######################################
## set goal 2
print("on goal 2")
T_2 = np.zeros((4,4))
# R = np.array([[0,-1,0],[0,0,-1],[1,0,0]])
R = np.array([[0,0,1],[0,-1,0],[1,0,0]])
T_2[0:3,0:3] = R
T_2[0:3,3] = np.array(ref_1_pos).reshape((3,))
T_2[3,3] = 1
thetas2 = inverse_kinematics(T_2)

# thetas1 = np.array([[np.pi/2],[0],[0],[0],[0],[0]])
current_thetas = np.zeros((6,1))
for i in range(6):
	current_thetas[i][0] = get_joint_value(i)

path = plan_path(current_thetas, thetas2, p_obstacles, r_obstacles)

prev_theta = current_thetas[:]
for i in range(path.shape[1]):
	theta = path[:,i].reshape((6,1))
	for k in range(4):
		ts = prev_theta + (theta-prev_theta)/4.0 * (k+1)
		for j in range(6):
			set_joint_value(j, ts[j], vrep.simx_opmode_oneshot_wait)
		time.sleep(1.2)
	prev_theta = theta

vrep.simxSetJointTargetVelocity(clientID, fingers_handles[0], 0.02, vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetVelocity(clientID, fingers_handles[1], 0.02, vrep.simx_opmode_oneshot)


cup_0_pos = get_obj_position(cup_0_handle, "cup 0 pos")
# p_obstacles[:,-1] = np.array(cup_0_pos).reshape((3,))

###########################################
##########################################
#######################################
## set goal 3
print("on goal 3")
T_2 = np.zeros((4,4))
# R = np.array([[0,-1,0],[0,0,-1],[1,0,0]])
R = np.array([[0,-1,0],[0,0,-1],[1,0,0]])
T_2[0:3,0:3] = R
T_2[0:3,3] = np.array(cup_pos).reshape((3,))
T_2[1][3] += 0.1
T_2[3,3] = 1
thetas2 = inverse_kinematics(T_2)

# thetas1 = np.array([[np.pi/2],[0],[0],[0],[0],[0]])
current_thetas = np.zeros((6,1))
for i in range(6):
	current_thetas[i][0] = get_joint_value(i)

path = plan_path(current_thetas, thetas2, p_obstacles, r_obstacles)

for i in range(path.shape[1]):
	theta = path[:,i].reshape((6,1))
	for j in range(6):
		set_joint_value(j, theta[j], vrep.simx_opmode_oneshot_wait)
	time.sleep(1.2)



thetas_old = path[:,path.shape[1]-1].reshape((6,1))
T_2[1][3] -= 0.1
thetas = inverse_kinematics(T_2)
while np.max(np.abs(thetas-thetas_old)) > np.pi/2:
	thetas = inverse_kinematics(T_2)
for i in range(3):
	ts = thetas_old + ((thetas-thetas_old)/3.0) * (i+1)
	for j in range(6):
		set_joint_value(j, ts[j], vrep.simx_opmode_oneshot_wait)
	time.sleep(1.2)

vrep.simxSetJointTargetVelocity(clientID, fingers_handles[0], -0.02, vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetVelocity(clientID, fingers_handles[1], -0.02, vrep.simx_opmode_oneshot)


p_obstacles[:,-2] = np.array([[1000],[1000],[1000]]).reshape((3,))
###########################################
##########################################
#######################################
## set goal 4
print("on goal 4")
T_2 = np.zeros((4,4))
# R = np.array([[0,-1,0],[0,0,-1],[1,0,0]])
R = np.array([[0,0,1],[0,-1,0],[1,0,0]])
T_2[0:3,0:3] = R
T_2[0:3,3] = np.array(ref_2_pos).reshape((3,))
T_2[3,3] = 1
thetas2 = inverse_kinematics(T_2)

# thetas1 = np.array([[np.pi/2],[0],[0],[0],[0],[0]])
current_thetas = np.zeros((6,1))
for i in range(6):
	current_thetas[i][0] = get_joint_value(i)

path = plan_path(current_thetas, thetas2, p_obstacles, r_obstacles)

for i in range(path.shape[1]):
	theta = path[:,i].reshape((6,1))
	for k in range(4):
		ts = prev_theta + (theta-prev_theta)/4.0 * (k+1)
		for j in range(6):
			set_joint_value(j, ts[j], vrep.simx_opmode_oneshot_wait)
	time.sleep(1.2)

vrep.simxSetJointTargetVelocity(clientID, fingers_handles[0], 0.02, vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetVelocity(clientID, fingers_handles[1], 0.02, vrep.simx_opmode_oneshot)





'''
###########################################
##########################################
#######################################
## set goal 2
T_2 = np.zeros((4,4))
R = quat2R(ref_quat)
# R = np.array([[0,-1,0],[0,0,-1],[1,0,0]])
R = np.array([[0,0,-1],[0,1,0],[1,0,0]])
T_2[0:3,0:3] = R
T_2[0:3,3] = np.array(ref_pos).reshape((3,))
T_2[3,3] = 1
thetas2 = inverse_kinematics(T_2)

# thetas1 = np.array([[np.pi/2],[0],[0],[0],[0],[0]])
current_thetas = np.zeros((6,1))
for i in range(6):
	current_thetas[i][0] = get_joint_value(i)

path = plan_path(current_thetas, thetas2, p_obstacles, r_obstacles)

for i in range(path.shape[1]):
	theta = path[:,i].reshape((6,1))
	for j in range(6):
		set_joint_value(j, theta[j], vrep.simx_opmode_oneshot_wait)
	time.sleep(2)

vrep.simxSetJointTargetVelocity(clientID, fingers_handles[0], 0.02, vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetVelocity(clientID, fingers_handles[1], 0.02, vrep.simx_opmode_oneshot)
'''
##############################
##############################
###############################

# p_r = get_robot_config(thetas1)
# # p_r = p_robot

# for i in range(len(dummy_handles)):
# 	print(i)
# 	p_j = p_r[0:3, i].reshape((3,1))
# 	set_obj_position(dummy_handles[i], p_j, vrep.simx_opmode_oneshot_wait)


# ##### get the goal pose 1
# T_1 = np.zeros((4,4))
# T_1[0:3,0:3] = R
# T_1[0:3,3] = np.array(cup_0_pos).reshape((3,))
# T_1[3,3] = 1
# thetas1 = inverse_kinematics(T_1)
# print("theta: ", thetas1)
# for i in range(6):
# 	print(i)
# 	set_joint_value(i, thetas1[i], vrep.simx_opmode_oneshot_wait)

# Wait two seconds
time.sleep(1000)

for i in range(len(dummy_handles)):
	vrep.simxRemoveObject(clientID, dummy_handles[i], vrep.simx_opmode_oneshot_wait)

# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)

# Close the connection to V-REP
vrep.simxFinish(clientID)
