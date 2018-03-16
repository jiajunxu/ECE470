import numpy as np
import vrep
from scipy.linalg import expm, logm, norm
from helper import get_s, screw2twist, get_quat
import time
import math

joint_handles = []
theta1 = np.array([[np.pi/2],[0],[np.pi/4],[0],[np.pi/6],[0]])
theta2 = np.array([[np.pi/4],[0],[np.pi/2],[0],[np.pi/4],[0]])
theta3 = np.array([[-np.pi/4],[np.pi/4],[np.pi/4],[0],[np.pi/4],[np.pi/4]])

theta = np.concatenate((theta1, theta2, theta3), axis=1)

def check_error(result, name):
	if result != vrep.simx_return_ok:
		exception = 'could not get' + name
		raise Exception(exception)

# Close all open connections (just in case)
vrep.simxFinish(-1)

# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
    raise Exception('Failed connecting to remote API server')

def get_handle(joint_number):
	joint_name = 'UR3_joint' + str(joint_number+1)
	result, joint_handle = vrep.simxGetObjectHandle(clientID, joint_name, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		exception = 'could not get object handle for the ' + str(joint_number) + ' joint'
		raise Exception(exception)

	return joint_handle

def set_joint_value(joint_number, theta):
	vrep.simxSetJointTargetPosition(clientID, joint_handles[joint_number], theta, vrep.simx_opmode_oneshot)
	
result, frame_handle = vrep.simxGetObjectHandle(clientID, 'ReferenceFrame', vrep.simx_opmode_blocking)
check_error(result, 'ReferenceFrame')

result, frame0_handle = vrep.simxGetObjectHandle(clientID, 'ReferenceFrame0', vrep.simx_opmode_blocking)
check_error(result, 'ReferenceFrame0')



# get joint handles
for i in range(0, 6):
	h = get_handle(i)
	joint_handles.append(h)

result, end_handle = vrep.simxGetObjectHandle(clientID, 'UR3_connection', vrep.simx_opmode_blocking)
check_error(result, 'UR3_connection')


# Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

# Wait two seconds
time.sleep(2)



# Forward Kinematics
a1 = np.array([[0],[0],[1]])
q1 = np.array([[0], [0], [0.1045]])
s1 = get_s(a1, q1)

a2 = np.array([[-1],[0],[0]])
q2 = np.array([[-0.1115], [0], [0.1089]])
s2 = get_s(a2, q2)

a3 = np.array([[-1],[0],[0]])
q3 = np.array([[-0.1115], [0], [0.3525]])
s3 = get_s(a3, q3)

a4 = np.array([[-1],[0],[0]])
q4 = np.array([[-0.1115], [0], [0.5658]])
s4 = get_s(a4, q4)

a5 = np.array([[0],[0],[1]])
q5 = np.array([[-0.1122], [0], [0.65]])
s5 = get_s(a5, q5)

a6 = np.array([[-1],[0],[0]])
q6 = np.array([[0.1115], [0], [0.6511]])
s6 = get_s(a6, q6)

M = np.array([[0,0,-1, -0.1940], [0,1,0,0], [1,0,0,0.6511], [0,0,0,1]])

for i in range(3):
	print(i)
	T = expm(screw2twist(s1)*theta[0][i])
	T = np.dot(T, expm(screw2twist(s2)*theta[1][i]))
	T = np.dot(T, expm(screw2twist(s3)*theta[2][i]))
	T = np.dot(T, expm(screw2twist(s4)*theta[3][i]))
	T = np.dot(T, expm(screw2twist(s5)*theta[4][i]))
	T = np.dot(T, expm(screw2twist(s6)*theta[5][i]))

	end = np.dot(T, M)
	# result, position = vrep.simxGetObjectPosition(clientID, frame_handle, -1, vrep.simx_opmode_streaming)
	# check_error(result, "get frame1 position")
	# result = vrep.simxSetObjectPosition(clientID, frame0_handle, -1, position, vrep.simx_opmode_oneshot)
	# check_error(result, "frame1 position")

	quat = get_quat(end[0:3, 0:3])
	print('quat: ', end[0:3, 0:3])
	# result, quat = vrep.simxGetObjectQuaternion(clientID, frame0_handle, -1, vrep.simx_opmode_streaming)
	result = vrep.simxSetObjectQuaternion(clientID, frame0_handle, -1, quat, vrep.simx_opmode_oneshot)
	result = vrep.simxSetObjectPosition(clientID, frame0_handle, -1, end[0:3, 3], vrep.simx_opmode_oneshot)
	# check_error(result, "frame1 position")
	print(quat)
	print("ending position: \n", end[0:3, 3])

	time.sleep(2)


	for j in range(0, 6):
		set_joint_value(j, theta[j][i])
		# time.sleep(1)

	time.sleep(5)



# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)

# Close the connection to V-REP
vrep.simxFinish(clientID)

