import vrep
import time
import numpy as np

move_all_joint = True
move_cup = ~move_all_joint

joint_handles = []
joint_values = []

def get_handle(joint_number):
	joint_name = 'UR3_joint' + str(joint_number+1)
	result, joint_handle = vrep.simxGetObjectHandle(clientID, joint_name, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		exception = 'could not get object handle for the ' + str(joint_number) + ' joint'
		raise Exception(exception)

	return joint_handle

def get_joint_value(joint_number):
	result, theta = vrep.simxGetJointPosition(clientID, joint_handles[joint_number], vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		exception = 'could not get' + str(joint_number) + ' joint variable'
		raise Exception('could not get first joint variable')
	print('current value of ', str(joint_number), ' joint variable: theta = {:f}'.format(theta))

	return theta

def set_joint_value(joint_number, theta):
	vrep.simxSetJointTargetPosition(clientID, joint_handles[joint_number], joint_values[joint_number] + theta, vrep.simx_opmode_oneshot)
	

# Close all open connections (just in case)
vrep.simxFinish(-1)

# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
    raise Exception('Failed connecting to remote API server')

# get joint handles
for i in range(0, 6):
	h = get_handle(i)
	joint_handles.append(h)

# Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

# Wait two seconds
time.sleep(2)

# get joint values
for i in range(0, 6):
	t = get_joint_value(i)
	joint_values.append(t)

# set joint values
# if not move_all_joint:
# 	set_joint_value(2, np.pi/2)
# 	time.sleep(2)
# 	set_joint_value(0, -np.pi/2)
# else:
# 	for i in range(0, 6):
# 		set_joint_value(i, 0)
# 		time.sleep(1)

set_joint_value(0, np.pi/2)


time.sleep(10)
# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)

# Close the connection to V-REP
vrep.simxFinish(clientID)