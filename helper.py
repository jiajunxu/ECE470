import numpy as np
from scipy.linalg import expm, logm, norm, inv
from pyquaternion import Quaternion
import lib as lib
from scipy.spatial import KDTree

num_joints = 6

def skew(w):
    a = w[0]
    b = w[1]
    c = w[2]
    return np.array([(0, -c, b), (c, 0, -a), (-b, a, 0)])

# get the screw axis
def get_s(a, q):
    result = np.zeros((6,))
    result[0:3] = a.reshape((3,))
    aq = -np.dot(skew(a), q)
    result[3:6] = aq.reshape((3,))
    return result.reshape((6, 1))
# UR3 properties
q0 = np.array([[0], [0], [0]])

a1 = np.array([[0],[0],[1]])
q1 = np.array([[0], [0], [0.1045]])
s1 = get_s(a1, q1)

a2 = np.array([[-1],[0],[0]])
q2 = np.array([[-0.1215], [0], [0.1089]])
s2 = get_s(a2, q2)

q2_5 = np.array([[-0.1215], [0], [0.2267]])

a3 = np.array([[-1],[0],[0]])
q3 = np.array([[-0.1115], [0], [0.3525]])
s3 = get_s(a3, q3)
qq3 = np.array([[-0.1215],[0],[0.3525]])

q3_51 = np.array([[-0.028],[0],[0.366]])
q3_5 = np.array([[-0.028],[0], [0.466]])

a4 = np.array([[-1],[0],[0]])
q4 = np.array([[-0.1115], [0], [0.5658]])
s4 = get_s(a4, q4)
qq4 = np.array([[-0.025],[0], [0.555]])


a5 = np.array([[0],[0],[1]])
q5 = np.array([[-0.1122], [0], [0.65]])
s5 = get_s(a5, q5)
qq5 = np.array([[-0.115],[0],[0.565]])

a6 = np.array([[-1],[0],[0]])
q6 = np.array([[-0.1115], [0], [0.6511]])
s6 = get_s(a6, q6)

q7 = np.array([[-0.194],[0],[0.6511]])

qEnd = np.array([[-0.225],[0],[0.65]])

p_robot = np.concatenate((q0,q1,q2,q2_5,qq3,q3_51,q3_5,qq4,qq5,q6,q7), axis=1)
d_robot = np.array([[.1,.13,.15,.12,.12,.1, .1,.1,.1,.1,.1]])
r_robot = d_robot/2
# r_robot = np.array([[.01,.09,.11,.08,.08,.1, .1,.1,.1,.1,.1]])
# screws = np.concatenate((s1,s2,s2,s3,s3,s3,s4,s5,s6,s6), axis = 1)
screws = np.concatenate((s1,s2,s3,s4,s5,s6), axis=1)
M = np.array([[0,0,-1, -0.3440], [0,1,0,0], [1,0,0,0.6511], [0,0,0,1]])

# position of the joints with 1's appended at the bottom, 4x10
p_joints = np.concatenate((p_robot, np.ones((1,p_robot.shape[1]))), axis=0)


def skew(w):
    a = w[0]
    b = w[1]
    c = w[2]
    return np.array([(0, -c, b), (c, 0, -a), (-b, a, 0)])

# get the screw axis
def get_s(a, q):
    result = np.zeros((6,))
    result[0:3] = a.reshape((3,))
    aq = -np.dot(skew(a), q)
    result[3:6] = aq.reshape((3,))
    return result.reshape((6, 1))

# get the twist from a screw axis: s -> [s]
def screw2twist(s):
    result = np.zeros((4, 4))
    result[0:3,0:3] = skew(s[0:3])
    result[0:3, 3] = s[3:6].reshape(3,)
    return result

# get the adjoint matrix
def adjoint(M):
    R = M[0:3, 0:3]
    p = M[0:3, 3]
    result = np.zeros((6, 6))
    result[0:3, 0:3] = R
    result[3:6, 0:3] = np.dot(skew(p), R)
    result[3:6, 3:6] = R
    return result

# get the quaternion
def get_quat(R):
    W = logm(R)
    w1 = W[2][1]
    w2 = W[0][2]
    w3 = W[1][0]
    w = np.array([[w1],[w2],[w3]])
    a = w / norm(w)
    theta = norm(w)
    q0 = np.cos(theta/2)
    q123 = a * np.sin(theta/2)
    return q123[0], q123[1], q123[2], q0

def quat2R(quat):
    q0 = quat[3]
    q1 = quat[0]
    q2 = quat[1]
    q3 = quat[2]
    quat = Quaternion([q0, q1, q2, q3])
    return quat.rotation_matrix



# get the screw axis from the given twist
def twist2screw(twist):
    v = np.zeros((6, 1))
    v[0][0] = twist[2][1]
    v[1][0] = twist[0][2]
    v[2][0] = twist[1][0]
    v[3][0] = twist[0][3]
    v[4][0] = twist[1][3]
    v[5][0] = twist[2][3]
    return v

def inverse_kinematics(T_2in0):
    theta = np.random.rand(6, 1)

    bs1 = screw2twist(s1)
    bs2 = screw2twist(s2)
    bs3 = screw2twist(s3)
    bs4 = screw2twist(s4)
    bs5 = screw2twist(s5)
    bs6 = screw2twist(s6)

    j1 = s1[:]

    i = 0
    while True:
        if i >10000:
            print("couldn't find the thetas")
            break
        m1 = expm(bs1*theta[0][0])
        m2 = np.dot(m1, expm(bs2*theta[1][0]))
        m3 = np.dot(m2, expm(bs3*theta[2][0]))
        m4 = np.dot(m3, expm(bs4*theta[3][0]))
        m5 = np.dot(m4, expm(bs5*theta[4][0]))
        m6 = np.dot(m5, expm(bs6*theta[5][0]))

        T = np.dot(m6, M)
        invt = inv(T)
        twist = logm(np.dot(T_2in0, invt))

        v = twist2screw(twist)

        # print(norm(v))
        if norm(v) < 0.01:
            print("found theta")
            return theta

        j2 = np.dot(adjoint(m1), s2)
        j3 = np.dot(adjoint(m2), s3)
        j4 = np.dot(adjoint(m3), s4)
        j5 = np.dot(adjoint(m4), s5)
        j6 = np.dot(adjoint(m5), s6)
        
        J = np.concatenate((j1,j2,j3,j4,j5,j6), axis=1)

        # dtheta = np.dot(inv(J), v)
        dtheta = np.dot(np.dot(inv(np.dot(J.T, J)+0.01*np.eye(6)), J.T), v)
        theta += dtheta

        i += 1


    return 0


def get_robot_config(theta):
    # s1,s2,s2,s3,s3,s3,s4,s5,s6,s6
    config = np.zeros((4, p_robot.shape[1]))
    config[:,0] = p_joints[:,0]
    config[:,1] = p_joints[:,1]
    Ts = lib.sequential_Ts(screws, theta)
    T = [Ts[0],Ts[1], Ts[1], Ts[2],Ts[2],Ts[2],Ts[3],Ts[4],Ts[5],Ts[5]]
    for i in range(len(T)):
        config[:,i+1] = np.dot(T[i], p_joints[:,i+1].reshape((4,1))).reshape((4,))

    return config[:3,:]



def within_joint_limit(theta):
    for i in range(theta.shape[0]):
        if theta[i] < -np.pi or theta[i] > np.pi:
            return 0
    return 1

def collision_2_points(p1, p2, r1, r2):
    if norm(p1-p2) <= r1 + r2:
        return 1
    else:
        return 0


def collision_robot(p_r, p_o, r_r, r_o):
    for i in range(p_r.shape[1]):
        pi = p_r[:,i].reshape((3,1))
        ri = r_r[0][i]
        # collision with obstacles
        for j in range(p_o.shape[1]):
            pj = p_o[:,j].reshape((3,1))
            rj = r_o[0][j]
            if collision_2_points(pi, pj, ri, rj):
                return 1
        # self collision
        for j in range(i+2, p_r.shape[1]):
            pj = p_r[:,j].reshape((3,1))
            rj = r_r[0][j]
            if collision_2_points(pi, pj, ri, rj):
                return 1
        # collision with the ground
        if pi[2][0] < ri and i != 0 and i != 1:
                return 1
    return 0


def collision_given_theta(theta, p_o, r_o):
    p_r = get_robot_config(theta)
    return collision_robot(p_r, p_o, r_robot, r_o)


"""
find the closest node in the tree given the new theta

theta_start: N x K

neighbor_start: K x 1
"""
def get_closest_node(theta, theta_start, theta_goal):
    t_start = KDTree(theta_start)
    t_goal = KDTree(theta_goal)
    neighbor_start = t_start.query(theta.T)[1]
    # print("neighbor_start: ", neighbor_start)
    # print("theta_start: ", theta_start)
    neighbor_goal = t_goal.query(theta.T)[1]
    # print("neighbor_goal: ", neighbor_goal)
    # print("theta_goal: ", theta_goal)
    return theta_start[neighbor_start[0]].reshape(6,1), theta_goal[neighbor_goal[0]].reshape(6,1)
 

"""
detect collision along the line from theta0 to theta1
"""
def line_collision(theta_a, theta_b, p_o, r_o):
    for s in range(1, 100, 4):
        s /= 100
        theta = (1-s)*theta_a + s*theta_b
        if collision_given_theta(theta, p_o, r_o):
            return 1
    return 0

"""
return the path of nodes given the theta
""" 
def get_path(theta, T_start, T_goal):
    start = []
    curr = theta[:]
    while True:
        curr = T_start[tuple(curr.reshape((num_joints,)))]
        if curr is None:
            break
        else:
            start.append(curr)
    start.reverse()
    mat_start = np.asarray(start)
    mat_start = mat_start.reshape((mat_start.shape[0],mat_start.shape[1])).T
    goal = []
    curr = theta[:]
    while True:
        curr = T_goal[tuple(curr.reshape((num_joints,)))]
        if curr is None:
            break
        else:
            goal.append(curr)
    mat_goal = np.asarray(goal)
    mat_goal = mat_goal.reshape((mat_goal.shape[0],mat_goal.shape[1])).T
    
    return np.concatenate((mat_start, theta, mat_goal), axis=1)


# s_config: starting config
# g_config: goal config
def plan_path(start_config, goal_config, p_o, r_o):
    theta_start = start_config[:]
    theta_goal = goal_config[:]
    T_start = {}
    T_goal = {}
    T_start[tuple(theta_start.reshape((6,)))] = None
    T_goal[tuple(theta_goal.reshape((6,)))] = None

    iterations = 0
    while iterations < 10000:
        iterations += 1
        # sample a random, within-limit, collision-free point
        theta = 2*np.pi*np.random.rand(6, 1)-np.pi
        while not within_joint_limit(theta) and not collision_given_theta(theta, p_o, r_o):
            theta = 2*np.pi*np.random.rand(6, 1)-np.pi

        # find closest node to T-start and T-goal
        node_start, node_goal = get_closest_node(theta, theta_start.T, theta_goal.T)
        # print("start node: ", node_start.tolist())
        # print("goal node: ", node_goal.tolist())

        flag = 0
        # if no collision from closest theta to the current theta, add the current theta to the start tree
        if not line_collision(node_start, theta, p_o, r_o):
            print("adding the the start tree\n")
            theta_start = np.concatenate((theta_start, theta), axis = 1)
            T_start[tuple(theta.reshape((num_joints,)))] = node_start
            flag += 1
        if not line_collision(node_goal, theta, p_o, r_o):
            print("adding to the goal tree\n")
            theta_goal = np.concatenate((theta_goal, theta), axis = 1)
            T_goal[tuple(theta.reshape((num_joints,)))] = node_goal
            flag += 1
        # if theta connects both trees without collision, we have found the solution
        if flag == 2:
            path = get_path(theta, T_start, T_goal)
            print("found path: ", path.tolist())
            return path
            




