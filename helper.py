import numpy as np
from scipy.linalg import expm, logm, norm, inv

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

screws = np.concatenate((s1,s2,s3,s4,s5,s6), axis = 1)
M = np.array([[0,0,-1, -0.1940], [0,1,0,0], [1,0,0,0.6511], [0,0,0,1]])

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
        if i >1000:
            print("couldn't find the thetas")
            break
        m1 = expm(bs1*theta[0][0])
        m2 = np.dot(m1, expm(bs2*theta[1][0]))
        m3 = np.dot(m2, expm(bs3*theta[2][0]))
        m4 = np.dot(m3, expm(bs4*theta[3][0]))
        m5 = np.dot(m4, expm(bs5*theta[4][0]))
        m6 = np.dot(m5, expm(bs6*theta[5][0]))

        T = np.dot(m6, M)
        twist = logm(np.dot(T_2in0, inv(T)))

        v = twist2screw(twist)

        if norm(v) < 0.01:
            print("found theta")
            return theta

        j2 = np.dot(adjoint(m1), s2)
        j3 = np.dot(adjoint(m2), s3)
        j4 = np.dot(adjoint(m3), s4)
        j5 = np.dot(adjoint(m4), s5)
        j6 = np.dot(adjoint(m5), s6)
        
        J = np.concatenate((j1,j2,j3,j4,j5,j6), axis=1)

        dtheta = np.dot(inv(J), v)
        theta += dtheta

        i += 1


    return 0
