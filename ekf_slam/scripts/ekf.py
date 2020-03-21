# READ THIS BEFORE IMPLEMENTING YOUR FILTER!
# This is a model template for your landmark-based SLAM filter.
# In this model, a known-correspondences EKF filter is implemented. But feel free to implement an
# unknown correspondence one :)
# Notice that the Python implementation might be slow, and parameters might be changed by another program. Therefore, making deep copies is necessary


import numpy as np
import math
import csv

def wrap_angle_Pi(x):
    max = np.pi
    min = -np.pi
    return min+ (max - min + ( (x - min) % (max - min) ) ) % (max - min)


def initialize_ekf_sigma(landmark_num):
    #Initialize sigma for EKF
    #u_t: [v, w]
    #miu_t_1: [x, y, theta, mx1, my1 ... ].T
    #z_t = array of [range, bearing, id]
    #z_t_i = 3x1
    #----------------------------params:
    # delta t
    sigma = np.zeros((2 * landmark_num + 3, 2 * landmark_num + 3))
    for i in range(3, 2 * landmark_num + 3):
        sigma[i, i] = 10000.0
    return sigma

def initialize_fk(landmark_num):
    F_k = np.zeros(( 3, 3 + 2 * landmark_num))
    for i in range(3):
        F_k[i, i] = 1.0
    return F_k


def theta_2_SO2(theta):
    """
    Input theta is a scalar vector
    Output: SO2 Rotation matrix
    """
    c = np.cos(theta)
    s = np.sin(theta)
    R = np.array([[c, -s],
                  [s, c]])
    return R

def pose_twist_to_SE2(pose_twist):
    '''
    In this program, we represent a robot's pose in a twist-like way: [theta,x,y]. However, this
    is NOT an actual twist, and the corresponding SE2 does not follow the standard "twist way".
    Args:
        pose_twist: [theta, x, y]
    Output:
        SE2 [R p; 0 1]
    '''
    theta = pose_twist[0]
    x = pose_twist[1]
    y = pose_twist[2]
    R = theta_2_SO2(theta)
    p = np.array([x,y]).reshape(2,1)
    T = np.vstack([np.hstack([R, p]), np.array([[0, 0, 1]]) ])
    return T

def twist_to_SE2(twist):
    """
    Input: twist in unit time [theta, x, y], which is screw x theta. screw is [1, vx, vy].
    Output: T matrix SE2
        T = np.array([[R, P],
                      [0, 1]])
    """
    theta = twist[0]
    R = theta_2_SO2(theta)
    x = twist[1]
    y = twist[2]

    if theta == 0:
        G_theta_v = np.array([[x], [y]])
    else:
        vx = x/theta
        vy = y/theta
        R = theta_2_SO2(theta)
        w = 1 # this is the "omega axis "
        w_hat = hat(w)
        G_theta = np.identity(2)*theta + (1.0 - np.cos(theta))*w_hat + (theta - np.sin(theta))* w_hat.dot(w_hat)
        G_theta_v = G_theta.dot(np.array([[vx], [vy]]))

    T = np.vstack([np.hstack([R, G_theta_v]), np.array([[0, 0, 1]]) ])
    return T


def hat(w):
    """
    Returns the hatted version of a scalar w. The result is in so(3)
    """
    w_hat = np.array([[0.0, -w],
                      [w, 0.0]])
    return w_hat

def SO2_2_theta(R):
    """
    Input R is SO2 Rotation matrix.
    Sample Input: [[cos(theta) -sin(theta)],
        [sin(theta) cos(theta)]]
        Output is the angle of R
    Sample output: theta = pi/2
    """
    R = np.array(R)
    theta = np.arctan2(R[1,0], R[0,0])
    return theta

def SE2_to_Pose_Twist(T):
    '''
    In this program, we represent a robot's pose in a twist-like way: [theta,x,y]. However, this
    is NOT an actual twist, and the corresponding SE2 does not follow the standard "twist way".
    Args:
        T = np.array([[R, P],
           [0, 1]])
    Output: twist representation of the robot pose [theta, x, y]
    '''
    T = np.copy(T)
    R= T[: 2, : 2]
    p = T[:2, -1]
    theta = SO2_2_theta(R)
    pose_twist = np.append(theta, p)
    return pose_twist


def SE2_to_Twist(T):
    """
    Input: T matrix SE2
            T = np.array([[R, P],
                       [0, 1]])
    Output: twist in unit time [theta, x, y], which is screw x theta. screw is [1, vx, vy].
    """
    T = np.copy(T)
    R= T[: 2, : 2]
    p = T[:2, -1]
    theta = SO2_2_theta(R)
    if abs(theta)<1e-6:
        xy = p
    else:
        w = 1 #this is the omega axis
        w_hat = hat(w)
        G_minus1 = 1/theta * np.identity(2) - 1.0/2.0*w_hat + ( 1/theta - 1.0/2.0*(1.0/np.tan(theta/2.0)))*w_hat.dot(w_hat)
        v = G_minus1.dot(np.array([[p[0]], [p[1]]]))
        xy = np.array([v[0], v[1]])* theta

    twist = np.append(theta, xy)
    return twist


SIGMA_V = 0.05
SIGMA_W = 0.01
SIGMA_RANGE = 0.1
SIGMA_BEARING = 0.1
TOTAL_STEP_NUM = 1000


class ekf_object():

    def __init__(self, x = 0.0, y = 0.0, theta = 0.0, landmark_num = 1, delta_t=0.01):
        #number of landmarks:
        self.landmark_num = landmark_num
        #miu = ([2*n + 3]x1) matrix [x,y,theta]
        self.miu = np.zeros((2 * landmark_num + 3, 1))
        self.miu[:3, 0] = np.array([x, y, theta])
        #3+2xn square matrix. large sigma means I don't trust the initial value. The initial covariance is 0 for x, y, theta, and inf for landmark poses
        self.sigma = initialize_ekf_sigma(self.landmark_num)
        #delta_t: time step
        self.delta_t = delta_t
        #measurement covariance matrix (range, bearing)
        self.Q = np.array([[SIGMA_RANGE**2, 0.0],
                           [0.0, SIGMA_BEARING**2]])

    def motion_predict(self, u_t, delta_t):
        # alphas: motion covariance weights.
        # args:
        #     u_t: [v_t, w_t]
        #     delta_t: double
        alphas = np.array([1, 0, 0, 1])
        theta = self.miu[2,0]
        v_t = u_t[0]
        w_t = u_t[1]

        F_k = initialize_fk(self.landmark_num)

        rot = self.delta_t * u_t[1]
        halfRot = rot / 2.0
        trans = u_t[0] * self.delta_t


        u_twist = np.array([w_t, v_t, 0])*delta_t

        T_bb1 = twist_to_SE2(u_twist)
        pose_twist = np.array([self.miu[2,0], self.miu[0,0], self.miu[1,0]])
        T_sb = pose_twist_to_SE2(pose_twist)
        T_sb1 = T_sb.dot(T_bb1)
        new_pose = SE2_to_Pose_Twist(T_sb1)

        self.miu[0,0] = new_pose[1]
        self.miu[1,0] = new_pose[2]
        self.miu[2,0] = new_pose[0]

        g_t = np.array([[0.0, 0.0, trans * -1.0*np.sin(theta + halfRot)],
                        [0.0, 0.0, trans * np.cos(theta + halfRot)],
                        [0.0, 0.0, 0.0] ])
        G_t = np.identity(3 + 2*self.landmark_num) + (F_k.T).dot(g_t).dot(F_k)


        M_t = np.array([[SIGMA_V**2, 0],
                        [0, SIGMA_W**2]])

        V_t = np.array([[np.cos(theta + halfRot), -0.5 * np.sin(theta + halfRot)],
                        [np.sin(theta + halfRot), 0.5 * np.cos(theta + halfRot)],
                        [0, 1.0]])
        R_t = V_t.dot(M_t).dot(V_t.T)

        # sigma_t_bar = (G_t.dot(self.sigma)).dot(G_t.T) + R_t
        sigma_t_bar = (G_t.dot(self.sigma)).dot(G_t.T) + ((F_k.T).dot(R_t)).dot(F_k)
        self.sigma = sigma_t_bar


    def measurement_update(self, z_t):
        #z_t is [z_t_1, z_t_2 ... ], in which z_t_i = [r_i, phi_i, j_i]. j_i is the landmark index [0,1 ... landmark_num -1]
        miu_copy = np.copy(self.miu)
        total_innovation = np.zeros(miu_copy.shape)


        for z_t_i in z_t:
            z_t_i = z_t_i.reshape(3,1)
            j = int(z_t_i[2, 0])            #j might be a float type

            if j >= self.landmark_num:  #we only add landmarks that we know.
                continue

            if miu_copy[3 + 2*j, 0] == 0 and miu_copy[3 + 2*j + 1, 0]==0:     #if the landmark has never been seen before, add it to the state vector
                r_j = z_t_i[0, 0]
                phi_j = z_t_i[1, 0]
                observed_landmark_pos = np.array([r_j * np.cos(phi_j + miu_copy[2, 0]), r_j * np.sin(phi_j + miu_copy[2, 0])]) + miu_copy[:2,0]
                miu_copy[3 + 2*j: 3 + 2*j+2, 0] = observed_landmark_pos
                self.miu[3 + 2*j: 3 + 2*j+2, 0] = observed_landmark_pos

        # ---------------- Step 1: Measurement update -----------------#
            F_1 = np.append( np.identity(3), np.zeros((2,3)) , axis=0)
            F_2 = np.append( np.zeros((3,2)), np.identity(2), axis=0)
            q = np.power(np.linalg.norm(miu_copy[3 + 2*j: 3 + 2*j+2, 0] - miu_copy[0:2, 0]), 2)    #landmark to robot distance ^2
            #make sure angle is wrapped into [-pi, pi], and set z[2] = 0
            m = np.copy(miu_copy[3 + 2*j: 3 + 2*j+2, 0])
            #this is the predicted range and bearing, note that this is 2x1
            z_t_i_hat = np.array([[np.sqrt(q)],
                                  [wrap_angle_Pi( np.arctan2( m[1] - miu_copy[1, 0], m[0] - miu_copy[0, 0]) - miu_copy[2, 0] )]
                                 ])

        # ------ Step 2: Linearize Measurement Model with Jacobian ------#
            #F_x makes the landmark z_t_i hatbecome a state.
            #        1 0 0  0 ...... 0   0 0   0 ...... 0
            #        0 1 0  0 ...... 0   0 0   0 ...... 0
            # F_x =  0 0 1  0 ...... 0   0 0   0 ...... 0
            #        0 0 0  0 ...... 0   1 0   0 ...... 0
            #        0 0 0  0 ...... 0   0 1   0 ...... 0
            #                   2*j           3+2*landmark_num - (3+2*j+2)
            #          -delta_x/sqrt_q  -delta_y/sqrt_q  0  delta_x/sqrt_q  delta_y/q
            # H_low =   delta_y/q   -delta_x/q  -1  -delta_y/q  delta_x/q
            # delta_x is the x coordinate difference bw updated landmark and robot pose


            F_x_j = np.hstack( (F_1, np.zeros((5, 2*j)), F_2, np.zeros((5, 2*self.landmark_num - 2*j-2)) ) )

            delta_x = miu_copy[3 + 2*j, 0] - miu_copy[0, 0]
            delta_y = miu_copy[3 + 2*j +1, 0] - miu_copy[1, 0]
            H_1 = np.array([-delta_x/np.sqrt(q), -delta_y/np.sqrt(q), 0.0, delta_x/np.sqrt(q), delta_y/np.sqrt(q)])
            H_2 = np.array([delta_y/q, -delta_x/q, -1.0, -delta_y/q, delta_x/q])
            # H_3 = np.array([0, 0, 0, 0, 0])       #since we are dealing with 2d with no landmark signature, we don't need these zeros.
            H_i_t = np.array([H_1, H_2]).dot(F_x_j)

        # ---------------- Step 3: Kalman gain update -----------------#
            S_i_t = H_i_t.dot(self.sigma).dot(H_i_t.T) + self.Q
            # K_i_t = ( self.sigma.dot(H_i_t.T) ).dot( np.linalg.pinv(S_i_t, rcond=1e-14) )
            K_i_t = ( self.sigma.dot(H_i_t.T) ).dot( np.linalg.inv(S_i_t) )


        # ------------------- Step 4: mean update ---------------------#
            z_t = z_t_i[0:2, 0].reshape(2,1)     #since we are dealing with 2d with no landmark signature

            z_t_difference = z_t - z_t_i_hat
            z_t_difference[1] = wrap_angle_Pi(z_t_difference[1])    #bearing difference should be normalized.



            innovation = K_i_t.dot(z_t_difference)
            self.sigma = ( np.identity(3 + 2 * self.landmark_num) - K_i_t.dot(H_i_t) ).dot(self.sigma)

            total_innovation += innovation

        return total_innovation


