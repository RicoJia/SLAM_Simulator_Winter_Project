# Assume Known Landmarks

import numpy as np
import math
import csv
import matplotlib.pyplot as plt


# def get_landmark(map):
#     #converting map ([subject, x, y]) into a dictionary
#
#     # ground_truth_table = {"1": (1,2)}
#     ground_truth_table = {}
#     for landmark in map:
#         ground_truth_table[landmark[0]] = np.array(landmark[1:])
#     return ground_truth_table

def wrap_angle_Pi(x):
    max = np.pi
    min = -np.pi
    return min+ (max - min + ( (x - min) % (max - min) ) ) % (max - min)

#u_t: [v, w]
#miu_t_1: [x, y, theta, mx1, my1 ... ].T
#z_t = array of [range, bearing, id]
#z_t_i = 3x1
#----------------------------params:
# delta t
# alphas: motion covariance weights.

class ekf_object ():
    def __init__(self, x = 0.0, y = 0.0, theta = 0.0):
        #miu = (3x1) matrix
        self.miu = (np.array([x, y, theta])).reshape(3,1)
        #large sigma means I don't trust the initial value
        self.sigma = np.array([[10000.0, 0.0, 0.0],
                                [0.0, 10000.0, 0.0],
                                [0.0, 0.0, 10000.0]])


def ekf(miu_t_1, sigma_t_1, u_t, z_t, map, delta_t):
    #motion update:
    #params
    alphas = [0.1, 0.1, 0.1, 0.1]

    theta = miu_t_1[2,0]
    v_t = u_t[0]
    w_t = u_t[1]

    # #what to do when you have w_t = 0? when you have np.inf - np.inf = np.nan pinv for S_i_t doesn't evaluate nan
    # G_t = np.array([[1.0, 0.0, -v_t/w_t*np.cos(theta) + v_t/w_t*np.cos(theta + w_t*delta_t)],
    #                 [0.0, 1.0, -v_t/w_t*np.sin(theta) + v_t/w_t*np.sin(theta + w_t*delta_t)],
    #                 [0.0, 0.0, 1.0]])
    #
    # V_t = np.array([[(-np.sin(theta) + np.sin(theta + w_t*delta_t))/w_t,  v_t*(np.sin(theta) - np.sin(theta+w_t*delta_t))/(w_t*w_t) + v_t*np.cos(theta + w_t*delta_t)*delta_t/w_t ],
    #                 [(np.cos(theta) - np.cos(theta + w_t*delta_t))/w_t,  -v_t*(np.cos(theta) - np.cos(theta+w_t*delta_t))/(w_t*w_t) + v_t*np.sin(theta + w_t*delta_t)*delta_t/w_t ],
    #                 [0 , delta_t]])
    #
    # miu_t_bar = miu_t_1+ np.array([[-v_t/w_t*np.sin(theta) + v_t/w_t*np.sin(theta+ w_t* delta_t)],
    #                            [v_t/w_t*np.cos(theta) - v_t/w_t*np.cos(theta+w_t*delta_t)],
    #                            [w_t*delta_t]])

    #TEST - simple motion model------------
    rot = delta_t * u_t[1]
    halfRot = rot / 2.0
    trans = u_t[0] * delta_t

    G_t = np.array([[1, 0, trans * -1.0*np.sin(theta + halfRot)],
                    [0.0, 1.0 , trans * np.cos(theta + halfRot)],
                    [0, 0, 1] ])

    V_t = np.array([[np.cos(theta + halfRot), -0.5 * np.sin(theta + halfRot)],
                    [np.sin(theta + halfRot), 0.5 * np.cos(theta + halfRot)],
                    [0, 1.0]])

    poseUpdate = np.array([[trans * np.cos(theta + halfRot)],
                           [trans * np.sin(theta + halfRot)],
                           [rot] ])

    miu_t_bar = miu_t_1+ poseUpdate

    #Test ends------------
    M_t = np.array([[alphas[0]*v_t*v_t + alphas[1]*w_t*w_t, 0],
                    [0, alphas[2]*v_t*v_t + alphas[3]*w_t*w_t]])



    sigma_t_bar = (G_t.dot(sigma_t_1)).dot(G_t.T) + (V_t.dot(M_t)).dot(V_t.T)

    Q_t = np.array([[0.0001, 0.0, 0.0],
                    [0.0, 0.0001, 0.0],
                    [0.0, 0.0, 0.0]])


    #correction
    for z_t_i in z_t:
        z_t_i = z_t_i.reshape(3,1)
        j = z_t_i[2, 0]
        z_t_i[2, 0] = 0.0

        #find ground truth here from the table
        if j in  get_landmark(map):
            m = get_landmark(map)[j]   #[x,y]
            q = np.power(np.linalg.norm(np.array(m) - miu_t_bar[0:2, 0]), 2)    #?

            #make sure angle is wrapped into [-pi, pi], and set z[2] = 0
            z_t_i_hat = np.array([[np.sqrt(q)],
                                [wrap_angle_Pi( np.arctan2( m[1] - miu_t_bar[1, 0], m[0] - miu_t_bar[0, 0])  - miu_t_bar[2, 0] )],
                                [0]])

            H_i_t = np.array([[-( m[0] - miu_t_bar[0,0] )/( np.sqrt(q) ), -( m[1] - miu_t_bar[1,0])/np.sqrt(q), 0.0],
                            [( m[1] - miu_t_bar[1, 0])/q, -( m[0] - miu_t_bar[0, 0] )/q, -1.0],
                            [0.0, 0.0, 0.0]])

            S_i_t = (H_i_t.dot(sigma_t_bar)).dot(H_i_t.T) + Q_t

            K_i_t = ( sigma_t_bar.dot(H_i_t.T) ).dot( np.linalg.pinv(S_i_t, rcond=1e-14) )
            # K_i_t = ( sigma_t_bar.dot(H_i_t.T) ).dot( np.linalg.inv(S_i_t) )
            miu_t_bar = miu_t_bar + K_i_t.dot( z_t_i - z_t_i_hat )
            sigma_t_bar = ( np.identity(3) - K_i_t.dot(H_i_t) ).dot(sigma_t_bar)


    miu_t = miu_t_bar
    sigma_t = sigma_t_bar

    return miu_t, sigma_t




#delta_t
#u_t: [v, w]
#z_t = array of [range, bearing, id]
#ground_truth: n x 3 [x,y,theta]
#self.delta_t
def init_tables(delta_t = 0.01):
    #assume we have v = (1,0,0), robot starts from 0,0,0, and there is a wall at (100,0,0). Everything is in [x,y,theta]
    #Gaussian noise is here.
    total_step_num = 1000
    wall_position = np.array([100.0, 1.5])

    time_arr = np.linspace(0, (total_step_num-1)*delta_t, num = total_step_num)

    u_t_arr = np.empty((0, 2))
    z_t_arr = np.empty((0,3))
    ground_truth_arr = np.empty((0,3))


    for step_num in range(total_step_num):
        np.random.seed(step_num)
        miu_v, sigma_v = 1.0, 0.1
        miu_w, sigma_w = 0.01, 0.5
        v = np.random.normal(miu_v, sigma_v)
        w = np.random.normal(miu_w, sigma_w)
        u_t = np.array([v,w])
        u_t_arr = np.append(u_t_arr, u_t.reshape(1,2), axis=0)

        miu_range, sigma_range = wall_position[0] - step_num * delta_t * miu_v, 0.001
        _range = np.random.normal(miu_range, sigma_range)
        z_t = np.array([_range, 0.0, 0])
        z_t_arr = np.append(z_t_arr, z_t.reshape(1,3), axis=0)

        x_t = miu_v * step_num * delta_t
        real_location = np.array([x_t, 1.5, 0.0])
        ground_truth_arr = np.append(ground_truth_arr, real_location.reshape(1,3), axis=0)

    map = np.append(0, wall_position).reshape(1,3)
    return time_arr, u_t_arr, z_t_arr, map, ground_truth_arr, sigma_range


class test_ekf_filter_1D():
    def __init__(self):
        self.ekf_obj = ekf_object(x=0.0, y=1.5, theta=0.0)     #TODO: this assumption should be changed to 0,0,0, after everything is working
        self.ekf_path = (self.ekf_obj.miu).reshape(1,3)
        self.ground_truth_table = None
        self.u_t_arr = None
        self.delta_t = 0.01
        self.sigma_range= None       #this is for plotting. Can be deleted TODO

    def run_ekf(self):
        time_arr, self.u_t_arr, z_t_arr, map, self.ground_truth_table, self.sigma_range = init_tables(self.delta_t)
        for i in range(time_arr.shape[0]):
            self.ekf_obj.miu, self.ekf_obj.sigma = ekf(self.ekf_obj.miu, self.ekf_obj.sigma, self.u_t_arr[i], z_t_arr[i].reshape(1,3), map,self.delta_t)
            self. ekf_path = np.append(self.ekf_path, self.ekf_obj.miu.reshape(1,3), axis=0)
            #
            # print("------------------")
            # print("z_t[0]: ",u_t_arr[i])
            # print("u_t: ", u_t_arr[i])
            # print("ground truth: ", self.ground_truth_table[i])
            # print("estimate", self.ekf_obj.miu[0,0])

        # #visualizing 1d
        # pure_odom = np.array([np.sum(u_t_arr[:i]) for i in range(u_t_arr.shape[0])])*delta_t
        # plt.plot(time_arr, self.ground_truth_table[:,0],color='blue')
        # # print ("pure_odom: ", pure_odom)
        # plt.plot(time_arr, pure_odom, color='green')
        # plt.plot(time_arr, self.ekf_path[: -1,0],color='red')
        # plt.legend(("ground truth", "pure odometry", "filtered output"), loc="lower right")
        # plt.show()


    def plot_ekf_ground_truth(self):
        #extract the corresponding ground truth data and plot it against the filtered path

        # thetas_odom = np.array([np.sum(self.u_t_arr[:i, 1]) for i in range(self.u_t_arr.shape[0])])*self.delta_t
        pure_odom_x = np.array([0])     #(n+1) array
        pure_odom_y = np.array([0])
        theta = 0
        for i in range(self.u_t_arr[:, 1].shape[0]):
            theta = theta + self.delta_t * self.u_t_arr[i, 1]
            x = pure_odom_x[-1] + self.u_t_arr[i,0] * np.cos(theta) * self.delta_t
            y = pure_odom_y[-1] + self.u_t_arr[i,0] * np.sin(theta) * self.delta_t
            pure_odom_x = np.append(pure_odom_x, x)
            pure_odom_y = np.append(pure_odom_y, y)
            print("x: ", x, "y: ", y, "theta: ", theta, "w: ", self.u_t_arr[i,1])

        print("w: ", self.u_t_arr[: 100, 1])
        ground_truth = self.ground_truth_table
        plt.plot(ground_truth[:, 0], ground_truth[:, 1], color='blue')
        plt.plot(self.ekf_path[:,0], self.ekf_path[:,1], color='red')
        # print("pure odom: ", pure_odom_x)
        pure_odom_y = np.array([y+1.5 for y in pure_odom_y])
        plt.plot(pure_odom_x[: -1], pure_odom_y[: -1], color="green")
        plt.title("EKF Filter vs groundtruth, sigma_range = " + str( self.sigma_range))
        plt.xlabel("x (m)")
        plt.ylabel("y (m)")
        plt.show()




if __name__== '__main__':
    # tef = test_ekf_filter(time_out=10)
    # tef.run_ekf()
    # tef.plot_ekf_ground_truth()

    tef = test_ekf_filter_1D()
    tef.run_ekf()
    tef.plot_ekf_ground_truth()