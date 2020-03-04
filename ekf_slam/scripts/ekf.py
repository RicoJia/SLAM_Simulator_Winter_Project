# Assume Known Landmarks

import numpy as np
import math
import csv
import matplotlib.pyplot as plt

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

def initialize_ekf_sigma(landmark_num):
    #Initialize sigma for EKF
    sigma = np.zeros((2 * landmark_num + 3, 2 * landmark_num + 3))
    for i in range(3, 2 * landmark_num + 3):
        sigma[i, i] = 10000.0
    return sigma

def initialize_fk(landmark_num):
    F_k = np.zeros(( 3, 3 + 2 * landmark_num))
    for i in range(3):
        F_k[i, i] = 1.0
    return F_k



SIGMA_V = 1.0
SIGMA_W = 0.01
SIGMA_RANGE = 0.1
TOTAL_STEP_NUM = 1000

class ekf_object():

    def __init__(self, x = 0.0, y = 0.0, theta = 0.0, landmark_num = 1, delta_t=0.01):
        #number of landmarks:
        self.landmark_num = landmark_num
        #miu = ([2*n + 3]x1) matrix
        self.miu = np.zeros((2 * landmark_num + 3, 1))
        self.miu[:3, 0] = np.array([x, y, theta])
        #3+2xn square matrix. large sigma means I don't trust the initial value. The initial covariance is 0 for x, y, theta, and inf for landmark poses
        self.sigma = initialize_ekf_sigma(self.landmark_num)
        #delta_t: time step
        self.delta_t = delta_t
        #measurement covariance matrix (range, bearing)
        self.Q = np.array([[SIGMA_RANGE**2, 0.0],
                           [0.0, 0]])

    def motion_predict(self, u_t, delta_t = None):
        # alphas: motion covariance weights.
        if delta_t != None:
            self.delta_t = delta_t

        alphas = np.array([1, 0, 0, 1])
        theta = self.miu[2,0]
        v_t = u_t[0]        #u_t = [v, w]
        w_t = u_t[1]

        F_k = initialize_fk(self.landmark_num)

        rot = self.delta_t * u_t[1]
        halfRot = rot / 2.0
        trans = u_t[0] * self.delta_t
        poseUpdate = np.array([[trans * np.cos(theta + halfRot)],
                       [trans * np.sin(theta + halfRot)],
                       [rot] ])
        self.miu += (F_k.T).dot(poseUpdate)

        g_t = np.array([[0.0, 0.0, trans * -1.0*np.sin(theta + halfRot)],
                        [0.0, 0.0, trans * np.cos(theta + halfRot)],
                        [0.0, 0.0, 0.0] ])
        G_t = np.identity(3 + 2*self.landmark_num) + (F_k.T).dot(g_t).dot(F_k)

        # TODO: Change this for the actual robot, as its noise will be a function of both v and w.
        # M_t = np.array([[alphas[0]*abs(v_t)**2 + alphas[1]*abs(w_t)**2, 0],
        #                 [0, alphas[2]*abs(v_t)**2 + alphas[3]*abs(w_t)**2 ]])

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
        for z_t_i in z_t:
            z_t_i = z_t_i.reshape(3,1)
            j = int(z_t_i[2, 0])            #j might be a float type

            if j >= self.landmark_num:  #we only add landmarks that we know.
                continue

            if self.miu[3 + 2*j, 0] == 0 and self.miu[3 + 2*j + 1, 0]==0:     #if the landmark has never been seen before, add it to the state vector
                r_j = z_t_i[0, 0]
                phi_j = z_t_i[1, 0]
                observed_landmark_pos = np.array([r_j * np.cos(phi_j + self.miu[2, 0]), r_j * np.sin(phi_j + self.miu[2, 0])]) + self.miu[:2,0]
                self.miu[3 + 2*j: 3 + 2*j+2, 0]= observed_landmark_pos

        # ---------------- Step 1: Measurement update -----------------#
            q = np.power(np.linalg.norm(self.miu[3 + 2*j: 3 + 2*j+2, 0] - self.miu[0:2, 0]), 2)    #landmark to robot distance ^2
            #make sure angle is wrapped into [-pi, pi], and set z[2] = 0
            m = np.copy(self.miu[3 + 2*j: 3 + 2*j+2, 0])
            #this is the predicted range and bearing, note that this is 2x1
            z_t_i_hat = np.array([[np.sqrt(q)],
                                  [wrap_angle_Pi( np.arctan2( m[1] - self.miu[1, 0], m[0] - self.miu[0, 0]) - self.miu[2, 0] )]
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

            F_1 = np.append( np.identity(3), np.zeros((2,3)) , axis=0)
            F_2 = np.append( np.zeros((3,2)), np.identity(2), axis=0)
            F_x_j = np.hstack( (F_1, np.zeros((5, 2*j)), F_2, np.zeros((5, 2*self.landmark_num - 2*j-2)) ) )

            delta_x = self.miu[3 + 2*j, 0] - self.miu[0, 0]
            delta_y = self.miu[3 + 2*j +1, 0] - self.miu[1, 0]
            H_1 = np.array([-delta_x/np.sqrt(q), -delta_y/np.sqrt(q), 0.0, delta_x/np.sqrt(q), delta_y/np.sqrt(q)])
            H_2 = np.array([delta_y/q, -delta_x/q, -1.0, -delta_y/q, delta_x/q])
            # H_3 = np.array([0, 0, 0, 0, 0])       #since we are dealing with 2d with no landmark signature, we don't need these zeros.
            H_i_t = np.array([H_1, H_2]).dot(F_x_j)

        # ---------------- Step 3: Kalman gain update -----------------#
            S_i_t = H_i_t.dot(self.sigma).dot(H_i_t.T) + self.Q
            K_i_t = ( self.sigma.dot(H_i_t.T) ).dot( np.linalg.pinv(S_i_t, rcond=1e-15) )

        # ------------------- Step 4: mean update ---------------------#
            z_t = z_t_i[0:2, 0].reshape(2,1)     #since we are dealing with 2d with no landmark signature
            innovation = K_i_t.dot(z_t - z_t_i_hat)

            self.miu = self.miu + innovation
            self.sigma = ( np.identity(3 + 2 * self.landmark_num) - K_i_t.dot(H_i_t) ).dot(self.sigma)


    def filter(self, u_t, z_t):
        self.motion_predict(u_t)
        self.measurement_update(z_t)



# Initialize test data
# Args:
#     delta_t: time from the last update
def init_tables(delta_t = 0.01):
    #assume we have v = (1,0,0), robot starts from 0,0,0, and there is a wall at (100,0,0). Everything is in [x,y,theta]
    #Gaussian noise is here.
    #u_t: [v, w]
    #z_t = array of [range, bearing, id]
    #ground_truth: n x 3 [x,y,theta]

    wall_position = np.array([100.0, 1.5])

    time_arr = np.linspace(0, (TOTAL_STEP_NUM-1)*delta_t, num = TOTAL_STEP_NUM)

    u_t_arr = np.empty((0, 2))
    z_t_arr = np.empty((0,3))
    ground_truth_arr = np.empty((0,3))


    for step_num in range(TOTAL_STEP_NUM):
        # np.random.seed(step_num)
        miu_v, sigma_v = 1.0, SIGMA_V
        miu_w, sigma_w = 0.0, SIGMA_W
        v = np.random.normal(miu_v, sigma_v)
        w = np.random.normal(miu_w, sigma_w)
        u_t = np.array([v,w])
        u_t_arr = np.append(u_t_arr, u_t.reshape(1,2), axis=0)

        miu_range, sigma_range = wall_position[0] - step_num * delta_t * miu_v, SIGMA_RANGE
        _range = np.random.normal(miu_range, sigma_range)
        z_t = np.array([_range, 0.0, 0])
        z_t_arr = np.append(z_t_arr, z_t.reshape(1,3), axis=0)

        x_t = miu_v * step_num * delta_t
        real_location = np.array([x_t, 1.5, 0.0])
        ground_truth_arr = np.append(ground_truth_arr, real_location.reshape(1,3), axis=0)

    map = np.append(0, wall_position).reshape(1,3)
    return time_arr, u_t_arr, z_t_arr, map, ground_truth_arr, sigma_range



import scipy.stats as stats

class test_ekf_filter_2D():
    def __init__(self):
        self.ekf_obj = ekf_object(x=0.0, y=1.5, theta=0.0, landmark_num=1, delta_t=0.01)     #TODO: this assumption should be changed to 0,0,0, after everything is working
        self.ekf_path = (self.ekf_obj.miu[:3, 0]).reshape(1,3)
        self.ground_truth_table = None
        self.u_t_arr = None
        self.delta_t = 0.01
        self.sigma_range= None       #this is for plotting. Can be deleted TODO

    def run_ekf(self):
        time_arr, self.u_t_arr, z_t_arr, map, self.ground_truth_table, self.sigma_range = init_tables(self.delta_t)
        sigma_arr = np.zeros((3, time_arr.shape[0]))        #3 x n, for plotting, [sigma_x, sigma_y, sigma_theta]
        for i in range(time_arr.shape[0]):
            self.ekf_obj.filter(self.u_t_arr[i], z_t_arr[i].reshape(1,3))   #z_t_i must be in the form [zt1],[zt2]
            self. ekf_path = np.append(self.ekf_path, (self.ekf_obj.miu[:3,0]).reshape(1,3), axis=0)
            print("sigma_arr: ", self.ekf_obj.sigma.diagonal())
            sigma_arr[0,i] = self.ekf_obj.sigma[0,0]
            sigma_arr[1,i] = self.ekf_obj.sigma[1,1]
            sigma_arr[2,i] = self.ekf_obj.sigma[2,2]

        # plot sigmas
        plt.plot(time_arr, sigma_arr[0,:],color='blue')
        plt.plot(time_arr, sigma_arr[1,:],color='green')
        plt.plot(time_arr, sigma_arr[2,:],color='red')
        plt.legend(("x covariance", "y covariance", "theta covariance"), loc="lower right")
        plt.title("covariance of states")
        plt.show()


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

        ground_truth = self.ground_truth_table
        plt.plot(ground_truth[:, 0], ground_truth[:, 1], color='blue')
        plt.plot(self.ekf_path[:,0], self.ekf_path[:,1], color='red')
        # print("pure odom: ", pure_odom_x)
        pure_odom_y = np.array([y+1.5 for y in pure_odom_y])
        plt.plot(pure_odom_x[: -1], pure_odom_y[: -1], color="green")
        plt.title("EKF Filter vs groundtruth, sigma_range = " + str( self.sigma_range))
        plt.xlabel("x (m)")
        plt.ylabel("y (m)")
        plt.legend(("ground truth", "filtered path", "path from odom only"))
        plt.show()


    def plot_error_covariance(self):
        #plot the distruibution of x and theta values of 200 runs, and see if the gaussian fits the distribution
        run_num = 30
        x_arr = np.zeros(run_num)
        theta_arr = np.zeros(run_num)

        for run in range(run_num):
            time_arr, self.u_t_arr, z_t_arr, map, self.ground_truth_table, self.sigma_range = init_tables(self.delta_t)
            for i in range(time_arr.shape[0]):
                self.ekf_obj.filter(self.u_t_arr[i], z_t_arr[i].reshape(1,3))   #z_t_i must be in the form [zt1],[zt2]
                self. ekf_path = np.append(self.ekf_path, (self.ekf_obj.miu[:3,0]).reshape(1,3), axis=0)
                if i == time_arr.shape[0]-1 :
                    x_arr[run] = self.ekf_obj.miu[0,0]
                    theta_arr[run] = self.ekf_obj.miu[2,0]

        print("x_arr: ", x_arr)
        print("theta_arr: ", theta_arr)
        print("x_arr cov: ", np.std(x_arr))
        print("theta_arr cov: ", np.std(theta_arr))


        plt.figure(1)
        actual_mean_x = TOTAL_STEP_NUM*0.01*1.0
        cov_x = SIGMA_V
        xx = np.arange(actual_mean_x-SIGMA_V*1.0, actual_mean_x+SIGMA_V*1.0, 0.001)
        y_pdf_x = stats.norm.pdf(xx, loc=actual_mean_x, scale=SIGMA_V)
        plt.plot(xx, y_pdf_x)
        mu, std = stats.norm.fit(x_arr)
        p = stats.norm.pdf(xx, mu, std)
        print("std of x_arr: ", std, "std of V: ", SIGMA_V)
        plt.plot(xx, p, 'k', linewidth=2)
        plt.title("x distribution. Number of runs: " + str(run_num))




        #
        # plt.figure(2)
        # cov_theta = SIGMA_W
        # actual_mean_theta = 0.0
        # xtheta = np.arange(actual_mean_theta-SIGMA_W*4.0, actual_mean_theta+SIGMA_W*4.0, 0.001)
        # y_pdf_theta = stats.norm.pdf(xtheta, loc=actual_mean_theta, scale=np.sqrt(SIGMA_W))
        # plt.plot(xtheta, y_pdf_theta)
        # plt.title("theta distribution. Number of runs: " + str(run_num))
        plt.show()






if __name__== '__main__':
    # ekf = ekf_object(x=0,y=0,theta=0)
    # # ekf.motion_predict([1,np.pi/2])
    # ekf.measurement_update(np.array([[1,-np.pi/4,0]]))        #test for landmark initialization z = [range, bearing, id]
    # # print(" mean: ", ekf.miu)

    test = test_ekf_filter_2D()
    test.run_ekf()
    test.plot_ekf_ground_truth()
    # test.plot_error_covariance()
