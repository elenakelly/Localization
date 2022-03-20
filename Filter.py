import numpy as np


class KalmanFilter:

    def __init__(self, dt, state, sigma_mov, sigma_rot):
        # state = [x,y,theta]
        self.state = state
        self.theta = state[2]
        # prediction/transition matrix
        self.A = np.eye(3)
        # control matrix
        self.B = [[dt * np.cos(self.theta), 0],
                  [dt * np.sin(self.theta), 0], [0, dt]]
        # sensor information
        self.C = np.eye(3)
        # sensor noise, initialize with small value
        self.Q = np.dot(np.eye(3), 1) * np.array([sigma_mov, sigma_mov, sigma_rot])  # TODO change when
        # process noise, initialize with small values
        self.R = np.dot(np.eye(3), 1) * np.array([sigma_mov, sigma_mov, sigma_rot])
        # previous coveriance, #initialize with small values
        self.S = np.eye(3) * np.array([sigma_mov, sigma_mov, sigma_rot])
        # get z and C from the sensors
        self.predictiontrack = [(state[0], state[1])]

        #nick added
        self.m = self.state
        print(sigma_mov)
        print(sigma_rot)

    def localization(self, z, v, w, m):

        # -----prediction-----
        '''self.u = [v, w]
        # state prediction
        self.m = np.matmul(self.A, self.state) + np.matmul(self.B, self.u)'''

        #nick added
        self.m = m


        # covariance prediction
        self.S = np.matmul(np.matmul(self.A, self.S), np.transpose(self.A)) + self.R

        # ------correction-----
        # get z and C from the sensors
        # z = [x,y,theta]
        a = np.matmul(np.matmul(self.C, self.S), np.transpose(self.C)) + self.Q
        a1 = np.linalg.matrix_power(a, -1)
        K = np.matmul(np.matmul(self.S, np.transpose(self.C)), a1)
        # new state
        if z == None:
            m = self.m
            self.predictiontrack.append([m[0], m[1], m[2]])

            self.state = m
        else:
            m = self.m + np.matmul(K, (z - np.matmul(self.C, self.m)))
            S = np.matmul(self.A - np.matmul(K, self.C), self.S)
            self.predictiontrack.append([m[0], m[1], m[2]])

            self.state = m
            self.S = S
            self.m = m
        # new covariance

        # print("new state:" , m)
        # print("new covariance:", S)

        # ------visualize-----




