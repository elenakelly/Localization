from re import I, U
import numpy as np

class KalmanFilter:
    
    def __init__(self,dt,state,v,w):
        #state = [x,y,theta]  
        self.state = state
        self.theta = state[2]
        #prediction/transition matrix
        self.A = np.eye(3) 
        #control matrix
        self.B = [[dt*np.cos(self.theta), 0],[dt*np.sin(self.theta), 0],[ 0 , dt]] 
        #sensor information
        self.C = np.eye(3)
        #sensor noise, initialize with small value
        self.Q = np.dot(np.eye(3),0.01) 
        #process noise, initialize with small values
        self.R = np.dot(np.eye(3),0.01) 
        self.u = [v,w]
        #previous coveriance, #initialize with small values
        self.S = np.dot(np.eye(3),0.01) 
        #get z and C from the sensors
        print(self.C)
      
    def prediction(self):

        #state prediction
        self.m = np.dot(self.A , self.state) + np.dot(self.B , self.u)
        #covariance prediction
        self.S = np.dot(np.dot(self.A, self.S), self.A) + self.R

    def correction(self,z):
        #get z and C from the sensors
        # z = [x,y,theta]

        a = np.dot(np.dot(self.C,S), self.C) + self.Q
        a1 = np.linalg.matrix_power(a, -1)
        K = np.dot(np.dot(S,self.C), a1)
        #new state
        m = self.m +np.dot(K,(z - np.dot(self.C,self.m)))
        #new covariance
        S = np.dot(self.A - np.dot(K,self.C),self.S) 
        print("new state:" , m)
        print("new covariance:", S)

