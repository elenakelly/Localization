import numpy as np

class KalmanFilter:
    
    def __init__(self,dt,state,v,w):
        #state = [x,y,theta]  
        self.state = state
        self.theta = state[2]
        #prediction/transition matrix
        self.A = np.eye(3) 
        #control matrix
        self.B = [[dt*np.cos(self.theta), 0],
        [dt*np.sin(self.theta), 0],[0 , dt]] 
        #sensor information
        self.C = np.eye(3)
        #sensor noise, initialize with small value
        self.Q = np.dot(np.eye(3),0.01) 
        #process noise, initialize with small values
        self.R = np.dot(np.eye(3),0.01) 
        self.u = [v,w]
        #previous coveriance, #initialize with small values
        self.S = np.dot(np.eye(3),0.001) 
        #get z and C from the sensors
        self.predictiontrack = [(state[0],state[1])]
        
      
    def localization(self,z):

        #-----prediction-----

        #state prediction
        self.m = np.matmul(self.A , self.state) + np.matmul(self.B , self.u)
        #covariance prediction
        self.S = np.matmul(np.matmul(self.A, np.transpose(self.S)), self.A) + self.R

        #------correction-----
        #get z and C from the sensors
        # z = [x,y,theta]
        a = np.matmul(np.matmul(self.C,S), np.transpose(self.C)) + self.Q
        a1 = np.linalg.matrix_power(a, -1)
        K = np.matmul(np.matmul(S,np.transpose(self.C)), a1)
        #new state
        m = self.m +np.matmul(K,(z - np.matmul(self.C,self.m)))
        #new covariance
        S = np.matmul(self.A - np.matmul(K,self.C),self.S) 
        #print("new state:" , m)
        #print("new covariance:", S)

        #------visualize-----
        self.predictiontrack.append = [(m[0],m[1])]
        self.state = m
        self.S = S

    


