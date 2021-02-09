import pymatrix as pym

class KalmanFilter:
   'Common base class for Kalman Filter'
   def __init__(self,x):          
                                                         # (Higher == More Prediction Weight)
       # Kalman Parameters initial parameters   
       # 3x9
       # Q, R, and H matrices dont change
       self.H = pym.matrix([[1,0,0,0,0,0,0,0,0],\
                            [0,0,0,1,0,0,0,0,0],\
                            [0,0,0,0,0,0,1,0,0]])                   # Observation Matrix
       # 9x9
       Qnoise = 0.5
       self.Q = pym.matrix([[0,0,0,0,0,0,0,0,0],\
              [0,0,0,0,0,0,0,0,0],\
              [0,0,Qnoise**2,0,0,0,0,0,0],\
              [0,0,0,0,0,0,0,0,0],\
              [0,0,0,0,0,0,0,0,0],\
              [0,0,0,0,0,Qnoise**2,0,0,0],\
              [0,0,0,0,0,0,0,0,0],\
              [0,0,0,0,0,0,0,0,0],\
              [0,0,0,0,0,0,0,0,Qnoise**2]]) 
       # 3x3
       Rnoise = 5
       self.R = pym.matrix([[Rnoise**2,0,0],[0,Rnoise**2,0],[0,0,(2*Rnoise)**2]])
       # 9x1
       self.x = x   # Start state variable vector
       # 9x9
       self.P = pym.matrix([[100,0,0,0,0,0,0,0,0],\
                           [0,20,0,0,0,0,0,0,0],\
                           [0,0,5,0,0,0,0,0,0],\
                           [0,0,0,100,0,0,0,0,0],\
                           [0,0,0,0,20,0,0,0,0],\
                           [0,0,0,0,0,5,0,0,0],\
                           [0,0,0,0,0,0,100,0,0],\
                           [0,0,0,0,0,0,0,20,0],\
                           [0,0,0,0,0,0,0,0,5]])                # Initial Prediction Estimate Covariance
   def update(self,dt,Measurements):
      # 9x9                                 
      self.A = pym.matrix([[1,dt,dt**2/2,0,0,0,0,0,0],\
                           [0,1,dt,0,0,0,0,0,0],\
                           [0,0,1,0,0,0,0,0,0],\
                           [0,0,0,1,dt,dt**2/2,0,0,0],\
                           [0,0,0,0,1,dt,0,0,0],\
                           [0,0,0,0,0,1,0,0,0],\
                           [0,0,0,0,0,0,1,dt,dt**2/2],\
                           [0,0,0,0,0,0,0,1,dt],\
                           [0,0,0,0,0,0,0,0,1]])                # State Transition Matrix
      
      # set acc measurement
      z = pym.matrix([[Measurements[0]],[Measurements[1]],[Measurements[2]]])
      # Kalman Filter
      self.xp = self.A*self.x                                       # I. Prediction of the estimate
      

      
     
      self.Pp = self.A*self.P*(self.A.trans()) + self.Q       #    Prediction of the Predicted error covariance
    
      # II. Kalman Gain
      self.K = self.Pp*(self.H.trans())*(((self.H*self.Pp*(self.H.trans())) + self.R).inv())
      
      self.x = self.xp + self.K*(z - self.H*self.xp)                # III. Computation of the estimate
      # Error Covariance
      self.P = self.Pp - self.K*self.H*self.Pp                      # IV. Computation of the error covariance
      
      # Return the smoothed states
      return self.x
