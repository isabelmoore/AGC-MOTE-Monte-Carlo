import numpy as np


class KalmanFilter:
    def __init__(self):
        self.damping = 0.02 # Stronger acceleration damping

        # Augmented State: [x, y, v, theta, yaw_rate_bias, heading_bias, a]
        # 0: x
        # 1: y
        # 2: v (linear velocity)
        # 3: theta (true heading)
        # 4: yaw_rate_bias (bias in angular velocity measurement)
        # 5: heading_bias (absolute offset of the sensor)
        # 6: a (linear acceleration)
        self.x = np.zeros((7, 1)) 
        
        # Initial Covariance
        self.P = np.eye(7) * 1.0

        # Process Noise
        self.Q = np.eye(7) * 0.01
        self.Q[4, 4] = 0.0001   # rate_bias changes slowly
        self.Q[5, 5] = 0.000001 # heading_bias is very stable
        self.Q[6, 6] = 0.1      # acceleration can change moderately

        # Observation Matrices (7 columns now)
        self.H_pos = np.zeros((2, 7))
        self.H_pos[0, 0] = 1
        self.H_pos[1, 1] = 1
        
        self.H_vel = np.zeros((1, 7))
        self.H_vel[0, 2] = 1
        
        self.H_yaw = np.zeros((1, 7))
        self.H_yaw[0, 3] = 1

        self.H_sensor = np.zeros((1, 7))
        self.H_sensor[0, 3] = 1
        self.H_sensor[0, 5] = 1

        # Measurement Noise Covariances
        self.R_pos = np.eye(2) * 1.0
        self.R_vel = np.eye(1) * 0.1
        self.R_yaw = np.eye(1) * 0.1

    def predict_ekf(self, omega_measured=0.0):
        """
        7-state EKF Prediction: Constant Acceleration Model
        """
        x = self.x[0, 0]
        y = self.x[1, 0]
        v = self.x[2, 0]
        theta = self.x[3, 0]
        rate_bias = self.x[4, 0]
        head_bias = self.x[5, 0]
        a = self.x[6, 0]
        dt = self.dt

        # 1. State Transition
        omega_corrected = omega_measured - rate_bias
        
        # x_new = x + v*dt + 0.5*a*dt^2
        dist = v * dt + 0.5 * a * dt**2
        new_x = x + dist * np.cos(theta)
        new_y = y + dist * np.sin(theta)
        new_v = v + a * dt
        new_theta = theta + omega_corrected * dt
        new_rate_bias = rate_bias
        new_head_bias = head_bias
        new_a = a * (1.0 - self.damping) # Moderate damping on acceleration

        self.x = np.array([[new_x], [new_y], [new_v], [new_theta], [new_rate_bias], [new_head_bias], [new_a]])

        # 2. Jacobian (F) - 7x7
        F = np.eye(7)
        # dx/dtheta, dy/dtheta
        F[0, 3] = -dist * np.sin(theta)
        F[1, 3] = dist * np.cos(theta)
        # dx/dv, dy/dv
        F[0, 2] = np.cos(theta) * dt
        F[1, 2] = np.sin(theta) * dt
        # dx/da, dy/da
        F[0, 6] = 0.5 * np.cos(theta) * dt**2
        F[1, 6] = 0.5 * np.sin(theta) * dt**2
        # dv/da
        F[2, 6] = dt
        # dtheta/drate_bias
        F[3, 4] = -dt
        # da/da (damping)
        F[6, 6] = 1.0 - self.damping

        self.P = F @ self.P @ F.T + self.Q

    def predict(self, omega_measured=0.0):
        self.predict_ekf(omega_measured=omega_measured)

    def update(self, z, H, R, angle_indices=[]):
        # Resize H if necessary for 7-state
        if H.shape[1] != 7:
            H_new = np.zeros((H.shape[0], 7))
            min_cols = min(H.shape[1], 7)
            H_new[:, :min_cols] = H[:, :min_cols]
            H = H_new
            
        y = z - np.dot(H, self.x)
        
        # Angle Wrapping (Residuals)
        for idx in angle_indices:
            y[idx, 0] = (y[idx, 0] + np.pi) % (2.0 * np.pi) - np.pi
            
        S = np.dot(H, np.dot(self.P, H.T)) + R
        K = np.dot(np.dot(self.P, H.T), np.linalg.inv(S))
        
        self.x = self.x + np.dot(K, y)
        
        # Wrap Heading (Index 3)
        self.x[3, 0] = (self.x[3, 0] + np.pi) % (2.0 * np.pi) - np.pi
        
        I = np.eye(7) 
        self.P = np.dot(I - np.dot(K, H), self.P)
    
        return y

