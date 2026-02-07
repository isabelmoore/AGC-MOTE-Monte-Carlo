import numpy as np


class KalmanFilter:
    def __init__(self):
        self.dt = 0.01  # Time step

        # Augmented State: [x, y, v, yaw, yaw_bias]
        # Note: We removed yaw_rate as a state because it's usually an input (IMU), 
        # but if you need to filter it, we can treat 'yaw_bias' as the correction.
        self.x = np.zeros((5, 1)) 
        
        # 0: x
        # 1: y
        # 2: v (linear velocity)
        # 3: yaw
        # 4: yaw_bias

        # Initial Covariance
        self.P = np.eye(5) * 1.0

        # Process Noise
        self.Q = np.eye(5) * 0.01
        self.Q[4, 4] = 0.0001 # Bias changes slowly

        # Standard Matrices (Legacy/Placeholder)
        # These linear matrices don't fully apply to EKF, but we keep them initialized 
        # to avoid instant crashes if old code references them, 
        # though logic should move to predict_ekf
        self.H_pos = np.zeros((2, 5))
        self.H_pos[0, 0] = 1
        self.H_pos[1, 1] = 1
        
        self.H_vel = np.zeros((1, 5))
        self.H_vel[0, 2] = 1
        
        self.H_yaw = np.zeros((1, 5))
        self.H_yaw[0, 3] = 1

        self.H_yawrate = np.zeros((1, 5))
        self.H_yawrate[0, 4] = 1  # Measures yaw_bias (state index 4)

        # Measurement Noise Covariances
        self.R_pos = np.eye(2) * 1.0
        self.R_vel = np.eye(1) * 0.1
        self.R_yaw = np.eye(1) * 0.1
        self.R_yawrate = np.eye(1) * 0.01

    def predict_ekf(self, omega_measured=0.0):
        """
        Extended Kalman Filter Prediction
        Assumes Constant Velocity Model for linear motion
        """
        x = self.x[0, 0]
        y = self.x[1, 0]
        v = self.x[2, 0]
        theta = self.x[3, 0]
        bias = self.x[4, 0]
        dt = self.dt

        # 1. Non-linear State Transition
        # theta_new = theta + (omega_meas - bias) * dt
        # We assume omega_measured is passed in, or we use a stored value.
        # If we don't have omega_measured, we rely on the state? 
        # For now, let's assume v is state, and we just integrate it.
        
        # Corrected Yaw Rate
        omega_corrected = omega_measured - bias
        
        new_x = x + v * np.cos(theta) * dt
        new_y = y + v * np.sin(theta) * dt
        new_v = v # Const vel model
        new_theta = theta + omega_corrected * dt
        new_bias = bias

        self.x = np.array([[new_x], [new_y], [new_v], [new_theta], [new_bias]])

        # 2. Jacobian (F)
        F = np.eye(5)
        # dx/dtheta
        F[0, 3] = -v * np.sin(theta) * dt
        # dy/dtheta
        F[1, 3] = v * np.cos(theta) * dt
        # dx/dv
        F[0, 2] = np.cos(theta) * dt
        # dy/dv
        F[1, 2] = np.sin(theta) * dt
        
        # dtheta/dbias
        F[3, 4] = -dt

        self.P = F @ self.P @ F.T + self.Q

    def predict(self, omega_measured=0.0):
        # Wrapper for legacy calls
        self.predict_ekf(omega_measured=omega_measured)

    def update(self, z, H, R, angle_indices=[]):
        # Standard Linear Update works for Measurement if measurement function is linear
        # (e.g. direct measurement of position)
        
        # Check dimensions if H expects 4 states (old) vs 5 states (new)
        if H.shape[1] != 5:
            # Resize H to match new state dimension
            H_new = np.zeros((H.shape[0], 5))
            min_cols = min(H.shape[1], 5)
            H_new[:, :min_cols] = H[:, :min_cols]
            H = H_new
            
        y = z - np.dot(H, self.x)
        
        # Handle Angle Wrapping for residuals
        for idx in angle_indices:
            y[idx, 0] = (y[idx, 0] + np.pi) % (2.0 * np.pi) - np.pi
            
        S = np.dot(H, np.dot(self.P, H.T)) + R
        K = np.dot(np.dot(self.P, H.T), np.linalg.inv(S))
        
        self.x = self.x + np.dot(K, y)
        
        # Wrap Yaw State (Index 3)
        self.x[3, 0] = (self.x[3, 0] + np.pi) % (2.0 * np.pi) - np.pi
        
        I = np.eye(H.shape[1]) # This assumes H is square? No.
        # Identity size should match State size (5)
        I = np.eye(5) 
        self.P = np.dot(I - np.dot(K, H), self.P)

