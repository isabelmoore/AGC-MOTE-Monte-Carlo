
import numpy as np
import matplotlib.pyplot as plt

class EKF_Localization:
    def __init__(self, x, y, theta, theta_bias=0.0):
        # State: [x, y, theta, theta_bias]
        self.x = np.array([x, y, theta, theta_bias])
        
        # Covariance Matrix
        # Heuristic initialization
        self.P = np.diag([1.0, 1.0, 0.1, 0.1])
        
        # Process Noise Covariance (Q)
        # Tuning knobs: How much we trust the model vs how much we think it jitters
        self.Q = np.diag([0.1, 0.1, 0.01, 0.0001]) 
        
        # Measurement Noise (will be set during update)

    def predict(self, v, omega_measured, dt):
        """
        EKF Prediction Step
        v: Linear Velocity (Control input)
        omega_measured: Angular Velocity (IMU input, contains bias)
        """
        theta = self.x[2]
        bias = self.x[3]
        
        # 1. State Prediction (Non-linear)
        # theta_new = theta + (omega_meas - bias) * dt
        # x_new = x + v * cos(theta) * dt
        # y_new = y + v * sin(theta) * dt
        # bias_new = bias (constant assumption, with noise)
        
        omega_true_est = omega_measured - bias
        
        new_theta = theta + omega_true_est * dt
        new_x = self.x[0] + v * np.cos(theta) * dt
        new_y = self.x[1] + v * np.sin(theta) * dt
        new_bias = bias
        
        self.x = np.array([new_x, new_y, new_theta, new_bias])
        
        # 2. Jacobian Calculation (F)
        # F = df/dx
        # Row 0 (x): [1, 0, -v*sin(theta)*dt, 0]
        # Row 1 (y): [0, 1,  v*cos(theta)*dt, 0]
        # Row 2 (theta): [0, 0, 1, -dt]
        # Row 3 (bias): [0, 0, 0, 1]
        
        F = np.eye(4)
        F[0, 2] = -v * np.sin(theta) * dt
        F[1, 2] =  v * np.cos(theta) * dt
        F[2, 3] = -dt
        
        # 3. Covariance Prediction
        # P = F * P * F.T + Q
        self.P = F @ self.P @ F.T + self.Q

    def update_position(self, z, R):
        """
        Update with GPS Position Measurement
        z: [x_meas, y_meas]
        R: Measurement Noise Covariance (2x2)
        """
        # Observation Model (Linear)
        # z = H * x
        # H maps state to measurement: [x, y, theta, bias] -> [x, y]
        H = np.zeros((2, 4))
        H[0, 0] = 1
        H[1, 1] = 1
        
        self.update(z, H, R)
        
    def update_yaw(self, yaw_meas, R_yaw):
        """
        Update with Yaw Measurement (rad)
        """
        H = np.zeros((1, 4))
        H[0, 2] = 1 # theta is index 2
        
        # We need to handle yaw wrapping in the residual calculation
        # Instead of calling self.update directly, we'll implement it here or handle it in update
        self.update(np.array([yaw_meas]), H, R_yaw, angle_indices=[0])

    def update(self, z, H, R, angle_indices=[]):
        """
        Generic Kalman Filter Update
        angle_indices: List of indices in 'y' (residual) that are angles and need wrapping
        """
        # Residual
        y = z - H @ self.x
        
        # Handle Angle Wrapping
        for idx in angle_indices:
            y[idx] = (y[idx] + np.pi) % (2.0 * np.pi) - np.pi
        
        # Innovation Covariance
        S = H @ self.P @ H.T + R
        
        # Kalman Gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update State
        self.x = self.x + K @ y
        
        # Update Covariance
        # Wrap theta in state to [-pi, pi]
        self.x[2] = (self.x[2] + np.pi) % (2.0 * np.pi) - np.pi
        
        I = np.eye(len(self.x))
        self.P = (I - K @ H) @ self.P


def run_ekf_simulation():
    # True State
    true_x, true_y, true_theta = 0.0, 0.0, 0.0
    true_bias = 0.2 # Constant Yaw Drift of 0.2 rad/s
    
    ekf = EKF_Localization(x=0, y=0, theta=0)
    
    # Sim Params
    dt = 0.1
    steps = 200
    v_cmd = 1.0
    omega_cmd = 0.0 # Straight line command
    
    history_true = []
    history_est = []
    
    plt.ion()
    fig, ax = plt.subplots(figsize=(10,6))
    
    for i in range(steps):
        # 1. Physics (Robot moves)
        true_theta += (omega_cmd + true_bias) * dt
        true_x += v_cmd * np.cos(true_theta) * dt
        true_y += v_cmd * np.sin(true_theta) * dt
        
        # 2. Sensor (IMU)
        # IMU measures rotation. If we are perfectly still (omega_cmd=0), but drifting (bias=0.2), 
        # Does the IMU see the drift? 
        # Case A: Bias is in the actuator/kinematics (e.g. wheel slip). IMU reads 0, Robot turns 0.2.
        # Case B: Bias is in the sensor (Gyro drift). Robot turns 0, IMU reads 0.2.
        # Here we model "Bias" as a mismatch between Measured Omega and True Omega.
        # Using the formulation: True = Measured - Bias
        # Let's assume the IMU reads 0.0 (perfectly straight command), but the robot drifts.
        # This implies: True=0.2, Measured=0.0. 
        # So 0.2 = 0.0 - Bias => Bias = -0.2. 
        # Let's stick to the sign convention: omega_est = omega_meas - bias.
        
        imu_measured = 0.0 
        
        # 3. Predict (EKF)
        ekf.predict(v=v_cmd, omega_measured=imu_measured, dt=dt)
        
        # 4. Measurement (GPS)
        gps_noise = np.random.normal(0, 0.5, 2)
        z = np.array([true_x, true_y]) + gps_noise
        ekf.update_position(z, R=np.eye(2)*0.5)
        
        # Store
        history_true.append([true_x, true_y, true_theta])
        history_est.append(ekf.x.copy())
        
        if i % 10 == 0:
            ax.clear()
            hist_t = np.array(history_true)
            hist_e = np.array(history_est)
            
            ax.plot(hist_t[:,0], hist_t[:,1], 'k-', label='Truth')
            ax.plot(hist_e[:,0], hist_e[:,1], 'r--', label='EKF Est')
            ax.set_title(f"Step {i}, Est Bias: {ekf.x[3]:.3f} (True: -0.2)")
            ax.legend()
            plt.pause(0.01)
            
    plt.show(block=True)

if __name__ == "__main__":
    run_ekf_simulation()
