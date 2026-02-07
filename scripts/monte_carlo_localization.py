
import numpy as np
import matplotlib.pyplot as plt

class ParticleFilter:
    def __init__(self, num_particles, x_range, y_range, heading_range):
        self.num_particles = num_particles
        # State: [x, y, theta, bias_theta]
        self.particles = np.zeros((num_particles, 4))
        
        # Initialize particles randomly around the start (or uniform if unknown)
        self.particles[:, 0] = np.random.uniform(x_range[0], x_range[1], num_particles)
        self.particles[:, 1] = np.random.uniform(y_range[0], y_range[1], num_particles)
        self.particles[:, 2] = np.random.uniform(heading_range[0], heading_range[1], num_particles)
        # Initialize drift/bias guess (e.g., small random drift)
        self.particles[:, 3] = np.random.normal(0, 0.1, num_particles) 
        
        self.weights = np.ones(num_particles) / num_particles

    def predict(self, v, omega, dt):
        """
        Dead Reckoning Step using Motion Model
        v: linear velocity control input
        omega: angular velocity control input (measured from IMU, containing bias!)
        """
        # Add noise to inputs
        v_noise = np.random.normal(0, 0.1, self.num_particles)
        omega_noise = np.random.normal(0, 0.05, self.num_particles)
        
        # Motion model
        # theta_new = theta + (omega_input - bias) * dt
        # Note: If input is from IMU, True Omega = Measured - Bias. 
        # Here we assume 'omega' is the measured input.
        # So we adjust by the estimated bias of each particle.
        
        theta = self.particles[:, 2]
        bias = self.particles[:, 3]
        
        # Update Heading (including bias correction)
        # We model that the particle's bias 'explains' the error in omega
        effective_omega = omega - bias # Correct the input using the particle's bias estimate
        
        theta_new = theta + (effective_omega + omega_noise) * dt
        
        # Update Position
        x = self.particles[:, 0]
        y = self.particles[:, 1]
        
        x_new = x + (v + v_noise) * np.cos(theta) * dt
        y_new = y + (v + v_noise) * np.sin(theta) * dt
        
        # Random Walk for Bias (it changes slowly over time)
        bias_new = bias + np.random.normal(0, 0.001, self.num_particles)
        
        self.particles[:, 0] = x_new
        self.particles[:, 1] = y_new
        self.particles[:, 2] = theta_new
        self.particles[:, 3] = bias_new

    def update(self, measurement, R_cov):
        """
        Monte Carlo Step: Correction based on Position Measurement (e.g., GPS)
        measurement: [x_meas, y_meas]
        """
        # Distance from particle to measurement
        dist = np.linalg.norm(self.particles[:, :2] - measurement, axis=1)
        
        # Gaussian likelihood
        # weight ~ exp(-dist^2 / (2 * sigma^2))
        sigma_pos = np.sqrt(R_cov[0,0])
        likelihood = np.exp(- (dist**2) / (2 * sigma_pos**2))
        
        # Avoid zero weights
        likelihood += 1.e-300
        
        self.weights *= likelihood
        self.weights /= np.sum(self.weights) # Normalize

    def update_yaw(self, yaw_meas, R_yaw):
        """
        Correction based on Yaw Measurement (rad)
        """
        # Difference in angles (handled wrapping)
        diff = self.particles[:, 2] - yaw_meas
        diff = (diff + np.pi) % (2.0 * np.pi) - np.pi
        
        sigma_yaw = np.sqrt(R_yaw)
        likelihood = np.exp(- (diff**2) / (2 * sigma_yaw**2))
        
        likelihood += 1.e-300
        
        self.weights *= likelihood
        self.weights /= np.sum(self.weights) # Normalize

    def resample(self):
        indices = np.random.choice(range(self.num_particles), size=self.num_particles, p=self.weights)
        self.particles = self.particles[indices]
        self.weights.fill(1.0 / self.num_particles)

    def estimate(self):
        """Return mean and variance of the particles"""
        mean = np.average(self.particles, weights=self.weights, axis=0)
        var = np.average((self.particles - mean)**2, weights=self.weights, axis=0)
        return mean, var

# --- Simulation to demonstrate correction ---

def run_simulation():
    # True State
    true_x, true_y, true_theta = 0.0, 0.0, 0.0
    true_bias = 0.2 # Constant Yaw Drift of 0.2 rad/s
    
    # Filter
    pf = ParticleFilter(num_particles=500, x_range=(-1,1), y_range=(-1,1), heading_range=(-0.1, 0.1))
    
    # Simulation Parameters
    dt = 0.1
    steps = 200
    v_cmd = 1.0 # Constant forward velocity
    omega_cmd = 0.0 # Commands straight line
    
    history_true = []
    history_est = []
    history_measurements = []
    
    plt.ion()
    fig, ax = plt.subplots(figsize=(10,6))
    
    for i in range(steps):
        # 1. Simulate Real Robot (Ground Truth) transforms
        true_theta += (omega_cmd + true_bias) * dt # Robot actually drifts
        true_x += v_cmd * np.cos(true_theta) * dt
        true_y += v_cmd * np.sin(true_theta) * dt
        
        # 2. Simulate Sensor Reading (IMU)
        # IMU measures rotational velocity. 
        # Ideally IMU = (True Omega + Noise). 
        # Here we model the 'Drift' as an external disturbance we want to estimate/cancel.
        # Let's say we receive 'omega_cmd' (0) but the robot is actually turning.
        # Or more realistically: We measure 'omega_measured' which effectively includes the bias? 
        # Actually usually Bias is additive to the measurement: Measurement = Truth + Bias.
        # So Truth = Measurement - Bias.
        # Let's assume our IMU reads '0' (perfect line) but the robot drifts.
        imu_read = 0.0 # The sensor implies we are going straight
        
        # 3. Predict (Dead Reckon)
        pf.predict(v=v_cmd, omega=imu_read, dt=dt)
        
        # 4. Measure (GPS update) - Simulating low frequency, or every step
        # GPS measures Position with noise
        gps_noise = np.random.normal(0, 0.5, 2)
        z = np.array([true_x, true_y]) + gps_noise
        
        # 5. Update (Monte Carlo Correction)
        pf.update(z, R_cov=np.eye(2)*0.5)
        
        # 6. Resample
        if i % 5 == 0: # Resample occasionally or based on effective sample size
            pf.resample()
            
        # Store
        est_mean, est_var = pf.estimate()
        history_true.append([true_x, true_y, true_theta])
        history_est.append(est_mean)
        history_measurements.append(z)
        
        # Visualization (every 10 steps)
        if i % 10 == 0:
            ax.clear()
            hist_t = np.array(history_true)
            hist_e = np.array(history_est)
            hist_z = np.array(history_measurements)
            
            ax.plot(hist_t[:,0], hist_t[:,1], 'k-', label='Ground Truth')
            ax.plot(hist_e[:,0], hist_e[:,1], 'b-', label='PF Estimate')
            ax.scatter(hist_z[:,0], hist_z[:,1], c='r', s=5, alpha=0.3, label='GPS Meas')
            
            # Draw particles
            ax.scatter(pf.particles[:,0], pf.particles[:,1], c='g', s=1, alpha=0.3)
            
            ax.legend()
            ax.set_title(f"Step {i}: True Bias={true_bias:.2f}, Est Bias={est_mean[3]:.2f}")
            plt.pause(0.01)

    print(f"Final Estimated Drift Bias: {est_mean[3]:.4f} (True: {true_bias})")
    plt.show(block=True)

if __name__ == "__main__":
    run_simulation()
