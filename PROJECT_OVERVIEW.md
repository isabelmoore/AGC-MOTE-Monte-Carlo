# AGC-MOTE-Monte-Carlo (mote_ros) Project Overview

## 📖 Bridging the Gap in Robotic Localization

In the world of autonomous systems, precise localization is the foundation of every mission. **AGC-MOTE-Monte-Carlo** is designed to solve one of the most persistent hurdles in robotics: **Sensor Fusion and Drift Correction.**

This project transforms messy, drifting raw sensor data into a rock-solid, accurate trajectory using an **Extended Kalman Filter (EKF)** and **Monte Carlo Localization (Particle Filter)**.

---

## 🛠 The Three Layers of the System

### 1. The "Truth Learner"
Robots live in two worlds at once. We use this system to compare them and find the reality:
*   **The Robot's Ego**: What the onboard sensors (IMU/Odometry) *think* is happening. Over time, these sensors "drift," causing the robot's belief to curl away from reality (visualized as the **Red Line**).
*   **The Ground Truth**: What the GPS and geometric course actually show (visualized as the **Blue Line**).
*   **The Estimate**: The EKF's job is to **"bridge the gap"** between them, merging the best of both worlds (the **Green Line**).

### 2. The "Bias Hunter"
Real-world sensors have "hidden" secrets, like **Yaw Bias**. If your IMU thinks you are turning slightly when you are actually moving in a straight line, your navigation will fail. Our system acts as a mathematical detective, hunting down these hidden biases and "subtracting" them from the robot's perception in real-time.

### 3. The "Digital Twin" Framework
The system is divided into three distinct phases to ensure mathematical accuracy:
*   **Backtesting**: Record data from a real car and replay it to tune the filters.
*   **Deployment**: Deploy validated math to a live ROS-based autonomous system.
*   **Real-Time QA**: Monitor the Estimate vs Ground Truth on a 3D dashboard (RViZ).

---

## 🔬 Localization & Drift Correction

This project implement two advanced sensor fusion approaches to estimate and cancel yaw drift in real-time.

### 1. Extended Kalman Filter (EKF)
The EKF treats the system as a 7-state estimation problem where bias and acceleration are hidden variables.
- **State Vector**: `[x, y, v, theta, yaw_rate_bias, heading_bias, acceleration]`
- **Prediction**: Constant Acceleration Model with moderate damping. It predicts angular change using `(omega_measured - yaw_rate_bias)`.
- **Update**: Corrects the state based on GPS (position), IMU (yaw/heading), and linear velocity measurements.

#### 📐 Mathematical Formulation (EKF)
**State Prediction Model:**
$$
\begin{align*}
d &= v_k \Delta t + \frac{1}{2} a_k \Delta t^2 \\
x_{k+1} &= x_k + d \cos(\theta_k) \\
y_{k+1} &= y_k + d \sin(\theta_k) \\
v_{k+1} &= v_k + a_k \Delta t \\
\theta_{k+1} &= \theta_k + (\omega_{meas} - \dot{\psi}_{bias}) \Delta t \\
a_{k+1} &= a_k (1 - \lambda_{damping})
\end{align*}
$$

**Bias Correction:**
The filter estimates the sensor bias $\dot{\psi}_{bias}$ as a random walk process. This estimated bias is subtracted from the raw IMU reading $\omega_{meas}$ at every step, effectively calibrating the sensor in real-time.

### 2. Particle Filter (Monte Carlo Localization)
For complex, non-linear scenarios, the Particle Filter maintains a set of 500+ "hypotheses" (particles).
- **Process**: Each particle represents a potential state `[x, y, theta, bias]`. 
- **Resampling**: Particles that better explain the observed GPS/Yaw measurements are "survived," while others are discarded, naturally converging on the most likely trajectory.

#### 🎲 Probability Update
**Weight Calculation:**
For each particle $i$, the weight $w^{(i)}$ is updated based on the Gaussian likelihood of the measurement $z$:
$$
w^{(i)} \propto \exp\left( -\frac{||pos^{(i)} - z_{gps}||^2}{2\sigma_{pos}^2} \right)
$$
This rewards particles that are spatially close to the GPS signal while penalizing outliers.
