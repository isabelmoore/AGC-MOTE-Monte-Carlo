import numpy as np
import matplotlib.pyplot as plt

# Selecting a random IMU drift, which the Monte-Carlo approach will not be made aware of
imu_drift = np.random.normal(0, 0.5, 1)[0]

actual_points = np.random.normal(0, 1, (10, 2))
cumulative_drift = imu_drift * np.arange(10)
cumulative_drift = cumulative_drift.repeat(2).reshape((-1, 2))
drifted_points = actual_points + cumulative_drift

# Monte-Carlo approach to estimate the IMU drift
best_estimate = None
best_cost = None
best_corrected_points = None
costs = []
for imu_drift_guess in np.arange(-1, 1, 0.01):
    cumulative_drift = imu_drift_guess * np.arange(10)
    cumulative_drift = cumulative_drift.repeat(2).reshape((-1, 2))
    this_guess = actual_points + cumulative_drift
    cost = np.mean((this_guess - actual_points) ** 2)
    costs.append(cost)
    if best_cost is None or cost < best_cost:
        best_cost = cost
        best_estimate = imu_drift_guess
        best_corrected_points = this_guess.copy()
assert best_corrected_points is not None

print(f"Actual IMU drift: {imu_drift}")
print(f"Best estimate: {best_estimate}")

# Plot of the actual, drifted, and corrected points
plt.figure(figsize=(10, 6))
for n in range(len(actual_points)):
    plt.text(actual_points[n, 0], actual_points[n, 1], str(n), color='blue')
    plt.text(drifted_points[n, 0], drifted_points[n, 1], str(n), color='red')

plt.plot(actual_points[:, 0], actual_points[:, 1], "o-", label="actual", linewidth=3)
plt.plot(drifted_points[:, 0], drifted_points[:, 1], "o-", label="drifted")
plt.plot(
    best_corrected_points[:, 0], best_corrected_points[:, 1], "o--", label="corrected"
)
plt.legend()
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.title('Actual, Drifted, and Corrected Points')
plt.grid(True)
plt.show()

# Plot of the cost function
plt.figure(figsize=(10, 6))
plt.plot(np.arange(-1, 1, 0.01), costs)
plt.xlabel("IMU drift guess")
plt.ylabel("Cost")
plt.title('Cost Function vs IMU Drift Guess')
plt.grid(True)
plt.show()
