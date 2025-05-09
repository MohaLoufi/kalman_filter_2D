import numpy as np
import matplotlib.pyplot as plt

# --- Function to simulate noisy 2D measurements ---
def simulate_data_2d(num_steps, initial_position, velocity, measurement_noise_cov):
    true_position = np.array(initial_position, dtype=float)
    measurements = []
    true_positions = []

    chol_L = np.linalg.cholesky(measurement_noise_cov)

    for _ in range(num_steps):
        true_position += velocity
        true_positions.append(true_position.copy())
        noise = chol_L @ np.random.randn(2, 1)
        noisy_measurement = true_position + noise.flatten()

        measurements.append(noisy_measurement)

    return true_positions, measurements

# --- Function to run the 2D Kalman Filter ---
def run_kalman_filter_2d(measurements, initial_state_estimate, initial_state_covariance,
                         measurement_noise_covariance, process_noise_covariance,
                         state_transition_matrix, measurement_matrix):
    x_est = np.array(initial_state_estimate, dtype=float).reshape(-1, 1) # State vector [x, y, vx, vy] (4x1)
    P = np.array(initial_state_covariance, dtype=float) # State covariance matrix (4x4)

    estimates = []
    uncertainties = []
    kalman_gains = []

    I = np.eye(x_est.shape[0]) # Identity matrix (4x4)

    for z in measurements:
        z = np.array(z, dtype=float).reshape(-1, 1) # Measurement vector [x, y] (2x1)

        # 1. Predict Step
        x_pred = state_transition_matrix @ x_est
        P_pred = state_transition_matrix @ P @ state_transition_matrix.T + process_noise_covariance

        # 2. Update Step
        y = z - measurement_matrix @ x_pred # Measurement residual
        S = measurement_matrix @ P_pred @ measurement_matrix.T + measurement_noise_covariance # Residual covariance
        K = P_pred @ measurement_matrix.T @ np.linalg.inv(S) # Kalman Gain

        x_est = x_pred + K @ y # Updated state estimate
        P = (I - K @ measurement_matrix) @ P_pred # Updated estimate uncertainty

        estimates.append(x_est.flatten())
        uncertainties.append(P.copy())
        kalman_gains.append(K.copy())

    return estimates, uncertainties, kalman_gains

# --- Simulation Parameters ---
num_steps = 100
initial_true_position = np.array([0.0, 0.0])
velocity = np.array([0.5, 0.8])

# Measurement noise covariance matrix (R)
measurement_noise_std = 2.0
measurement_noise_variance = measurement_noise_std**2
measurement_noise_cov = np.array([[measurement_noise_variance, 0.0],
                                  [0.0, measurement_noise_variance]])

np.random.seed(42)

# Simulate 2D data
true_positions_2d, measurements_2d = simulate_data_2d(
    num_steps,
    initial_true_position,
    velocity,
    measurement_noise_cov
)

true_positions_2d = np.array(true_positions_2d)
measurements_2d = np.array(measurements_2d)

# --- 2D Kalman Filter Parameters ---
# State vector: [x, y, vx, vy]
initial_state_estimate = np.array([0.0, 0.0, 0.0, 0.0])

# Initial state covariance matrix (P)
initial_position_uncertainty = 10.0
initial_velocity_uncertainty = 1.0
initial_state_covariance = np.diag([initial_position_uncertainty**2,
                                      initial_position_uncertainty**2,
                                      initial_velocity_uncertainty**2,
                                      initial_velocity_uncertainty**2])

# Process noise covariance matrix (Q)
process_noise_velocity_variance = 0.01
process_noise_covariance = np.diag([0.0,
                                    0.0,
                                    process_noise_velocity_variance,
                                    process_noise_velocity_variance])

# State transition matrix (A) - Constant velocity model with dt=1
dt = 1.0
state_transition_matrix = np.array([[1.0, 0.0, dt, 0.0],
                                    [0.0, 1.0, 0.0, dt],
                                    [0.0, 0.0, 1.0, 0.0],
                                    [0.0, 0.0, 0.0, 1.0]])

# Measurement matrix (H) - Measures position [x, y]
measurement_matrix = np.array([[1.0, 0.0, 0.0, 0.0],
                               [0.0, 1.0, 0.0, 0.0]])

# Run the 2D Kalman Filter
estimates_2d, uncertainties_2d, kalman_gains_2d = run_kalman_filter_2d(
    measurements_2d,
    initial_state_estimate,
    initial_state_covariance,
    measurement_noise_cov,
    process_noise_covariance,
    state_transition_matrix,
    measurement_matrix
)

estimates_2d = np.array(estimates_2d)

# --- Plotting 2D Results ---

# Plotting 2D Position Estimation
plt.figure(figsize=(10, 8))
plt.plot(measurements_2d[:, 0], measurements_2d[:, 1], 'o', label='Noisy Measurements', alpha=0.6)
plt.plot(estimates_2d[:, 0], estimates_2d[:, 1], '-', label='Kalman Filter Estimate')
plt.plot(true_positions_2d[:, 0], true_positions_2d[:, 1], '--', label='True Position')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('2D Kalman Filter: Position Estimation')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.tight_layout()
plt.show()

# Plotting Uncertainty (Trace of Covariance Matrix) and Kalman Gain (Frobenius Norm)
fig, ax = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

uncertainty_trace = [np.trace(P) for P in uncertainties_2d]
ax[0].plot(uncertainty_trace, 'r-', marker='s', markersize=5)
ax[0].set_ylabel('Trace of Covariance (P)')
ax[0].set_title('Estimate Uncertainty (Trace of P) Over Time')
ax[0].grid(True)

kalman_gain_norm = [np.linalg.norm(K, 'fro') for K in kalman_gains_2d]
ax[1].plot(kalman_gain_norm, 'g-', marker='o', markersize=5)
ax[1].set_ylabel('Frobenius Norm of Kalman Gain (K)')
ax[1].set_xlabel('Time Step')
ax[1].set_title('Kalman Gain (Frobenius Norm of K) Over Time')
ax[1].grid(True)

plt.tight_layout()
plt.show()
