import matplotlib.pyplot as plt
import numpy as np


def rot2d(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


class IMU:
    def __init__(
        self,
        gyro_bias=0.02,
        accel_bias=np.array([0.1, -0.05]),
        gyro_noise_std=0.01,
        accel_noise_std=0.1,
        gravity=9.81,
    ):

        self.bg = gyro_bias
        self.ba = accel_bias
        self.gyro_noise_std = gyro_noise_std
        self.accel_noise_std = accel_noise_std
        self.g_world = np.array([0.0, gravity])

    def measure(self, v_k, theta_k, v_kp1, theta_kp1, dt):
        omega_true = (theta_kp1 - theta_k) / dt
        omega_meas = omega_true + self.bg + np.random.randn() * self.gyro_noise_std

        a_world = (v_kp1 - v_k) / dt
        accel_body = rot2d(theta_k).T @ (a_world - self.g_world)
        accel_meas = accel_body + self.ba + np.random.randn(2) * self.accel_noise_std
        return omega_true, omega_meas, accel_body, accel_meas


if __name__ == "__main__":
    dt = 0.01
    T = 10.0
    N = int(T / dt)
    t = np.linspace(0, T, N)

    # Trajectory
    radius = 5.0
    speed = 1.0
    omega = speed / radius  # rad/s

    theta = omega * t
    vx = -speed * np.sin(theta)
    vy = speed * np.cos(theta)

    v = np.stack([vx, vy], axis=1)

    # SIM
    imu = IMU()

    omega_true_log = []
    omega_meas_log = []
    acc_true_log = []
    acc_meas_log = []

    for k in range(N - 1):
        omega_t, omega_m, acc_t, acc_m = imu.measure(
            v[k], theta[k], v[k + 1], theta[k + 1], dt
        )

        omega_true_log.append(omega_t)
        omega_meas_log.append(omega_m)
        acc_true_log.append(acc_t)
        acc_meas_log.append(acc_m)

    omega_true_log = np.array(omega_true_log)
    omega_meas_log = np.array(omega_meas_log)
    acc_true_log = np.array(acc_true_log)
    acc_meas_log = np.array(acc_meas_log)

    plt.figure(figsize=(12, 8))

    plt.subplot(3, 1, 1)
    plt.plot(t[:-1], omega_true_log, label="Gyro Ground Truth")
    plt.plot(t[:-1], omega_meas_log, label="Gyro Measured", alpha=0.7)
    plt.ylabel("Angular Rate [rad/s]")
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 1, 2)
    plt.plot(t[:-1], acc_true_log[:, 0], label="Accel X Ground Truth")
    plt.plot(t[:-1], acc_meas_log[:, 0], label="Accel X Measured", alpha=0.7)
    plt.ylabel("Accel X [m/s²]")
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 1, 3)
    plt.plot(t[:-1], acc_true_log[:, 1], label="Accel Y Ground Truth")
    plt.plot(t[:-1], acc_meas_log[:, 1], label="Accel Y Measured", alpha=0.7)
    plt.xlabel("Time [s]")
    plt.ylabel("Accel Y [m/s²]")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()
