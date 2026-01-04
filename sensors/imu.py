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
        gyro_bg_rw_std=0.1,
        acc_bg_rw_std=0.1,
        accel_noise_std=0.2,
        gravity=9.81,
        seed=100,
    ):

        self.bg0 = gyro_bias
        self.ba0 = accel_bias
        self.bg = gyro_bias
        self.bg_rw_std = gyro_bg_rw_std
        self.ba_rw_std = acc_bg_rw_std
        self.ba = accel_bias
        self.gyro_noise_std = gyro_noise_std
        self.accel_noise_std = accel_noise_std
        self.g_world = np.array([0.0, -1 * gravity])
        self.seed = seed
        self.rng = np.random.default_rng(seed)

    def measure(self, theta, theta_dot, a_world, dt):
        omega_true = theta_dot
        omega_meas = omega_true + self.bg + self.rng.normal(0.0, self.gyro_noise_std)

        accel_true_body = rot2d(theta).T @ (a_world - self.g_world)
        accel_meas = (
            accel_true_body
            + self.ba
            + self.rng.normal(0.0, self.accel_noise_std, size=2)
        )

        self.bg += self.rng.normal(0.0, self.bg_rw_std * np.sqrt(dt))
        self.ba += self.rng.normal(0.0, self.ba_rw_std * np.sqrt(dt))
        return omega_meas, accel_meas

    def reset(self):
        self.bg = self.bg0
        self.ba = self.ba0
        self.rng = np.random.default_rng(self.seed)


if __name__ == "__main__":
    dt = 0.01
    T = 10.0
    N = int(T / dt)
    t = np.linspace(0, T, N)

    R = 5.0
    omega = 0.5  # rad/s

    theta = omega * t
    theta_dot = np.full_like(t, omega)

    a_world = np.zeros((N, 2))
    a_world[:, 0] = -R * omega**2 * np.cos(theta)
    a_world[:, 1] = -R * omega**2 * np.sin(theta)

    imu = IMU()

    omega_true_log = []
    omega_meas_log = []
    acc_true_log = []
    acc_meas_log = []

    for k in range(N):
        omega_m, acc_m = imu.measure(theta[k], theta_dot[k], a_world[k], dt)

        omega_true_log.append(theta_dot[k])
        omega_meas_log.append(omega_m)
        acc_true_body = rot2d(theta[k]).T @ (a_world[k] - imu.g_world)
        acc_true_log.append(acc_true_body)
        acc_meas_log.append(acc_m)

    omega_true_log = np.array(omega_true_log)
    omega_meas_log = np.array(omega_meas_log)
    acc_true_log = np.array(acc_true_log)
    acc_meas_log = np.array(acc_meas_log)

    plt.figure(figsize=(12, 8))

    plt.subplot(3, 1, 1)
    plt.plot(t, omega_true_log, label="Gyro Ground Truth")
    plt.plot(t, omega_meas_log, label="Gyro Measured", alpha=0.7)
    plt.ylabel("Angular Rate [rad/s]")
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 1, 2)
    plt.plot(t, acc_true_log[:, 0], label="Accel X Ground Truth")
    plt.plot(t, acc_meas_log[:, 0], label="Accel X Measured", alpha=0.7)
    plt.ylabel("Accel X [m/s²]")
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 1, 3)
    plt.plot(t, acc_true_log[:, 1], label="Accel Y Ground Truth")
    plt.plot(t, acc_meas_log[:, 1], label="Accel Y Measured", alpha=0.7)
    plt.xlabel("Time [s]")
    plt.ylabel("Accel Y [m/s²]")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()
