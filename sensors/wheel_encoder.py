import matplotlib.pyplot as plt
import numpy as np


class WheelEncoder:
    def __init__(
        self,
        pos_noise_std: float = 1e-3,
        vel_noise_std: float = 5e-3,
        seed: int | None = None,
    ):
        self.x_noise_std = pos_noise_std
        self.x_dot_noise_std = vel_noise_std
        self.rng = np.random.default_rng(seed)

    def measure(self, x: float, x_dot: float):
        x_meas = x + self.rng.normal(0.0, self.x_noise_std)
        x_dot_meas = x_dot + self.rng.normal(0.0, self.x_dot_noise_std)
        return x_meas, x_dot_meas


if __name__ == "__main__":
    wheel = WheelEncoder(
        pos_noise_std=0.1,
        vel_noise_std=0.01,
        seed=42,
    )

    a = 0.5
    v0 = 2.0
    x0 = 0.0

    dt = 0.1
    T = 10.0
    time = np.arange(0.0, T, dt)

    X_true = np.zeros((len(time), 2))
    X_meas = np.zeros((len(time), 2))

    for k, t in enumerate(time):
        x = x0 + v0 * t + 0.5 * a * t**2
        v = v0 + a * t

        x_m, v_m = wheel.measure(x, v)

        X_true[k] = [x, v]
        X_meas[k] = [x_m, v_m]

    fig, axs = plt.subplots(2, 1, sharex=True, figsize=(8, 6))

    axs[0].plot(time, X_true[:, 0], label="True x")
    axs[0].plot(time, X_meas[:, 0], ".", label="Measured x", alpha=0.6)
    axs[0].set_ylabel("Position [m]")
    axs[0].legend()
    axs[0].grid(True)

    axs[1].plot(time, X_true[:, 1], label="True x_dot")
    axs[1].plot(time, X_meas[:, 1], ".", label="Measured x_dot", alpha=0.6)
    axs[1].set_ylabel("Velocity [m/s]")
    axs[1].set_xlabel("Time [s]")
    axs[1].legend()
    axs[1].grid(True)

    plt.tight_layout()
    plt.show()
