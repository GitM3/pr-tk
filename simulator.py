import numpy as np

from controller.lqr import LQRController
from dynamics.inverted_pendulum import InvertedPendulumCart
from estimation.ekf import EKF
from plotting.plotting import (
    animate_cart_pendulum,
    plot_ekf_statistics,
    plot_imu_ekf_error_overlay,
    plot_imu_ekf_vs_truth,
    plot_kalman_gain,
    plot_true_vs_meas,
)
from sensors.imu import IMU, rot2d
from sensors.wheel_encoder import WheelEncoder

SEED = 69 * 69 * 42 * 9


def rk4_step(f, x, u, dt):
    k1 = f(x, u)
    k2 = f(x + 0.5 * dt * k1, u)
    k3 = f(x + 0.5 * dt * k2, u)
    k4 = f(x + dt * k3, u)
    return x + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)


def simulate(system, x0, T, dt, controller=None):
    N = int(T / dt)
    x = x0.copy()

    history = np.zeros((N, len(x0)))
    time = np.linspace(0, T, N)

    for k in range(N):
        u = controller(x, time[k]) if controller else 0.0
        history[k] = x
        x = rk4_step(system.dynamics, x, u, dt)

    return time, history


def integrate_gyro(omega_meas, theta0, dt):
    theta_imu = np.zeros_like(omega_meas)
    theta_imu[0] = theta0

    for k in range(1, len(omega_meas)):
        theta_imu[k] = theta_imu[k - 1] + omega_meas[k - 1] * dt

    return theta_imu


def simulate_with_imu(system, imu, encoder, x0, T, dt, controller=None):
    N = int(T / dt)
    time = np.linspace(0, T, N)

    x = x0.copy()
    x_hat = x0.copy()  # Prior unrealistic?
    state_hist = np.zeros((N, len(x0)))
    est_hist = np.zeros((N, len(x0)))

    acc_true_log = np.zeros((N, 2))
    acc_meas_log = np.zeros((N, 2))
    u_hist = np.zeros(N)
    theta_meas = x0[2]

    for k in range(N):
        u = controller(x_hat[:4], time[k]) if controller else 0.0
        x = rk4_step(system.dynamics, x, u, dt)

        x_dot, x_ddot, theta_dot, theta_ddot = system.dynamics(x, u)
        v_tip, a_tip = system.tip_kinematics(x, x_ddot, theta_ddot)

        x_enc, x_dot_enc = encoder.measure(x[0], x[1])
        omega_meas, acc_meas = imu.measure(
            theta=x[2], theta_dot=theta_dot, a_world=a_tip, dt=dt
        )
        theta_meas = theta_meas + omega_meas * dt

        x_hat = np.array([x_enc, x_dot_enc, theta_meas, omega_meas])

        u_hist[k] = u
        state_hist[k] = x
        acc_true_log[k] = rot2d(x[2]).T @ (a_tip - imu.g_world)
        acc_meas_log[k] = acc_meas
        est_hist[k] = x_hat

    return {
        "time": time,
        "x_true": state_hist,
        "x_meas": est_hist,
        "acc_true": acc_true_log,
        "acc_meas": acc_meas_log,
        "u": u_hist,
    }


def f_disc(x, u):
    # State: [x, x_dot, theta, theta_dot, b_g, b_ax, b_ay]
    s = x[:4]
    b = x[4:]
    s_next = rk4_step(system.dynamics, s, u, dt)
    return np.hstack([s_next, b])


def simulate_with_ekf(system, imu, encoder, x0, T, dt, controller=None):
    N = int(T / dt)
    time = np.linspace(0, T, N)
    g_world = imu.g_world

    def h_meas(x, u):
        x_pos, x_dot, theta, theta_dot = x[:4]
        b_g, b_ax, b_ay = x[4:]

        x_dot_d, x_ddot, theta_dot_d, theta_ddot = system.dynamics(x[:4], u)
        _, a_tip = system.tip_kinematics(x[:4], x_ddot, theta_ddot)

        acc_body = rot2d(theta).T @ (a_tip - g_world)

        return np.array(
            [
                x_pos,
                x_dot,
                theta_dot + b_g,
                acc_body[0] + b_ax,
                acc_body[1] + b_ay,
            ]
        )

    x = x0.copy()
    state_hist = np.zeros((N, len(x0)))
    est_hist = np.zeros((N, len(x0)))

    acc_true_log = np.zeros((N, 2))
    acc_meas_log = np.zeros((N, 2))
    u_hist = np.zeros(N)
    theta_meas = x0[2]

    Q = np.diag(
        [
            encoder.x_noise_std**2,
            encoder.x_dot_noise_std**2,
            imu.gyro_noise_std**2,
            imu.accel_noise_std**2,
            imu.accel_noise_std**2,
        ]
    )
    # Process noise for [x, x_dot, theta, theta_dot, b_g, b_ax, b_ay]
    R = np.diag([1e-5, 1e-3, 1e-6, 1e-3, 1e-8, 1e-6, 1e-6])

    ekf = EKF(
        g_motion=f_disc,
        h_meas=h_meas,
        R_t=R,
        Q_t=Q,
        state_dim=7,
        meas_dim=5,
        eps_jac=1e-6,
    )
    x_hat = np.hstack([x0.copy(), np.array([0.0, 0.0, 0.0])])
    P = np.diag([1e-2, 1e-1, 1e-3, 1e-1, 1e-2, 1e-1, 1e-1])
    K_hist = np.zeros((N, 7, 5))
    P_diag_hist = np.zeros((N, 7))

    for k in range(N):
        u = controller(x_hat[:4], time[k]) if controller else 0.0
        x = rk4_step(system.dynamics, x, u, dt)

        x_dot, x_ddot, theta_dot, theta_ddot = system.dynamics(x, u)
        v_tip, a_tip = system.tip_kinematics(x, x_ddot, theta_ddot)

        x_enc, x_dot_enc = encoder.measure(x[0], x[1])
        omega_meas, acc_meas = imu.measure(
            theta=x[2], theta_dot=theta_dot, a_world=a_tip, dt=dt
        )
        theta_meas = theta_meas + omega_meas * dt
        z = np.array([x_enc, x_dot_enc, omega_meas, acc_meas[0], acc_meas[1]])
        x_hat, P, _, _, K, _, _ = ekf.step(x_hat, P, z, u)

        P_diag_hist[k] = np.diag(P)
        K_hist[k] = K
        u_hist[k] = u
        state_hist[k] = x
        acc_true_log[k] = rot2d(x[2]).T @ (a_tip - imu.g_world)
        acc_meas_log[k] = acc_meas
        est_hist[k] = x_hat[:4]

    return {
        "time": time,
        "x_true": state_hist,
        "x_meas": est_hist,
        "acc_true": acc_true_log,
        "acc_meas": acc_meas_log,
        "u": u_hist,
        "K": K_hist,
        "P": P_diag_hist,
    }


def run_simulate_imu_only(system, T, dt):
    """
    Without EKF or LQR controller
    """
    imu = IMU(seed=SEED)
    encoder = WheelEncoder()
    # x  # x_dot  # theta  # theta_dot
    x0 = np.array([0.0, 0.0, np.deg2rad(1), 0.0])
    results = simulate_with_imu(system, imu, encoder, x0, T, dt)
    plot_true_vs_meas(
        time=results["time"], x_true=results["x_true"], x_meas=results["x_meas"]
    )
    animate_cart_pendulum(
        results["time"],
        results["x_true"],
        L,
        trace=True,
        trace_length=200,
        state_est_history=results["x_meas"],
        est_trace=True,
    )


def run_simulate_ekf_only(system, T, dt):
    """
    Without LQR controller
    """
    imu = IMU()
    encoder = WheelEncoder()
    # x  # x_dot  # theta  # theta_dot
    x0 = np.array([0.0, 0.0, np.deg2rad(5), 0.0])
    results = simulate_with_ekf(system, imu, encoder, x0, T, dt)
    plot_true_vs_meas(
        time=results["time"], x_true=results["x_true"], x_meas=results["x_meas"]
    )
    animate_cart_pendulum(
        results["time"],
        results["x_true"],
        L,
        trace=True,
        trace_length=200,
        state_est_history=results["x_meas"],
        est_trace=True,
    )
    state_labels = ["x", "x_dot", "theta", "theta_dot", "b_g", "b_ax", "b_ay"]
    meas_labels = ["x", "x_dot", "omega", "acc_x", "acc_y"]
    plot_kalman_gain(
        results["time"],
        results["K"],
        meas_labels=meas_labels,
        state_labels=state_labels,
    )
    plot_ekf_statistics(
        results["time"],
        P_diag=results["P"],
        meas_labels=meas_labels,
        state_labels=state_labels,
    )


if __name__ == "__main__":
    M = 1.0
    m = 0.2  # TODO: butler bot params
    L = 0.5
    system = InvertedPendulumCart(M, m, L, B_M=0.9, B_m=0.05)

    T = 10.0
    dt = 0.01

    # run_simulate_imu_only(system, T, dt)
    run_simulate_ekf_only(system, T, dt)

    # x0 = np.array([0.0, -0.5, np.deg2rad(45), -0.01])
    # imu = IMU()
    # lqr = LQRController(
    #     system,
    #     Q=np.diag([1.0, 1.0, 10.0, 10.0]),
    #     R=np.array([[10.0]]),
    #     x_ref=np.array([3.5, 0.0, 0.0, 0.0]),
    #     u_limit=50.0,
    #     alpha=1.0,
    # )
    # (
    #     time,
    #     state_hist,
    #     est_hist,
    #     theta_t,
    #     theta_m,
    #     omega_t,
    #     omega_m,
    #     acc_t,
    #     acc_m,
    #     u_hist,
    #     stats,
    # ) = simulate_with_imu_and_ekf(system, imu, x0, T, dt, controller=lqr)
    #
    #
    # plot_imu_ekf_error_overlay(
    #     time=time,
    #     system=system,
    #     imu=imu,
    #     x_true=state_hist,
    #     x_ekf=est_hist,
    #     theta_gyro=theta_m,
    #     omega_meas=omega_m,
    #     acc_true_body=acc_t,
    #     acc_meas=acc_m,
    #     u_hist=u_hist,
    # )
    #
    # animate_cart_pendulum(
    #     time,
    #     state_hist,
    #     L,
    #     trace=True,
    #     trace_length=100,
    #     state_est_history=est_hist,
    #     est_trace=True,
    # )
