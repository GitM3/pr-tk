import numpy as np

from controller.lqr import LQRController
from dynamics.inverted_pendulum import InvertedPendulumCart
from estimation.ekf import EKF
from plotting.plotting import (
    animate_cart_pendulum,
    plot_imu_ekf_error_overlay,
    plot_imu_ekf_vs_truth,
)
from sensors.imu import IMU, rot2d


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


def simulate_with_imu(system, imu, x0, T, dt, controller=None):
    N = int(T / dt)
    time = np.linspace(0, T, N)

    x = x0.copy()
    state_hist = np.zeros((N, len(x0)))

    theta_true_log = np.zeros(N)
    omega_true_log = np.zeros(N)
    omega_meas_log = np.zeros(N)
    acc_true_log = np.zeros((N, 2))
    acc_meas_log = np.zeros((N, 2))

    for k in range(N):
        u = controller(x, time[k]) if controller else 0.0
        state_hist[k] = x

        x_dot, x_ddot, theta_dot, theta_ddot = system.dynamics(x, u)
        v_tip, a_tip = system.tip_kinematics(x, x_ddot, theta_ddot)
        theta = x[2]
        omega_meas, acc_meas = imu.measure(
            theta=theta, theta_dot=theta_dot, a_world=a_tip
        )
        theta_true_log[k] = theta
        omega_true_log[k] = theta_dot
        omega_meas_log[k] = omega_meas
        acc_true_log[k] = rot2d(x[2]).T @ (a_tip - imu.g_world)
        acc_meas_log[k] = acc_meas

        # Integrate plant forward
        x = rk4_step(system.dynamics, x, u, dt)
    theta_meas_log = integrate_gyro(omega_meas_log, theta_true_log[0], dt)
    return (
        time,
        state_hist,
        theta_true_log,
        theta_meas_log,
        omega_true_log,
        omega_meas_log,
        acc_true_log,
        acc_meas_log,
    )


def f_disc(x, u):
    # State: [x, x_dot, theta, theta_dot, b_g, b_ax, b_ay]
    s = x[:4]
    b = x[4:]
    s_next = rk4_step(system.dynamics, s, u, dt)
    return np.hstack([s_next, b])


def simulate_with_imu_and_ekf(system, imu, x0, T, dt, controller=None):
    N = int(T / dt)
    time = np.linspace(0, T, N)

    g_world = imu.g_world

    def h_meas(x, u):
        # Measurement model: [omega, ax_body, ay_body] including biases
        s = x[:4]
        b_g, b_ax, b_ay = x[4:]
        x_dot, x_ddot, theta_dot, theta_ddot = system.dynamics(
            s, u
        )  # TODO: xdot from wheel encoder?
        _, a_tip = system.tip_kinematics(s, x_ddot, theta_ddot)
        theta = s[2]
        acc_body = rot2d(theta).T @ (a_tip - g_world)
        return np.array([theta_dot + b_g, acc_body[0] + b_ax, acc_body[1] + b_ay])

    Q = np.diag([imu.gyro_noise_std**2, imu.accel_noise_std**2, imu.accel_noise_std**2])
    # Process noise for [x, x_dot, theta, theta_dot, b_g, b_ax, b_ay]
    R = np.diag([1e-5, 1e-3, 1e-6, 1e-3, 1e-8, 1e-6, 1e-6])

    ekf = EKF(
        g_motion=f_disc,
        h_meas=h_meas,
        R_t=R,
        Q_t=Q,
        state_dim=7,
        meas_dim=3,
        eps_jac=1e-6,
    )

    x = x0.copy()
    x_hat = np.hstack([x0.copy(), np.array([0.0, 0.0, 0.0])])
    P = np.diag([1e-2, 1e-1, 1e-3, 1e-1, 1e-2, 1e-1, 1e-1])

    state_hist = np.zeros((N, len(x0)))
    est_hist = np.zeros((N, 7))
    theta_true_log = np.zeros(N)
    omega_true_log = np.zeros(N)
    omega_meas_log = np.zeros(N)
    acc_true_log = np.zeros((N, 2))
    acc_meas_log = np.zeros((N, 2))
    u_hist = np.zeros(N)

    for k in range(N):
        u = controller(x, time[k]) if controller else 0.0
        u_hist[k] = u
        state_hist[k] = x

        x_dot, x_ddot, theta_dot, theta_ddot = system.dynamics(x, u)
        _, a_tip = system.tip_kinematics(x, x_ddot, theta_ddot)

        theta = x[2]
        omega_meas, acc_meas = imu.measure(
            theta=theta, theta_dot=theta_dot, a_world=a_tip
        )

        theta_true_log[k] = theta
        omega_true_log[k] = theta_dot
        omega_meas_log[k] = omega_meas
        acc_true_log[k] = rot2d(theta).T @ (a_tip - imu.g_world)
        acc_meas_log[k] = acc_meas

        # EKF update with IMU z = [omega, ax, ay]
        z = np.array([omega_meas, acc_meas[0], acc_meas[1]])
        x_hat, P, _, _ = ekf.step(x_hat, P, z, u)
        est_hist[k] = x_hat

        # Integrate plant forward
        x = rk4_step(system.dynamics, x, u, dt)

    theta_meas_log = integrate_gyro(omega_meas_log, theta_true_log[0], dt)

    return (
        time,
        state_hist,
        est_hist,
        theta_true_log,
        theta_meas_log,
        omega_true_log,
        omega_meas_log,
        acc_true_log,
        acc_meas_log,
        u_hist,
    )


if __name__ == "__main__":
    M = 1.0
    m = 0.2  # TODO: butler bot params
    L = 0.5
    system = InvertedPendulumCart(M, m, L, B_M=0.5)
    # x  # x_dot  # theta  # theta_dot
    x0 = np.array([0.0, 0.0, np.deg2rad(10), 0.0])

    T = 10.0
    dt = 0.01

    # imu = IMU()
    # # WITHOUT CONTROLLER:
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
    # ) = simulate_with_imu_and_ekf(system, imu, x0, T, dt)
    #
    # plot_imu_ekf_vs_truth(
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
    #     trace_length=200,
    #     state_est_history=est_hist,
    #     est_trace=True,
    #     theta_gyro=theta_m,
    #     gyro_trace=True,
    # )

    # WITH controller
    lqr = LQRController(
        system,
        Q=np.diag([1.0, 1.0, 10.0, 100.0]),
        R=np.array([[10.0]]),
        x_ref=np.array([0.0, 0.0, 0.0, 0.0]),
        u_limit=50.0,
        alpha=1.0,
    )
    imu = IMU()
    (
        time,
        state_hist,
        est_hist,
        theta_t,
        theta_m,
        omega_t,
        omega_m,
        acc_t,
        acc_m,
        u_hist,
    ) = simulate_with_imu_and_ekf(system, imu, x0, T, dt, controller=lqr)

    plot_imu_ekf_vs_truth(
        time=time,
        system=system,
        imu=imu,
        x_true=state_hist,
        x_ekf=est_hist,
        theta_gyro=theta_m,
        omega_meas=omega_m,
        acc_true_body=acc_t,
        acc_meas=acc_m,
        u_hist=u_hist,
    )

    plot_imu_ekf_error_overlay(
        time=time,
        system=system,
        imu=imu,
        x_true=state_hist,
        x_ekf=est_hist,
        theta_gyro=theta_m,
        omega_meas=omega_m,
        acc_true_body=acc_t,
        acc_meas=acc_m,
        u_hist=u_hist,
    )

    animate_cart_pendulum(
        time,
        state_hist,
        L,
        trace=True,
        trace_length=200,
        state_est_history=est_hist,
        est_trace=True,
        theta_gyro=theta_m,
        gyro_trace=True,
    )
