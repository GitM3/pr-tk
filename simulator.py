import numpy as np

from dynamics.inverted_pendulum import InvertedPendulumCart
from plotting.plotting import (
    animate_cart_pendulum,
    plot_imu_errors,
    plot_imu_vs_truth,
    plot_states,
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

        # Use current state's accelerations from dynamics
        x_dot, x_ddot, theta_dot, theta_ddot = system.dynamics(x, u)

        # Tip kinematics
        v_tip, a_tip = system.tip_kinematics(x, x_ddot, theta_ddot)

        # IMU measurement at the tip
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


if __name__ == "__main__":
    M = 1.0
    m = 0.2  # TODO: simulate butler bot
    L = 0.5
    system = InvertedPendulumCart(M, m, L, B_M=0.5)
    # x  # x_dot  # theta  # theta_dot
    x0 = np.array([0.5, 0.0, np.deg2rad(10), 0.0])

    T = 10.0
    dt = 0.01

    # without IMU:
    # t, history = simulate(system, x0, T, dt)
    # plot_states(t, history)
    # animate_cart_pendulum(
    #     t,
    #     history,
    #     L=L,
    #     trace=True,
    #     trace_length=10,
    # )

    imu = IMU()
    time, state_hist, theta_t, theta_m, omega_t, omega_m, acc_t, acc_m = (
        simulate_with_imu(
            system=system,
            imu=imu,
            x0=x0,
            T=T,
            dt=dt,
        )
    )

    plot_imu_vs_truth(
        time,
        theta_t,
        theta_m,
        omega_t,
        omega_m,
        acc_t,
        acc_m,
    )
    plot_imu_errors(time, theta_t, theta_m, omega_t, omega_m, acc_t, acc_m)
