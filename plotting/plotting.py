import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np

from sensors.imu import rot2d


def animate_cart_pendulum(
    t,
    state_history,
    L,
    trace=False,
    trace_length=None,
    state_est_history=None,
    est_trace=True,
):
    x_cart = state_history[:, 0]
    theta = state_history[:, 2]

    pend_x = x_cart + L * np.sin(theta)
    pend_y = L * np.cos(theta)

    if state_est_history is not None:
        x_cart_est = state_est_history[:, 0]
        theta_est = state_est_history[:, 2]
        pend_x_est = x_cart + L * np.sin(theta_est)
        pend_y_est = L * np.cos(theta_est)
        # pend_x_est = x_cart_est + L * np.sin(theta_est) # TODO: if wheel decoder needed add x_est back
        # pend_y_est = L * np.cos(theta_est)
    else:
        pend_x_est = pend_y_est = None

    fig, ax = plt.subplots(figsize=(8, 4))

    ax.set_xlim(np.min(x_cart) - 1, np.max(x_cart) + 1)
    ax.set_ylim(-0.5, L + 0.5)
    ax.set_aspect("equal")
    ax.grid(True)

    cart_width = 0.3
    cart_height = 0.2
    (cart_line,) = ax.plot([], [], "k-", lw=2)
    (rod_line,) = ax.plot([], [], "r-", lw=2, label="True")

    (bob_point,) = ax.plot([], [], "ro", markersize=6)

    if trace:
        (trace_line,) = ax.plot([], [], "b--", lw=1, alpha=0.6, label="True tip")
    else:
        trace_line = None

    if state_est_history is not None and est_trace:
        (est_trace_line,) = ax.plot([], [], "g--", lw=1, alpha=0.6, label="EKF tip")
    else:
        est_trace_line = None

    def init():
        cart_line.set_data([], [])
        rod_line.set_data([], [])
        bob_point.set_data([], [])
        if trace_line:
            trace_line.set_data([], [])
        if est_trace_line:
            est_trace_line.set_data([], [])
        return (
            cart_line,
            rod_line,
            bob_point,
            trace_line,
            est_trace_line,
        )

    def update(i):
        x = x_cart[i]
        cart_x = [
            x - cart_width / 2,
            x + cart_width / 2,
            x + cart_width / 2,
            x - cart_width / 2,
            x - cart_width / 2,
        ]
        cart_y = [
            0,
            0,
            cart_height,
            cart_height,
            0,
        ]
        cart_line.set_data(cart_x, cart_y)

        rod_line.set_data(
            [x_cart[i], pend_x[i]],
            [cart_height, pend_y[i] + cart_height],
        )

        bob_point.set_data(
            [pend_x[i]],
            [pend_y[i] + cart_height],
        )

        if trace_line:
            if trace_length:
                start = max(0, i - trace_length)
            else:
                start = 0
            trace_line.set_data(
                pend_x[start:i],
                pend_y[start:i] + cart_height,
            )

        if est_trace_line and pend_x_est is not None:
            if trace_length:
                start = max(0, i - trace_length)
            else:
                start = 0
            est_trace_line.set_data(
                pend_x_est[start:i],
                pend_y_est[start:i] + cart_height,
            )

        return (
            cart_line,
            rod_line,
            bob_point,
            trace_line,
            est_trace_line,
        )

    ani = animation.FuncAnimation(
        fig,
        update,
        frames=len(t),
        init_func=init,
        interval=30,
        blit=True,
    )

    plt.show()


def plot_imu_ekf_vs_truth(
    time,
    system,
    imu,
    x_true,
    x_ekf,
    theta_gyro,
    omega_meas,
    acc_true_body,
    acc_meas,
    u_hist,
):
    N = len(time)
    acc_ekf = np.zeros_like(acc_true_body)
    for k in range(N):
        xk = x_ekf[k]
        s = xk[:4]
        uk = u_hist[k] if k < len(u_hist) else 0.0
        x_dot, x_ddot, theta_dot, theta_ddot = system.dynamics(s, uk)
        _, a_tip = system.tip_kinematics(s, x_ddot, theta_ddot)
        theta = s[2]
        acc_ekf[k] = rot2d(theta).T @ (a_tip - imu.g_world)

    plt.figure(figsize=(12, 12))

    # Angle
    plt.subplot(4, 1, 1)
    plt.plot(time, x_true[:, 2], label="True theta")
    plt.plot(time, x_ekf[:, 2], label="EKF theta")
    if theta_gyro is not None:
        plt.plot(time, theta_gyro, label="Gyro-integrated theta", alpha=0.7)
    plt.ylabel("theta [rad]")
    plt.grid(True)
    plt.legend()

    # Angular rate
    plt.subplot(4, 1, 2)
    plt.plot(time, x_true[:, 3], label="True theta_dot")
    plt.plot(time, x_ekf[:, 3], label="EKF theta_dot")
    plt.plot(time, omega_meas, label="Gyro measured", alpha=0.7)
    plt.ylabel("theta_dot [rad/s]")
    plt.grid(True)
    plt.legend()

    # Accel X (body)
    plt.subplot(4, 1, 3)
    plt.plot(time, acc_true_body[:, 0], label="True acc_x (body)")
    plt.plot(time, acc_ekf[:, 0], label="EKF acc_x (body)")
    plt.plot(time, acc_meas[:, 0], label="IMU acc_x", alpha=0.7)
    plt.ylabel("acc_x [m/s^2]")
    plt.grid(True)
    plt.legend()

    # Accel Y (body)
    plt.subplot(4, 1, 4)
    plt.plot(time, acc_true_body[:, 1], label="True acc_y (body)")
    plt.plot(time, acc_ekf[:, 1], label="EKF acc_y (body)")
    plt.plot(time, acc_meas[:, 1], label="IMU acc_y", alpha=0.7)
    plt.ylabel("acc_y [m/s^2]")
    plt.xlabel("Time [s]")
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.show()


def plot_imu_ekf_error_overlay(
    time,
    system,
    imu,
    x_true,
    x_ekf,
    theta_gyro,
    omega_meas,
    acc_true_body,
    acc_meas,
    u_hist,
    angle_wrap=True,
):
    """
    Overlay error plots: EKF vs IMU errors on same axes.
    - Angle error: (EKF theta - true) vs (gyro-integrated - true)
    - Angular rate error: (EKF theta_dot - true) vs (gyro - true)
    - Accel errors: EKF predicted specific force vs IMU vs true specific force
    """
    N = len(time)
    # EKF predicted body-frame accelerations
    acc_ekf = np.zeros_like(acc_true_body)
    for k in range(N):
        xk = x_ekf[k]
        s = xk[:4]
        uk = u_hist[k] if k < len(u_hist) else 0.0
        x_dot, x_ddot, theta_dot, theta_ddot = system.dynamics(s, uk)
        _, a_tip = system.tip_kinematics(s, x_ddot, theta_ddot)
        theta = s[2]
        acc_ekf[k] = rot2d(theta).T @ (a_tip - imu.g_world)

    # Errors
    e_theta_ekf = x_ekf[:, 2] - x_true[:, 2]
    e_theta_gyro = theta_gyro - x_true[:, 2]
    if angle_wrap:
        e_theta_ekf = wrap_angle_pi(e_theta_ekf)
        e_theta_gyro = wrap_angle_pi(e_theta_gyro)

    e_w_ekf = x_ekf[:, 3] - x_true[:, 3]
    e_w_imu = omega_meas - x_true[:, 3]

    e_ax_ekf = acc_ekf[:, 0] - acc_true_body[:, 0]
    e_ax_imu = acc_meas[:, 0] - acc_true_body[:, 0]

    e_ay_ekf = acc_ekf[:, 1] - acc_true_body[:, 1]
    e_ay_imu = acc_meas[:, 1] - acc_true_body[:, 1]

    print("\nCombined Error Summary (EKF vs IMU)")
    print("-" * 80)
    for err, name in [
        (e_theta_ekf, "theta EKF (rad)"),
        (e_theta_gyro, "theta gyro (rad)"),
        (e_w_ekf, "theta_dot EKF (rad/s)"),
        (e_w_imu, "theta_dot gyro (rad/s)"),
        (e_ax_ekf, "acc_x EKF (m/s^2)"),
        (e_ax_imu, "acc_x IMU (m/s^2)"),
        (e_ay_ekf, "acc_y EKF (m/s^2)"),
        (e_ay_imu, "acc_y IMU (m/s^2)"),
    ]:
        s = summarize_error(err, name)
        print(
            f"{s['name']:<28}  MAE={s['MAE']:.6f}  RMSE={s['RMSE']:.6f}  "
            f"Bias={s['Bias(mean)']:.6f}  Std(zm)={s['Std(zero-mean)']:.6f}"
        )
    print("-" * 80)

    # Plots
    plt.figure(figsize=(12, 10))

    plt.subplot(4, 1, 1)
    plt.plot(time, e_theta_ekf, label="EKF theta error")
    plt.plot(time, e_theta_gyro, label="Gyro-integrated theta error", alpha=0.7)
    plt.ylabel("rad")
    plt.title("Angle Error")
    plt.grid(True)
    plt.legend()

    plt.subplot(4, 1, 2)
    plt.plot(time, e_w_ekf, label="EKF theta_dot error")
    plt.plot(time, e_w_imu, label="Gyro error", alpha=0.7)
    plt.ylabel("rad/s")
    plt.title("Angular Rate Error")
    plt.grid(True)
    plt.legend()

    plt.subplot(4, 1, 3)
    plt.plot(time, e_ax_ekf, label="EKF acc_x error")
    plt.plot(time, e_ax_imu, label="IMU acc_x error", alpha=0.7)
    plt.ylabel("m/s^2")
    plt.title("Accel X Error (body)")
    plt.grid(True)
    plt.legend()

    plt.subplot(4, 1, 4)
    plt.plot(time, e_ay_ekf, label="EKF acc_y error")
    plt.plot(time, e_ay_imu, label="IMU acc_y error", alpha=0.7)
    plt.ylabel("m/s^2")
    plt.xlabel("Time [s]")
    plt.title("Accel Y Error (body)")
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.show()


def wrap_angle_pi(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def summarize_error(e, name):
    mae = np.mean(np.abs(e))
    rmse = np.sqrt(np.mean(e**2))
    bias = np.mean(e)
    std_zeromean = np.std(e - bias)
    return {
        "name": name,
        "MAE": mae,
        "RMSE": rmse,
        "Bias(mean)": bias,
        "Std(zero-mean)": std_zeromean,
    }


def plot_kalman_gain(
    time,
    K_hist,
    state_labels=None,
    meas_labels=None,
    state_rows=None,
    meas_cols=None,
):
    N, n, m = K_hist.shape
    if state_rows is None:
        state_rows = list(range(n))
    if meas_cols is None:
        meas_cols = list(range(m))
    if state_labels is None:
        state_labels = [f"x[{i}]" for i in range(n)]
    if meas_labels is None:
        meas_labels = [f"z[{j}]" for j in range(m)]

    cols = len(meas_cols)
    plt.figure(figsize=(12, max(3, 2.5 * cols)))
    for idx, j in enumerate(meas_cols, start=1):
        plt.subplot(cols, 1, idx)
        for i in state_rows:
            plt.plot(time, K_hist[:, i, j], label=state_labels[i])
        plt.title(f"Kalman Gain for measurement: {meas_labels[j]}")
        plt.ylabel("K entries")
        plt.grid(True)
        plt.legend(ncol=4, fontsize=9)
        if idx == cols:
            plt.xlabel("Time [s]")
    plt.tight_layout()
    plt.show()


def plot_ekf_statistics(
    time,
    P_diag,
    meas_labels=None,
    state_labels=None,
):
    _, n = P_diag.shape
    if meas_labels is None:
        meas_labels = [f"z[{j}]" for j in range(m)]
    if state_labels is None:
        state_labels = [f"x[{i}]" for i in range(n)]

    plt.figure(figsize=(12, 12))

    split_idx = 4 if n >= 7 else n // 2

    plt.subplot(2, 1, 1)
    for i in range(min(split_idx, n)):
        plt.plot(time, P_diag[:, i], label=state_labels[i])
    plt.ylabel("Var")
    plt.title("Covariance diag (dynamics)")
    plt.grid(True)
    plt.legend(ncol=4, fontsize=9)

    plt.subplot(2, 1, 2)
    for i in range(split_idx, n):
        plt.plot(time, P_diag[:, i], label=state_labels[i])
    plt.ylabel("Var")
    plt.title("Covariance diag (bias/others)")
    plt.grid(True)
    if n - split_idx > 0:
        plt.legend(ncol=4, fontsize=9)
    plt.legend()

    plt.tight_layout()
    plt.show()
