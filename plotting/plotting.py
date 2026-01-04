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
        pend_y_est = L * np.cos(theta_est)
        pend_x_est = x_cart_est + L * np.sin(theta_est)
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
        (trace_line,) = ax.plot([], [], "r--", lw=1, alpha=0.8, label="True tip")
    else:
        trace_line = None

    if state_est_history is not None and est_trace:
        (est_trace_line,) = ax.plot([], [], "b--", lw=1, alpha=0.8, label="EKF tip")
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


def plot_true_vs_meas(time, x_true, x_meas, u=None, return_fig: bool = False):
    """
    x_true: (N, 4) [x, x_dot, theta, theta_dot]
    x_meas: (N, 4)
    """

    labels = [
        ("x", "Position [m]"),
        ("x_dot", "Velocity [m/s]"),
        ("theta", "Angle [rad]"),
        ("theta_dot", "Angular rate [rad/s]"),
    ]

    fig, axs = plt.subplots(4, 1, sharex=True, figsize=(9, 10))

    for i, (name, ylabel) in enumerate(labels):
        axs[i].plot(time, x_true[:, i], label=f"true {name}")
        axs[i].plot(time, x_meas[:, i], "--", label=f"measured {name}")
        axs[i].set_ylabel(ylabel)
        axs[i].grid(True)
        axs[i].legend()

    axs[-1].set_xlabel("Time [s]")

    if u is not None:
        ax_u = axs[-1].twinx()
        ax_u.plot(time, u, "k:", alpha=0.4, label="control u")
        ax_u.set_ylabel("Control")
        ax_u.legend(loc="lower right")

    plt.tight_layout()
    if return_fig:
        return fig
    plt.show()


def plot_imu_vs_ekf(
    system,
    imu,
    results: dict,
    return_fig: bool = False,
):
    time = results["time"]
    x_true = results["x_true"]
    x_ekf = results["x_meas"]
    x_imu = results["imu_meas"]

    acc_true_body = results["acc_true"]
    acc_meas = results["acc_meas"]
    u_hist = results["u"]

    N = len(time)

    acc_ekf = np.zeros_like(acc_true_body)
    for k in range(N):
        s = x_ekf[k][:4]
        uk = u_hist[k]
        _, x_ddot, _, theta_ddot = system.dynamics(s, uk)
        _, a_tip = system.tip_kinematics(s, x_ddot, theta_ddot)
        acc_ekf[k] = rot2d(s[2]).T @ (a_tip - imu.g_world)

    fig = plt.figure(figsize=(12, 12))

    plt.subplot(4, 1, 1)
    plt.plot(time, x_true[:, 2], label="True")
    plt.plot(time, x_ekf[:, 2], label="EKF")
    plt.plot(time, x_imu[:, 2], label="IMU-only", alpha=0.7)
    plt.ylabel("theta [rad]")
    plt.grid(True)
    plt.legend()

    plt.subplot(4, 1, 2)
    plt.plot(time, x_true[:, 3], label="True")
    plt.plot(time, x_ekf[:, 3], label="EKF")
    plt.plot(time, x_imu[:, 3], label="IMU-only", alpha=0.7)
    plt.ylabel("theta_dot [rad/s]")
    plt.grid(True)
    plt.legend()

    plt.subplot(4, 1, 3)
    plt.plot(time, acc_true_body[:, 0], label="True")
    plt.plot(time, acc_ekf[:, 0], label="EKF")
    plt.plot(time, acc_meas[:, 0], label="IMU", alpha=0.7)
    plt.ylabel("acc_x [m/s²]")
    plt.grid(True)
    plt.legend()

    plt.subplot(4, 1, 4)
    plt.plot(time, acc_true_body[:, 1], label="True")
    plt.plot(time, acc_ekf[:, 1], label="EKF")
    plt.plot(time, acc_meas[:, 1], label="IMU", alpha=0.7)
    plt.ylabel("acc_y [m/s²]")
    plt.xlabel("Time [s]")
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    if return_fig:
        return fig
    plt.show()


def plot_imu_ekf_errors(
    results: dict,
    angle_wrap: bool = True,
    return_fig: bool = False,
):
    time = results["time"]
    x_true = results["x_true"]
    x_ekf = results["x_meas"]
    x_imu = results["imu_meas"]

    e_theta_ekf = x_ekf[:, 2] - x_true[:, 2]
    e_theta_imu = x_imu[:, 2] - x_true[:, 2]

    if angle_wrap:
        e_theta_ekf = wrap_angle_pi(e_theta_ekf)
        e_theta_imu = wrap_angle_pi(e_theta_imu)

    e_w_ekf = x_ekf[:, 3] - x_true[:, 3]
    e_w_imu = x_imu[:, 3] - x_true[:, 3]

    print("\nCombined Error Summary (EKF vs IMU-only)")
    print("-" * 80)
    for err, name in [
        (e_theta_ekf, "theta EKF (rad)"),
        (e_theta_imu, "theta IMU-only (rad)"),
        (e_w_ekf, "theta_dot EKF (rad/s)"),
        (e_w_imu, "theta_dot IMU-only (rad/s)"),
    ]:
        s = summarize_error(err, name)
        print(
            f"{s['name']:<28}  "
            f"MAE={s['MAE']:.6f}  "
            f"RMSE={s['RMSE']:.6f}  "
            f"Bias={s['Bias(mean)']:.6f}  "
            f"Std(zm)={s['Std(zero-mean)']:.6f}"
        )
    print("-" * 80)

    fig = plt.figure(figsize=(12, 8))

    plt.subplot(2, 1, 1)
    plt.plot(time, e_theta_ekf, label="EKF")
    plt.plot(time, e_theta_imu, label="IMU-only", alpha=0.7)
    plt.ylabel("theta error [rad]")
    plt.grid(True)
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(time, e_w_ekf, label="EKF")
    plt.plot(time, e_w_imu, label="IMU-only", alpha=0.7)
    plt.ylabel("theta_dot error [rad/s]")
    plt.xlabel("Time [s]")
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    if return_fig:
        return fig
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
    return_fig: bool = False,
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
    fig = plt.figure(figsize=(12, max(3, 2.5 * cols)))
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
    if return_fig:
        return fig
    plt.show()


def plot_ekf_statistics(
    time,
    P_diag,
    meas_labels=None,
    state_labels=None,
    return_fig: bool = False,
):
    _, n = P_diag.shape
    if meas_labels is None:
        meas_labels = [f"z[{j}]" for j in range(m)]
    if state_labels is None:
        state_labels = [f"x[{i}]" for i in range(n)]

    fig = plt.figure(figsize=(12, 12))

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
    if return_fig:
        return fig
    plt.show()


def plot_control_comparison(
    ekf_results: dict,
    imu_results: dict,
    return_fig: bool = False,
):
    time = ekf_results["time"]
    u_ekf = ekf_results["u"]
    u_imu = imu_results["u"]

    fig = plt.figure(figsize=(12, 6))

    plt.plot(time, u_ekf, label="EKF control")
    plt.plot(time, u_imu, label="IMU-only control", alpha=0.7)
    plt.ylabel("u")
    plt.title("Control Signal Comparison")
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    if return_fig:
        return fig
    plt.show()
