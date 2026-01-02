import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np


def plot_states(t, x):
    labels = ["x", "x_dot", "theta", "theta_dot"]
    _, axs = plt.subplots(4, 1, sharex=True, figsize=(8, 8))

    for i in range(4):
        axs[i].plot(t, x[:, i])
        axs[i].set_ylabel(labels[i])
        axs[i].grid(True)

    axs[-1].set_xlabel("Time [s]")
    plt.tight_layout()
    plt.show()


def animate_cart_pendulum(
    t,
    state_history,
    L,
    trace=False,
    trace_length=None,
):
    x_cart = state_history[:, 0]
    theta = state_history[:, 2]

    pend_x = x_cart + L * np.sin(theta)
    pend_y = L * np.cos(theta)

    fig, ax = plt.subplots(figsize=(8, 4))

    ax.set_xlim(np.min(x_cart) - 1, np.max(x_cart) + 1)
    ax.set_ylim(-0.5, L + 0.5)
    ax.set_aspect("equal")
    ax.grid(True)

    cart_width = 0.3
    cart_height = 0.2
    (cart_line,) = ax.plot([], [], "k-", lw=2)
    (rod_line,) = ax.plot([], [], "r-", lw=2)

    (bob_point,) = ax.plot([], [], "ro", markersize=6)

    if trace:
        (trace_line,) = ax.plot([], [], "b--", lw=1, alpha=0.6)
    else:
        trace_line = None

    def init():
        cart_line.set_data([], [])
        rod_line.set_data([], [])
        bob_point.set_data([], [])
        if trace_line:
            trace_line.set_data([], [])
        return cart_line, rod_line, bob_point, trace_line

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

        return cart_line, rod_line, bob_point, trace_line

    ani = animation.FuncAnimation(
        fig,
        update,
        frames=len(t),
        init_func=init,
        interval=30,
        blit=True,
    )

    plt.show()


def plot_imu_vs_truth(
    time,
    theta_true,
    theta_meas,
    omega_true,
    omega_meas,
    acc_true,
    acc_meas,
):
    plt.figure(figsize=(12, 8))

    plt.subplot(4, 1, 1)
    plt.plot(time, theta_true, label="True Angle (Simulator)")
    plt.plot(time, theta_meas, label="Angle from IMU Gyro Integration", alpha=0.8)
    plt.ylabel("Angle [rad]")
    plt.legend()
    plt.grid(True)

    plt.subplot(4, 1, 2)
    plt.plot(time, omega_true, label="Gyro Ground Truth")
    plt.plot(time, omega_meas, label="Gyro Measured", alpha=0.7)
    plt.ylabel("Angular Rate [rad/s]")
    plt.legend()
    plt.grid(True)

    plt.subplot(4, 1, 3)
    plt.plot(time, acc_true[:, 0], label="Accel X Ground Truth")
    plt.plot(time, acc_meas[:, 0], label="Accel X Measured", alpha=0.7)
    plt.ylabel("Accel X [m/s²]")
    plt.legend()
    plt.grid(True)

    plt.subplot(4, 1, 4)
    plt.plot(time, acc_true[:, 1], label="Accel Y Ground Truth")
    plt.plot(time, acc_meas[:, 1], label="Accel Y Measured", alpha=0.7)
    plt.xlabel("Time [s]")
    plt.ylabel("Accel Y [m/s²]")
    plt.legend()
    plt.grid(True)

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


def plot_imu_errors(
    time,
    theta_true,
    theta_meas,
    theta_dot_true,
    omega_meas,
    acc_true_body,
    acc_meas,
    angle_wrap=True,
):
    e_omega = omega_meas - theta_dot_true
    e_ax = acc_meas[:, 0] - acc_true_body[:, 0]
    e_ay = acc_meas[:, 1] - acc_true_body[:, 1]

    e_theta = theta_meas - theta_true
    if angle_wrap:
        e_theta = wrap_angle_pi(e_theta)

    summaries = [
        summarize_error(e_ax, "accel_x (m/s^2)"),
        summarize_error(e_ay, "accel_y (m/s^2)"),
        summarize_error(e_omega, "theta_dot (rad/s)"),
        summarize_error(e_theta, "theta (rad) [gyro integrated]"),
    ]

    print("\nIMU Error Summary Metrics")
    print("-" * 80)
    for s in summaries:
        print(
            f"{s['name']:<28}  "
            f"MAE={s['MAE']:.6f}  RMSE={s['RMSE']:.6f}  "
            f"Bias={s['Bias(mean)']:.6f}  Std(zm)={s['Std(zero-mean)']:.6f}"
        )
    print("-" * 80)

    plt.figure(figsize=(12, 10))

    plt.subplot(4, 1, 1)
    plt.plot(time, e_ax)
    plt.title("X Acceleration Error ")
    plt.ylabel("m/s²")
    plt.grid(True)

    plt.subplot(4, 1, 2)
    plt.plot(time, e_ay)
    plt.title("Y Acceleration Error")
    plt.ylabel("m/s²")
    plt.grid(True)

    plt.subplot(4, 1, 3)
    plt.plot(time, e_omega)
    plt.title("Gyro Error ")
    plt.ylabel("rad/s")
    plt.grid(True)

    plt.subplot(4, 1, 4)
    plt.plot(time, e_theta)
    plt.title("Angle Error (Integrated Gyro) ")
    plt.xlabel("Time [s]")
    plt.ylabel("rad")
    plt.grid(True)

    plt.tight_layout()
    plt.show()

    return {
        "errors": {"ax": e_ax, "ay": e_ay, "omega": e_omega, "theta": e_theta},
        "summaries": summaries,
    }
