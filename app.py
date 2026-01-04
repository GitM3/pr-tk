import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import streamlit as st
from streamlit.components.v1 import html as st_html

import simulator as sim
from controller.lqr import LQRController
from dynamics.inverted_pendulum import InvertedPendulumCart
from sensors.imu import IMU
from sensors.wheel_encoder import WheelEncoder
from plotting.plotting import (
    plot_imu_ekf_errors,
    plot_imu_vs_ekf,
    plot_true_vs_meas,
    plot_kalman_gain,
    plot_ekf_statistics,
)


def build_animation_html(
    t: np.ndarray,
    state_history: np.ndarray,
    L: float,
    state_est_history: np.ndarray | None = None,
    trace: bool = True,
    est_trace: bool = True,
    trace_length: int | None = 200,
    interval_ms: int = 30,
) -> str:
    """Create a simple cart-pendulum animation and return it as JS HTML."""
    x_cart = state_history[:, 0]
    theta = state_history[:, 2]

    pend_x = x_cart + L * np.sin(theta)
    pend_y = L * np.cos(theta)

    if state_est_history is not None and state_est_history.shape[1] >= 3:
        theta_est = state_est_history[:, 2]
        pend_x_est = x_cart + L * np.sin(theta_est)
        pend_y_est = L * np.cos(theta_est)
    else:
        pend_x_est = pend_y_est = None

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.set_xlim(float(np.min(x_cart) - 1.0), float(np.max(x_cart) + 1.0))
    ax.set_ylim(-0.5, L + 0.5)
    ax.set_aspect("equal")
    ax.grid(True)

    cart_width = 0.3
    cart_height = 0.2
    (cart_line,) = ax.plot([], [], "k-", lw=2)
    (rod_line,) = ax.plot([], [], "r-", lw=2, label="True")
    (bob_point,) = ax.plot([], [], "ro", markersize=6)

    trace_line = None
    if trace:
        (trace_line,) = ax.plot([], [], "b--", lw=1, alpha=0.6, label="True tip")

    est_trace_line = None
    if est_trace and pend_x_est is not None:
        (est_trace_line,) = ax.plot([], [], "g--", lw=1, alpha=0.6, label="EKF tip")

    def init():
        cart_line.set_data([], [])
        rod_line.set_data([], [])
        bob_point.set_data([], [])
        if trace_line is not None:
            trace_line.set_data([], [])
        if est_trace_line is not None:
            est_trace_line.set_data([], [])
        return tuple(filter(None, [cart_line, rod_line, bob_point, trace_line, est_trace_line]))

    def update(i):
        x = x_cart[i]
        cart_x = [
            x - cart_width / 2,
            x + cart_width / 2,
            x + cart_width / 2,
            x - cart_width / 2,
            x - cart_width / 2,
        ]
        cart_y = [0, 0, cart_height, cart_height, 0]
        cart_line.set_data(cart_x, cart_y)

        rod_line.set_data([x_cart[i], pend_x[i]], [cart_height, pend_y[i] + cart_height])
        bob_point.set_data([pend_x[i]], [pend_y[i] + cart_height])

        if trace_line is not None:
            start = max(0, i - trace_length) if trace_length else 0
            trace_line.set_data(pend_x[start:i], pend_y[start:i] + cart_height)

        if est_trace_line is not None and pend_x_est is not None:
            start = max(0, i - trace_length) if trace_length else 0
            est_trace_line.set_data(
                pend_x_est[start:i],
                pend_y_est[start:i] + cart_height,
            )

        return tuple(filter(None, [cart_line, rod_line, bob_point, trace_line, est_trace_line]))

    ani = animation.FuncAnimation(
        fig,
        update,
        frames=len(t),
        init_func=init,
        interval=interval_ms,
        blit=True,
    )

    return ani.to_jshtml()


def main():
    st.set_page_config(page_title="Cart-Pendulum Simulator", layout="wide")
    st.title("Interactive Cart-Pendulum Simulator")

    st.header("Parameters")
    with st.form(key="params_form"):
        st.subheader("System")
        cols1 = st.columns(4)
        with cols1[0]:
            M = st.number_input("Cart mass M", value=1.0, step=0.1)
        with cols1[1]:
            m = st.number_input("Pendulum mass m", value=0.2, step=0.05)
        with cols1[2]:
            L = st.number_input("Pendulum length L", value=0.5, step=0.1)
        with cols1[3]:
            gravity = st.number_input("Gravity g [m/s^2]", value=9.81, format="%.2f")

        cols2 = st.columns(2)
        with cols2[0]:
            B_M = st.number_input("Cart damping B_M", value=0.5, step=0.1)
        with cols2[1]:
            B_m = st.number_input("Pendulum damping B_m", value=0.01, step=0.01, format="%.3f")

        st.subheader("Initial State")
        cols3 = st.columns(4)
        with cols3[0]:
            x0_x = st.number_input("x [m]", value=0.0)
        with cols3[1]:
            x0_xdot = st.number_input("x_dot [m/s]", value=0.0)
        with cols3[2]:
            x0_theta_deg = st.number_input("theta [deg]", value=10.0)
        with cols3[3]:
            x0_thetadot = st.number_input("theta_dot [rad/s]", value=0.0)

        st.subheader("Simulation")
        cols4 = st.columns(2)
        with cols4[0]:
            T = st.number_input("Duration T [s]", value=10.0, step=1.0)
        with cols4[1]:
            dt = st.number_input("Time step dt [s]", value=0.01, step=0.005, format="%.3f")

        st.subheader("IMU Noise")
        cols5 = st.columns(3)
        with cols5[0]:
            gyro_bias = st.number_input("Gyro bias [rad/s]", value=0.02, format="%.4f")
            gyro_noise_std = st.number_input("Gyro noise std [rad/s]", value=0.01, format="%.4f")
        with cols5[1]:
            accel_bias_x = st.number_input("Accel bias X [m/s^2]", value=0.10, format="%.3f")
            accel_noise_std = st.number_input("Accel noise std [m/s^2]", value=0.10, format="%.3f")
        with cols5[2]:
            accel_bias_y = st.number_input("Accel bias Y [m/s^2]", value=-0.05, format="%.3f")

        st.subheader("Controller & Estimator")
        cols6 = st.columns(3)
        with cols6[0]:
            use_lqr = st.checkbox("Use LQR", value=True)
        with cols6[1]:
            u_limit = st.number_input("u_limit (saturation)", value=50.0, step=5.0)
        with cols6[2]:
            alpha = st.number_input("alpha (linearization)", value=1.0, step=0.1)

        use_ekf = st.checkbox("Use EKF (estimation)", value=True)

        submitted = st.form_submit_button("Play")

    system = InvertedPendulumCart(M, m, L, g=gravity, B_M=B_M, B_m=B_m)
    controller = None
    if use_lqr:
        controller = LQRController(
            system,
            Q=np.diag([1.0, 1.0, 10.0, 100.0]),
            R=np.array([[10.0]]),
            x_ref=np.array([0.0, 0.0, 0.0, 0.0]),
            u_limit=u_limit,
            alpha=alpha,
        )

    imu = IMU(
        gyro_bias=gyro_bias,
        accel_bias=np.array([accel_bias_x, accel_bias_y]),
        gyro_noise_std=gyro_noise_std,
        accel_noise_std=accel_noise_std,
        gravity=gravity,
    )
    encoder = WheelEncoder()

    # Simulate on click
    if submitted:
        x0 = np.array([x0_x, x0_xdot, np.deg2rad(x0_theta_deg), x0_thetadot])

        if use_ekf:
            # Set globals used by sim.f_disc inside simulate_with_ekf
            sim.system = system
            sim.dt = float(dt)
            results = sim.simulate_with_ekf(
                system,
                imu,
                encoder,
                x0,
                float(T),
                float(dt),
                controller=controller,
            )
        else:
            results = sim.simulate_with_imu(
                system,
                imu,
                encoder,
                x0,
                float(T),
                float(dt),
                controller=controller,
            )

        time = results["time"]
        state_hist = results["x_true"]
        est_hist = results["x_meas"]

        with st.spinner("Building animation..."):
            anim_html = build_animation_html(time, state_hist, L, state_est_history=est_hist)
        st.subheader("Animation")
        st_html(anim_html, height=420)

        if use_ekf:
            st.subheader("EKF vs IMU-only")
            fig1 = plot_imu_vs_ekf(system=system, imu=imu, results=results, return_fig=True)
            st.pyplot(fig1)

            st.subheader("EKF Error Overlay")
            fig2 = plot_imu_ekf_errors(results=results, return_fig=True)
            st.pyplot(fig2)

            st.subheader("Kalman Gain")
            state_labels = ["x", "x_dot", "theta", "theta_dot", "b_g", "b_ax", "b_ay"]
            meas_labels = ["x", "x_dot", "omega", "acc_x", "acc_y"]
            fig3 = plot_kalman_gain(
                time=time,
                K_hist=results["K"],
                meas_labels=meas_labels,
                state_labels=state_labels,
                return_fig=True,
            )
            st.pyplot(fig3)

            st.subheader("EKF Covariance Diagonal")
            fig4 = plot_ekf_statistics(
                time=time,
                P_diag=results["P"],
                meas_labels=meas_labels,
                state_labels=state_labels,
                return_fig=True,
            )
            st.pyplot(fig4)
        else:
            st.subheader("IMU-only vs Truth")
            fig = plot_true_vs_meas(
                time=time,
                x_true=state_hist,
                x_meas=est_hist,
                u=results["u"],
                return_fig=True,
            )
            st.pyplot(fig)


if __name__ == "__main__":
    main()
