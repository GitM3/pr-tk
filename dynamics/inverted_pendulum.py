import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np


class InvertedPendulumCart:
    def __init__(self, M, m, L, g=9.81, B_M=0.05, B_m=0.01):
        """
        M   : cart mass
        m   : pendulum mass
        L   : pendulum length
        g   : gravity
        B_M : cart damping
        B_m : pendulum damping
        """
        self.M = M
        self.m = m
        self.L = L
        self.g = g
        self.B_M = B_M
        self.B_m = B_m

    def dynamics(self, state, u):
        """
        Continuous-time dynamics

        state = [x, x_dot, theta, theta_dot]
        u     = force applied to cart
        """
        x, x_dot, theta, theta_dot = state

        M, m, L, g = self.M, self.m, self.L, self.g
        B_M, B_m = self.B_M, self.B_m

        s = np.sin(theta)
        c = np.cos(theta)
        denom = L * (M + m - m * c**2)

        x_ddot = (
            L * u
            + B_m * theta_dot * c
            - m * L * g * s * c
            + m * L**2 * theta_dot**2 * s
            - B_M * L * x_dot
        ) / denom

        theta_ddot = (
            -m * L * c * u
            - m**2 * L**2 * theta_dot**2 * s * c
            + B_M * x_dot * m * L * c
            - (M + m) * B_m * theta_dot
            + (M + m) * m * g * L * s
        ) / (m * L**2 * (M + m - m * c**2))

        return np.array([x_dot, x_ddot, theta_dot, theta_ddot])


# Testing
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


if __name__ == "__main__":
    M = 1.0
    m = 0.2  # TODO: simulate butler bot
    L = 0.5
    system = InvertedPendulumCart(M, m, L,B_M=0.5)
    # x  # x_dot  # theta  # theta_dot
    x0 = np.array([0.0, 0.0, np.deg2rad(10), 0.0])

    T = 10.0
    dt = 0.01

    t, history = simulate(system, x0, T, dt)

    #plot_states(t, history)
    animate_cart_pendulum(
        t,
        history,
        L=L,
        trace=True,
        trace_length=10,
    )
