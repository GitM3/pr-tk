import numpy as np


def linearize_upright(system, alpha: float = 1.0):
    """
    Continuous-time linearization around upright (theta = 0)
    Please see references/MATLAB for reference.
    Returns A, B for state x = [x, x_dot, theta, theta_dot].
    """
    M, m, L, g = system.M, system.m, system.L, system.g
    B_M, B_m = system.B_M, system.B_m

    A = np.array(
        [
            [0.0, 1.0, 0.0, 0.0],
            [0.0, -B_M / M, -(g * m) / M, (alpha * B_m) / (L * M)],
            [0.0, 0.0, 0.0, 1.0],
            [
                0.0,
                (alpha * B_M) / (L * M),
                (alpha * (M + m) * g) / (L * M),
                -((M + m) * B_m) / (L * L * M * m),
            ],
        ]
    )

    B = np.array([[0.0], [1.0 / M], [0.0], [-(alpha) / (L * M)]])
    return A, B


def solve(A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: np.ndarray) -> np.ndarray:
    """
    Solve the continuous-time Algebraic Riccati Equation via the Hamiltonian method:
        A'P + P A - P B R^{-1} B' P + Q = 0
    Returns stabilizing P
    Please see references/MATLAB for reference.
    """
    n = A.shape[0]
    R_inv = np.linalg.inv(R)
    H = np.block(
        [
            [A, -B @ R_inv @ B.T],
            [-Q, -A.T],
        ]
    )

    w, V = np.linalg.eig(H)
    select = np.real(w) < 0.0
    if np.count_nonzero(select) != n:
        select = np.real(w) <= 1e-9
    Vs = V[:, select]
    X = Vs[:n, :]
    Y = Vs[n:, :]

    try:
        P = np.real(Y @ np.linalg.inv(X))
    except np.linalg.LinAlgError:
        P = np.real(Y @ np.linalg.pinv(X))

    P = 0.5 * (P + P.T)
    return P


class LQRController:
    """LQR controller u = -K (x - x_ref)."""

    def __init__(
        self,
        system,
        Q: np.ndarray | None = None,
        R: np.ndarray | None = None,
        x_ref: np.ndarray | None = None,
        u_limit: float | None = None,
        alpha: float = 1.0,
    ) -> None:
        self.system = system
        self.x_ref = np.zeros(4) if x_ref is None else np.asarray(x_ref).reshape(4)
        self.u_limit = u_limit

        if Q is None:
            Q = np.diag([1.0, 1.0, 10.0, 100.0])
        if R is None:
            R = np.array([[10.0]])
        self.Q = Q
        self.R = R

        A, B = linearize_upright(system, alpha=alpha)
        self.A = A
        self.B = B
        P = solve(A, B, Q, R)
        self.P = P
        self.K = np.linalg.solve(R, B.T @ P)

    def __call__(self, x: np.ndarray, t: float) -> float:
        e = x[:4] - self.x_ref
        u = float(-(self.K @ e).squeeze())
        if self.u_limit is not None:
            u = np.clip(u, -self.u_limit, self.u_limit)
        return u
