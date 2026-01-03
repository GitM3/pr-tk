import numpy as np


class EKF:
    def __init__(
        self,
        g_motion,
        h_meas,
        R_t,
        Q_t,
        state_dim,
        meas_dim,
        eps_jac=1e-6,
    ):
        """
        g_motion(mu, u): returns next state
        h_meas(mu, u): returns measurement prediction
        R_t: process (motion) noise covariance (state_dim x state_dim)
        Q_t: measurement noise covariance (meas_dim x meas_dim)
        eps_jac: finite difference epsilon for numerical Jacobians
        """
        self.g = g_motion
        self.h = h_meas
        self.R_t = R_t
        self.Q_t = Q_t
        self.n = state_dim
        self.m = meas_dim
        self.eps = eps_jac

    def _jacobian_g(self, mu, u):
        G = np.zeros((self.n, self.n))
        g_mu = self.g(mu, u)
        for i in range(self.n):
            d = np.zeros(self.n)
            d[i] = self.eps
            g_plus = self.g(mu + d, u)
            g_minus = self.g(mu - d, u)
            G[:, i] = (g_plus - g_minus) / (2 * self.eps)
        return G

    def _jacobian_h(self, mu, u):
        H = np.zeros((self.m, self.n))
        h_mu = self.h(mu, u)
        for i in range(self.n):
            d = np.zeros(self.n)
            d[i] = self.eps
            h_plus = self.h(mu + d, u)
            h_minus = self.h(mu - d, u)
            H[:, i] = (h_plus - h_minus) / (2 * self.eps)
        return H

    def step(self, mu, Sigma, z, u):
        # Prediction
        mu_bar = self.g(mu, u)
        G = self._jacobian_g(mu, u)
        Sigma_bar = G @ Sigma @ G.T + self.R_t

        # Update
        H = self._jacobian_h(mu_bar, u)
        z_hat = self.h(mu_bar, u)
        S = H @ Sigma_bar @ H.T + self.Q_t
        K = Sigma_bar @ H.T @ np.linalg.inv(S)
        innov = z - z_hat
        mu_new = mu_bar + K @ innov
        I = np.eye(self.n)
        Sigma_new = (I - K @ H) @ Sigma_bar

        return mu_new, Sigma_new, mu_bar, Sigma_bar, K, innov, S
