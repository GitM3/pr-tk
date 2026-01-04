# Inverted Pendulum Controllers (MATLAB References)

- Source: `references/MATLAB/PART2_Inverted_Pendulum.m`, `references/MATLAB/PART3_Inverted_Pendulum.m`
- Scope: Controller design and logic only (dynamics/animation omitted)

## State & Linearization

- State: \(x = \begin{bmatrix} x & \dot{x} & \theta & \dot{\theta} \end{bmatrix}^\top\)
- Upright linearization flag: \(\alpha = +1\) (use \(\alpha=-1\) for downward)
- Input: cart force \(u\) [N]
- Continuous-time linear model near upright:

$$
A = \begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & -\tfrac{B_M}{M} & -\tfrac{g\,m}{M} & \tfrac{\alpha\,B_m}{L\,M} \\
0 & 0 & 0 & 1 \\
0 & \tfrac{\alpha\,B_M}{L\,M} & \tfrac{\alpha\,(M+m)\,g}{L\,M} & -\tfrac{(M+m)\,B_m}{L^2\,M\,m}
\end{bmatrix},\quad
B = \begin{bmatrix}
0 \\
\tfrac{1}{M} \\
0 \\
-\tfrac{\alpha}{L\,M}
\end{bmatrix}.
$$

- Controllability check: \(\mathrm{rank}\,\mathcal{C}=4\), with \(\mathcal{C}=[B\,|\,AB\,|\,A^2B\,|\,A^3B]\).

## LQR Design

- Weights used: \(Q=\mathrm{diag}(1,\,1,\,10,\,100)\), \(R=10\).
- Continuous-time ARE:

$$
A^\top P + P A - P B R^{-1} B^\top P + Q = 0.
$$

- Optimal gain and control law:

$$
K = R^{-1} B^\top P,\quad u = -K\,\bigl(x - x_{\mathrm{ref}}\bigr),\quad x_{\mathrm{ref}}=\mathbf{0}.
$$

## Energy-Based Swing-Up

- Desired upright energy: \(E_d = 2\,m\,g\,L\).
- Current energy (as used in code):

$$
E(x) = m\,g\,L\,\bigl(1+\cos\theta\bigr) + \tfrac{1}{2}\,m\,L^2\,\dot{\theta}^2.
$$

- Energy-shaping control with direction term:

$$\nu = -k\,\bigl(E_d - E(x)\bigr)\;\mathrm{sgn}\!\bigl(\dot{\theta}\cos\theta\bigr),\quad k=0.5,$$

with saturation in code \(|u|\le 70\,\mathrm{N}|\).

## Mode Switching Logic

- Switch criterion: near-upright angle \(|\theta| < 0.5\,\mathrm{rad}|\) ⇒ use LQR; otherwise use swing-up.
- Summary:
  - If \(|\theta| \ge 0.5\): apply swing-up control.
  - If \(|\theta| < 0.5\): apply LQR \(u=-K(x-x_{\mathrm{ref}})\).

## Example Parameters (from scripts)

- \(M=10\) kg, \(m=5\) kg, \(L=20\) m, \(g=9.81\) m/s²
- Friction: \(B_M=0.1\) (cart), \(B_m=0.1\) (pendulum)
- Linearization: \(\alpha=+1\) (upright)

## EKF Bias Random-Walk Process Noise (IMU Consistency)

To let the EKF bias states adapt over time (matching the simulated IMU), model
gyro and accelerometer biases as random walks:

$$
b_{k+1} = b_k + w_b,\quad w_b \sim \mathcal{N}(0, \sigma_{rw}^2\,\Delta t)
$$

This implies a per-step process noise variance:

$$
\mathrm{Var}(w_b) = \sigma_{rw}^2\,\Delta t
$$

So the EKF process noise entries for the bias states are:

$$
R_{b_g} = \sigma_{g, rw}^2\,\Delta t,\quad
R_{b_{ax}} = \sigma_{a, rw}^2\,\Delta t,\quad
R_{b_{ay}} = \sigma_{a, rw}^2\,\Delta t
$$

This is consistent with the IMU update:

$$
b_g \leftarrow b_g + \mathcal{N}(0,\sigma_{g,rw}\sqrt{\Delta t}),\quad
b_a \leftarrow b_a + \mathcal{N}(0,\sigma_{a,rw}\sqrt{\Delta t})
$$
