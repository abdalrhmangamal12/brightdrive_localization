# ESKF derivation — IMU prediction and GNSS correction

Step-by-step derivation of the error-state Kalman filter implemented in `eskf/`: a 6-axis IMU drives the
prediction, GNSS position fixes drive the correction. The derivation follows J. Solà, *Quaternion kinematics for the
error-state Kalman filter*, arXiv:1711.02508, ch. 5–6. It is written in the notation of this repository; Solà
equation numbers are given where they apply, and §2 maps the notation to his.

1. [State definition](#1-state-definition)
2. [Conventions](#2-conventions)
3. [SO(3) tools](#3-so3-tools)
4. [IMU measurement model and inputs](#4-imu-measurement-model-and-inputs)
5. [Nominal-state propagation](#5-nominal-state-propagation-discrete--solà-eq-260)
6. [True vs nominal kinematics and the error-state derivation](#6-true-vs-nominal-kinematics-and-the-error-state-derivation--solà-533--542-eq-238--261)
7. [Discrete error dynamics, F and Q, covariance prediction](#7-discrete-error-dynamics-f-and-q-covariance-prediction--solà-eq-261270)
8. [Measurement update](#8-measurement-update--solà-6-eq-272288)
9. [Injection and reset](#9-injection-and-reset--solà-eq-282288)
10. [Summary: the complete algorithm](#10-summary-the-complete-algorithm)

---

## 1. State definition

The state is what must be carried from one instant to the next to predict the next instant. It has five blocks:
position, velocity, orientation, accelerometer bias, gyroscope bias. The IMU reading $(\tilde a, \tilde\omega)$ is
the input $u$, not a state: it arrives fresh every sample. Gravity $g$ is a constant, not a state.

**Nominal state** $x$, 16 numbers, stored:

$$
x = \left(p,\ v,\ q,\ b_a,\ b_g\right), \qquad p,\ v,\ b_a,\ b_g \in \mathbb{R}^3, \qquad q \ \text{a unit quaternion (4 numbers)}
$$

**Error state** $\delta x$, 15 numbers ($\delta\theta$ has 3 components, not 4):

$$
\delta x = \left(\delta p,\ \delta v,\ \delta\theta,\ \delta b_a,\ \delta b_g\right) \in \mathbb{R}^{15}
$$

**True state** $x_t = x \oplus \delta x$: plain addition for every block, quaternion composition with a local
rotation error for $q$ (Solà Table 3):

$$
p_t = p + \delta p, \qquad
v_t = v + \delta v, \qquad
q_t = q \otimes \operatorname{Exp}(\delta\theta), \qquad
b_{a,t} = b_a + \delta b_a, \qquad
b_{g,t} = b_g + \delta b_g
$$

Because $\delta x$ has 15 entries, the covariance $P$ is $15 \times 15$.

| block | error | size | rows / columns | code name |
|---|---|---|---|---|
| position | $\delta p$ | 3 | 0–2 | `kP = 0` |
| velocity | $\delta v$ | 3 | 3–5 | `kV = 3` |
| orientation | $\delta\theta$ | 3 | 6–8 | `kTh = 6` |
| accelerometer bias | $\delta b_a$ | 3 | 9–11 | `kBa = 9` |
| gyroscope bias | $\delta b_g$ | 3 | 12–14 | `kBg = 12` |

```cpp
static constexpr int kP = 0, kV = 3, kTh = 6, kBa = 9, kBg = 12;
```

Sizes: nominal 16 · error 15 · $P$ 15×15 · $F$ 15×15 · $Q$ 15×15 · $H$ 3×15 · $K$ 15×3.

---

## 2. Conventions

| item | choice |
|---|---|
| quaternion | Hamilton, $(w, x, y, z)$, `Eigen::Quaterniond`, normalized after every multiply |
| rotation error | local: $q_t = q \otimes \operatorname{Exp}(\delta\theta)$, multiplied on the **right** |
| rotation matrix | $R = R(q) = {}^{\mathrm{world}}R_{\mathrm{imu}}$, IMU → world |
| frames | "world" = the map frame. $p$, $v$, $g$ and GNSS positions live in it; IMU readings and biases are in IMU axes |
| gravity | $g = (0, 0, -9.81)$, constant, **not** a state. Solà estimates $g$; his eq. 260f is dropped here |
| error index order | $\delta p$ 0–2 · $\delta v$ 3–5 · $\delta\theta$ 6–8 · $\delta b_a$ 9–11 · $\delta b_g$ 12–14 |
| values inside $F$ | $R$, $b_a$, $b_g$ at the **start** of the step ($R_{k-1}$, $b_{a,k-1}$, $b_{g,k-1}$): build $F$ before updating the nominal |
| $\Delta t$ | $\Delta t = t_k - t_{k-1}$ from the timestamps, never hard-coded |
| noise names | $Q$ = process noise (IMU) · $V$ = measurement noise (GNSS) |
| time index | one step is $k-1 \to k$. No subscript = nominal, subscript $t$ = true, $\delta$ = true − nominal. Everything on a right-hand side is at the start of the step |

Notation map to Solà:

| here | Solà | meaning |
|---|---|---|
| $\tilde a,\ \tilde\omega$ | $a_m,\ \omega_m$ | IMU reading, IMU axes |
| $b_a,\ b_g$ | $a_b,\ \omega_b$ | biases (states) |
| $\delta b_a,\ \delta b_g$ | $\delta a_b,\ \delta\omega_b$ | bias errors |
| $a_n,\ \omega_n$ | $a_n,\ \omega_n$ | white noise in the reading |
| $a_w,\ \omega_w$ | $a_w,\ \omega_w$ | bias random-walk increments (subscript $w$ = walk) |
| $\operatorname{Exp}(\theta)$ | $q\{\theta\}$ | rotation from a rotation vector: as a quaternion (`ExpQ`) or as a $3 \times 3$ matrix (`ExpSO3`) |
| $a_c = \tilde a - b_a,\ \ \omega_c = \tilde\omega - b_g$ | — | bias-corrected ("clean") readings, IMU axes |
| $X_{\delta x} = \mathrm{blockdiag}(I_6, Q_{\delta\theta}, I_6)$ | $\mathrm{blockdiag}(I_6, Q_{\delta\theta}, I_9)$ | Solà's $I_9$ includes $\delta g$; there is no $\delta g$ here |

---

## 3. SO(3) tools

**Skew-symmetric matrix** (Solà eq. 20). The cross product written as a matrix, so that it can sit inside $F$ and
$H$ and the product $F\,\delta x$ is one matrix multiplication:

$$
[a]_\times =
\begin{bmatrix}
0 & -a_z & a_y \\
a_z & 0 & -a_x \\
-a_y & a_x & 0
\end{bmatrix},
\qquad
[a]_\times\, b = a \times b
$$

**Swap rule.** From $a \times b = -\,b \times a$:

$$
[a]_\times\, b = -[b]_\times\, a
$$

**Exponential map, quaternion form** (Solà eq. 101), Hamilton $(w, x, y, z)$:

$$
\operatorname{Exp}(\theta) = \left[\cos\tfrac{\lVert\theta\rVert}{2},\ \frac{\theta}{\lVert\theta\rVert}\sin\tfrac{\lVert\theta\rVert}{2}\right]
$$

Tiny-angle case: if $\lVert\theta\rVert < 10^{-8}$ use $[1,\ \theta/2]$ and normalize, so there is no division by
zero when still. In Eigen: `Quaterniond(AngleAxisd(|θ|, θ/|θ|))`.

**Exponential map, matrix form (Rodrigues).** `ExpSO3`$(\theta)$ is the $3 \times 3$ rotation matrix of the
quaternion `ExpQ`$(\theta)$. Both are written $\operatorname{Exp}(\cdot)$ below; which one is meant follows from
where it is used (quaternion product or matrix product).

**Small-angle approximation**, first order in $\delta\theta$:

$$
\operatorname{Exp}(\delta\theta) \approx I + [\delta\theta]_\times \quad\text{(matrix)}, \qquad
\operatorname{Exp}(\delta\theta) \approx \left[1,\ \tfrac12\,\delta\theta\right] \quad\text{(quaternion)}
$$

**Identities used in the derivations:**

$$
\operatorname{Exp}(x)^\top = \operatorname{Exp}(-x), \qquad (AB)^\top = B^\top A^\top
$$

$$
\operatorname{Exp}(-x)\,\operatorname{Exp}(\delta\theta)\,\operatorname{Exp}(x) = \operatorname{Exp}\!\left(\operatorname{Exp}(-x)\,\delta\theta\right)
$$

*The same small rotation, expressed in axes that have been rotated by $x$.*

$$
\operatorname{Exp}(a + b) \approx \operatorname{Exp}(a)\,\operatorname{Exp}(b) \qquad \text{for small } a,\ b
$$

*Two small rotations compose, to first order, into one rotation of their sum.*

---

## 4. IMU measurement model and inputs

Every sample at time $t_k$ delivers $\tilde a_k$ [m/s²] and $\tilde\omega_k$ [rad/s] in IMU axes.
$\Delta t = t_k - t_{k-1}$ is taken from the timestamps: samples can drop or jitter.

Each reading is wrong in two ways. A **bias**: the same offset on every sample, drifting slowly, which is why
$b_a$ and $b_g$ are states. **White noise**: a new zero-mean random value every sample, of which only the size is
known.

**True quantities behind a reading** (what the true state experienced during the step):

$$
a_{t,k} = R_{t,k-1}\left(\tilde a_k - b_{a,t,k-1} - a_{n,k}\right) + g,
\qquad
\omega_{t,k} = \tilde\omega_k - b_{g,t,k-1} - \omega_{n,k}
$$

*Remove the true bias and the noise. The acceleration is rotated to the world frame and gravity is added back; the
turn rate stays in IMU axes (why: §6.7).*

**Bias random walk:**

$$
b_{a,t,k} = b_{a,t,k-1} + a_{w,k}, \qquad
b_{g,t,k} = b_{g,t,k-1} + \omega_{w,k},
\qquad
a_{w,k} \sim \mathcal N\!\left(0,\ \sigma_{ba}^2\,\Delta t\, I\right), \quad
\omega_{w,k} \sim \mathcal N\!\left(0,\ \sigma_{bg}^2\,\Delta t\, I\right)
$$

**Noise densities.** Four rate-independent parameters, SI units:

| symbol | meaning | unit |
|---|---|---|
| $\sigma_a$ | accelerometer noise density | m/s²/√Hz |
| $\sigma_g$ | gyroscope noise density | rad/s/√Hz |
| $\sigma_{ba}$ | accelerometer bias random-walk density | m/s³/√Hz |
| $\sigma_{bg}$ | gyroscope bias random-walk density | rad/s²/√Hz |

**Discrete-time form.** A density is not yet the standard deviation of one reading; the per-sample value depends on
the rate:

| | per-sample standard deviation | variance that enters $Q$ |
|---|---|---|
| white noise $a_n$, $\omega_n$ | $\sigma_a / \sqrt{\Delta t}$, $\ \sigma_g / \sqrt{\Delta t}$ | $\sigma_a^2\,\Delta t$, $\ \sigma_g^2\,\Delta t$ |
| bias increments $a_w$, $\omega_w$ | $\sigma_{ba}\sqrt{\Delta t}$, $\ \sigma_{bg}\sqrt{\Delta t}$ | $\sigma_{ba}^2\,\Delta t$, $\ \sigma_{bg}^2\,\Delta t$ |

If a sensor model gives a per-reading standard deviation $s$ instead of a density, the same entry is
$\sigma^2\,\Delta t = (s\,\Delta t)^2$.

The parameters store only the four densities. The factors in $\Delta t$ are applied when $Q$ is built, so changing
the IMU rate never means retuning.

**Inputs to one prediction step**, with the nominal biases at the start of the step:

$$
a_c = \tilde a_k - b_{a,k-1}, \qquad \omega_c = \tilde\omega_k - b_{g,k-1}
$$

The initial nominal state $x_0$ and covariance $P_0$ are supplied once by an initializer outside the filter.

---

## 5. Nominal-state propagation (discrete) — Solà eq. 260

Every IMU sample pushes the nominal state forward by $\Delta t$, using the reading as if it were exact after
removing the believed bias: dead reckoning, no noise, no covariance. $R_{k-1} = R(q_{k-1})$ is built from the
**old** $q$.

$$
a_k = R_{k-1}\,a_c + g = R_{k-1}\left(\tilde a_k - b_{a,k-1}\right) + g
$$

*Remove the bias, rotate the reading to the world frame, add gravity back: the reading contains the reaction to
gravity, which is not motion.*

$$
p_k = p_{k-1} + v_{k-1}\,\Delta t + \tfrac12\,a_k\,\Delta t^2
$$

*Uses the old velocity $v_{k-1}$; the new one would count the acceleration twice.*

$$
v_k = v_{k-1} + a_k\,\Delta t
$$

$$
q_k = \big(q_{k-1} \otimes \operatorname{Exp}(\omega_c\,\Delta t)\big)\ \text{normalized}
$$

*Remove the gyro bias, scale by $\Delta t$ to a small rotation, multiply on the right: the gyro measures about the
IMU's own axes.*

$$
b_{a,k} = b_{a,k-1}, \qquad b_{g,k} = b_{g,k-1}
$$

*The direction of the bias drift is unknown, so the nominal keeps the biases. That they can drift is tracked in $P$
(§7).*

Order matters in code: compute $R$ and $a$ from the old state, update $p$ with the old $v$, then $v$, then $q$.
There is no noise, no $\sigma$, no Jacobian here; all uncertainty lives in $P$, propagated alongside (§7).
Difference from Solà: his eq. 260f ($g \leftarrow g$) is dropped because $g$ is constant.

---

## 6. True vs nominal kinematics and the error-state derivation — Solà §5.3.3 → §5.4.2, eq. 238 → 261

### 6.1 Method

Write the true step, write the nominal step, subtract, keep first order: drop products of two small quantities
(small × small) and terms in error × $\Delta t^2$. The continuous rate form (eq. 238) and the discrete step form
(eq. 261) are the same rule: per step = old + (rate-form right-hand side) × $\Delta t$.

Notation rule: everything on a right-hand side is at $k-1$, the start of the step. $R = R_{k-1}$; $a_c$ and
$\omega_c$ use the nominal biases at $k-1$; an error written without an index ($\delta\theta$, $\delta b_a$,
$\delta b_g$) is at $k-1$.

### 6.2 True kinematics (unknown, contains noise)

$$
\begin{aligned}
a_{t,k} &= R_{t,k-1}\left(\tilde a_k - b_{a,t,k-1} - a_{n,k}\right) + g \\
p_{t,k} &= p_{t,k-1} + v_{t,k-1}\,\Delta t + \tfrac12\,a_{t,k}\,\Delta t^2 \\
v_{t,k} &= v_{t,k-1} + a_{t,k}\,\Delta t \\
\omega_{t,k} &= \tilde\omega_k - b_{g,t,k-1} - \omega_{n,k} \qquad \text{(IMU axes, no } R\text{)} \\
R_{t,k} &= R_{t,k-1}\,\operatorname{Exp}(\omega_{t,k}\,\Delta t) \\
b_{a,t,k} &= b_{a,t,k-1} + a_{w,k} \\
b_{g,t,k} &= b_{g,t,k-1} + \omega_{w,k}
\end{aligned}
$$

### 6.3 Nominal kinematics (the code of §5: no subscript $t$, no noise)

$$
\begin{aligned}
a_k &= R_{k-1}\left(\tilde a_k - b_{a,k-1}\right) + g \\
p_k &= p_{k-1} + v_{k-1}\,\Delta t + \tfrac12\,a_k\,\Delta t^2 \\
v_k &= v_{k-1} + a_k\,\Delta t \\
\omega_k &= \tilde\omega_k - b_{g,k-1} \\
R_k &= R_{k-1}\,\operatorname{Exp}(\omega_k\,\Delta t) \\
b_{a,k} &= b_{a,k-1} \\
b_{g,k} &= b_{g,k-1}
\end{aligned}
$$

### 6.4 The link (holds at every $k$)

$$
p_t = p + \delta p, \qquad
v_t = v + \delta v, \qquad
R_t = R\,\operatorname{Exp}(\delta\theta), \qquad
b_{a,t} = b_a + \delta b_a, \qquad
b_{g,t} = b_g + \delta b_g
$$

### 6.5 Position error $\delta p$

$$
p_{t,k} - p_k = \left(p_{t,k-1} - p_{k-1}\right) + \left(v_{t,k-1} - v_{k-1}\right)\Delta t + \tfrac12\left(a_{t,k} - a_k\right)\Delta t^2
$$

*Subtract the nominal position line from the true one.*

$$
\delta p_k = \delta p_{k-1} + \delta v_{k-1}\,\Delta t
$$

*Substitute the link; drop the error × $\Delta t^2$ term. The position line uses the old velocity error, exactly as
the nominal uses the old $v$.*

### 6.6 Velocity error $\delta v$ — six moves

Short names: $R = R_{k-1}$, $a_c = \tilde a_k - b_{a,k-1}$, $\delta\theta = \delta\theta_{k-1}$,
$\delta b_a = \delta b_{a,k-1}$, $a_n = a_{n,k}$.

Start from the difference of the two acceleration lines:

$$
a_{t,k} - a_k = R_{t,k-1}\left(\tilde a_k - b_{a,t,k-1} - a_{n,k}\right) + g - R_{k-1}\left(\tilde a_k - b_{a,k-1}\right) - g
$$

$$
a_{t,k} - a_k = R_{t,k-1}\left(\tilde a_k - b_{a,t,k-1} - a_{n,k}\right) - R_{k-1}\left(\tilde a_k - b_{a,k-1}\right)
$$

*Move 1: $g$ cancels.*

$$
a_{t,k} - a_k = R\,\operatorname{Exp}(\delta\theta)\left(\tilde a_k - b_{a,k-1} - \delta b_a - a_n\right) - R\left(\tilde a_k - b_{a,k-1}\right)
$$

*Move 2: substitute the true-state composition (§6.4): $R_{t,k-1} = R\,\operatorname{Exp}(\delta\theta)$ and
$b_{a,t,k-1} = b_{a,k-1} + \delta b_a$.*

$$
a_{t,k} - a_k = R\,\operatorname{Exp}(\delta\theta)\left(a_c - \delta b_a - a_n\right) - R\,a_c
$$

*Move 3: write the clean reading $a_c = \tilde a_k - b_{a,k-1}$.*

$$
a_{t,k} - a_k = R\left(I + [\delta\theta]_\times\right)\left(a_c - \delta b_a - a_n\right) - R\,a_c
$$

*Move 4: small-angle approximation $\operatorname{Exp}(\delta\theta) \approx I + [\delta\theta]_\times$.*

$$
a_{t,k} - a_k = R\,[\delta\theta]_\times\, a_c - R\,\delta b_a - R\,a_n
$$

*Move 5: multiply out ($I$ times three terms, $[\delta\theta]_\times$ times three terms); $R\,a_c$ cancels; drop
the two second-order terms $R[\delta\theta]_\times\delta b_a$ and $R[\delta\theta]_\times a_n$.*

$$
a_{t,k} - a_k = -R\,[a_c]_\times\,\delta\theta - R\,\delta b_a - R\,a_n
$$

*Move 6: swap so that $\delta\theta$ stands alone on the right:
$[\delta\theta]_\times a_c = \delta\theta \times a_c = -\,a_c \times \delta\theta = -[a_c]_\times\delta\theta$.*

Result, multiplying by $\Delta t$ and adding the old velocity error:

$$
\delta v_k = \delta v_{k-1} - R\,[a_c]_\times\,\Delta t\ \delta\theta_{k-1} - R\,\Delta t\ \delta b_{a,k-1} - R\,a_{n,k}\,\Delta t
$$

Meaning of the $\delta\theta$ term: the code rotates the reading with $R$, but the true rotation is
$R\,\operatorname{Exp}(\delta\theta)$, so the same reading really points slightly elsewhere. The missed acceleration
has size $\lVert a_c\rVert\,\lVert\delta\theta\rVert$, and $\delta v$ grows by that times $\Delta t$ every step.
The filter never knows $a_{t,k}$; only the difference $a_{t,k} - a_k$ is needed, and it is written entirely in the
errors.

### 6.7 Orientation error $\delta\theta$ — six moves

Short name: $\omega_c = \tilde\omega_k - b_{g,k-1}$.

Start from the two rotation lines:

$$
R_{t,k} = R_{t,k-1}\,\operatorname{Exp}(\omega_{t,k}\,\Delta t), \qquad
R_k = R_{k-1}\,\operatorname{Exp}(\omega_c\,\Delta t)
$$

$$
\operatorname{Exp}(\delta\theta_k) = R_k^\top R_{t,k}
$$

*Move 1: the rotation error is "undo the nominal, then go to the true", from $R_t = R\,\operatorname{Exp}(\delta\theta)$.*

$$
\operatorname{Exp}(\delta\theta_k) = \operatorname{Exp}(-\omega_c\,\Delta t)\; R_{k-1}^\top R_{t,k-1}\; \operatorname{Exp}(\omega_{t,k}\,\Delta t)
$$

*Move 2: substitute both rotation lines, using $(AB)^\top = B^\top A^\top$ and
$\operatorname{Exp}(x)^\top = \operatorname{Exp}(-x)$.*

$$
\operatorname{Exp}(\delta\theta_k) = \operatorname{Exp}(-\omega_c\,\Delta t)\; \operatorname{Exp}(\delta\theta_{k-1})\; \operatorname{Exp}(\omega_{t,k}\,\Delta t)
$$

*Move 3: the middle factor is the old error, $R_{k-1}^\top R_{t,k-1} = \operatorname{Exp}(\delta\theta_{k-1})$.*

$$
\operatorname{Exp}(\delta\theta_k) \approx \operatorname{Exp}(-\omega_c\,\Delta t)\; \operatorname{Exp}(\delta\theta_{k-1})\; \operatorname{Exp}(\omega_c\,\Delta t)\; \operatorname{Exp}(\delta\omega\,\Delta t)
$$

*Move 4: split the true turn rate,
$\omega_{t,k} = \tilde\omega_k - (b_{g,k-1} + \delta b_g) - \omega_n = \omega_c + \delta\omega$ with
$\delta\omega = -\delta b_g - \omega_n$; $\omega_c\,\Delta t$ is small, so
$\operatorname{Exp}((\omega_c + \delta\omega)\Delta t) \approx \operatorname{Exp}(\omega_c\Delta t)\,\operatorname{Exp}(\delta\omega\,\Delta t)$.*

$$
\operatorname{Exp}(\delta\theta_k) \approx \operatorname{Exp}\!\left(A\,\delta\theta_{k-1}\right)\,\operatorname{Exp}(\delta\omega\,\Delta t),
\qquad A = \operatorname{Exp}(-\omega_c\,\Delta t) \ \ (3 \times 3)
$$

*Move 5: sandwich identity
$\operatorname{Exp}(-x)\,\operatorname{Exp}(\delta\theta)\,\operatorname{Exp}(x) = \operatorname{Exp}(\operatorname{Exp}(-x)\,\delta\theta)$:
the same small rotation, written in axes that turned by $x$.*

$$
\delta\theta_k \approx A\,\delta\theta_{k-1} + \delta\omega\,\Delta t
$$

*Move 6: two small rotations compose into one rotation of their sum; drop small × small.*

Result, with $\delta\omega$ expanded:

$$
\delta\theta_k = \operatorname{Exp}\!\left(-(\tilde\omega_k - b_{g,k-1})\,\Delta t\right)\delta\theta_{k-1} - \Delta t\ \delta b_{g,k-1} - \omega_{n,k}\,\Delta t
$$

| term | meaning |
|---|---|
| $\operatorname{Exp}(-\omega_c\Delta t)\,\delta\theta_{k-1}$ | the old orientation error re-expressed in the IMU axes after this step's turn; driving straight it is $\approx I$ and the error just stays |
| $-\Delta t\,\delta b_{g,k-1}$ | a gyro bias error adds a wrong turn every step, so the orientation error grows |
| $-\omega_{n,k}\,\Delta t$ | gyro noise; goes to $Q$ |

Why $\omega_t$ carries no $R$ while $a_t$ does: the acceleration is combined with $v$, $p$ and $g$, which live in
the world frame, so it is rotated there. The turn is applied on the right of $R$, in IMU axes, so it stays in IMU
axes. The equivalent world-frame form puts the turn on the left, $\operatorname{Exp}(R\,\omega\,\Delta t)\,R$;
never $R$ inside and on the right at the same time, which would rotate twice.

### 6.8 Bias errors $\delta b_a$, $\delta b_g$

$$
\delta b_{a,k} = \delta b_{a,k-1} + a_{w,k}, \qquad
\delta b_{g,k} = \delta b_{g,k-1} + \omega_{w,k}
$$

*Straight subtraction of the nominal bias lines from the true ones; nothing is dropped.*

### 6.9 Collected error equations (Solà eq. 261 without $\delta g$)

$$
\begin{aligned}
\delta p_k &= \delta p_{k-1} + \delta v_{k-1}\,\Delta t \\
\delta v_k &= \delta v_{k-1} - R_{k-1}\,[\tilde a_k - b_{a,k-1}]_\times\,\Delta t\ \delta\theta_{k-1} - R_{k-1}\,\Delta t\ \delta b_{a,k-1} - R_{k-1}\,a_{n,k}\,\Delta t \\
\delta\theta_k &= \operatorname{Exp}\!\left(-(\tilde\omega_k - b_{g,k-1})\,\Delta t\right)\delta\theta_{k-1} - \Delta t\ \delta b_{g,k-1} - \omega_{n,k}\,\Delta t \\
\delta b_{a,k} &= \delta b_{a,k-1} + a_{w,k} \\
\delta b_{g,k} &= \delta b_{g,k-1} + \omega_{w,k}
\end{aligned}
$$

Tools used: $\operatorname{Exp}(\delta\theta) \approx I + [\delta\theta]_\times$ · drop small × small and
error × $\Delta t^2$ · $[a]_\times b = -[b]_\times a$ ·
$\operatorname{Exp}(-x)\operatorname{Exp}(\delta\theta)\operatorname{Exp}(x) = \operatorname{Exp}(\operatorname{Exp}(-x)\,\delta\theta)$.
The $\operatorname{Exp}(-\cdots)$ in the $\delta\theta$ line is a $3 \times 3$ rotation matrix.

---

## 7. Discrete error dynamics, F and Q, covariance prediction — Solà eq. 261–270

### 7.1 Stacked form

The five lines of §6.9 are linear in the old errors and in the noise. Stacked:

$$
\delta x_k = F\,\delta x_{k-1} + F_i\,n_k
$$

- $\delta x = [\delta p;\ \delta v;\ \delta\theta;\ \delta b_a;\ \delta b_g]$ (15×1). Each row of $F$ is one of
  the five lines; each column is the old error it multiplies, 0 where that error does not appear.
- $n_k = [-R\,a_{n,k}\,\Delta t;\ -\omega_{n,k}\,\Delta t;\ a_{w,k};\ \omega_{w,k}]$ (12×1): the noise kicks of
  the step. Never computed, they are unknown random values.
- $F_i$ (15×12) places each noise in its row: identities and zeros, the $\delta p$ row is zero.

The linearization is already done inside the moves ($\operatorname{Exp}(\delta\theta) \approx I + [\delta\theta]_\times$,
$\operatorname{Exp}(a+b) \approx \operatorname{Exp}(a)\operatorname{Exp}(b)$, small × small dropped). What remains
is linear in the errors, so $F = \partial\delta x_k / \partial\delta x_{k-1}$ at $\delta x = 0$ is read off as the
coefficients.

### 7.2 F row by row

With $R = R_{k-1}$, $a_c = \tilde a_k - b_{a,k-1}$, $\omega_c = \tilde\omega_k - b_{g,k-1}$, every block $3 \times 3$;
rows and columns in the order $\delta p,\ \delta v,\ \delta\theta,\ \delta b_a,\ \delta b_g$:

$$
F =
\begin{bmatrix}
I & I\,\Delta t & 0 & 0 & 0 \\
0 & I & -R\,[a_c]_\times\,\Delta t & -R\,\Delta t & 0 \\
0 & 0 & \operatorname{Exp}(-\omega_c\,\Delta t) & 0 & -I\,\Delta t \\
0 & 0 & 0 & I & 0 \\
0 & 0 & 0 & 0 & I
\end{bmatrix}
$$

Index form, identity first, then the non-trivial blocks:

```
F = I₁₅
F[kP , kV ] =  I·Δt
F[kV , kTh] = −R·[a_c]×·Δt
F[kV , kBa] = −R·Δt
F[kTh, kTh] =  Exp(−ω_c·Δt)
F[kTh, kBg] = −I·Δt
```

Which $R$: $R_{k-1}$ is the nominal rotation at the start of the step and goes into $F$.
$R_k = R_{k-1}\operatorname{Exp}(\omega_c\Delta t)$ is the nominal after the step (gyro turn, no $\delta\theta$).
$R_t = R\,\operatorname{Exp}(\delta\theta)$ is the true one, unknown.

| $F$ block | meaning |
|---|---|
| $I\,\Delta t$ | velocity error → position error |
| $-R[a_c]_\times\Delta t$ | orientation error → gravity leaks into velocity |
| $-R\,\Delta t$ | accelerometer bias error → velocity error |
| $\operatorname{Exp}(-\omega_c\Delta t)$ | old orientation error, seen from the frame that just turned |
| $-I\,\Delta t$ | gyro bias error → orientation error |

**Row 1, $\delta p$**

| column | block | reason |
|---|---|---|
| $\delta p$ | $I$ | the old position error is still there; no mixing between axes |
| $\delta v$ | $I\,\Delta t$ | velocity error × time → position error |
| $\delta\theta$, $\delta b_a$, $\delta b_g$ | 0 | no direct effect in one step; they reach $\delta p$ through $\delta v$ on the next step |

A zero means "no direct effect in one step", not "no effect": bias → $\delta v$ → $\delta p$, one step at a time.

**Row 2, $\delta v$**

| column | block | reason |
|---|---|---|
| $\delta p$ | 0 | a position error does not change velocity |
| $\delta v$ | $I$ | the old velocity error is still there |
| $\delta\theta$ | $-R[a_c]_\times\Delta t$ | gravity leak: an orientation error rotates the reading the wrong way and produces a false acceleration |
| $\delta b_a$ | $-R\,\Delta t$ | an accelerometer bias error is a wrong acceleration in IMU axes; $R$ turns it to the world, × $\Delta t$ gives velocity |
| $\delta b_g$ | 0 | the gyro bias reaches $\delta v$ only through $\delta\theta$ (row 3) |

Gravity leak with the vehicle at rest: the accelerometer reads the reaction to gravity; the code rotates it with $R$
and adds $g$. With the right $R$ the two cancel and the result is "not moving". With $R$ wrong by $\delta\theta$
they do not cancel; the remainder is a false acceleration that integrates into $\delta v$. Level and still, $a_c$ is
along $z$, so:

| orientation error | velocity error grows along | why |
|---|---|---|
| roll $\delta\theta_x$ | $y$ | gravity leaks sideways |
| pitch $\delta\theta_y$ | $x$ | gravity leaks forward / back |
| yaw $\delta\theta_z$ | nothing | gravity is along $z$; a rotation about $z$ does not move it |

Yaw is therefore invisible while standing still. It shows up only when the vehicle accelerates or turns, when $a_c$
gains $x$ and $y$ components.

**Row 3, $\delta\theta$**

| column | block | reason |
|---|---|---|
| $\delta p$, $\delta v$ | 0 | position and velocity do not change the orientation error |
| $\delta\theta$ | $\operatorname{Exp}(-\omega_c\Delta t)$ | the old orientation error carried over, re-described in the IMU's new axes; same size, a rotation keeps length |
| $\delta b_a$ | 0 | accelerometer bias does not affect orientation |
| $\delta b_g$ | $-I\,\Delta t$ | a gyro bias error is a wrong turn every step, so the orientation error grows |

$\operatorname{Exp}(-\omega_c\Delta t)$ is a rotation matrix (diagonal near 1), not a skew matrix, though it is built
from one: $\operatorname{Exp}(\theta) \approx I + [\theta]_\times$. Driving straight it is $\approx I$ and the error
just stays; turning, the IMU axes turn, so the same physical error gets new components. The rate form (eq. 238)
shows $-[\omega]_\times\delta\theta$; the step form (eq. 261) the full rotation. The code uses the full rotation.
In prediction $\delta\theta$ grows only through the gyro bias error and the gyro noise; the diagonal block only
tracks it as the vehicle turns.

**Rows 4 and 5, $\delta b_a$, $\delta b_g$**

| row | blocks | reason |
|---|---|---|
| $\delta b_a$ | $[0\ \ 0\ \ 0\ \ I\ \ 0]$ | the bias error stays; nothing else changes the accelerometer bias. Its kick $a_w$ goes to $Q$ |
| $\delta b_g$ | $[0\ \ 0\ \ 0\ \ 0\ \ I]$ | the same for the gyro; kick $\omega_w$ goes to $Q$ |

In prediction a bias error never shrinks by itself. It shrinks only at a correction, which sees the velocity or
position error the bias caused (through rows 2 and 3) and traces it back.

### 7.3 Noise: n, F_i, Q_i → Q

| object | content | computed? |
|---|---|---|
| $n$ (12×1) | the actual kicks $-R\,a_n\Delta t$, $-\omega_n\Delta t$, $a_w$, $\omega_w$ | never: unknown random values |
| $F_i$ (15×12) | which noise lands in which row; identities and zeros, $\delta p$ row zero | fixed |
| $Q_i$ (12×12) | variance of each kick: $\mathrm{diag}(\sigma_a^2\Delta t\,I,\ \sigma_g^2\Delta t\,I,\ \sigma_{ba}^2\Delta t\,I,\ \sigma_{bg}^2\Delta t\,I)$ | the four parameters |
| $Q$ (15×15) | $F_i\,Q_i\,F_i^\top$: $Q_i$ placed in the 15×15 shape, zero for $\delta p$ | yes, built directly |

$$
Q = F_i\,Q_i\,F_i^\top = \mathrm{diag}\!\left(0,\ \sigma_a^2\,\Delta t\,I,\ \sigma_g^2\,\Delta t\,I,\ \sigma_{ba}^2\,\Delta t\,I,\ \sigma_{bg}^2\,\Delta t\,I\right)
$$

*Same rule as for $F$: $\mathrm{var}(A\,x) = A\,\Sigma\,A^\top$. $R$ drops out of the $\delta v$ noise because the
noise is the same in every direction: $R\,\sigma^2 I\,R^\top = \sigma^2 I$.*

Index form:

```
Q = 0₁₅
Q[kV , kV ] = σ_a²·Δt · I      (per-reading std s_a instead: (s_a·Δt)²)
Q[kTh, kTh] = σ_g²·Δt · I      (per-reading std s_g instead: (s_g·Δt)²)
Q[kBa, kBa] = σ_ba²·Δt · I
Q[kBg, kBg] = σ_bg²·Δt · I
```

$F$ changes every step; $Q$ changes only through $\Delta t$. Build it from the real $\Delta t$ of each step (safer
with jittery timestamps), or once at startup for a fixed rate.

### 7.4 Covariance prediction — Solà eq. 269

$$
P_k = F\,P_{k-1}\,F^\top + Q
$$

Never skipped. $P$ grows every IMU sample and shrinks at every correction.

### 7.5 Why the error mean stays zero in prediction — Solà eq. 268

The IMU motion goes into the nominal. The error receives only what is wrong with the motion (noise, bias drift),
which is zero-mean. After every correction the mean is injected into the nominal and reset to zero. During
prediction, therefore,

$$
\hat{\delta x}_k = F\,\hat{\delta x}_{k-1} + 0 = F \cdot 0 = 0
$$

*so the mean is not computed in code; the prediction of the error state is only the covariance update of §7.4.*

| | every IMU sample (predict) | measurement arrives (correct) |
|---|---|---|
| nominal $x$ | IMU model (§5) | $+\,\hat{\delta x}$ (inject) |
| error mean $\hat{\delta x}$ | stays 0, skipped | computed: $\hat{\delta x} = K\,r$ |
| $P$ | grows: $F\,P\,F^\top + Q$ | shrinks |
| after | — | reset $\hat{\delta x} = 0$ |

Assumption behind it: whatever drives the error is zero-mean. Anything that leans one way must be modelled as a
state, which is why $b_a$ and $b_g$ are states. If the assumption is broken, the innovation keeps the same sign.

### 7.6 One prediction step

```
build F from R_{k−1}, a_c, ω_c, Δt      (values at the start of the step)
build Q from Δt
P ← F P Fᵀ + Q
update the nominal (§5)
```

---

## 8. Measurement update — Solà §6, eq. 272–288

### 8.1 Observation model and the standard update

When a measurement arrives, prediction has been run up to its time, and the filter holds the nominal $x$ (16), the
error mean $\hat{\delta x} = 0$ (15) and $P$ (15×15).

$$
y = h(x_t) + v, \qquad v \sim \mathcal N(0, V) \qquad \text{(eq. 272–273)}
$$

$y$ is the measurement itself, not the innovation. $h$ maps the true state to what the sensor should read. $V$ is the
measurement noise covariance.

$$
K = P\,H^\top\left(H\,P\,H^\top + V\right)^{-1}, \qquad
\hat{\delta x} \leftarrow K\left(y - h(\hat x_t)\right), \qquad
P \leftarrow (I - K H)\,P \qquad \text{(eq. 274–276)}
$$

with $\hat x_t = x$, the nominal, because $\hat{\delta x} = 0$ at this point. The only piece left to derive is $H$,
the Jacobian of $h$ with respect to the **error**, because the filter estimates the error, evaluated at
$\hat x_t = x$ (eq. 277):

$$
H = \left.\frac{\partial h}{\partial\,\delta x}\right|_{x} \qquad (3 \times 15 \text{ for a position measurement})
$$

### 8.2 Chain rule H = H_x · X_δx — eq. 278

$$
H = H_x\,X_{\delta x}, \qquad
H_x = \frac{\partial h}{\partial x_t}\ \ (3 \times 16), \qquad
X_{\delta x} = \frac{\partial x_t}{\partial\,\delta x}\ \ (16 \times 15)
$$

| piece | what | depends on |
|---|---|---|
| $H_x$ | the ordinary EKF Jacobian: how the reading changes with the true state | the sensor |
| $X_{\delta x}$ | how the true state changes with the error, from $x_t = x \oplus \delta x$ | only the state composition: the same for every sensor |

### 8.3 X_δx — eq. 279–281

Block by block from §1: $p$, $v$, $b_a$, $b_g$ compose by plain addition, so their derivative is $I_3$;
$q_t = q \otimes \delta q$ gives the block $Q_{\delta\theta}$ (4×3). Each true block depends only on its own error,
so the matrix is block-diagonal:

$$
X_{\delta x} = \mathrm{blockdiag}\left(I_3,\ I_3,\ Q_{\delta\theta},\ I_3,\ I_3\right) \qquad (16 \times 15)
$$

*Solà writes $\mathrm{blockdiag}(I_6, Q_{\delta\theta}, I_9)$; his $I_9$ includes $\delta g$, ours is $I_6$.
Written once, reused for GNSS, a pose measurement, any sensor.*

### 8.4 The quaternion block Q_δθ — eq. 281

The question it answers: if $\delta\theta$ changes a little, how do the four numbers of $q_t$ change? Four out,
three in: a $4 \times 3$ matrix.

$$
q_t = q \otimes \delta q = [q]_L\,\delta q
$$

*The quaternion product as a matrix (eq. 18): pull $\delta q$ out of the product formula, the same trick as the
skew matrix.*

$$
\delta q = \operatorname{Exp}(\delta\theta) \approx \left[1,\ \tfrac12\,\delta\theta\right]
\quad\Rightarrow\quad
\frac{\partial\,\delta q}{\partial\,\delta\theta} = \tfrac12 \begin{bmatrix} 0_{1\times3} \\ I_3 \end{bmatrix}
$$

*Small-angle quaternion: the first component is always 1, derivative 0; quaternions store half the angle, hence the
factor ½.*

$$
Q_{\delta\theta} = [q]_L \cdot \tfrac12 \begin{bmatrix} 0_{1\times3} \\ I_3 \end{bmatrix}
= \tfrac12
\begin{bmatrix}
-q_x & -q_y & -q_z \\
 q_w & -q_z &  q_y \\
 q_z &  q_w & -q_x \\
-q_y &  q_x &  q_w
\end{bmatrix}
$$

*Chain rule: ½ times columns 2–4 of $[q]_L$.*

$Q_{\delta\theta}$ matters only when the reading depends on orientation. For a position measurement with the antenna
at the IMU, $h = p_t$, the $q$ columns of $H_x$ are zero and $Q_{\delta\theta}$ is multiplied by zero. With a lever
arm, or a pose measurement, it is needed. If $h$ is instead differentiated with respect to $\delta\theta$ directly
through $R$, $Q_{\delta\theta}$ is not needed and the same $H$ comes out (§8.5).

### 8.5 GNSS with a lever arm

$\ell$ is the antenna position in the IMU frame.

**Case A, antenna at the IMU ($\ell = 0$):**

$$
h(x_t) = p_t, \qquad
H_x = \begin{bmatrix} I_3 & 0_{3\times3} & 0_{3\times4} & 0_{3\times3} & 0_{3\times3} \end{bmatrix} \ (3 \times 16), \qquad
H = H_x\,X_{\delta x} = \begin{bmatrix} I & 0 & 0 & 0 & 0 \end{bmatrix} \ (3 \times 15)
$$

*Only the $p$ columns move the reading; the $q$ columns are zero.*

**Case B, lever arm $\ell$:**

$$
h(x_t) = p_t + R(q_t)\,\ell
$$

*Antenna = IMU position + lever arm rotated to the world.*

$$
H_x = \begin{bmatrix} I_3 & 0 & \dfrac{\partial\left(R(q)\,\ell\right)}{\partial q} & 0 & 0 \end{bmatrix}, \qquad
H = \begin{bmatrix} I_3 & 0 & \dfrac{\partial\left(R(q)\,\ell\right)}{\partial q}\,Q_{\delta\theta} & 0 & 0 \end{bmatrix}
$$

*Chain rule: the $q$ block of $H_x$ is $3 \times 4$; times the $4 \times 3$ $Q_{\delta\theta}$ it gives the
$3 \times 3$ block $-R[\ell]_\times$.*

Shortcut, differentiating with respect to $\delta\theta$ directly:

$$
R_t\,\ell = R\,\operatorname{Exp}(\delta\theta)\,\ell \approx R\,\ell + R\,[\delta\theta]_\times\,\ell = R\,\ell - R\,[\ell]_\times\,\delta\theta
\quad\Rightarrow\quad
\frac{\partial h}{\partial\,\delta\theta} = -R\,[\ell]_\times
$$

*Substitute the composition; small-angle approximation; swap rule $[\delta\theta]_\times\ell = -[\ell]_\times\delta\theta$,
the same swap as in the gravity-leak term. Both routes give the same block.*

**Result:**

$$
H = \begin{bmatrix} I_3 & 0 & -R\,[\ell]_\times & 0 & 0 \end{bmatrix} \qquad (3 \times 15)
$$

A position error moves the antenna directly ($I$); an orientation error swings the arm ($-R[\ell]_\times$). GNSS
therefore sees orientation only a little directly, mostly through velocity (row 2 of $F$). Ignoring $\ell$ tells the
filter at every fix that it is displaced by the length of the arm.

### 8.6 Innovation, gate, gain, Joseph form

$$
\hat z = h(x) = p + R\,\ell
$$

*Predicted reading from the nominal, since $\hat{\delta x} = 0$.*

$$
r = y - \hat z \qquad (3 \times 1)
$$

*Innovation.*

$$
S = H\,P\,H^\top + V \qquad (3 \times 3)
$$

*The predicted spread of $r$: the filter's own uncertainty plus the measurement noise. If $P$ is large (uncertain
filter) or $V$ is large (noisy sensor), a large $r$ is normal.*

$$
d^2 = r^\top S^{-1} r, \qquad d^2 > 7.81 \ \Rightarrow\ \text{reject the fix}
$$

*Chi-square gate, 3 degrees of freedom, 95 % (11.34 for 99 %). $d^2$ measures $r$ in units of its own expected
spread. Too large means an outlier (multipath, jump): skip the update, keep predicting, log the rejected fix.*

$$
K = P\,H^\top S^{-1} \qquad (15 \times 3) \qquad \text{(eq. 274)}
$$

*How much of $r$ to believe, separately for each of the 15 errors: large $P$ or small $V$ means trusting the
measurement more. Computed by solving with $S$ (`S.ldlt().solve`), no explicit inverse.*

$$
\hat{\delta x} = K\,r \qquad (15 \times 1) \qquad \text{(eq. 275)}
$$

*The only time the error mean is non-zero. The measurement has 3 numbers but $K$ has 15 rows: every error is
corrected through the off-diagonal blocks of $P$, the links $F$ built in prediction ($\delta v \to \delta p$,
$\delta\theta \to \delta v$, $\delta b_g \to \delta\theta$).*

$$
P \leftarrow (I - K H)\,P\,(I - K H)^\top + K\,V\,K^\top, \qquad P \leftarrow \tfrac12\left(P + P^\top\right)
$$

*Joseph form of eq. 276, then symmetrize: the same as $(I - KH)\,P$ on paper, but it keeps $P$ symmetric and
positive definite in floating point.*

---

## 9. Injection and reset — Solà eq. 282–288

**Inject** the error mean into the nominal (eq. 282–283):

$$
p \leftarrow p + \hat{\delta p}, \qquad
v \leftarrow v + \hat{\delta v}, \qquad
q \leftarrow \big(q \otimes \operatorname{Exp}(\hat{\delta\theta})\big)\ \text{normalized}, \qquad
b_a \leftarrow b_a + \hat{\delta b}_a, \qquad
b_g \leftarrow b_g + \hat{\delta b}_g
$$

*Plain addition for everything except the rotation, which is composed on the right: the local error, the same as the
composition in §1.*

**Reset** (eq. 284–288):

$$
\hat{\delta x} \leftarrow 0, \qquad
P \leftarrow G\,P\,G^\top, \qquad
G = \mathrm{blockdiag}\!\left(I_6,\ \ I - \left[\tfrac12\,\hat{\delta\theta}\right]_\times,\ \ I_6\right)
$$

*After injection the error is measured from the new nominal, whose axes turned by $\hat{\delta\theta}$, so the
$\delta\theta$ part of $P$ is rotated by half of it. $\hat{\delta\theta}$ is tiny, so $G \approx I$: v1 uses
$G = I$ and skips this step.*

---

## 10. Summary: the complete algorithm

```
startup
  x₀, P₀ from the initializer (outside the filter)
  Q from σ_a, σ_g, σ_ba, σ_bg            (or rebuilt every step from the real Δt)

every IMU sample (ã_k, ω̃_k, t_k)                                            §5, §7
  Δt  = t_k − t_last                       from the timestamps
  R   = R(q)                               OLD q
  a_c = ã_k − b_a ;   ω_c = ω̃_k − b_g       clean readings, IMU axes
  F   = I₁₅
        F[kP ,kV ] =  I·Δt
        F[kV ,kTh] = −R·[a_c]×·Δt
        F[kV ,kBa] = −R·Δt
        F[kTh,kTh] =  Exp(−ω_c·Δt)
        F[kTh,kBg] = −I·Δt
  Q   = 0₁₅
        Q[kV ,kV ] = σ_a²·Δt·I ;  Q[kTh,kTh] = σ_g²·Δt·I ;  Q[kBa,kBa] = σ_ba²·Δt·I ;  Q[kBg,kBg] = σ_bg²·Δt·I
  P  ← F P Fᵀ + Q
  a   = R·a_c + g
  p  ← p + v·Δt + ½·a·Δt²                  OLD v
  v  ← v + a·Δt
  q  ← (q ⊗ Exp(ω_c·Δt)).normalized()
  b_a, b_g unchanged ;   t_last = t_k
  (error mean stays 0: not computed)

every GNSS fix (y, V, t)                                                     §8, §9
  predict with IMU up to t
  ẑ  = p + R·ℓ
  r  = y − ẑ
  H  = [ I   0   −R·[ℓ]×   0   0 ]
  S  = H P Hᵀ + V
  d² = rᵀ S⁻¹ r ;   if d² > 7.81: log, return
  K  = P Hᵀ S⁻¹                            solve, no explicit inverse
  δx̂ = K·r
  A  = I − K H ;   P ← A P Aᵀ + K V Kᵀ ;   P ← ½(P + Pᵀ)
  inject:  p += δx̂[kP] ;  v += δx̂[kV] ;  q ← (q ⊗ Exp(δx̂[kTh])).normalized() ;  b_a += δx̂[kBa] ;  b_g += δx̂[kBg]
  reset:   δx̂ = 0 ;   P ← G P Gᵀ with G = I (v1)
```
