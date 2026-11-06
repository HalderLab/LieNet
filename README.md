# LieNet

**Fast Optimal Control Through Geometry and Learning**

LieNet is an ongoing research project investigating the integration of **geometric optimal control and self-supervised neural networks** to accelerate motion planning on Lie groups.

Classical optimal-control methods often require computationally expensive iterative solvers to generate trajectories for each new target configuration, making real-time implementation challenging. LieNet seeks to reduce this computational burden by learning to predict the **initial costate** of an optimal-control problem, from which the corresponding controls and state trajectories can be obtained by integrating known Hamiltonian dynamics.

Unlike purely data-driven approaches, LieNet incorporates the geometric structure and necessary optimality conditions of the underlying control problem directly into its learning framework.

## Mathematical Formulation

### Optimal Control on Lie Groups

Consider a matrix Lie group $G$ with associated Lie algebra $\mathfrak{g}$. We consider a fixed-horizon optimal-control problem of the form

$$
\begin{aligned}
\min_{u(\cdot)} \quad
J(u)
&=
\int_0^T \ell(u(t))\,dt
+
\Phi(g(T),g_d),\\
\text{subject to}\quad
\dot g(t)&=g(t)\xi(u(t)),\\
g(0)&=g_0,
\end{aligned}
$$

where:

- $g(t)\in G$ is the system configuration.
- $\xi(u)\in\mathfrak{g}$ describes the system's controlled Lie-algebra velocity.
- $u(t)\in\mathbb{R}^m$ is the control input.
- $\ell(u)$ is the running control cost.
- $\Phi(g(T),g_d)$ penalizes deviation from the desired configuration $g_d\in G$.
- $T>0$ is the fixed terminal time.

The objective balances control effort against terminal configuration accuracy while respecting the system's geometric dynamics.

### Pontryagin's Maximum Principle

Pontryagin's Maximum Principle (PMP) provides necessary conditions for optimality through a Hamiltonian system involving the state and an associated costate.

Using left trivialization, the costate can be represented as $\mu\in\mathfrak{g}^*$, the dual of the Lie algebra. The corresponding Hamiltonian is

$$
H(g,\mu,u)
=
\langle\mu,\xi(u)\rangle-\ell(u).
$$

The optimal control satisfies the Hamiltonian maximization condition

$$
u^*(t)
\in
\underset{u}{\operatorname{argmax}}
\;H(g(t),\mu(t),u).
$$

For a quadratic control cost,

$$
\ell(u)=\frac12 u^\top Wu,
\qquad W\succ0,
$$

the maximization condition can often be solved explicitly, expressing the optimal control in terms of the costate.

The resulting control law, together with the Hamiltonian state–costate equations, defines a system of differential equations. For left-invariant problems, the costate dynamics admit a reduced Lie–Poisson formulation on $\mathfrak{g}^*$.

At the terminal time, the costate must satisfy the transversality condition

$$
\boxed{
\mu(T)+d_g^L\Phi(g(T),g_d)=0,
}
$$

where $d_g^L\Phi$ denotes the left-trivialized differential of the terminal cost.

Traditionally, numerical shooting methods iteratively adjust the unknown initial costate $\mu(0)$ until this terminal boundary condition is satisfied.

## LieNet: Learning the Initial Costate

LieNet replaces the iterative search for an initial costate with a neural-network approximation.

For a fixed initial configuration and time horizon, the network learns the mapping

$$
\boxed{
\mathcal N_\theta:g_d\longmapsto\widehat\mu(0),
}
$$

where $\theta$ denotes the trainable network parameters.

The predicted initial costate is propagated through the known PMP equations to generate the state, costate, and control trajectories:

$$
\widehat\mu(0)
\;\xrightarrow{\text{PMP integration}}\;
\bigl(g(t),\mu(t),u(t)\bigr).
$$

This approach retains the known geometric dynamics instead of requiring the network to learn them from data.

### Self-Supervised Training

A conventional supervised-learning approach would require a dataset of optimal trajectories and their corresponding initial costates, typically obtained through expensive numerical optimization.

LieNet instead uses the **terminal transversality condition as a self-supervised training signal**.

For each sampled desired configuration $g_d$, the network predicts an initial costate and integrates the PMP equations. The resulting terminal residual is

$$
r_T(\theta,g_d)
=
\mu_\theta(T)
+
d_g^L\Phi(g_\theta(T),g_d).
$$

The network is trained by minimizing

$$
\boxed{
\mathcal L(\theta)
=
\mathbb E_{g_d}
\left[
r_T(\theta,g_d)^\top
W_r
r_T(\theta,g_d)
\right],
}
$$

where $W_r$ is a positive-definite residual-weighting matrix.

Gradients are propagated through the numerical integration to update the network parameters.

**No precomputed optimal trajectories or numerical solver labels are required during training.**

The learned initial costate approximates a solution of the PMP boundary-value problem. Since PMP provides necessary rather than sufficient conditions, satisfaction of the transversality condition alone does not guarantee global optimality.

## Applications

The framework is investigated using two representative matrix Lie groups.

### 1. Planar Motion on SE(2)

The first example considers a unit-speed unicycle moving in the plane, with steering rate $u$ as its control input.

Its dynamics are

$$
\begin{aligned}
\dot x &= \cos\phi,\\
\dot y &= \sin\phi,\\
\dot\phi &= u.
\end{aligned}
$$

Equivalently, the system can be expressed on $SE(2)$:

$$
\dot g=g\xi(u),
\qquad
\xi(u)=
\begin{bmatrix}
0&-u&1\\
u&0&0\\
0&0&0
\end{bmatrix}.
$$

The optimal-control objective penalizes steering effort and terminal position error:

$$
J=
\frac12\int_0^T u(t)^2\,dt
+
\frac{k_p}{2}
\left[
(x(T)-x_d)^2+
(y(T)-y_d)^2
\right].
$$

Using an appropriate left-trivialized costate $\mu=(\mu_1,\mu_2,\mu_3)^\top$, the optimal control is $u^*=\mu_1$, and the reduced costate dynamics are

$$
\begin{aligned}
\dot\mu_1&=-\mu_3,\\
\dot\mu_2&=\mu_1\mu_3,\\
\dot\mu_3&=-\mu_1\mu_2.
\end{aligned}
$$

LieNet learns to predict the initial costate for different desired planar positions and generates the corresponding trajectories through forward integration.

### 2. Three-Dimensional Rotation on SO(3)

The second example considers rotational motion on $SO(3)$, where the configuration is represented by a rotation matrix $R$ satisfying

$$
R^\top R=I,\qquad \det R=1.
$$

The system evolves according to angular-velocity-controlled kinematics:

$$
\dot R=R\widehat u,
\qquad
\widehat u=
\begin{bmatrix}
0&-u_3&u_2\\
u_3&0&-u_1\\
-u_2&u_1&0
\end{bmatrix}.
$$

The objective balances angular control effort against terminal orientation error:

$$
J=
\frac12\int_0^T u(t)^\top Wu(t)\,dt
+
\frac{k_R}{2}
\left\|
\log(R_d^\top R(T))^\vee
\right\|^2.
$$

Here, the logarithm measures the geodesic rotation discrepancy between the achieved and desired orientations.

For the fully actuated model, the Hamiltonian optimality condition yields

$$
u^*=W^{-1}\mu,
$$

with reduced costate dynamics

$$
\dot\mu=\mu\times u^*.
$$

This example demonstrates how the same learning framework extends from planar motion to three-dimensional rotational configurations.

## Numerical Evaluation

LieNet is compared against classical numerical optimal-control methods, including **shooting methods** and **forward-backward sweep algorithms**.

Performance is evaluated using three metrics:

**Computation Time:** The wall-clock time required to generate a control trajectory, including neural inference and PMP integration for LieNet.

**Original Objective Cost:** The value of the optimal-control objective $J$, measuring the quality of the generated solution under the prescribed control and terminal penalties.

**RMS Trajectory Error:** The discrepancy between the trajectory generated by LieNet and a converged numerical reference:

$$
E_{\mathrm{RMS}}
=
\left(
\frac1T
\int_0^T
d_G(g_{\mathrm{NN}}(t),g_{\mathrm{ref}}(t))^2\,dt
\right)^{1/2}.
$$

For $SE(2)$, the trajectory metric measures Euclidean positional discrepancy. For $SO(3)$, it measures geodesic angular discrepancy.

Evaluations include target configurations both within and outside the neural network's training distribution to investigate computational efficiency, solution quality, and generalization.

## Publication and Code Availability

**This research is currently unpublished and remains under active development.**

The source code, numerical experiments, and implementation details will be made publicly available in this repository following publication.

## Research Team

- **[Xincheng Li](https://xincheng.li)** — University of South Florida
- **[Udit Halder](https://sites.google.com/view/udithalder/)** — University of South Florida
- **[Biswadip Dey](https://d-biswa.github.io/)** — Meta

## Correspondence

**Prof. Udit Halder**  
Department of Mechanical and Aerospace Engineering  
University of South Florida  
Email: [udithalder@usf.edu](mailto:udithalder@usf.edu)  
Website: [Udit Halder](https://sites.google.com/view/udithalder/)

## References

1. V. Jurdjevic, *Geometric Control Theory*. Cambridge University Press, 1997.
2. A. A. Agrachev and Y. L. Sachkov, *Control Theory from the Geometric Viewpoint*. Springer, 2004.
3. R. M. Murray, Z. Li, and S. S. Sastry, *A Mathematical Introduction to Robotic Manipulation*. CRC Press, 1994.