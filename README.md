# LieNet

**Fast Optimal Control Through Geometry and Learning**

LieNet is a research project that uses geometry and self-supervised learning for motion planning on Lie groups. Given a desired configuration, it predicts the initial costate for a fixed-horizon control problem and integrates the governing equations to obtain a trajectory and its control inputs.

## Approach

Pontryagin’s maximum principle describes candidate solutions through coupled state and costate equations. Lie–Poisson reduction expresses the costate in the dual of the Lie algebra. LieNet learns to predict that reduced costate at the start of the trajectory.

During training, the network integrates the equations and measures how closely the final costate satisfies the terminal transversality condition. The control problem supplies the training signal directly, without precomputed solver trajectories.

## Examples

- **Planar motion on SE(2).** A unit-speed unicycle steers toward a desired position. The objective balances steering effort and terminal position error.
- **Attitude control on SO(3).** A system steers its orientation in three dimensions using body angular rates. The objective balances control effort and terminal rotation error.

The project compares LieNet with a classical forward–backward solver using computation time, objective cost, and RMS trajectory discrepancy. The trajectory comparison uses planar position distance for SE(2) and geodesic rotation angle for SO(3).

## Project team

Xincheng Li and Udit Halder, University of South Florida, and Bishwadip Dey, Meta.
