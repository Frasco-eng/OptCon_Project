# Optimal Control of a Flexible Robotic Link

**OPTCON-RL 2025/2026 — Course Project #2**
Optimal Control and Reinforcement Learning · Master's Degree in Automation Engineering
Alma Mater Studiorum – Università di Bologna

This project designs and tests optimal control strategies for a **flexible double pendulum** (a two-link robotic arm with an elastic joint). It has two stages:

1. **Offline trajectory generation**: a Newton method for optimal control, with Armijo line search, turns a rough reference into a dynamically feasible optimal trajectory between two equilibrium configurations. An optional variant extends the time horizon automatically.
2. **Online trajectory tracking**: the optimal trajectory is tracked from perturbed initial conditions with either a **Linear Time-Varying LQR** or a **Model Predictive Controller** (CasADi).

The full derivation and results are in [`report/report_group_10.pdf`](report/report_group_10.pdf). The assignment text is in [`Flexible_Assignment.pdf`](Flexible_Assignment.pdf).

---

## Repository Structure

```
.
├── code/                        # Current implementation (run from here)
│   ├── main.py                  # Entry point: configuration + pipeline orchestration
│   ├── model.py                 # DoublePendulumFlexible: dynamics, Jacobians, equilibria
│   ├── trajectory_reference_generation.py   # Equilibria, task list, reference curves
│   ├── trajectory_generation_newton.py      # Newton's method for optimal control
│   ├── trajectory_generation_newton_extended.py  # Newton + dynamic horizon extension
│   ├── armijo.py                # Armijo step-size selection
│   ├── trajectory_tracking_LQR.py   # LTV-LQR tracking
│   ├── trajectory_tracking_MPC.py   # MPC tracking (CasADi)
│   ├── solver.py                # Integrators, Riccati recursions, MPC problem
│   ├── costs.py                 # Stage/terminal costs and their derivatives
│   ├── plots.py                 # Plotting utilities
│   └── animation.py             # Arm animations
├── code_project_old_versions/   # Earlier development branches (archive only)
├── report/                      # LaTeX report, sections and figures
└── Flexible_Assignment.pdf      # Project assignment
```

---

## Requirements

- Python 3.10 or newer (developed on 3.12)
- `numpy`, `scipy`, `matplotlib`, `control`, `casadi`

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install numpy scipy matplotlib control casadi
```

> `casadi` is needed for the MPC tracker. `solver.py` imports it at load time, so install it even if you only use LQR.

---

## Usage

Run the pipeline from the **repository root**, so figures are saved under `report/figs/code/`:

```bash
python code/main.py
```

`main.py` is the only script you need to run. For each selected task it:

1. computes the start and end equilibria and builds a reference trajectory,
2. computes the optimal trajectory with the chosen generator,
3. tracks it with the chosen controller over several perturbed runs (unless `GENERATION_ONLY = True`).

### Configuration

All options are set at the top of `code/main.py`.

**Algorithm selection**

```python
GENERATOR = "Newton"        # "Newton" | "Newton-Extended"
TRACKER = "LQR"             # "LQR" | "MPC"
GENERATION_ONLY = False     # True: skip tracking
TASKS = ['swing_up', 'elbow_up', 'swing_down', 'complex_maneuver']
```

- **Newton**: Newton's method for optimal control over a fixed horizon.
- **Newton-Extended**: if the terminal error is still above `terminal_error_tol` after convergence, the horizon is extended by `extension_duration` and optimization continues.
- **LQR**: time-varying feedback gains from linearizing along the optimal trajectory.
- **MPC**: a receding-horizon optimal control problem solved at every step, with explicit input constraints.

**Output flags**

```python
save_flag = False     # Save figures/animations to report/figs/code/...
plot_flag = True      # Show figures
debug_flag = True     # Verbose console output
plot_armijo = False   # Plot Armijo line-search diagnostics
```

**Main hyperparameters** (the defaults work as they are)

| Parameter | Default | Meaning |
|---|---|---|
| `discr_method` | `"fe"` | Discretization: Forward Euler (`fe`) or Runge–Kutta 4 (`rk4`) |
| `dt` | `0.01` | Sampling time [s] |
| `method` | `'poly5'` | Reference curve: `poly3`, `poly5`, `stepwise` |
| `max_iters` | `300` | Maximum Newton iterations |
| `descent_threshold` | `1e-6` | Stopping threshold on the descent direction |
| `QQt_gen`, `RRt_gen` | `I`, `1e-3·I` | Generation cost weights |
| `stepsize_0`, `c`, `beta` | `1.0`, `0.5`, `0.7` | Armijo parameters |
| `runs` | `5` | Tracking runs with random initial perturbations |
| `perturb_scale` | `2` | Size of the initial perturbation |
| `T_pred` | `100` | MPC prediction horizon (steps) |
| `QQt_track`, `RRt_track` | `I`, `1e-4·I` | Tracking cost weights |

### Available Tasks

| Task | Maneuver |
|---|---|
| `swing_up` | Down-Down → Up-Up |
| `elbow_up` | Down-Down → Right-Up |
| `swing_down` | Up-Up → Down-Down |
| `complex_maneuver` | Right-Down → Down-Up |
| `topside_pump` | Up-Down → Up-Up |
| `diagonal_swing` | Left-Diagonal-Down → Right-Diagonal-Up |
| `top_to_elbow_up` | Up-Up → Right-Up |
| `low_diagonal_hold` | Down-Down → Right-Diagonal-Down |
| `half_swing_up` | Down-Up → Up-Down |

### Output

- **Console**: Newton progress (cost, descent direction, step size) and tracking errors.
- **Figures and animations** (when `save_flag = True`): written to `report/figs/code/<generator>_<tracker>/<task>/`, for example `report/figs/code/traj_gen_newton/swing_up/` and `report/figs/code/traj_track_LQR/swing_up/`.

---

## Model

The model has two links. Only the first joint is actuated. The second joint is underactuated and has a nonlinear elastic term (flexibility coefficient `k`), and both joints have viscous friction. The state is `x = [θ₁, θ₂, θ̇₁, θ̇₂]` and the input is the torque on the first joint. `model.py` provides:

- the equations of motion (mass matrix, Coriolis, gravity, elastic and friction terms),
- discrete-time dynamics (Forward Euler or RK4) with analytic Jacobians `A`, `B`,
- equilibrium computation, forward kinematics, and a PBH stabilizability test.

The parameters used in this project are in `PARAM_SET_2`: `m₁ = m₂ = 2 kg`, `l₁ = l₂ = 1.5 m`, `I₁ = I₂ = 1.5 kg·m²`, `f₁ = f₂ = 1 N·m·s/rad`, `k = 0.6 N·m/rad`.

---

## Authors

- **Francesco Cenni**
- **Giulio Mazzaroppi**
- **Giacomo Pezzolati**

Professor: Giuseppe Notarstefano · Tutor: Marco Falotico
Alma Mater Studiorum – Università di Bologna, Academic Year 2025/2026

© 2026 Francesco Cenni, Giulio Mazzaroppi, Giacomo Pezzolati. **All rights reserved.**
No part of this repository may be copied, modified, or redistributed without the authors' prior written permission.
