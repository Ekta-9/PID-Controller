# PID Control from Scratch

A beginner's guide to PID control, with every graph and every number made by my own Python code.

- **Guide:** [`PID_Control_Guide.pdf`](PID_Control_Guide.pdf) (13 pages)
- **Code:** NumPy + Matplotlib only. The PID controller and the simulator (RK4 integration) are written by hand. No control libraries (no `python-control`, `scipy.signal` or `simple-pid`).
- **Reproducible:** all random noise uses fixed seeds, so every run gives exactly the same figures and numbers.

**Test system:** a 1 kg block on a spring (k = 20 N/m) with a damper (c = 10 N·s/m), moved to a target of 1 m.
**Final gains:** Kp = 300, Ki = 200, Kd = 20, which give 0.49 % overshoot, 0.24 s settling time and zero steady-state error.

---

## Setup

You need Python 3.9 or newer.

```bash
git clone [GitHub link]
cd PID
python -m venv .venv
# Windows:      .venv\Scripts\activate
# macOS/Linux:  source .venv/bin/activate
pip install -r requirements.txt
```

## Recreate everything

Run this from the repo's top folder:

```bash
python run_all.py
```

This takes about 30 seconds. It rewrites every PNG in `figures/` and every JSON file in `results/`, and prints the checks and tables to the terminal.

## Recreate one graph

Run each command from the repo's top folder. The `-m` form lets the scripts find the `pid_control` package.

| Figure | Shows | Command |
|---|---|---|
| `fig01_loop.png` | Feedback loop diagram | `python -m experiments.section01_loop_diagram` |
| `fig02_pid_terms.png` | P, I and D parts of the force, and the discrete sum/difference | `python -m experiments.section02_pid_terms` |
| `fig03_test_system.png` | Block with a constant push (why 20 N is needed for 1 m) | `python -m experiments.section03_test_system` |
| `fig04_metrics.png` | Rise time, overshoot, settling time and steady-state error marked on one response | `python -m experiments.section04_metrics` |
| `fig05a_P.png`, `fig05b_PI.png`, `fig05c_PD.png` | P only (4 values of Kp), then adding I, then adding D | `python -m experiments.section05_each_part` |
| `fig06_compare.png` | P, PI, PD and PID compared | `python -m experiments.section06_all_together` |
| `fig07a_windup.png` … `fig07d_slow.png` | Windup, sensor noise, derivative kick, slow updates (each with its fix) | `python -m experiments.section07_real_world` |
| `fig08a_ku.png`, `fig08b_zn.png` | Ziegler–Nichols on 1/(s+1)³, then hand-improved | `python -m experiments.section08_ziegler_nichols` |
| `fig09_full_demo.png` | Final PID class with all fixes (+ printed step log) | `python -m experiments.section09_full_demo` |
| (terminal table) | Hand calculation vs simulation for P-only control | `python -m experiments.check_theory` |

## Folder structure

```
PID/
├── pid_control/              reusable code
│   ├── pid.py                  PID class: limits + anti-windup, D filter, D on measurement
│   ├── plant.py                mass-spring-damper, 3rd-order lag, RK4 simulator
│   ├── metrics.py              rise time, overshoot, settling time, steady-state error
│   └── common.py               test-system values, final gains, plot style, save helpers
├── experiments/              one script per section of the guide
│   ├── check_theory.py         theory vs simulation table (P only)
│   └── section01_*.py … section09_*.py
├── figures/                  all graphs (made by the scripts)
├── results/                  the numbers quoted in the PDF, as JSON
├── docs/
│   ├── pid_guide.html          source of the PDF
│   └── build_pdf.py            HTML → PDF
├── PID_Control_Guide.pdf
├── run_all.py
└── requirements.txt
```

## Using the PID class

```python
from pid_control.pid import PID
from pid_control.plant import MassSpringDamper, simulate

pid = PID(kp=300, ki=200, kd=20, dt=0.001,
          u_min=-30, u_max=30, anti_windup=True,     # motor limit + anti-windup
          d_filter_tau=0.01,                         # low-pass filter on D (10 ms)
          d_on_measurement=True)                     # no derivative kick

log = simulate(MassSpringDamper(m=1, c=10, k=20), pid, setpoint=1.0, t_end=5.0,
               noise_std=0.001, seed=1)              # 1 mm sensor noise, fixed seed
print(log["y"][-1])                                  # final position [m]
```

`pid.update(setpoint, measurement)` returns the force for one time step. `pid.reset()` clears its memory.

## How the metrics are defined

| Measure | Definition |
|---|---|
| Rise time | time to go from 10 % to 90 % of the final value |
| Overshoot | (peak − final) ÷ final × 100 %, measured from the **final value**, not the target |
| Settling time | last time the response is outside ±2 % of the final value |
| Steady-state error | target − final value, where final = mean of the last 5 % of the run |

Physics runs in 1 ms RK4 steps. The controller runs every Ts seconds and holds its force between updates. Ts is 1 ms unless a section says otherwise; Section 8 uses 5 ms.
