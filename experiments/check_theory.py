"""
check_theory.py - Proves the simulator and PID class are correct before we draw any graph.

Test: P-only control of the mass-spring-damper (m=1, c=10, k=20), target 1 m.
Hand calculation (Section 5):
    At rest, x'' = x' = 0, so  k*x = u = Kp*(1 - x)   ->   x_ss = Kp / (Kp + k)
    Steady-state error         e_ss = 1 - x_ss = k / (Kp + k)
    Closed loop: x'' + c x' + (k + Kp) x = Kp   ->   wn = sqrt(k + Kp),  zeta = c / (2*wn)
    Predicted overshoot (if zeta < 1) = exp(-pi*zeta / sqrt(1 - zeta^2)) * 100 %
Run:  python -m experiments.check_theory
"""

import numpy as np
from pid_control.pid import PID
from pid_control.plant import MassSpringDamper, simulate
from pid_control.metrics import step_metrics

m, c, k = 1.0, 10.0, 20.0
print(f"{'Kp':>5} | {'x_ss theory':>11} {'x_ss sim':>9} | {'e_ss theory':>11} {'e_ss sim':>9} |"
      f" {'zeta':>5} {'OS% theory':>10} {'OS% sim':>8}")
print("-" * 86)
for kp in [10, 30, 100, 300, 400]:
    log = simulate(MassSpringDamper(m, c, k), PID(kp=kp, dt=0.001), setpoint=1.0, t_end=5.0)
    res = step_metrics(log["t"], log["y"])

    x_theory = kp / (kp + k)
    wn = np.sqrt(k + kp)
    zeta = c / (2 * wn)
    os_theory = 100 * np.exp(-np.pi * zeta / np.sqrt(1 - zeta**2)) if zeta < 1 else 0.0

    print(f"{kp:>5} | {x_theory:>11.4f} {res['final']:>9.4f} | {1 - x_theory:>11.4f}"
          f" {res['ss_error']:>9.4f} | {zeta:>5.3f} {os_theory:>10.2f} {res['overshoot_pct']:>8.2f}")
