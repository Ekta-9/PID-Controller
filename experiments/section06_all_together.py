"""
Section 6 - All together: P, PI, PD and full PID on one graph plus a table.
All four use the same Kp = 300 so the comparison is fair; I and D are added on top.
Final gains: Kp = 300, Ki = 200, Kd = 20.
Run:  python -m experiments.section06_all_together   ->  figures/fig06_compare.png
"""

import matplotlib.pyplot as plt
from pid_control.pid import PID
from pid_control.plant import MassSpringDamper, simulate
from pid_control.metrics import step_metrics
from pid_control.common import M, C, K, COLORS, TARGET_STYLE, finish, save_results

cases = {"P   (Kp=300)":                (300, 0, 0),
         "PI  (Kp=300, Ki=200)":        (300, 200, 0),
         "PD  (Kp=300, Kd=20)":         (300, 0, 20),
         "PID (Kp=300, Ki=200, Kd=20)": (300, 200, 20)}

fig, (a1, a2) = plt.subplots(1, 2, figsize=(7.2, 3.0))
table = {}
print(f"  {'controller':<29}{'rise [s]':>9}{'OS [%]':>8}{'settle [s]':>11}{'ss-err [m]':>11}{'max u [N]':>10}")
for j, (name, g) in enumerate(cases.items()):
    log = simulate(MassSpringDamper(M, C, K), PID(*g, dt=0.001), 1.0, t_end=8.0)
    m = step_metrics(log["t"], log["y"])
    m["max_force"] = float(abs(log["u"]).max())
    table[name] = m
    for ax in (a1, a2):
        ax.plot(log["t"], log["y"], color=COLORS[j], label=name)
    print(f"  {name:<29}{m['rise_time']:>9.3f}{m['overshoot_pct']:>8.2f}"
          f"{m['settling_time']:>11.3f}{m['ss_error']:>11.4f}{m['max_force']:>10.0f}")

for ax in (a1, a2):
    ax.axhline(1.0, label="target 1 m", **TARGET_STYLE)
a1.set(title="P, PI, PD and PID compared", xlabel="time t [s]", ylabel="position x [m]",
       xlim=(0, 1.5), ylim=(0, 1.5))
a1.legend(loc="lower right", fontsize=7)
a2.set(title="Zoom near the target (first 3 s)", xlabel="time t [s]", ylabel="position x [m]",
       xlim=(0, 3.0), ylim=(0.9, 1.06))
finish(fig, "fig06_compare",
       "Only PID has all four: overshoot 0.5 %, settles in 0.24 s, and zero steady-state error.")
save_results("section06", table)
