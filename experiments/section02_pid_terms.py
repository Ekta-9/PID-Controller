"""
Section 2 - The PID formula: what P, I and D each "look at".

Left : the error curve from a real run. At one moment t* we mark
       P = the error value now, I = the shaded area so far, D = the slope (tangent line).
Right: the same idea the way a COMPUTER does it, with samples every dt:
       I = sum of thin rectangles (e_k * dt), D = (e_k - e_(k-1)) / dt.
Run:  python -m experiments.section02_pid_terms   ->  figures/fig02_pid_terms.png
"""

import numpy as np
import matplotlib.pyplot as plt
from pid_control.pid import PID
from pid_control.plant import MassSpringDamper, simulate
from pid_control.common import M, C, K, COLORS, INK2, finish, save_results

# A PI run (Kp=100, Ki=200) gives an error curve that overshoots, so the area changes sign.
log = simulate(MassSpringDamper(M, C, K), PID(kp=100, ki=200, dt=0.001), 1.0, t_end=0.6)
t, e = log["t"], log["r"] - log["y"]

t_star = 0.12                                   # the moment we "freeze" to look at
i_star = int(round(t_star / 0.001))
e_now = e[i_star]                               # what P looks at
area = np.sum(e[:i_star] * 0.001)               # what I looks at (area under the curve)
slope = np.gradient(e, t)[i_star]               # what D looks at (steepness)

fig, (a1, a2) = plt.subplots(1, 2, figsize=(7.2, 2.9))

# ---- Left panel: continuous view ----
a1.plot(t, e, color=COLORS[0], label="error e(t)")
a1.fill_between(t[: i_star + 1], 0, e[: i_star + 1], color=COLORS[2], alpha=0.25,
                label=f"I looks at: area = {area:.3f} m·s")
a1.plot([t_star], [e_now], "o", color=COLORS[1], ms=7, zorder=5,
        label=f"P looks at: e now = {e_now:.3f} m")
tt = np.array([t_star - 0.06, t_star + 0.06])
a1.plot(tt, e_now + slope * (tt - t_star), color=COLORS[6], lw=2,
        label=f"D looks at: slope = {slope:.2f} m/s")
a1.axhline(0, color=INK2, lw=0.8)
a1.set(title="What P, I and D look at (at t* = 0.12 s)", xlabel="time t [s]",
       ylabel="error e [m]", xlim=(0, 0.6))
a1.legend(loc="upper right")

# ---- Right panel: computer's view with a coarse dt so the steps are visible ----
dt_show = 0.02
tk = np.arange(0, 0.6 + 1e-9, dt_show)
ek = np.interp(tk, t, e)
a2.bar(tk, ek, width=dt_show, align="edge", color=COLORS[2], alpha=0.35,
       edgecolor="white", linewidth=1, label="I: sum of e_k · dt (rectangles)")
a2.plot(tk, ek, "o", color=COLORS[0], ms=3.5, label="samples e_k (every dt = 0.02 s)")
k = 6
a2.plot(tk[k - 1:k + 1], ek[k - 1:k + 1], color=COLORS[6], lw=2.4,
        label="D: (e_k − e_(k−1)) / dt")
a2.axhline(0, color=INK2, lw=0.8)
a2.set(title="The computer's version (samples every dt)", xlabel="time t [s]",
       ylabel="error e [m]", xlim=(0, 0.6))
a2.legend(loc="upper right")

finish(fig, "fig02_pid_terms",
       f"At t* = {t_star} s: P sees {e_now:.3f} m, I sees {area:.3f} m·s of built-up error, "
       f"D sees the error falling at {abs(slope):.2f} m/s.")
save_results("section02", dict(t_star=t_star, e_now=e_now, area=area, slope=slope))
