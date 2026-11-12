"""
Section 4 - How to judge a result: mark rise time, overshoot, settling time and
steady-state error on ONE graph. We use P-only control with Kp = 100 because that run
shows all four clearly (it overshoots AND stops short of the target).
Run:  python -m experiments.section04_metrics   ->  figures/fig04_metrics.png
"""

import numpy as np
import matplotlib.pyplot as plt
from pid_control.pid import PID
from pid_control.plant import MassSpringDamper, simulate
from pid_control.metrics import step_metrics
from pid_control.common import M, C, K, COLORS, INK, INK2, TARGET_STYLE, finish, save_results, fmt_metrics

log = simulate(MassSpringDamper(M, C, K), PID(kp=100, dt=0.001), 1.0, t_end=5.0)
t, y = log["t"], log["y"]
m = step_metrics(t, y)
yf = m["final"]
print("  P-only Kp=100:", fmt_metrics(m))

fig, ax = plt.subplots(figsize=(7.2, 3.6))
ax.plot(t, y, color=COLORS[0], lw=2, label="position x(t), P-only Kp = 100")
ax.axhline(1.0, label="target 1 m", **TARGET_STYLE)
ax.axhspan(0.98 * yf, 1.02 * yf, color=COLORS[2], alpha=0.18, label="±2 % band around final value")

# 1) Rise time: 10 % and 90 % crossings
t10 = t[np.argmax(y >= 0.1 * yf)]; t90 = t[np.argmax(y >= 0.9 * yf)]
for tt, lvl in [(t10, 0.1 * yf), (t90, 0.9 * yf)]:
    ax.plot([tt], [lvl], "o", color=COLORS[1], ms=5)
ax.annotate("", xy=(t90, 0.42), xytext=(t10, 0.42),
            arrowprops=dict(arrowstyle="<->", color=COLORS[1], lw=1.5))
ax.text(0.215, 0.36, f"① rise time\n{m['rise_time']:.3f} s\n(10 %→90 %)",
        color=INK, fontsize=8)

# 2) Overshoot: peak above final value
ax.annotate("", xy=(m["t_peak"], m["peak"]), xytext=(m["t_peak"], yf),
            arrowprops=dict(arrowstyle="<->", color=COLORS[6], lw=1.5))
ax.text(m["t_peak"] + 0.03, 1.035, f"② overshoot {m['overshoot_pct']:.1f} %\n"
        f"(peak {m['peak']:.3f} m)", color=INK, fontsize=8)

# 3) Settling time: last exit from the ±2 % band
ax.axvline(m["settling_time"], color=COLORS[3], lw=1.5, label="settling time")
ax.text(m["settling_time"] + 0.02, 0.25, f"③ settling time\n{m['settling_time']:.3f} s", color=INK, fontsize=8)

# 4) Steady-state error: gap between final value and target
ax.annotate("", xy=(1.35, 1.0), xytext=(1.35, yf),
            arrowprops=dict(arrowstyle="<->", color=COLORS[7], lw=1.5))
ax.text(1.32, 0.885, f"④ steady-state\nerror {m['ss_error']:.3f} m", color=INK, fontsize=8, ha="right")

ax.set(title="Figure 4 - The four measures marked on one step response",
       xlabel="time t [s]", ylabel="position x [m]", xlim=(0, 1.5), ylim=(0, 1.15))
ax.text(m["t_peak"] + 0.03, 0.66, "(overshoot is measured from the\nfinal value, not the target)",
        color=INK2, fontsize=7)
ax.legend(loc="lower right")
finish(fig, "fig04_metrics",
       f"Rise {m['rise_time']:.3f} s, overshoot {m['overshoot_pct']:.1f} %, settles in "
       f"{m['settling_time']:.2f} s, but stops {m['ss_error']:.3f} m short of the 1 m target.")
save_results("section04", m)
