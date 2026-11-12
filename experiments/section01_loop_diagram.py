"""
Section 1 - The basic idea: draw the feedback loop
    target -> controller -> machine -> sensor -> back to the start.
Each box also shows the shower example underneath it.
Run:  python -m experiments.section01_loop_diagram   ->  figures/fig01_loop.png
"""

import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Circle
from matplotlib.lines import Line2D
from pid_control.common import COLORS, INK, INK2, finish

FWD, BACK = COLORS[0], COLORS[1]   # blue = forward path, orange = feedback path

fig, ax = plt.subplots(figsize=(7.2, 3.0))
ax.set_xlim(0, 10.6); ax.set_ylim(0, 4.6); ax.axis("off")


def box(x, y, w, h, title, sub, color):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.05,rounding_size=0.12",
                                facecolor="white", edgecolor=color, linewidth=1.8))
    ax.text(x + w / 2, y + h * 0.64, title, ha="center", va="center", fontsize=9.5,
            color=INK, weight="bold")
    ax.text(x + w / 2, y + h * 0.28, sub, ha="center", va="center", fontsize=7.5, color=INK2)


def arrow(x0, y0, x1, y1, color, label=None, lx=None, ly=None):
    ax.annotate("", xy=(x1, y1), xytext=(x0, y0),
                arrowprops=dict(arrowstyle="-|>", color=color, lw=1.8, mutation_scale=12))
    if label:
        ax.text(lx if lx is not None else (x0 + x1) / 2, ly if ly is not None else y0 + 0.2,
                label, ha="center", va="bottom", fontsize=7.8, color=INK)


# Boxes (top row = forward path; bottom = sensor on the way back)
box(0.1, 2.6, 1.7, 1.2, "Target r", "wanted temp: 38 °C", FWD)
box(3.6, 2.6, 2.0, 1.2, "Controller (PID)", "your hand on the tap", FWD)
box(6.4, 2.6, 2.0, 1.2, "Machine (plant)", "tap, pipes, water", FWD)
box(6.4, 0.35, 2.0, 1.2, "Sensor", "your skin feels it", BACK)

# The comparison circle: error = target - measured
ax.add_patch(Circle((2.5, 3.2), 0.28, facecolor="white", edgecolor=INK, linewidth=1.5))
ax.text(2.5, 3.2, "Σ", ha="center", va="center", fontsize=11, color=INK)
ax.text(2.18, 3.62, "+", fontsize=9, color=INK); ax.text(2.62, 2.62, "−", fontsize=11, color=INK)

arrow(1.85, 3.2, 2.22, 3.2, FWD)
arrow(2.78, 3.2, 3.6, 3.2, FWD, "error e", ly=3.32)
arrow(5.65, 3.2, 6.4, 3.2, FWD, "push u", ly=3.32)
arrow(8.45, 3.2, 10.4, 3.2, FWD, "actual y", 9.45, 3.32)
# Feedback path: from the output down to the sensor, then back to the circle
ax.plot([9.45, 9.45], [3.2, 0.95], color=BACK, lw=1.8)
arrow(9.45, 0.95, 8.45, 0.95, BACK)
ax.plot([6.4, 2.5], [0.95, 0.95], color=BACK, lw=1.8)
arrow(2.5, 0.95, 2.5, 2.92, BACK)
ax.text(4.4, 1.07, "measured y  (feedback: back to the start)", ha="center", va="bottom",
        fontsize=7.8, color=INK)
ax.text(2.5, 4.15, "compare:  e = r − y", ha="center", fontsize=8, color=INK)

ax.set_title("Figure 1 - The feedback loop (shower example under each box)", loc="left")
ax.legend(handles=[Line2D([], [], color=FWD, lw=1.8, label="forward path (decide and act)"),
                   Line2D([], [], color=BACK, lw=1.8, label="feedback path (measure and report back)")],
          loc="lower left", bbox_to_anchor=(0.0, -0.06), ncol=1)
finish(fig, "fig01_loop",
       "The controller only ever sees the error e = r − y; it keeps acting until the error is close to zero.")
