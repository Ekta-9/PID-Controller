"""
common.py - Shared settings and plotting helpers used by every script in experiments/.

Keeping colours, the test-system values and the "save figure" code in ONE place means
every graph looks the same and every script uses exactly the same numbers.
"""

import os
import json
import textwrap
import matplotlib
matplotlib.use("Agg")           # draw straight to files; no window needed
import matplotlib.pyplot as plt

# ---------- The test system (Section 3) and our final gains (Section 6) ----------
M, C, K = 1.0, 10.0, 20.0                    # kg, N*s/m, N/m
FINAL = dict(kp=300.0, ki=200.0, kd=20.0)    # chosen in Section 6
DT = 0.001                                   # default controller + physics step [s]

# ---------- Colours: a colour-blind-checked palette, used in a FIXED order ----------
COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]
INK, INK2, GRID = "#0b0b0b", "#52514e", "#e2e1dc"
TARGET_STYLE = dict(color=INK2, linestyle="--", linewidth=1.2)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # repo root
FIG_DIR = os.path.join(ROOT, "figures")
RES_DIR = os.path.join(ROOT, "results")
os.makedirs(FIG_DIR, exist_ok=True)
os.makedirs(RES_DIR, exist_ok=True)

plt.rcParams.update({
    "font.size": 9, "axes.titlesize": 10, "axes.labelsize": 9, "legend.fontsize": 8,
    "axes.edgecolor": INK2, "axes.labelcolor": INK, "xtick.color": INK2, "ytick.color": INK2,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.7,
    "axes.spines.top": False, "axes.spines.right": False,
    "lines.linewidth": 1.8, "legend.frameon": False, "figure.dpi": 100,
    "axes.prop_cycle": matplotlib.cycler(color=COLORS),
})


def finish(fig, name, caption):
    """Add the one-line caption under the figure, then save it as figures/<name>.png."""
    lines = textwrap.wrap(caption, 105)                 # long captions go onto two lines
    h = fig.get_size_inches()[1]
    bottom = (0.06 + 0.15 * len(lines)) / h             # room (as a fraction) for the caption
    fig.tight_layout(rect=[0, bottom, 1, 1])
    fig.text(0.5, 0.04 / h, "\n".join(lines), ha="center", va="bottom", fontsize=8.5,
             color=INK2, style="italic", linespacing=1.3)
    path = os.path.join(FIG_DIR, name + ".png")
    fig.savefig(path, dpi=200, facecolor="white")
    plt.close(fig)
    print(f"  saved {os.path.relpath(path, ROOT)}")


def save_results(name, data):
    """Store the numbers each section reports, so the PDF quotes exactly what the code made."""
    path = os.path.join(RES_DIR, name + ".json")
    with open(path, "w") as f:
        json.dump(data, f, indent=2)
    print(f"  saved {os.path.relpath(path, ROOT)}")


def fmt_metrics(m):
    return (f"rise {m['rise_time']:.3f} s | overshoot {m['overshoot_pct']:.2f} % | "
            f"settle {m['settling_time']:.3f} s | ss-error {m['ss_error']:+.4f} m")
