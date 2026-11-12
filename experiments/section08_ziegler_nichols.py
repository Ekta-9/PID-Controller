"""
Section 8 - Choosing the gains with the Ziegler-Nichols (Z-N) recipe on G(s) = 1/(s+1)^3.

Step 1: P only. Raise Kp until the output swings steadily (neither growing nor shrinking).
        That gain is Ku ("ultimate gain"); the time between swings is Pu ("ultimate period").
        We find it automatically: measure how much each swing grows or shrinks
        (amplitude ratio), then home in on ratio = 1 by bisection (halving the search range).
Step 2: Apply the classic Z-N PID table:  Kp = 0.6 Ku,  Ti = Pu/2,  Td = Pu/8
        which in our form is  Ki = Kp/Ti = 1.2 Ku/Pu,  Kd = Kp*Td = 0.075 Ku*Pu
Step 3: Plot it, then improve it by hand and compare with numbers.

Hand calculation: the loop swings steadily when Kp*G(jw) = -1.
  The phase of 1/(jw+1)^3 is -3*atan(w) = -180 deg  ->  w = tan(60 deg) = sqrt(3) rad/s
  Size: 1/|jw+1|^3 = 1/(1+3)^(3/2) = 1/8  ->  Ku = 8,  Pu = 2*pi/sqrt(3) = 3.628 s
Run:  python -m experiments.section08_ziegler_nichols  ->  figures/fig08a_ku.png, fig08b_zn.png
"""

import numpy as np
import matplotlib.pyplot as plt
from pid_control.pid import PID
from pid_control.plant import ThirdOrderLag, simulate
from pid_control.metrics import step_metrics
from pid_control.common import COLORS, INK2, TARGET_STYLE, finish, save_results, fmt_metrics

TS = 0.005          # controller + physics step for this slower system [s]


def p_only(kp, t_end=60.0):
    return simulate(ThirdOrderLag(), PID(kp, dt=TS), 1.0, t_end, Ts=TS, dt_phys=TS)


def swing_ratio(log):
    """Size of the last full swing divided by the size of the 2nd swing (1 = steady)."""
    y, t = log["y"], log["t"]
    peaks = np.where((y[1:-1] > y[:-2]) & (y[1:-1] >= y[2:]))[0] + 1
    troughs = np.where((y[1:-1] < y[:-2]) & (y[1:-1] <= y[2:]))[0] + 1
    if len(peaks) < 4 or len(troughs) < 4:
        return 0.0, np.nan                       # no real oscillation: it just settles
    amps = [y[p] - y[troughs[troughs > p][0]] for p in peaks if np.any(troughs > p)]
    period = float(np.mean(np.diff(t[peaks][1:])))
    return amps[-1] / amps[1], period


# ---- Step 1a: raise Kp in coarse steps (what you would do by hand) ----
print("  Step 1: raise Kp until the swing is steady")
coarse = {}
for kp in [2, 4, 6, 7, 8, 9, 10]:
    r, per = swing_ratio(p_only(kp))
    coarse[kp] = r
    word = "settles" if r < 0.98 else ("GROWS" if r > 1.02 else "steady")
    print(f"    Kp = {kp:>4}: swing ratio {r:7.3f}  -> {word}")

# ---- Step 1b: bisection between the last 'settles' and first 'grows' ----
lo, hi = 7.0, 9.0
for _ in range(14):
    mid = 0.5 * (lo + hi)
    r, _ = swing_ratio(p_only(mid))
    lo, hi = (mid, hi) if r < 1 else (lo, mid)
Ku = 0.5 * (lo + hi)
_, Pu = swing_ratio(p_only(Ku))
print(f"    measured Ku = {Ku:.3f} (theory 8.000),  Pu = {Pu:.3f} s (theory {2 * np.pi / np.sqrt(3):.3f} s)")

fig, ax = plt.subplots(figsize=(7.2, 2.9))
for j, (kp, name) in enumerate([(0.75 * Ku, "0.75·Ku: swings shrink"), (Ku, "Ku: steady swing"),
                                (1.1 * Ku, "1.1·Ku: swings grow")]):
    log = p_only(kp, t_end=30)
    ax.plot(log["t"], log["y"], color=COLORS[j], lw=1.5, label=f"Kp = {kp:.2f} ({name})")
    if j == 1:
        y = log["y"]; t = log["t"]
        pk = np.where((y[1:-1] > y[:-2]) & (y[1:-1] >= y[2:]))[0] + 1
        t1, t2 = t[pk[3]], t[pk[4]]
        ax.annotate("", xy=(t2, 1.82), xytext=(t1, 1.82),
                    arrowprops=dict(arrowstyle="<->", color=INK2, lw=1.2))
        ax.text((t1 + t2) / 2, 1.88, f"Pu = {Pu:.2f} s", ha="center", fontsize=8)
ax.axhline(1.0, label="target 1", **TARGET_STYLE)
ax.set(title="Step 1 - P only on 1/(s+1)³: find the steady swing", xlabel="time t [s]",
       ylabel="output y [-]", xlim=(0, 30), ylim=(-0.6, 3.1))
ax.legend(loc="upper right", fontsize=7, ncol=2)
finish(fig, "fig08a_ku",
       f"At Kp = Ku = {Ku:.2f} the swing neither grows nor shrinks; peaks are Pu = {Pu:.2f} s apart "
       f"(theory: Ku = 8, Pu = 3.63 s).")

# ---- Step 2 and 3: Z-N table, then improve by hand ----
zn = dict(kp=0.6 * Ku, ki=1.2 * Ku / Pu, kd=0.075 * Ku * Pu)
better = dict(kp=4.0, ki=0.8, kd=3.0)
print(f"  Step 2: Z-N gains  Kp={zn['kp']:.3f}  Ki={zn['ki']:.3f}  Kd={zn['kd']:.3f}")
res = {}
fig, (a1, a2) = plt.subplots(1, 2, figsize=(7.2, 2.9))
for j, (name, g) in enumerate([("Z-N recipe", zn), ("improved by hand", better)]):
    log = simulate(ThirdOrderLag(), PID(**g, dt=TS), 1.0, 40.0, Ts=TS, dt_phys=TS)
    m = step_metrics(log["t"], log["y"])
    res[name] = dict(m, **g)
    lab = f"{name} (Kp={g['kp']:.1f}, Ki={g['ki']:.2f}, Kd={g['kd']:.2f})"
    a1.plot(log["t"], log["y"], color=COLORS[j], label=lab)
    a2.plot(log["t"], log["u"], color=COLORS[j], label=name)
    print(f"    {name:<17} {fmt_metrics(m)}")
a1.axhline(1.0, label="target 1", **TARGET_STYLE)
a1.set(title="Z-N result vs hand-improved", xlabel="time t [s]", ylabel="output y [-]",
       xlim=(0, 20), ylim=(0, 1.7))
a1.legend(loc="lower right", fontsize=6.5)
a2.set(title="Controller output", xlabel="time t [s]", ylabel="controller output u [-]", xlim=(0, 20))
a2.legend(loc="upper right", fontsize=7)
z, b = res["Z-N recipe"], res["improved by hand"]
finish(fig, "fig08b_zn",
       f"Z-N gives {z['overshoot_pct']:.0f} % overshoot and settles in {z['settling_time']:.1f} s; "
       f"cutting Ki and raising Kd gives {b['overshoot_pct']:.1f} % and {b['settling_time']:.1f} s.")
save_results("section08", dict(Ku=Ku, Pu=Pu, Ku_theory=8.0, Pu_theory=2 * np.pi / np.sqrt(3),
                               coarse_ratios=coarse, results=res))
