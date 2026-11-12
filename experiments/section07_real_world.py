"""
Section 7 - Real-world problems, each shown and then fixed. Final gains Kp=300, Ki=200, Kd=20.
  (a) motor limited to 30 N  -> integral windup  -> fix: anti-windup (clamping)
  (b) noisy sensor (1 mm)    -> jumpy D part     -> fix: low-pass filter on D (tau = 10 ms)
  (c) sudden target change   -> derivative kick  -> fix: take D from the measurement
  (d) controller updates too slowly (large Ts)   -> fix: faster loop, or softer gains
Each problem is shown ON ITS OWN (other fixes off) so you can see one cause and one effect.
Run:  python -m experiments.section07_real_world -> figures/fig07a_windup.png ... fig07d_slow.png
"""

import numpy as np
import matplotlib.pyplot as plt
from pid_control.pid import PID
from pid_control.plant import MassSpringDamper, simulate
from pid_control.metrics import step_metrics
from pid_control.common import M, C, K, FINAL, COLORS, INK2, TARGET_STYLE, finish, save_results, fmt_metrics

out = {}


def plant():
    return MassSpringDamper(M, C, K)


# ======================= (a) windup =======================
print("  (a) windup, force limit 30 N")
cases = {"no force limit (ideal)":       dict(),
         "30 N limit, no anti-windup":   dict(u_min=-30, u_max=30),
         "30 N limit + anti-windup":     dict(u_min=-30, u_max=30, anti_windup=True)}
fig, axs = plt.subplots(1, 3, figsize=(7.4, 2.8))
out["a"] = {}
for j, (name, extra) in enumerate(cases.items()):
    log = simulate(plant(), PID(**FINAL, dt=0.001, **extra), 1.0, t_end=10.0)
    m = step_metrics(log["t"], log["y"])
    m["max_I_part"] = float(log["i"].max())
    out["a"][name] = m
    style = dict(color=INK2, lw=1.2, ls=":") if j == 0 else dict(color=COLORS[j])
    axs[0].plot(log["t"], log["y"], label=name, **style)
    axs[1].plot(log["t"], log["u"], label=name, **style)
    axs[2].plot(log["t"], log["i"], label=name, **style)
    print(f"    {name:<28} {fmt_metrics(m)} | max I part {m['max_I_part']:.1f} N")
axs[0].axhline(1.0, label="target 1 m", **TARGET_STYLE)
axs[0].set(title="Position", xlabel="time t [s]", ylabel="position x [m]", xlim=(0, 4), ylim=(0, 1.2))
axs[1].set(title="Motor force", xlabel="time t [s]", ylabel="force u [N]", xlim=(0, 4), ylim=(-35, 60))
axs[1].axhline(30, color=COLORS[7], lw=1, ls="--", label="limit ±30 N")
axs[1].axhline(-30, color=COLORS[7], lw=1, ls="--")
axs[2].axhline(20, label="20 N needed to hold 1 m", **TARGET_STYLE)
axs[2].set(title="I part of the force", xlabel="time t [s]", ylabel="I part [N]", xlim=(0, 4))
axs[0].legend(loc="lower right", fontsize=6.5)
axs[1].legend(loc="upper right", fontsize=6.5, handles=axs[1].get_legend_handles_labels()[0][-1:])
axs[2].legend(loc="upper right", fontsize=6.5, handles=axs[2].get_legend_handles_labels()[0][-1:])
a_no, a_yes = out["a"]["30 N limit, no anti-windup"], out["a"]["30 N limit + anti-windup"]
finish(fig, "fig07a_windup",
       f"While stuck at 30 N the I part winds up to {a_no['max_I_part']:.0f} N: overshoot "
       f"{a_no['overshoot_pct']:.1f} %. Anti-windup caps it at {a_yes['max_I_part']:.0f} N: overshoot "
       f"{a_yes['overshoot_pct']:.2f} %.")

# ======================= (b) noisy sensor =======================
print("  (b) sensor noise 1 mm (std), seed 1")
NOISE = 0.001
fig, (a1, a2) = plt.subplots(1, 2, figsize=(7.2, 2.8))
out["b"] = {}
win = None
for j, (name, tau) in enumerate([("no D filter", 0.0), ("D filter, tau = 10 ms", 0.01)]):
    log = simulate(plant(), PID(**FINAL, dt=0.001, d_filter_tau=tau), 1.0, t_end=3.0,
                   noise_std=NOISE, seed=1)
    win = (log["t"] >= 1.0) & (log["t"] <= 2.0)       # steady part: only noise is moving things
    m = step_metrics(log["t"], log["y"])
    m.update(force_std=float(log["u"][win].std()), d_std=float(log["d"][win].std()),
             force_range=float(np.ptp(log["u"][win])), pos_std_mm=float(1000 * log["y"][win].std()))
    out["b"][name] = m
    a1.plot(log["t"][win], log["u"][win], color=COLORS[j], lw=1.0 if j == 0 else 1.8,
            label=f"{name}: force std {m['force_std']:.1f} N")
    if j == 0:
        a2.plot(log["t"][win], 1000 * (log["y_meas"][win] - 1), color="#c9c8c2", lw=0.6,
                label="what the sensor reports (noise)")
    a2.plot(log["t"][win], 1000 * (log["y"][win] - 1), color=COLORS[j],
            label=f"true position, {name}")
    print(f"    {name:<22} force std {m['force_std']:.2f} N (range {m['force_range']:.0f} N) | "
          f"D part std {m['d_std']:.2f} N | true position std {m['pos_std_mm']:.3f} mm | "
          f"OS {m['overshoot_pct']:.2f}% settle {m['settling_time']:.3f} s")
a1.axhline(20, label="20 N needed to hold 1 m", **TARGET_STYLE)
a1.set(title="Motor force (steady part, 1-2 s)", xlabel="time t [s]", ylabel="force u [N]",
       ylim=(-80, 210))
a1.legend(loc="upper right", fontsize=7)
a2.set(title="Position error (steady part)", xlabel="time t [s]", ylabel="x − 1 m [mm]", ylim=(-4, 6.5))
a2.legend(loc="upper right", fontsize=7, ncol=1)
b0, b1 = out["b"]["no D filter"], out["b"]["D filter, tau = 10 ms"]
finish(fig, "fig07b_noise",
       f"1 mm of noise makes the unfiltered force jump by ±{b0['force_std']:.0f} N (std); the 10 ms "
       f"filter cuts this to {b1['force_std']:.1f} N while the block's own jitter is only {b1['pos_std_mm']:.2f} mm.")

# ======================= (c) derivative kick =======================
print("  (c) target jumps 0 -> 1 m at t = 0.5 s")
step_at = lambda t: 0.0 if t < 0.5 else 1.0
fig, (a1, a2) = plt.subplots(1, 2, figsize=(7.2, 2.8))
out["c"] = {}
for j, (name, dom) in enumerate([("D on error", False), ("D on measurement", True)]):
    log = simulate(plant(), PID(**FINAL, dt=0.001, d_on_measurement=dom), step_at, t_end=6.0)
    after = log["t"] >= 0.5
    m = step_metrics(log["t"][after] - 0.5, log["y"][after])
    m.update(peak_force=float(np.abs(log["u"]).max()), peak_d=float(np.abs(log["d"]).max()))
    out["c"][name] = m
    a1.plot(log["t"], log["u"], color=COLORS[j], label=f"{name}: peak {m['peak_force']:,.0f} N")
    a2.plot(log["t"], log["y"], color=COLORS[j], label=f"{name}: OS {m['overshoot_pct']:.1f} %")
    print(f"    {name:<17} peak force {m['peak_force']:,.0f} N | peak D part {m['peak_d']:,.0f} N | "
          f"{fmt_metrics(m)}")
a1.set(title="Motor force when the target jumps", xlabel="time t [s]", ylabel="force u [N]",
       xlim=(0.45, 0.65))
a1.legend(loc="upper right", fontsize=7)
a1.annotate("D on measurement stays ≤ 300 N\n(too small to see at this scale)", xy=(0.52, 300), xytext=(0.54, 6000),
            fontsize=7, color=INK2, arrowprops=dict(arrowstyle="->", color=INK2, lw=0.8))
a2.plot(log["t"], log["r"], label="target", **TARGET_STYLE)
a2.set(title="Position", xlabel="time t [s]", ylabel="position x [m]", xlim=(0, 2), ylim=(-0.05, 1.4))
a2.legend(loc="lower right", fontsize=7)
c0, c1 = out["c"]["D on error"], out["c"]["D on measurement"]
finish(fig, "fig07c_kick",
       f"A 1 m jump in 1 ms gives D a {c0['peak_d']:,.0f} N spike (force {c0['peak_force']:,.0f} N); "
       f"taking D from the measurement removes it (peak {c1['peak_force']:.0f} N, all from P).")

# ======================= (d) slow updates =======================
print("  (d) controller update interval Ts")
fig, (a1, a2) = plt.subplots(1, 2, figsize=(7.2, 2.8))
out["d"] = {}
for j, Ts in enumerate([0.001, 0.03, 0.05, 0.07, 0.08]):
    log = simulate(plant(), PID(**FINAL, dt=Ts), 1.0, t_end=10.0, Ts=Ts)
    m = step_metrics(log["t"], log["y"])
    unstable = bool(np.abs(log["y"]).max() > 3)
    out["d"][f"{Ts}"] = dict(m, unstable=unstable)
    lab = f"Ts = {Ts * 1000:.0f} ms: " + ("UNSTABLE" if unstable else f"OS {m['overshoot_pct']:.1f} %")
    a1.plot(log["t"], log["y"], color=COLORS[j], lw=1.5, label=lab)
    print(f"    Ts={Ts * 1000:>3.0f} ms: " + ("UNSTABLE (swings grow)" if unstable else fmt_metrics(m)))
a1.axhline(1.0, label="target 1 m", **TARGET_STYLE)
a1.set(title="Same gains, slower updates", xlabel="time t [s]", ylabel="position x [m]",
       xlim=(0, 2), ylim=(-0.6, 3.7))
a1.legend(loc="upper right", fontsize=6.5, ncol=2)

out["d_fix"] = {}
for j, (name, g, Ts) in enumerate([("Ts = 1 ms, final gains (300/200/20)", (300, 200, 20), 0.001),
                                    ("Ts = 70 ms, final gains (300/200/20)", (300, 200, 20), 0.07),
                                    ("Ts = 70 ms, softer gains (100/100/8)", (100, 100, 8), 0.07)]):
    log = simulate(plant(), PID(*g, dt=Ts), 1.0, t_end=10.0, Ts=Ts)
    m = step_metrics(log["t"], log["y"])
    out["d_fix"][name] = m
    a2.plot(log["t"], log["y"], color=[COLORS[0], COLORS[3], COLORS[2]][j], lw=1.6,
            label=f"{name}: OS {m['overshoot_pct']:.1f} %")
    print(f"    fix: {name:<38} {fmt_metrics(m)}")
a2.axhline(1.0, label="target 1 m", **TARGET_STYLE)
a2.set(title="Fix: faster loop, or softer gains", xlabel="time t [s]", ylabel="position x [m]",
       xlim=(0, 2), ylim=(0, 2.15))
a2.legend(loc="upper right", fontsize=6.5)
d = out["d"]
finish(fig, "fig07d_slow",
       f"Overshoot {d['0.001']['overshoot_pct']:.1f} % at 1 ms, {d['0.05']['overshoot_pct']:.0f} % at 50 ms, "
       f"{d['0.07']['overshoot_pct']:.0f} % at 70 ms, unstable at 80 ms; softer gains bring 70 ms back to "
       f"{out['d_fix']['Ts = 70 ms, softer gains (100/100/8)']['overshoot_pct']:.1f} %.")
save_results("section07", out)
