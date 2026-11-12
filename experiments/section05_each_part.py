"""
Section 5 - Each part alone: P only (4 strengths), then add I, then add D.
Every graph has a "theory vs simulation" check on the right.

HAND CALCULATIONS checked here
  P only : at rest  k*x = Kp*(1-x)  ->  x_ss = Kp/(Kp+k),  e_ss = k/(Kp+k)
  PI     : at rest the error must be 0 (otherwise the I part keeps growing),
           so the I part alone must supply the spring force k*1 = 20 N.
  PD     : m x'' + (c+Kd) x' + (k+Kp) x = Kp     (D acts like an extra damper)
           wn = sqrt((k+Kp)/m),  zeta = (c+Kd) / (2*sqrt(m*(k+Kp)))
           overshoot = exp(-pi*zeta/sqrt(1-zeta^2))  (for zeta < 1)
Run:  python -m experiments.section05_each_part  ->  figures/fig05a_P.png, fig05b_PI.png, fig05c_PD.png
"""

import numpy as np
import matplotlib.pyplot as plt
from pid_control.pid import PID
from pid_control.plant import MassSpringDamper, simulate
from pid_control.metrics import step_metrics
from pid_control.common import M, C, K, COLORS, INK2, TARGET_STYLE, finish, save_results, fmt_metrics

out = {"P": {}, "PI": {}, "PD": {}}


def run(kp, ki=0.0, kd=0.0, t_end=4.0):
    log = simulate(MassSpringDamper(M, C, K), PID(kp, ki, kd, dt=0.001), 1.0, t_end)
    return log, step_metrics(log["t"], log["y"])


def overshoot_theory(zeta):
    return 100 * np.exp(-np.pi * zeta / np.sqrt(1 - zeta**2)) if zeta < 1 else 0.0


# ---------------- 5a: P only ----------------
fig, (a1, a2) = plt.subplots(1, 2, figsize=(7.2, 2.9), gridspec_kw=dict(width_ratios=[1.35, 1]))
print("  P only:")
for j, kp in enumerate([10, 30, 100, 300]):
    log, m = run(kp, t_end=2.0)
    x_th = kp / (kp + K)
    zeta = C / (2 * np.sqrt(M * (K + kp)))
    out["P"][kp] = dict(m, x_ss_theory=x_th, e_ss_theory=1 - x_th, zeta=zeta,
                        os_theory=overshoot_theory(zeta))
    a1.plot(log["t"], log["y"], color=COLORS[j], label=f"Kp = {kp}")
    a1.axhline(x_th, color=COLORS[j], lw=0.8, ls=":")
    print(f"    Kp={kp:>3}: x_ss theory {x_th:.4f} sim {m['final']:.4f} | "
          f"OS theory {out['P'][kp]['os_theory']:.2f}% sim {m['overshoot_pct']:.2f}%")
a1.axhline(1.0, label="target 1 m", **TARGET_STYLE)
a1.set(title="P only: four strengths", xlabel="time t [s]", ylabel="position x [m]",
       xlim=(0, 2), ylim=(0, 1.45))
a1.legend(loc="upper right", ncol=2)

kp_line = np.linspace(1, 500, 300)
a2.plot(kp_line, kp_line / (kp_line + K), color=INK2, lw=1.5, label="theory Kp/(Kp+20)")
for j, kp in enumerate(out["P"]):
    a2.plot(kp, out["P"][kp]["final"], "o", color=COLORS[j], ms=8, markeredgecolor="white",
            label="simulation" if j == 0 else None)
a2.axhline(1.0, label="target 1 m", **TARGET_STYLE)
a2.set(title="Where it stops: theory vs simulation", xlabel="Kp [N/m]",
       ylabel="final position [m]", ylim=(0, 1.1))
a2.legend(loc="lower right")
finish(fig, "fig05a_P",
       "P only always stops short: error 0.667 → 0.400 → 0.167 → 0.063 m as Kp rises, "
       "while overshoot climbs 0.1 % → 40.7 %.")

# ---------------- 5b: add I (PI, Kp = 100) ----------------
fig, (a1, a2) = plt.subplots(1, 2, figsize=(7.2, 2.9))
print("  PI (Kp=100):")
for j, ki in enumerate([0, 50, 200, 500]):
    log, m = run(100, ki=ki, t_end=4.0)
    out["PI"][ki] = dict(m, i_final=float(log["i"][-1]))
    a1.plot(log["t"], log["y"], color=COLORS[j], label=f"Ki = {ki}")
    a2.plot(log["t"], log["i"], color=COLORS[j], label=f"Ki = {ki}")
    print(f"    Ki={ki:>3}: {fmt_metrics(m)} | I part at end {log['i'][-1]:.2f} N")
a1.axhline(1.0, label="target 1 m", **TARGET_STYLE)
a1.set(title="Adding I (Kp = 100)", xlabel="time t [s]", ylabel="position x [m]",
       xlim=(0, 4), ylim=(0, 1.55))
a1.legend(loc="lower right", ncol=2)
a2.axhline(K * 1.0, label="spring force at 1 m = 20 N", **TARGET_STYLE)
a2.set(title="The I part's force builds up to 20 N", xlabel="time t [s]",
       ylabel="I part of force [N]", xlim=(0, 4))
a2.legend(loc="upper right", ncol=2, fontsize=7)
finish(fig, "fig05b_PI",
       "I removes the 0.167 m gap (error → 0.000 m for Ki = 200) because its force grows until it "
       "alone holds the spring's 20 N.")

# ---------------- 5c: add D (PD, Kp = 300) ----------------
fig, (a1, a2) = plt.subplots(1, 2, figsize=(7.2, 2.9), gridspec_kw=dict(width_ratios=[1.35, 1]))
print("  PD (Kp=300):")
kd_list = [0, 5, 10, 20]
for j, kd in enumerate(kd_list):
    log, m = run(300, kd=kd, t_end=2.0)
    zeta = (C + kd) / (2 * np.sqrt(M * (K + 300)))
    out["PD"][kd] = dict(m, zeta=zeta, os_theory=overshoot_theory(zeta))
    a1.plot(log["t"], log["y"], color=COLORS[j], label=f"Kd = {kd}  (ζ = {zeta:.2f})")
    print(f"    Kd={kd:>2}: zeta {zeta:.3f} | OS theory {overshoot_theory(zeta):.2f}% "
          f"sim {m['overshoot_pct']:.2f}% | {fmt_metrics(m)}")
a1.axhline(1.0, label="target 1 m", **TARGET_STYLE)
a1.set(title="Adding D (Kp = 300)", xlabel="time t [s]", ylabel="position x [m]",
       xlim=(0, 1.0), ylim=(0, 1.45))
a1.legend(loc="lower right", ncol=1)

kd_line = np.linspace(0, 30, 200)
zl = (C + kd_line) / (2 * np.sqrt(M * (K + 300)))
a2.plot(kd_line, [overshoot_theory(z) for z in zl], color=INK2, lw=1.5, label="theory from ζ")
for j, kd in enumerate(kd_list):
    a2.plot(kd, out["PD"][kd]["overshoot_pct"], "o", color=COLORS[j], ms=8,
            markeredgecolor="white", label="simulation" if j == 0 else None)
a2.set(title="Overshoot: theory vs simulation", xlabel="Kd [N·s/m]", ylabel="overshoot [%]",
       ylim=(0, 45))
a2.legend(loc="upper right")
finish(fig, "fig05c_PD",
       "D acts like an extra damper: Kd = 0 → 20 raises ζ from 0.28 to 0.84 and cuts overshoot "
       "from 40.7 % to 0.7 %.")

save_results("section05", {k: {str(kk): vv for kk, vv in v.items()} for k, v in out.items()})
