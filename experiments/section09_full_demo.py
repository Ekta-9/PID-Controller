"""
Section 9 - The code: the reusable PID class with ALL Section 7 fixes switched on,
running the test system in a realistic situation:
    * motor limited to +/-30 N           -> output limit + anti-windup
    * sensor noise 1 mm (std), seed 1    -> derivative filter, tau = 10 ms
    * target 1 m at t = 0, then 0.5 m at t = 5 s (a sudden jump) -> D on measurement
We run the same situation with a "naive" PID (no fixes) to show what the fixes buy us.
Run:  python -m experiments.section09_full_demo   ->  figures/fig09_full_demo.png + printed output
"""

import numpy as np
import matplotlib.pyplot as plt
from pid_control.pid import PID
from pid_control.plant import MassSpringDamper, simulate
from pid_control.metrics import step_metrics
from pid_control.common import M, C, K, FINAL, COLORS, INK2, TARGET_STYLE, finish, save_results, fmt_metrics

# ---- 1. Build the controller: final gains + every real-world fix ----
controller = PID(**FINAL, dt=0.001,
                 u_min=-30.0, u_max=30.0,      # the motor can push at most 30 N either way
                 anti_windup=True,             # freeze I while the motor is maxed out
                 d_filter_tau=0.01,            # smooth D over about 10 ms
                 d_on_measurement=True)        # no kick when the target jumps

target = lambda t: 1.0 if t < 5.0 else 0.5     # 0 -> 1 m at t = 0, then 1 -> 0.5 m at t = 5 s

# ---- 2. Run the closed loop ----
log = simulate(MassSpringDamper(M, C, K), controller, target, t_end=8.0, noise_std=0.001, seed=1)

# Same situation, naive PID: same gains, same 30 N motor, but no anti-windup / filter / D fix.
naive = PID(**FINAL, dt=0.001, u_min=-30.0, u_max=30.0)
log_n = simulate(MassSpringDamper(M, C, K), naive, target, t_end=8.0, noise_std=0.001, seed=1)


def two_steps(lg):
    """Judge both target changes with the Section 4 measures."""
    t, y = lg["t"], lg["y"]
    first = t < 5.0
    m1 = step_metrics(t[first], y[first], setpoint=1.0)
    # Second step goes DOWN from 1.0 to 0.5: flip it so it looks like a 0 -> 1 step.
    sec = t >= 5.0
    ys = (y[sec] - 1.0) / (0.5 - 1.0)
    m2 = step_metrics(t[sec] - 5.0, ys, setpoint=1.0)
    m2["ss_error"] *= -0.5                       # back to metres
    steady = (t > 4.0) & (t < 5.0)
    return m1, m2, float(lg["u"][steady].std())


m1, m2, ustd = two_steps(log)
n1, n2, nstd = two_steps(log_n)
print("  Fixed PID (all Section 7 fixes on):")
print(f"    step 0 -> 1 m  : {fmt_metrics(m1)}")
print(f"    step 1 -> 0.5 m: {fmt_metrics(m2)}")
print(f"    force jitter at rest (4-5 s): std {ustd:.2f} N")
print("  Naive PID (same gains, same 30 N motor, no fixes):")
print(f"    step 0 -> 1 m  : {fmt_metrics(n1)}")
print(f"    step 1 -> 0.5 m: {fmt_metrics(n2)}")
print(f"    force jitter at rest (4-5 s): std {nstd:.2f} N")

# A short printed log, so you can see what the controller is doing step by step.
print("\n     t [s]   target   measured   error     P [N]     I [N]     D [N]     u [N]")
for tq in [0.0, 0.1, 0.2, 0.5, 1.0, 2.0, 4.0, 5.0, 5.1, 6.0, 8.0]:
    n = int(round(tq / 0.001))
    e = log["r"][n] - log["y_meas"][n]
    print(f"    {tq:6.3f} {log['r'][n]:8.2f} {log['y_meas'][n]:10.4f} {e:8.4f} "
          f"{log['p'][n]:9.2f} {log['i'][n]:9.2f} {log['d'][n]:9.2f} {log['u'][n]:9.2f}")

# ---- 3. Plot ----
fig, axs = plt.subplots(3, 1, figsize=(7.2, 6.4), sharex=True)
axs[0].plot(log_n["t"], log_n["y"], color=COLORS[1], lw=1.4, label="naive PID (no fixes)")
axs[0].plot(log["t"], log["y"], color=COLORS[0], lw=1.8, label="PID with all fixes")
axs[0].plot(log["t"], log["r"], label="target", **TARGET_STYLE)
axs[0].set(title="Full PID on the test system: 30 N motor, noisy sensor, target jump at 5 s",
           ylabel="position x [m]", ylim=(0, 1.3))
axs[0].legend(loc="upper right", fontsize=7, ncol=2)

axs[1].plot(log_n["t"], log_n["u"], color=COLORS[1], lw=0.6, label="naive PID force")
axs[1].plot(log["t"], log["u"], color=COLORS[0], lw=1.2, label="fixed PID force")
axs[1].axhline(30, color=COLORS[7], ls="--", lw=1, label="motor limit ±30 N")
axs[1].axhline(-30, color=COLORS[7], ls="--", lw=1)
axs[1].set(ylabel="force u [N]", ylim=(-38, 45))
axs[1].legend(loc="upper right", fontsize=7, ncol=3)

axs[2].plot(log["t"], log["p"], color=COLORS[0], lw=1.2, label="P part")
axs[2].plot(log["t"], log["i"], color=COLORS[2], lw=1.8, label="I part")
axs[2].plot(log["t"], log["d"], color=COLORS[6], lw=1.0, label="D part (filtered)")
axs[2].set(xlabel="time t [s]", ylabel="force part [N]", xlim=(0, 8), ylim=(-165, 330))
axs[2].legend(loc="upper right", fontsize=7, ncol=3)
finish(fig, "fig09_full_demo",
       f"With all fixes: overshoot {m1['overshoot_pct']:.1f} % / {m2['overshoot_pct']:.1f} % on the two "
       f"steps vs {n1['overshoot_pct']:.1f} % / {n2['overshoot_pct']:.1f} % naive; force jitter "
       f"{ustd:.1f} N vs {nstd:.1f} N.")
save_results("section09", dict(fixed=dict(step1=m1, step2=m2, force_std=ustd),
                               naive=dict(step1=n1, step2=n2, force_std=nstd)))
