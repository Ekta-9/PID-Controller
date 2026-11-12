"""
Section 3 - The test system: 1 kg block, spring k = 20 N/m, damper c = 10 N*s/m.
No controller yet: we just push with a CONSTANT force and watch where the block ends up.
Hand calculation: at rest x'' = x' = 0, so  k*x = u  ->  x_rest = u / k.
    u = 10 N -> 0.5 m,   u = 20 N -> 1.0 m,   u = 30 N -> 1.5 m
Run:  python -m experiments.section03_test_system   ->  figures/fig03_test_system.png
"""

import numpy as np
import matplotlib.pyplot as plt
from pid_control.plant import MassSpringDamper, simulate
from pid_control.metrics import step_metrics
from pid_control.common import M, C, K, COLORS, TARGET_STYLE, finish, save_results


class ConstantPush:
    """A 'controller' that ignores the sensor and always pushes with the same force."""
    def __init__(self, force):
        self.force = force
        self.reset()

    def reset(self):
        self.last_p = self.last_i = self.last_d = 0.0

    def update(self, setpoint, measurement):
        return self.force


fig, (a1, a2) = plt.subplots(1, 2, figsize=(7.2, 2.9))
results = {}
for j, force in enumerate([10, 20, 30]):
    log = simulate(MassSpringDamper(M, C, K), ConstantPush(force), 1.0, t_end=6.0)
    m = step_metrics(log["t"], log["y"])
    results[f"{force}N"] = dict(theory=force / K, sim=m["final"], settle_2pct=m["settling_time"])
    a1.plot(log["t"], log["y"], color=COLORS[j],
            label=f"{force} N → {m['final']:.3f} m")
    if force == 20:
        log20 = log
a1.axhline(1.0, label="target 1 m", **TARGET_STYLE)
a1.set(title="Constant push, no controller", xlabel="time t [s]", ylabel="position x [m]",
       xlim=(0, 3), ylim=(0, 1.65))
a1.legend(loc="lower right", ncol=2, fontsize=7.5)

# Forces acting on the block during the 20 N run
x, v = log20["y"], np.gradient(log20["y"], log20["t"])
a2.plot(log20["t"], log20["u"], color=COLORS[0], label="motor push u = 20 N")
a2.plot(log20["t"], K * x, color=COLORS[1], label="spring pulls back: k·x")
a2.plot(log20["t"], C * v, color=COLORS[2], label="damper resists: c·v")
a2.set(title="Forces during the 20 N run", xlabel="time t [s]", ylabel="force [N]",
       xlim=(0, 3), ylim=(-1, 24))
a2.legend(loc="center right")

finish(fig, "fig03_test_system",
       "A steady 20 N push holds the block at exactly 1.000 m, because the spring then pulls back with k·x = 20 N.")
save_results("section03", results)
for kname, r in results.items():
    print(f"  {kname}: theory {r['theory']:.3f} m, simulated {r['sim']:.4f} m, settles in {r['settle_2pct']:.2f} s")
