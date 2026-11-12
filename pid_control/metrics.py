"""
metrics.py - The four measures used to judge a step response (Section 4).

    Rise time        : time to go from 10% to 90% of the final value      [s]
    Overshoot        : how far the peak goes past the final value          [%]
    Settling time    : last time the output is outside +/-2% of final value [s]
    Steady-state err : target minus final value                            [m]

"Final value" = average of the last 5% of the run (so a tiny ripple does not fool us).
Rise time, overshoot and settling are measured relative to the FINAL value (the standard
textbook convention). Steady-state error is measured relative to the TARGET.
"""

import numpy as np


def step_metrics(t, y, setpoint=1.0, band=0.02):
    n_tail = max(1, len(y) // 20)
    y_final = float(np.mean(y[-n_tail:]))

    # Rise time: first time we cross 10% and first time we cross 90% of the final value.
    i10 = np.argmax(y >= 0.1 * y_final)
    i90 = np.argmax(y >= 0.9 * y_final)
    rise_time = t[i90] - t[i10]

    # Overshoot: peak above the final value, as a percentage of the final value.
    overshoot = max(0.0, (np.max(y) - y_final) / abs(y_final) * 100.0)

    # Settling time: the last moment the response is outside the +/-2% band.
    outside = np.where(np.abs(y - y_final) > band * abs(y_final))[0]
    settling_time = t[outside[-1] + 1] if len(outside) and outside[-1] + 1 < len(t) else (
        t[-1] if len(outside) else 0.0)

    sse = setpoint - y_final
    return {"rise_time": rise_time, "overshoot_pct": overshoot,
            "settling_time": settling_time, "ss_error": sse, "final": y_final,
            "peak": float(np.max(y)), "t_peak": float(t[np.argmax(y)])}
