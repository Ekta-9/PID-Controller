"""
pid.py - A reusable PID controller written from scratch (NumPy only, no control libraries).

PLAIN-ENGLISH SUMMARY
    A PID controller looks at the ERROR (target minus measured value) and decides
    how hard to push. It adds three parts:
        P = Kp * error                      -> reacts to the error NOW
        I = Ki * (sum of error * dt)        -> reacts to error that has built up in the PAST
        D = Kd * (rate of change of error)  -> reacts to where the error is GOING

    This class also contains the four "real-world fixes" from Section 7:
        1. Output limit        (a motor cannot push harder than its maximum, e.g. 30 N)
        2. Anti-windup         (stop the I part from growing while the motor is maxed out)
        3. Derivative filter   (smooth the D part so sensor noise does not make it jumpy)
        4. Derivative on measurement (avoid the huge "kick" when the target jumps)

    Every fix can be switched on or off, so the same class is used for every graph.
"""

import numpy as np


class PID:
    def __init__(self, kp, ki=0.0, kd=0.0, dt=0.001,
                 u_min=-np.inf, u_max=np.inf,
                 anti_windup=False,
                 d_filter_tau=0.0,
                 d_on_measurement=False):
        # --- Gains: how strongly each part reacts ---
        self.kp = kp            # proportional gain   [N per m of error]
        self.ki = ki            # integral gain       [N per (m*s) of accumulated error]
        self.kd = kd            # derivative gain     [N per (m/s) of error change]
        self.dt = dt            # time between controller updates [s]

        # --- Fix 1: output limit (the motor's maximum push, in N) ---
        self.u_min = u_min
        self.u_max = u_max

        # --- Fix 2: anti-windup on/off ---
        self.anti_windup = anti_windup

        # --- Fix 3: derivative low-pass filter time constant [s]; 0 means "no filter" ---
        self.d_filter_tau = d_filter_tau

        # --- Fix 4: take the derivative of the measurement instead of the error ---
        self.d_on_measurement = d_on_measurement

        self.reset()

    def reset(self):
        """Forget everything from previous runs (memory of the past goes back to zero)."""
        self.integral = 0.0     # running sum of error * dt  [m*s]
        self.prev_error = None  # error at the previous update (None = first update)
        self.prev_meas = None   # measurement at the previous update
        self.d_filtered = 0.0   # smoothed derivative value
        # Store the last P, I, D contributions so we can plot them later.
        self.last_p = self.last_i = self.last_d = 0.0

    def update(self, setpoint, measurement):
        """
        Run ONE controller step. Call this once every dt seconds.
        Returns the force u [N] to send to the motor.
        """
        dt = self.dt

        # Step 1: how far are we from the target right now?
        error = setpoint - measurement

        # Step 2: P part - push in proportion to the present error.
        p_term = self.kp * error

        # Step 3: D part - how fast is the error changing?
        if self.prev_error is None:
            # First step: there is no "previous" value yet, so we cannot measure a rate.
            d_raw = 0.0
        elif self.d_on_measurement:
            # Fix 4: use minus the change in MEASUREMENT. When the setpoint is constant this
            # is exactly the same as the error rate, but a setpoint jump causes no spike.
            d_raw = -(measurement - self.prev_meas) / dt
        else:
            # Textbook version: change in error divided by time step.
            d_raw = (error - self.prev_error) / dt

        # Fix 3: first-order low-pass filter. alpha close to 1 = heavy smoothing.
        # new = alpha * old + (1 - alpha) * raw   (an "exponential moving average")
        if self.d_filter_tau > 0:
            alpha = self.d_filter_tau / (self.d_filter_tau + dt)
            self.d_filtered = alpha * self.d_filtered + (1 - alpha) * d_raw
        else:
            self.d_filtered = d_raw
        d_term = self.kd * self.d_filtered

        # Step 4: I part - add this step's error to the running sum (rectangle rule).
        new_integral = self.integral + error * dt
        i_term_candidate = self.ki * new_integral

        # Step 5: add the three parts, then clip to what the motor can really do (Fix 1).
        u_unclipped = p_term + i_term_candidate + d_term
        u = float(np.clip(u_unclipped, self.u_min, self.u_max))

        # Step 6 (Fix 2: anti-windup by "conditional integration" / clamping):
        # If the motor is already maxed out AND the error would push it even further
        # in the same direction, do NOT grow the integral. Otherwise, accept it.
        saturated = (u != u_unclipped)
        pushing_further = np.sign(error) == np.sign(u_unclipped)
        if self.anti_windup and saturated and pushing_further:
            pass                            # freeze the integral
        else:
            self.integral = new_integral    # normal case: keep the new sum

        # Step 7: remember values for next time and for plotting.
        self.prev_error = error
        self.prev_meas = measurement
        self.last_p = p_term
        self.last_i = self.ki * self.integral
        self.last_d = d_term
        return u
