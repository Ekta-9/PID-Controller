"""
plant.py - The machines we control ("plants") and the closed-loop simulator.

We write our own numerical integrator (Runge-Kutta 4, "RK4") instead of using a library.
RK4 estimates where the system will be after a small time step by checking the slope
4 times inside that step and taking a weighted average. It is far more accurate than
simply doing  new = old + slope * dt  (Euler), for the same step size.
"""

import numpy as np


class Plant:
    """Base class: any system described by  d(state)/dt = f(state, u)."""

    def __init__(self, n_states):
        self.state = np.zeros(n_states)

    def derivatives(self, state, u):
        raise NotImplementedError

    def output(self):
        raise NotImplementedError

    def step(self, u, dt):
        """Move the system forward by dt seconds with the force u held constant (RK4)."""
        s = self.state
        k1 = self.derivatives(s, u)
        k2 = self.derivatives(s + 0.5 * dt * k1, u)
        k3 = self.derivatives(s + 0.5 * dt * k2, u)
        k4 = self.derivatives(s + dt * k3, u)
        self.state = s + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)


class MassSpringDamper(Plant):
    """
    Section 3 test system:   m * x'' + c * x' + k * x = u
        m = 1 kg (mass), c = 10 N*s/m (damper), k = 20 N/m (spring), u = motor force [N]
    State = [x, v] = [position (m), velocity (m/s)].
    """

    def __init__(self, m=1.0, c=10.0, k=20.0):
        super().__init__(2)
        self.m, self.c, self.k = m, c, k

    def derivatives(self, state, u):
        x, v = state
        # Newton's 2nd law: acceleration = (sum of forces) / mass
        a = (u - self.c * v - self.k * x) / self.m
        return np.array([v, a])     # d(x)/dt = v,  d(v)/dt = a

    def output(self):
        return self.state[0]        # the sensor measures position x


class ThirdOrderLag(Plant):
    """
    Section 8 system:  G(s) = 1 / (s + 1)^3
    Think of three identical "slow buckets" in a row: each one smoothly follows the one
    before it with a 1-second delay-like lag.  x1' = -x1 + u,  x2' = -x2 + x1,  x3' = -x3 + x2
    Output y = x3.
    """

    def __init__(self):
        super().__init__(3)

    def derivatives(self, state, u):
        x1, x2, x3 = state
        return np.array([-x1 + u, -x2 + x1, -x3 + x2])

    def output(self):
        return self.state[2]


def simulate(plant, controller, setpoint, t_end, Ts=0.001, dt_phys=0.001,
             noise_std=0.0, seed=0):
    """
    Run the closed loop:  target -> controller -> plant -> sensor -> back to controller.

    plant      : a Plant object (its state is reset to zero first)
    controller : a PID object (reset first). Its dt must equal Ts.
    setpoint   : a number, or a function f(t) giving the target at time t
    t_end      : total simulated time [s]
    Ts         : how often the controller updates [s]  (the "computer" speed)
    dt_phys    : small step used to simulate the real physics [s]
    noise_std  : standard deviation of sensor noise [same unit as the output]
    seed       : fixes the random numbers so every run gives identical graphs

    Returns a dict of NumPy arrays sampled every dt_phys:
        t, y (true output), y_meas (what the sensor said), u (force), r (target), p, i, d
    """
    rng = np.random.default_rng(seed)
    plant.state = np.zeros_like(plant.state)
    controller.reset()
    sp = setpoint if callable(setpoint) else (lambda t, v=setpoint: v)

    n_sub = max(1, int(round(Ts / dt_phys)))   # physics steps per controller update
    n_steps = int(round(t_end / dt_phys))
    log = {key: np.zeros(n_steps + 1) for key in ("t", "y", "y_meas", "u", "r", "p", "i", "d")}

    u = 0.0
    y_meas = plant.output()
    for n in range(n_steps + 1):
        t = n * dt_phys
        r = sp(t)
        # The controller only "wakes up" every n_sub physics steps (zero-order hold between).
        if n % n_sub == 0:
            y_meas = plant.output() + noise_std * rng.standard_normal()
            u = controller.update(r, y_meas)
        log["t"][n], log["y"][n], log["y_meas"][n] = t, plant.output(), y_meas
        log["u"][n], log["r"][n] = u, r
        log["p"][n], log["i"][n], log["d"][n] = controller.last_p, controller.last_i, controller.last_d
        if n < n_steps:
            plant.step(u, dt_phys)
    return log
