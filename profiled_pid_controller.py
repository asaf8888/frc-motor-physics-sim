import math

import numpy as np
class ProfiledPIDController:
    def __init__(self, pid_controller, max_voltage, max_velocity, max_acceleration):
        self.profile = lambda x:(0,0,0)
        self.controller = pid_controller
        self.max_voltage = max_voltage
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.target = 0

    def set_target(self, state, target, sim_time):
        self.target = target
        self.profile = self.create_profile(state["position"], state["velocity"], sim_time)

    def create_profile(self, x0, v0, sim_time):
        target = self.target - (x0 % (2*math.pi))
        sign = np.sign(target)
        target = abs(target)
        v0 = abs(v0)
        t_max = max(target/2 - v0 / self.max_acceleration, 0)
        max_accelable_velocity = v0 + t_max * self.max_acceleration
        print(sim_time, t_max)
        if t_max == 0:
            def profile(t):
                vel = v0 - self.max_acceleration * t
                pos = x0 + v0 * t - self.max_acceleration * t * t/2
                return pos, vel, 0
        elif v0 + t_max * self.max_acceleration > self.max_velocity:
            t1 = t_max - (max_accelable_velocity - self.max_velocity) / self.max_acceleration
            area = (t_max - t1) * (max_accelable_velocity - self.max_velocity) / 2
            t2_unpushed = t_max + (max_accelable_velocity - self.max_velocity) / self.max_acceleration
            t2 = t2_unpushed + area / self.max_velocity
            print(sim_time, t2)
            def profile(t):
                if t < t1:
                    vel = v0 + self.max_acceleration * t
                    pos = x0 + vel * t / 2
                    return pos, vel, sign
                if t1 < t < t2:
                    vel = self.max_velocity
                    pos = x0 + vel * t1 / 2 + vel * t
                    return pos, vel, sign
                if t > t2:
                    vel = self.max_velocity - self.max_acceleration * (t - t2)
                    pos = x0 + target - vel * self.max_velocity / self.max_acceleration / 2
                    return pos, vel, 0

        else:
            def profile(t):
                if t < t_max:
                    vel = v0 + self.max_acceleration * t
                    pos = x0 + vel * t / 2
                    return pos, vel, sign
                else:
                    vel = max_accelable_velocity - self.max_acceleration * (t - t_max)
                    pos = x0 + self.target - vel * (max_accelable_velocity / self.max_acceleration / 2 + t_max - t)
                    return pos, vel, 0

        return lambda t: profile(t-sim_time)

    def calc(self, state, sim_time, target=None):
        if target:
            self.set_target(state, target, sim_time)
        pos, vel, volt = self.profile(sim_time)
        volt *= self.max_voltage
        return volt + self.controller.calc(state, sim_time, setpoint=pos)

