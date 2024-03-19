class PIDController:
    def __init__(self, p=0, i=0, d=0, i_zone=0):
        self.p = p
        self.i = i
        self.d = d
        self.i_zone = i_zone
        self.last_error = 0
        self.last_calc_t = 0
        self.i_accum = 0
        self.setpoint = 0

    def set_setpoint(self, setpoint, sim_time=None):
        if sim_time:
            self.last_calc_t = sim_time
        self.setpoint = setpoint

    def calc(self, state, sim_time, setpoint=None):
        if setpoint:
            self.setpoint = setpoint
        error = self.setpoint - state["position"]
        dt = (sim_time - self.last_calc_t)
        p_part = self.p * error
        if abs(error) < self.i_zone > 0:
            self.i_accum += self.i * error * dt
        if sim_time > 0:
            d_part = self.d * (error - self.last_error) / dt
        else:
            d_part = 0
        self.last_calc_t = sim_time
        self.last_error = error
        return p_part + self.i_accum + d_part