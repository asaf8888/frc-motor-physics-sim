class UErrorCompensator:
    def __init__(self, Ku, state_to_presumed_voltage):
        self.Ku = Ku
        self.u_error_accum = 0
        self.state_to_presumed_voltage = state_to_presumed_voltage
        self.last_calc_t = 0

    def calc(self, state, last_voltage, sim_time):
        dt = sim_time - self.last_calc_t
        self.last_calc_t = sim_time
        presumed_voltage = self.state_to_presumed_voltage(state)
        last_voltage_ignoring_compensation = last_voltage - self.u_error_accum
        self.u_error_accum += self.Ku * (last_voltage_ignoring_compensation - presumed_voltage) * dt
        return self.u_error_accum
