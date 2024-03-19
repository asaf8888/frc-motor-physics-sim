import math
import numpy as np
class BLDCMotor:
    def __init__(self, Ks=0, Kv=0, Ka=0):
        self.Ks = Ks
        self.Kv = Kv
        self.Ka = Ka
        self.position_in_rads = 0
        self.velocity_in_rads_per_sec = 0
        self.t = 0
        self.encoders = []

    def step(self, voltage, sim_time):
        dt = sim_time - self.t
        for encoder in self.encoders:
            if sim_time > encoder.last_measurement + encoder.get_measurement_period():
                (pos, vel) = self.calculate_motor_state(voltage,  dt - (sim_time % encoder.get_measurement_period()))
                encoder.update_pos(pos, sim_time - sim_time % encoder.get_measurement_period())
        (pos, vel) = self.calculate_motor_state(voltage, dt)
        self.position_in_rads = pos
        self.velocity_in_rads_per_sec = vel
        self.t = sim_time

    def calculate_motor_state(self, voltage, dt):
        v0 = self.velocity_in_rads_per_sec
        if v0 == 0:
            voltage_post_friction = max((abs(voltage) - self.Ks), 0) * np.sign(voltage)
        else:
            voltage_post_friction = voltage - self.Ks * np.sign(v0)
        Kv = self.Kv
        Ka = self.Ka
        log_inside = -voltage_post_friction / (-voltage_post_friction + (Kv * v0)) if voltage_post_friction else 0
        if log_inside > 0:
            t_0_vel = -(Ka / Kv) * math.log(log_inside)
            if dt > t_0_vel > 0:
                v0 = 0
                dt = dt - t_0_vel
                voltage_post_friction = max((abs(voltage) - self.Ks), 0) * np.sign(voltage)
        velocity_in_rads_per_sec = (-(voltage_post_friction/Kv) + v0) * math.exp(-(Kv/Ka)*dt) + voltage_post_friction/Kv
        position_in_rads = self.position_in_rads + ((voltage_post_friction * Ka/(Kv**2) - (Ka/Kv * v0)) * math.exp(-(Kv/Ka)*dt) + (voltage_post_friction/Kv)*dt - (voltage_post_friction * Ka/(Kv**2) - (Ka/Kv * v0)))
        return position_in_rads, velocity_in_rads_per_sec

    def add_encoder(self, encoder):
        self.encoders.append(encoder)
    def get_position_in_rads(self):
        return self.position_in_rads % (2*math.pi)

expo = BLDCMotor(4.35, 4.4, 4.4)
expo.velocity_in_rads_per_sec = 0.4
expo.step(6.06, 5)

