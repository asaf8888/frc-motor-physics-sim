import numpy as np
class Encoder:
    def __init__(self, motor, backlash_in_rads, measuring_frequency, noise_std):
        self.motor = motor
        motor.add_encoder(self)
        self.backlash_in_rads = backlash_in_rads
        self.measuring_frequency = measuring_frequency
        self.noise_std = noise_std
        self.last_measurement = 0
        self.position_in_rads = 0
        self.velocity_in_rads_per_sec = 0
        self.acceleration = 0

    def get_measurement_period(self):
        return 1/self.measuring_frequency

    def update_pos(self, new_pos, measurement_time):
        noisy_pos = new_pos + np.random.normal(scale=self.noise_std)
        backlashed_pos = noisy_pos if not self.backlash_in_rads else noisy_pos - (noisy_pos % self.backlash_in_rads)
        new_vel = (backlashed_pos - self.position_in_rads) / (measurement_time - self.last_measurement)
        self.acceleration = (new_vel - self.velocity_in_rads_per_sec) / (measurement_time - self.last_measurement)
        self.velocity_in_rads_per_sec = new_vel
        self.position_in_rads = backlashed_pos
        self.last_measurement = measurement_time

    def get_state(self):
        return {"position": self.position_in_rads,
                "velocity": self.velocity_in_rads_per_sec,
                "acceleration": self.acceleration}
