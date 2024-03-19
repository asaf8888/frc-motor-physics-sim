import math
import time

import pygame

from PIDController import PIDController
from u_error_compensator import UErrorCompensator
from profiled_pid_controller import ProfiledPIDController
from encoder import Encoder
from motor import BLDCMotor

pygame.init()

SCREEN_WIDTH = 500
SCREEN_HEIGHT = 500
dt = 0.001
LINE_LENGTH = 20
ITERATION_TIME = 0.001


def draw_position(color, position_in_rads, length):
    pygame.draw.line(screen, color, (position_in_rads / 2 / math.pi * SCREEN_WIDTH, SCREEN_HEIGHT / 2 - length / 2),
                     (position_in_rads / 2 / math.pi * SCREEN_WIDTH, SCREEN_HEIGHT / 2 + length / 2))


screen = pygame.display.set_mode([SCREEN_WIDTH, SCREEN_HEIGHT])

running = True
x0 = math.pi
motor = BLDCMotor(1, 0.5, 0.5)
encoder = Encoder(motor, 0.0, 50, 0.0)
pid_controller = PIDController(0, 0.0, 0, 0.0)
profiled_pid_controller = ProfiledPIDController(pid_controller, 8, 8, 6)
motor.position_in_rads = x0
pid_controller.set_setpoint(2.3, 0)

voltage = 0
sim_time = 0
last_time = time.time()
while running:


    for event in pygame.event.get():

        if event.type == pygame.QUIT:
            running = False

    sim_time += dt

    keys = pygame.key.get_pressed()
    mouse_pos = pygame.mouse.get_pos()
    mouse_x_in_rads = mouse_pos[0] / SCREEN_WIDTH * 2 * math.pi
    if not mouse_x_in_rads == profiled_pid_controller.target:
        profiled_pid_controller.set_target(encoder.get_state(), mouse_x_in_rads, sim_time)
    voltage = profiled_pid_controller.calc(encoder.get_state(), sim_time)

    motor.step(voltage, sim_time)
    # Fill the background with white

    screen.fill((255, 255, 255))

    # Draw a solid blue circle in the center
    motor_rads = motor.get_position_in_rads()

    pygame.draw.line(screen, (0, 0, 255), (0, SCREEN_HEIGHT / 2), (SCREEN_WIDTH, SCREEN_HEIGHT / 2))
    draw_position((255, 0, 0), motor_rads, LINE_LENGTH)
    draw_position((0, 255, 0), profiled_pid_controller.target, LINE_LENGTH)

    pygame.display.flip()
    curr_time = time.time()
    time_diff = curr_time - last_time
    if time_diff < ITERATION_TIME:
        time.sleep(ITERATION_TIME - time_diff)
    last_time = time.time()

pygame.quit()
