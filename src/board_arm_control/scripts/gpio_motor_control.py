"""
Source code adapted from:
https://github.com/curiores/ArduinoTutorials/tree/main/encoderControl/part4_NoAtomic
which was our reference for PID motor control on the Arduino.
"""

import Adafruit_BBIO.GPIO as gpio
import Adafruit_BBIO.PWM as pwm
from time import sleep
import rospy
import numpy as np
from enum import Enum
from board_arm_control.msg import JointConfiguration

# Encoder input pins
ENC2A = "P8_8"
ENC2B = "P8_10"
ENC1A = "P8_12"
ENC1B = "P8_14"

# Enable pins used to control speed with PWM
ENA2 = "P9_14"
ENA1 = "P8_19"

# Directional pins used to determine rotation direction
IN2A = "P8_7"
IN2B = "P8_9"
IN1A = "P8_15"
IN1B = "P8_17"

pos = np.array([0.0, 0.0])

class Direction(Enum):
    CW = 1
    CCW = 2


def read_encoder_1B(pin):
    global pos
    val = gpio.input(ENC1B)
    if val == gpio.HIGH:
        pos[0] += 1.0
    elif val == gpio.LOW:
        pos[0] -= 1.0

def read_encoder_2B(pin):
    global pos
    val = gpio.input(ENC2B)
    if val == gpio.HIGH:
        pos[1] += 1.0
    elif val == gpio.LOW:
        pos[1] -= 1.0


def set_motor(direction, power, motor):
    enable_pin = ""
    in_pin_A = ""
    in_pin_B = ""

    if motor == 1:
        enable_pin = ENA1
        in_pin_A = IN1A
        in_pin_B = IN1B
    elif motor == 2:
        enable_pin = ENA2
        in_pin_A = IN2A
        in_pin_B = IN2B

    if direction == Direction.CW:
        gpio.output(in_pin_A, gpio.HIGH)
        gpio.output(in_pin_B, gpio.LOW)
    elif direction == Direction.CCW:
        gpio.output(in_pin_A, gpio.LOW)
        gpio.output(in_pin_B, gpio.HIGH)
    pwm.start(enable_pin, power)


def pid_controller(data):
    e_prev = np.zeros(2)
    t_prev = monotonic()
    e_integral = np.zeros(2)
    target = np.array([data.configuration[0], data.configuration[1]])
    kp = 1.0
    ki = 0.001
    kd = 0.01
    global pos
    print(target)

    """
    while np.linalg.norm(e_prev) > 0.01:
        t_curr = monotonic()
        t_delta = (t_curr - t_prev)
        t_prev = t_curr

        e = pos - target
        de_dt = (e - e_prev) / t_delta
        e_integral = e_integral + e * t_delta
        u = kp * e + kd * de_dt + ki * e_integral
        e_prev = e

        pwm = min(abs(u), 100)
        direction = (u < 0) ? CW : CCW
        setMotor(direction, pwm)
    """


def init():
    gpio.setup(ENC1A, gpio.IN)
    gpio.setup(ENC1B, gpio.IN)
    gpio.setup(ENC2A, gpio.IN)
    gpio.setup(ENC2B, gpio.IN)

    gpio.setup(IN1A, gpio.OUT)
    gpio.setup(IN1B, gpio.OUT)
    gpio.setup(IN2A, gpio.OUT)
    gpio.setup(IN2B, gpio.OUT)

    gpio.add_event_detect(ENC1A, gpio.RISING, read_encoder_1B)
    gpio.add_event_detect(ENC2A, gpio.RISING, read_encoder_2B)

def main():
    """
    rospy.init_node("gpio_controller")
    rospy.Subscriber("joint_configuration", JointConfiguration, pid_controller)
    rospy.spin()
    """
    gpio.output(IN1B, gpio.HIGH)
    gpio.output(IN1A, gpio.LOW)
    pwm.start(ENA1, 60)
    #gpio.output(IN2A, gpio.LOW)
    #gpio.output(IN2B, gpio.HIGH)
    #pwm.start(ENA2, 100)

    sleep(1)
    pwm.start(ENA1, 30)
    sleep(1)
    pwm.stop(ENA1)
    #pwm.stop(ENA2)


if __name__ == "__main__":
    init()
    main()
    pwm.cleanup()
    gpio.cleanup()
