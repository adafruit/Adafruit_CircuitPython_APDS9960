# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import board
from digitalio import DigitalInOut, Pull
from adafruit_apds9960.apds9960 import APDS9960

i2c = board.I2C()

# Replace "D5" with the GPIO connected to the sensor's interrupt pin
int_pin = DigitalInOut(board.D5)
int_pin.switch_to_input(pull=Pull.UP)
apds = APDS9960(i2c)

# LED settings can be tuned to quickly increase/decrease range and sensitivity
# This configures the IR LED for 8 pulses, 8us each, 100mA @ x1 boost
apds.proximity_led_config = (7, 1, 0, 0)

# Interrupt thresholds can be tuned to decide when the interrupt pin gets asserted
# This configures the interrupt with 0 far threshold, 175 near threshold, 5 cycle persistence
apds.proximity_interrupt_threshold = (0, 175, 5)

# This enables assertion of the interrupt pin (pulling it to ground) when the thresholds are met
apds.enable_proximity_interrupt = True

# This starts running continously running proximity engine loops on the sensor
apds.enable_proximity = True

while True:
    # Print the proximity reading when the interrupt pin goes low
    if not int_pin.value:
        print(apds.proximity)

        # clear the interrupt
        apds.clear_interrupt()
