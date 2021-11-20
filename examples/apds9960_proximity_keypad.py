# SPDX-FileCopyrightText: 2021 fivesixzero
# SPDX-License-Identifier: MIT
#
# Proximity 'keypad key' Demo
#
# The APDS-9960's proximity function can be used like any other button or keyswitch input!
#
# In this example we'll be using the 'keypad' library, which is useful for cases where your inputs
#   need to be responsive even when your main loop has unpredictable timing.
#
# Adding additional buttons or keyswitches is as easy as adding them to the 'keys' tuple and making
#   sure they're accounted for in the event handling function.
#
# Once the proximity engine is enabled it'll keep running on its own. The interrupt pin will be
#   asserted (pulled to ground, or 'False'), only when the proximity stays within the 'window'
#   established by the high/low thresholds for enough of its internal measurement cycles, defined
#   by the 'persistence' variable.

import time
import board
import keypad
from adafruit_apds9960.apds9960 import APDS9960

i2c = board.I2C()
apds = APDS9960(i2c)

# Interrupt will trigger only if proximity value is >50 for at least 2 consecutive cycles
apds.proximity_interrupt_threshold = (0, 50, 1)
apds.enable_proximity = True
apds.enable_proximity_interrupt = True

# Clue: https://www.adafruit.com/product/4500
#
# The two buttons available on the Clue are easy to include for scanning with the keypad
#
keys = (board.PROXIMITY_LIGHT_INTERRUPT, board.BUTTON_A, board.BUTTON_B)

# Bluefruit Sense: https://www.adafruit.com/product/4516
# keys = (
#     board.PROXIMITY_LIGHT_INTERRUPT,
# )

# Breakout board sensor: https://www.adafruit.com/product/3595
#
# The breakout's "INT" pin will need to be attached to a GPIO pin on your device
#
# keys = (
#     board.D5,
# )

# Proximity Trinkey: https://www.adafruit.com/product/5022
#
#   The Proximity Trinky firmware doesn't include the 'keypad' module by default so this demo will
#     require a custom firmware build with the 'keypad' module manually included.
#
#   Note: Getting the ~3.6k module to fit in the Trinkey's flash storage will likely require
#     removing other modules, such as usb_midi (~1.9k) and/or usb_hid (~2k).
#
#   Details on building customized firmware can be found in the Building CircuitPython guide:
#   https://learn.adafruit.com/building-circuitpython
#
# keys = (
#     board.INTERRUPT,
# )

prox_keypad = keypad.Keys(keys, value_when_pressed=False, pull=True)

# Keypad constants/buffer to save memory
PROX_PRESS = keypad.Event(0, True)
PROX_RELEASE = keypad.Event(0, False)
KEY_BUFFER = keypad.Event()

# Simple function to see if we had a press or release come in since our last check-in
def handle_key_events(keys_to_check: keypad.Keys, currently_held: bool):
    new_press = False
    new_release = False

    # Get new events and quickly compare them against our buffers
    while keys_to_check.events.get_into(KEY_BUFFER):
        if KEY_BUFFER == PROX_PRESS:
            new_press = True
            currently_held = True
        if KEY_BUFFER == PROX_RELEASE:
            new_release = True
            currently_held = False

    # We should clear the proximity interrupt every time we check on it
    apds.clear_proximity_interrupt()

    return new_press, new_release, currently_held


held = False
while True:
    pressed, released, held = handle_key_events(prox_keypad, held)

    if pressed:
        print("A thing got pretty close!")
    elif released:
        print("That thing moved away!")
    elif held:
        print("That thing is still there!")
    else:
        print("Nothing nearby...")

    time.sleep(1)
