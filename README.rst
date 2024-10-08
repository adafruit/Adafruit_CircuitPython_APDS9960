
Introduction
============

.. image:: https://readthedocs.org/projects/adafruit-circuitpython-apds9960/badge/?version=latest
    :target: https://docs.circuitpython.org/projects/apds9960/en/latest/
    :alt: Documentation Status

.. image:: https://raw.githubusercontent.com/adafruit/Adafruit_CircuitPython_Bundle/main/badges/adafruit_discord.svg
    :target: https://adafru.it/discord
    :alt: Discord

.. image:: https://github.com/adafruit/Adafruit_CircuitPython_APDS9960/workflows/Build%20CI/badge.svg
    :target: https://github.com/adafruit/Adafruit_CircuitPython_APDS9960/actions/
    :alt: Build Status

.. image:: https://img.shields.io/badge/code%20style-black-000000.svg
    :target: https://github.com/psf/black
    :alt: Code Style: Black

The APDS-9960 is a specialized chip that detects hand gestures, proximity
and ambient light color over I2C. Its available from
`Adafruit as a breakout <https://www.adafruit.com/product/3595>`_ and as a built-in sensor on
several Adafruit development boards.

* `Adafruit CLUE <https://www.adafruit.com/product/4500>`_
* `Adafruit Feather nRF52840 Sense <https://www.adafruit.com/product/4516>`_
* `Adafruit Proximity Trinkey <https://www.adafruit.com/product/5022>`_

This driver provides easy access to proximity, gesture and color data from the APDS-9960 sensor
with a minimal footprint to allow it to work on all CircuitPython platforms.

Installation and Dependencies
=============================
This driver depends on:

* `Adafruit CircuitPython <https://github.com/adafruit/circuitpython>`_

Please ensure all dependencies are available on the CircuitPython filesystem.

This is easily achieved by downloading
`the Adafruit library and driver bundle <https://github.com/adafruit/Adafruit_CircuitPython_Bundle>`_.

Installing from PyPI
--------------------

On supported GNU/Linux systems like the Raspberry Pi, you can install the driver locally `from PyPI <https://pypi.org/project/adafruit-circuitpython-apds9960/>`_.

To install for current user:

.. code-block:: shell

    pip3 install adafruit-circuitpython-apds9960

To install system-wide (this may be required in some cases):

.. code-block:: shell

    sudo pip3 install adafruit-circuitpython-apds9960

To install in a virtual environment in your current project:

.. code-block:: shell

    mkdir project-name && cd project-name
    python3 -m venv .venv
    source .venv/bin/activate
    pip3 install adafruit-circuitpython-apds9960

Usage Example
=============

.. code-block:: python3

    import board
    import digitalio
    from adafruit_apds9960.apds9960 import APDS9960

    i2c = board.I2C()
    int_pin = digitalio.DigitalInOut(board.D5)
    int_pin.switch_to_input(pull=digitalio.Pull.UP)
    apds = APDS9960(i2c)

    apds.enable_proximity_interrupt = True
    apds.proximity_interrupt_threshold = (0, 175)
    apds.enable_proximity = True

    while True:
        if not int_pin.value:
            print(apds.proximity)
            apds.clear_interrupt()

Hardware Set-up
---------------

If you're using a board with a built-in APDS-9960, no hardware setup will be required.

If you're using a breakout board via the pin header, connect ``Vin`` to a 3.3 V or 5 V power source,
connect ``GND`` to ground, then connect ``SCL`` and ``SDA`` to the appropriate pins.

Optionally, if you'd like to use the sensor's interrupt pin connect ``INT`` to any available
digital I/O pin.

Basics
------

To get started, import ``board`` and, and this library:

.. code:: python3


    import board
    from adafruit_apds9960.apds9960 import APDS9960

To set up the sensor to gather data, initialize the I2C bus via ``board.I2C()``
then initialize the APDS-9960 library.

.. code:: python3

    i2c = board.I2C()
    apds = APDS9960(i2c)

Proximity
---------

To get a proximity result, enable the proximity engine then read the `proximity` value.

This will return a value between 0 and 255, with higher values indicating that something is close
to the sensor.

.. code:: python3

    apds.enable_proximity = True

    while True:
      print(apds.proximity)

Gestures
--------

First, enable both the proximity and gesture engines. The gesture engine relies on the proximity
engine to determine when to start itself up and, as a result, proximity readings won't be reliable
while the gesture engine is enabled.

To get a gesture, use the `gesture()` function to see if a gesture has been detected. If a value
greater than 0 is returned, a gesture has been detected.

.. code:: python3

    # Uncomment and set the rotation if depending on how your sensor is mounted.
    # apds.rotation = 270 # 270 for CLUE

    apds.enable_proximity = True
    apds.enable_gesture = True

    while True:
      gesture = apds.gesture()
      if gesture == 1:
        print("up")
      if gesture == 2:
        print("down")
      if gesture == 3:
        print("left")
      if gesture == 4:
        print("right")

Color/Light Measurement
-----------------------

To get a color measurement, first enable the color/light engine, wait for color data to arrive,
then read the `color_data` values.

.. code:: python3

    apds.enable_color = True

    while True:
        while not apds.color_data_ready:
            time.sleep(0.005)

        r, g, b, c = apds.color_data
        print("r: {}, g: {}, b: {}, c: {}".format(r, g, b, c))

Interrupt Pin
-------------

This sensor has an interrupt pin can be asserted (pulled low) if proximity is detected outside of a
specified window of values.

For boards with a built-in APDS-9960 this interupt pin will already be defined. For example, on the
Clue and Feather nRF52840 Sense boards this pin is mapped to ``board.PROXIMITY_LIGHT_INTERRUPT``
and on the Proximity Trinkey it is mapped to ``board.INTERRUPT``.

.. code:: python3

    int_pin = digitalio.DigitalInOut(board.D5)
    int_pin.switch_to_input(pull=digitalio.Pull.UP)

Proximity Detection
-------------------

With the interrupt pin set up we can define a threshold and enable the assertion of the sensor's
interrupt pin by the proximity engine before enabling the proximity engine itself.

In this configuration, the sensor's interrupt pin will be asserted when an object is close to the
sensor. After checking on the interrupt it can be cleared using `clear_interrupt()`

.. code:: python3

    apds.enable_proximity = True

    # set the interrupt threshold to fire when proximity reading goes above 175
    apds.proximity_interrupt_threshold = (0, 175)

    # assert interrupt pin on internal proximity interrupt
    apds.enable_proximity_interrupt = True

    # enable the sensor's proximity engine
    apds.enable_proximity = True

    while True:
      if not interrupt_pin.value:
        print(apds.proximity)

        # clear the interrupt
        apds.clear_interrupt()

Initiaization Options
----------------------

By default, when the driver is initialized, the APDS-9960 sensor's internal settings are reset and
sensible defaults are applied to several low-level settings that should work well for most use cases.

If either the "reset" or "set defaults" behaviors (or both) aren't desired, they can be individually
disabled via init kwargs.

.. code:: python3

    apds = APDS9960(i2c, reset=False, set_defaults=False)

Documentation
=============

API documentation for this library can be found on `Read the Docs <https://docs.circuitpython.org/projects/apds9960/en/latest/>`_.

For information on building library documentation, please check out `this guide <https://learn.adafruit.com/creating-and-sharing-a-circuitpython-library/sharing-our-docs-on-readthedocs#sphinx-5-1>`_.

Contributing
============

Contributions are welcome! Please read our `Code of Conduct
<https://github.com/adafruit/Adafruit_CircuitPython_APDS9960/blob/main/CODE_OF_CONDUCT.md>`_
before contributing to help this project stay welcoming.

Building locally
================

To build this library locally you'll need to install the
`circuitpython-travis-build-tools <https://github.com/adafruit/circuitpython-build-tools>`_ package.

.. code-block::shell

    python3 -m venv .venv
    source .venv/bin/activate
    pip install -r requirements.txt

Once installed, make sure you are in the virtual environment:

.. code-block::shell

    source .venv/bin/activate

Then run the build:

.. code-block::shell

    circuitpython-build-bundles --filename_prefix adafruit-circuitpython-apds --library_location .

Sphinx documentation
-----------------------

Sphinx is used to build the documentation based on rST files and comments in the code. First,
install dependencies (feel free to reuse the virtual environment from above):

.. code-block:: shell

    python3 -m venv .venv
    source .venv/bin/activate
    pip install Sphinx sphinx-rtd-theme

Now, once you have the virtual environment activated:

.. code-block:: shell

    cd docs
    sphinx-build -E -W -b html . _build/html

This will output the documentation to ``docs/_build/html``. Open the index.html in your browser to
view them. It will also (due to -W) error out on any warning like Travis will. This is a good way to
locally verify it will pass.
