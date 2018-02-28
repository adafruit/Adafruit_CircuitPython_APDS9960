
Introduction
============

.. image:: https://readthedocs.org/projects/adafruit-circuitpython-apds9960/badge/?version=latest
    :target: https://circuitpython.readthedocs.io/projects/apds9960/en/latest/
    :alt: Documentation Status

.. image :: https://img.shields.io/discord/327254708534116352.svg
    :target: https://discord.gg/nBQh6qu
    :alt: Discord

The APDS9960 is a specialize chip that detects hand gestures, proximity detection
and ambient light color over I2C. Its available on
`Adafruit as a breakout <https://www.adafruit.com/product/3595>`_.


Dependencies
=============
This driver depends on:

* `Adafruit CircuitPython <https://github.com/adafruit/circuitpython>`_

Please ensure all dependencies are available on the CircuitPython filesystem.
This is easily achieved by downloading
`the Adafruit library and driver bundle <https://github.com/adafruit/Adafruit_CircuitPython_Bundle>`_.

Usage Example
=============

Hardware Set-up
---------------

Connect Vin to 3.3 V or 5 V power source, GND to ground, SCL and SDA to the appropriate pins.

Basics
------

Of course, you must import i2c bus device, board pins, and the library:

.. code:: python


  from board import SCL, SDA, A1
  from adafruit_apds9960.apds9960 import APDS9960
  import busio
  import digitalio

To set-up the device to gather data, initialize the I2CDevice using SCL
and SDA pins.   Then initialize the library.  Optionally provide an interrupt
pin for proximity detection.

.. code:: python

  int_pin = digitalio.DigitalInOut(A1)
  i2c = busio.I2C(SCL, SDA)
  apds = APDS9960(i2c, interrupt_pin=int_pin)

Gestures
--------

To get a gesture, see if a gesture is available first, then get the gesture Code

.. code:: python

  gesture = apds.gesture()
  if gesture == 1:
    print("up")
  if gesture == 2:
    print("down")
  if gesture == 3:
    print("left")
  if gesture == 4:
    print("right")

Color Measurement
-----------------

To get a color measure, enable color measures, wait for color data,
then get the color data.

.. code:: python

  apds.enable_color = True

  while not apds.color_data_ready:
      time.sleep(0.005)

  r, g, b, c = apds.color_data
  print("r: {}, g: {}, b: {}, c: {}".format(r, g, b, c))

Proximity Detection
---------------------

To check for a object in proximity, see if a gesture is available first, then get the gesture Code

.. code:: python

  apds.enable_proximity = True

  # set the interrupt threshold to fire when proximity reading goes above 175
  apds.proximity_interrupt_threshold = (0, 175)

  # enable the proximity interrupt
  apds.enable_proximity_interrupt = True

  while True:
    if not interrupt_pin.value:
      print(apds.proximity())

      # clear the interrupt
      apds.clear_interrupt()


Contributing
============

Contributions are welcome! Please read our `Code of Conduct
<https://github.com/adafruit/Adafruit_CircuitPython_APDS9960/blob/master/CODE_OF_CONDUCT.md>`_
before contributing to help this project stay welcoming.

Building locally
================

To build this library locally you'll need to install the
`circuitpython-travis-build-tools <https://github.com/adafruit/circuitpython-build-tools>`_ package.

.. code-block::shell

    python3 -m venv .env
    source .env/bin/activate
    pip install -r requirements.txt

Once installed, make sure you are in the virtual environment:

.. code-block::shell

    source .env/bin/activate

Then run the build:

.. code-block::shell

    circuitpython-build-bundles --filename_prefix adafruit-circuitpython-apds --library_location .

Sphinx documentation
-----------------------

Sphinx is used to build the documentation based on rST files and comments in the code. First,
install dependencies (feel free to reuse the virtual environment from above):

.. code-block:: shell

    python3 -m venv .env
    source .env/bin/activate
    pip install Sphinx sphinx-rtd-theme

Now, once you have the virtual environment activated:

.. code-block:: shell

    cd docs
    sphinx-build -E -W -b html . _build/html

This will output the documentation to ``docs/_build/html``. Open the index.html in your browser to
view them. It will also (due to -W) error out on any warning like Travis will. This is a good way to
locally verify it will pass.