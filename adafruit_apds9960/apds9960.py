# SPDX-FileCopyrightText: 2017 Michael McWethy for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
`APDS9960`
====================================================

Driver class for the APDS9960 board.  Supports gesture, proximity, and color
detection.

* Author(s): Michael McWethy

Implementation Notes
--------------------

**Hardware:**

* Adafruit `APDS9960 Proximity, Light, RGB, and Gesture Sensor
  <https://www.adafruit.com/product/3595>`_ (Product ID: 3595)

* Adafruit `Adafruit CLUE
  <https://www.adafruit.com/product/4500>`_ (Product ID: 4500)

* Adafruit `Adafruit Feather nRF52840 Sense
  <https://www.adafruit.com/product/4516>`_ (Product ID: 4516)

* Adafruit `Adafruit Proximity Trinkey
  <https://www.adafruit.com/product/5022>`_ (Product ID: 5022)

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads

* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
"""
import time
from adafruit_register.i2c_bits import RWBits
from adafruit_register.i2c_bit import RWBit, ROBit
from adafruit_register.i2c_struct import UnaryStruct, ROUnaryStruct
from adafruit_bus_device.i2c_device import I2CDevice
from micropython import const

try:
    # Only used for typing
    from typing import Tuple, List
    from busio import I2C
except ImportError:
    pass

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_APDS9960.git"

# APDS9960_RAM        = const(0x00)
_APDS9960_ENABLE = const(0x80)
_APDS9960_ATIME = const(0x81)
# APDS9960_WTIME      = const(0x83)
# APDS9960_AILTIL     = const(0x84)
# APDS9960_AILTH      = const(0x85)
# APDS9960_AIHTL      = const(0x86)
# APDS9960_AIHTH      = const(0x87)
_APDS9960_PILT = const(0x89)
_APDS9960_PIHT = const(0x8B)
_APDS9960_PERS = const(0x8C)
# APDS9960_CONFIG1    = const(0x8D)
_APDS9960_PPULSE = const(0x8E)
_APDS9960_CONTROL = const(0x8F)
_APDS9960_CONFIG2 = const(0x90)
_APDS9960_ID = const(0x92)
_APDS9960_STATUS = const(0x93)
_APDS9960_CDATAL = const(0x94)
# APDS9960_CDATAH     = const(0x95)
# APDS9960_RDATAL     = const(0x96)
# APDS9960_RDATAH     = const(0x97)
# APDS9960_GDATAL     = const(0x98)
# APDS9960_GDATAH     = const(0x99)
# APDS9960_BDATAL     = const(0x9A)
# APDS9960_BDATAH     = const(0x9B)
_APDS9960_PDATA = const(0x9C)
# _APDS9960_POFFSET_UR = const(0x9D)
# _APDS9960_POFFSET_DL = const(0x9E)
# _APDS9960_CONFIG3 = const(0x9F)
_APDS9960_GPENTH = const(0xA0)
_APDS9960_GEXTH = const(0xA1)
_APDS9960_GCONF1 = const(0xA2)
_APDS9960_GCONF2 = const(0xA3)
# _APDS9960_GOFFSET_U  = const(0xA4)
# _APDS9960_GOFFSET_D  = const(0xA5)
# _APDS9960_GOFFSET_L  = const(0xA7)
# _APDS9960_GOFFSET_R  = const(0xA9)
_APDS9960_GPULSE = const(0xA6)
# _APDS9960_GCONF3 = const(0xAA)
_APDS9960_GCONF4 = const(0xAB)
_APDS9960_GFLVL = const(0xAE)
_APDS9960_GSTATUS = const(0xAF)
# APDS9960_IFORCE     = const(0xE4)
_APDS9960_PICLEAR = const(0xE5)
_APDS9960_CICLEAR = const(0xE6)
_APDS9960_AICLEAR = const(0xE7)
_APDS9960_GFIFO_U = const(0xFC)
# APDS9960_GFIFO_D    = const(0xFD)
# APDS9960_GFIFO_L    = const(0xFE)
# APDS9960_GFIFO_R    = const(0xFF)

_GESTURE_NAMES = ["None", "Up", "Down", "Left", "Right"]
_CYCLE_TIME = 0.00278  # Sensor internal cycle time in milliseconds

# pylint: disable-msg=too-many-instance-attributes, too-many-public-methods
class APDS9960:
    """
    APDS9900 provide basic driver services for the ASDS9960 breakout board

    :param ~busio.I2C i2c: The I2C bus the ASDS9960 is connected to
    :param int address: The I2C device address. Defaults to :const:`0x39`
    :param int rotation: Rotation of the device. Defaults to :const:`0`
    :param bool reset: If true, resets device registers during init. Defaults to :const:`True`
    :param bool set_defaults: If true, set sensible defaults during init. Defaults to :const:`True`
    :param int gesture_max_dataframes: Maxmium size gesture dataframe. Defaults to :const:`64`
    :param int gesture_persist_cycles: Cycles to wait for new gesture data. Defaults to :const:`5`
    :param int gesture_high_pass_threshold: Minimum value for gesture data. Defaults to :const:`30`


    **Quickstart: Importing and using the APDS9960**

        Here is an example of using the :class:`APDS9960` class.
        First you will need to import the libraries to use the sensor

        .. code-block:: python

            import board
            from adafruit_apds9960.apds9960 import APDS9960

        Once this is done you can define your `board.I2C` object and define your sensor object

        .. code-block:: python

            i2c = board.I2C()   # uses board.SCL and board.SDA
            apds = APDS9960(i2c)

        Now you have access to the :attr:`proximity_enable` and :attr:`sensor.proximity` attributes

        .. code-block:: python

            apds.proximity_enable = True
            proximity = apds.proximity

    """

    def __init__(
        self,
        i2c: I2C,
        *,
        address: int = 0x39,
        rotation: int = 0,
        reset: bool = True,
        set_defaults: bool = True,
        gesture_max_dataframes: int = 64,
        gesture_persist_cycles: int = 5,
        gesture_high_pass_threshold: int = 30
    ):

        self.gesture_buffer = None
        self.msg_buffer = bytearray(2)

        self.i2c_device = I2CDevice(i2c, address)

        if self.device_id != 0xAB:
            raise RuntimeError()

        self._rotation = rotation

        if reset:
            self.reset()

        if set_defaults:
            self.defaults()

        self._gesture_max_dataframes = gesture_max_dataframes
        self._data_gesture_persist_cycles = gesture_persist_cycles
        self._data_gesture_high_pass_threshold = gesture_high_pass_threshold
        self._data_stream_persist_sleep = (
            self._data_gesture_high_pass_threshold * _CYCLE_TIME
        )

    def defaults(self):
        """Apply sensible defaults to fit most use cases"""
        self.proximity_interrupt_threshold = (0, 150, 5)  # 0 near, 150 far, 5 cycles
        self.proximity_led_config = (7, 1, 0, 0)  # 8 pulses, 8us, 100mA x1
        self.proximity_gain = 1

        self.gesture_engine_config = (5, 100, 2, 2)  # ent: 5, ex: 100, per: 2, wait: 2
        self.gesture_led_config = (5, 2, 0, 2)  # 4 pulses, 32us, 100mA x2
        self.gesture_fifo_threshold = 1
        self.gesture_gain = 1

    def reset(self):
        """Reset device registers to power-on defaults"""
        self.enable_proximity = False
        self.enable_proximity_interrupt = False
        self.proximity_interrupt_threshold = (0, 0, 0)  # 0 near, 0 far, 0 cycles
        self.proximity_led_config = (0, 1, 0, 0)  # 1 pulse, 8us, 100mA x1
        self.proximity_gain = 0

        self.enable_gesture = False
        self.enable_gesture_interrupt = False
        self.gmode = False
        self.clear_gesture_fifo()
        self.gesture_engine_config = (0, 0, 0, 0)  # ent: 0, ex: 0, per: 0, wait: 0
        self.gesture_led_config = (0, 1, 0, 0)  # 1 pulse, 8us, 100mA x1
        self.gesture_gain = 0
        self.gesture_fifo_threshold = 0

        self.enable_color = False
        self.color_integration_time = 1

        self.clear_all_interrupts()

        self.enable = False
        time.sleep(0.010)
        self.enable = True
        time.sleep(0.010)

    # Device Configuration
    device_id = UnaryStruct(_APDS9960_ID, "<B")
    enable = RWBit(_APDS9960_ENABLE, 0)
    """If true, the sensor will be operational and will cycle through enabled engines

    If false, the sensor will enter a sleep state until re-enabled

    While in sleep state the internal oscillator and other circuits are disabled, resulting in
        very low power consumption but I2C messages will still be handled"""
    enable_proximity = RWBit(_APDS9960_ENABLE, 2)
    """Enables operation of proximity engine"""
    enable_gesture = RWBit(_APDS9960_ENABLE, 6)
    """Enables operation of gesture engine"""
    enable_color = RWBit(_APDS9960_ENABLE, 1)
    """Enables operation of color/light engine"""

    def clear_all_interrupts(self):
        """Clears all non-gesture internal interrupts"""
        self._writecmdonly(_APDS9960_AICLEAR)

    # _wtime = UnaryStruct(_APDS9960_WTIME, "<B")
    # cycle_wait_time_long = RWBit(_APDS9960_CONFIG1, 1)

    # @property
    # def cycle_wait_time(self) -> int:
    #     """Number of 2.78ms cycles to wait between sensor-wide cycles

    #     Multiplied by 12x if `cycle_wait_time_long` is true"""
    #     return 256 - self._wtime

    # @cycle_wait_time.setter
    # def cycle_wait_time(self, wait_time: int) -> None:
    #     """Number of 2.78ms cycles to wait between sensor-wide cycles

    #     Multiplied by 12x if `cycle_wait_time_long` is true"""
    #     if 1 <= wait_time <= 256:
    #         self._atime = 256 - self._wtime
    #     else:
    #         raise ValueError

    # Proximity Data
    proximity = ROUnaryStruct(_APDS9960_PDATA, "<B")
    """Proximity data, 0-255 with 0 being 'far' and 255 being 'near'"""
    proximity_valid = ROBit(_APDS9960_STATUS, 1)
    """True if new, valid proximity data is available

    Cleared when proximity data is read"""

    # Proximity LED/Sensor Configuration
    _ppulse = RWBits(6, _APDS9960_PPULSE, 0)
    _pplen = RWBits(2, _APDS9960_PPULSE, 6)
    _ldrive = RWBits(2, _APDS9960_CONTROL, 6)
    _ledboost = RWBits(2, _APDS9960_CONFIG2, 4)

    @property
    def proximity_led_config(self) -> Tuple[int, int, int, int]:
        """Tuple representing LED configuration for proximity measurements

        Pulse count (0-63): 0 = 1 pulse, 63 = 64 pulses (default: 0)
        Pulse length (0-3): 0 = 4 usec, 1 = 8 usec, 2 = 16 usec, 3 = 32 usec (default: 1)
        LED drive (0-3): 0 = 100 mA, 1 = 50 mA, 2 = 25 mA, 3 = 12.5 mA (default: 0)
        LED boost (0-3): 0 = 100%, 1 = 150%, 2 = 200%, 3 = 300% (default: 0)"""
        return (self._ppulse, self._pplen, self._ldrive, self._ledboost)

    @proximity_led_config.setter
    def proximity_led_config(self, led_config: Tuple[int, int, int, int]) -> None:
        """Tuple representing LED configuration for proximity measurements

        Pulse count (0-63): 0 = 1 pulse, 63 = 64 pulses (default: 0)
        Pulse length (0-3): 0 = 4 usec, 1 = 8 usec, 2 = 16 usec, 3 = 32 usec (default: 1)
        LED drive (0-3): 0 = 100 mA, 1 = 50 mA, 2 = 25 mA, 3 = 12.5 mA (default: 0)
        LED boost (0-3): 0 = 100%, 1 = 150%, 2 = 200%, 3 = 300% (default: 0)"""
        if 0 <= led_config[0] <= 63:
            self._ppulse = led_config[0]
        else:
            raise ValueError

        if 0 <= led_config[1] <= 3:
            self._pplen = led_config[1]
        else:
            raise ValueError

        if 0 <= led_config[2] <= 3:
            self._ldrive = led_config[2]
        else:
            raise ValueError

        if 0 <= led_config[3] <= 3:
            self._ledboost = led_config[3]
        else:
            raise ValueError

    # Proximity Sensor Configuration
    _pgain = RWBits(2, _APDS9960_CONTROL, 2)

    @property
    def proximity_gain(self) -> int:
        """Proximity sensor gain multipler

        Proximity Gain (0-3): 0 = 1x, 1 = 2x, 2 = 4x, 3 = 8x (default: 0)"""
        return self._pgain

    @proximity_gain.setter
    def proximity_gain(self, gain: int) -> None:
        """Proximity sensor gain multipler

        Proximity Gain (0-3): 0 = 1x, 1 = 2x, 2 = 4x, 3 = 8x (default: 0)"""
        if 0 <= gain <= 3:
            self._pgain = gain
        else:
            raise ValueError

    ## Proximity masking, offsets, gain compensation
    # _pmsk_u = RWBit(_APDS9960_CONFIG3, 3)
    # _pmsk_d = RWBit(_APDS9960_CONFIG3, 2)
    # _pmsk_l = RWBit(_APDS9960_CONFIG3, 1)
    # _pmsk_r = RWBit(_APDS9960_CONFIG3, 0)
    # _poffset_ur_sign = RWBit(APDS9960_POFFSET_UR, 1)
    # _poffset_ur = RWBits(7, APDS9960_POFFSET_UR)
    # _poffset_dl_sign = RWBit(APDS9960_POFFSET_DL, 1)
    # _poffset_dl = RWBits(7, APDS9960_POFFSET_DL)
    # _pcmp = RWBit(_APDS9960_CONFIG3, 5)

    # Proximity Interrupts and Interrupt Configuration
    proximity_interrupt = ROBit(_APDS9960_STATUS, 5)
    """Asserted when persistence threshold is met by sequential valid proximity measurements

    Cleared by manual 'proximity_interrupt_clear' or 'all_interrupt_clear'

    Can assert external interrupt pin if 'enable_proximity_interrupt' is true"""
    enable_proximity_interrupt = RWBit(_APDS9960_ENABLE, 5)
    """If true, internal proximity interrupt asserts interrupt pin"""

    # proximity_saturation_interrupt = ROBit(_APDS9960_STATUS, 6)
    # """Asserted if an analog saturation event occurs during proximity or gesture measurements

    # Cleared by 'proximity_interrupt_clear' or by disabling proximity engine

    # Can assert external interrupt pin if 'enable_proximity_interrupt' is true"""
    # enable_proximity_saturation_interrupt = RWBit(_APDS9960_CONFIG2, 5)
    # """If true, proximity saturation interrupt asserts interrupt pin"""

    def clear_proximity_interrupt(self):
        """Clears internal proximity interrupt"""
        self._writecmdonly(_APDS9960_PICLEAR)

    _pilt = UnaryStruct(_APDS9960_PILT, "<B")
    _piht = UnaryStruct(_APDS9960_PIHT, "<B")
    _ppers = RWBits(4, _APDS9960_PERS, 4)

    @property
    def proximity_interrupt_threshold(self) -> Tuple[int, int, int]:
        """Tuple representing proximity engnie low/high threshold (0-255) and persistence (0-15)

        Controls assertion of internal proximty interrupt

        Internal interrupt is only asserted when the number of valid, in-threshold measurements is
        equal to or greater than the persistence setting"""
        return (
            self._pilt,
            self._piht,
            self._ppers,
        )

    @proximity_interrupt_threshold.setter
    def proximity_interrupt_threshold(
        self, setting_tuple: Tuple[int, int, int]
    ) -> None:
        """Tuple representing proximity engnie low/high threshold (0-255) and persistence (0-15)

        Controls assertion of internal proximty interrupt

        Internal interrupt is only asserted when the number of valid, in-threshold measurements is
        equal to or greater than the persistence setting"""
        if 0 <= setting_tuple[0] <= 255:
            self._pilt = setting_tuple[0]
        else:
            raise ValueError

        if 0 <= setting_tuple[1] <= 255:
            self._piht = setting_tuple[1]
        else:
            raise ValueError

        if 0 <= setting_tuple[2] <= 15:
            self._ppers = setting_tuple[2]
        else:
            raise ValueError

    # Gesture Data and Status
    gesture_valid = ROBit(_APDS9960_GSTATUS, 0)
    _gflvl = ROUnaryStruct(_APDS9960_GFLVL, "<B")
    _gfov = ROBit(_APDS9960_GSTATUS, 1)
    _gmode = RWBit(_APDS9960_GCONF4, 0)

    @property
    def is_gesture_looping(self) -> bool:
        """True if the sensor's gesture engine is currently looping"""
        return self._gmode

    def force_gesture_loop_entry(self):
        """Force gesture engine to enter regardless of proximity entry threshold status"""
        self._gmode = True

    def force_gesture_loop_exit(self):
        """Forces gesture engine to halt if it is currently looping regardless of exit threshold"""
        self._gmode = False

    _gfifo_clr = RWBit(_APDS9960_GCONF4, 2)

    def clear_gesture_fifo(self):
        """Clears gestire FIFO, interrupt, overflow flag, and resets FIFO level"""
        self._gfifo_clr = True

    # Gesture Core Engine Configuration
    _gpenth = UnaryStruct(_APDS9960_GPENTH, "<B")
    _gexth = UnaryStruct(_APDS9960_GEXTH, "<B")
    _gpers = RWBits(2, _APDS9960_GCONF1, 0)
    _gwtime = RWBits(3, _APDS9960_GCONF2, 0)
    # _gexmsk = RWBits(4, _APDS9960_GCONF1, 2)

    @property
    def gesture_engine_config(self) -> Tuple[int, int, int, int, int]:
        """Tuple representing configuration for gesture engine

        Proximity entry threshold (0-255): Minimum proximity value for gesture engine entrance
        Exit threshold (0-255): Minimum proximity value for gesture engine loop persistence
        Persistence (0-3): Number of out-of-thresold loops to continue recording gesture data
            0 = 1 cycle, 1 = 2 cycles, 2 = 4 cycles, 3 = 7 cycles
        Wait time (0-7): Number of 2.78ms cycles to wait between gesture 1.39 ms engine loops"""
        return (self._gpenth, self._gexth, self._gpers, self._gwtime)

    @gesture_engine_config.setter
    def gesture_engine_config(
        self, gesture_config: Tuple[int, int, int, int, int]
    ) -> None:
        """Tuple representing configuration for gesture engine

        Proximity entry threshold (0-255): Minimum proximity value for gesture engine entrance
        Exit threshold (0-255): Minimum proximity value for gesture engine loop persistence
        Persistence (0-3): Number of out-of-thresold loops to continue recording gesture data
            0 = 1 cycle, 1 = 2 cycles, 2 = 4 cycles, 3 = 7 cycles
        Wait time (0-7): Number of 2.78ms cycles to wait between gesture 1.39 ms engine loops"""
        if 0 <= gesture_config[0] <= 255:
            self._gpenth = gesture_config[0]
        else:
            raise ValueError

        if 0 <= gesture_config[1] <= 255:
            self._gexth = gesture_config[1]
        else:
            raise ValueError

        if 0 <= gesture_config[2] <= 3:
            self._gpers = gesture_config[2]
        else:
            raise ValueError

        if 0 <= gesture_config[3] <= 7:
            self._gwtime = gesture_config[3]
        else:
            raise ValueError

    # Gesture LED/Sensor Configuration
    _gpulse = RWBits(6, _APDS9960_GPULSE, 0)
    _gplen = RWBits(2, _APDS9960_GPULSE, 6)
    _gldrive = RWBits(2, _APDS9960_GCONF2, 3)

    @property
    def gesture_led_config(self) -> Tuple[int, int, int, int]:
        """Tuple representing LED configuration for gesture measurements

        Pulse count (0-63): 0 = 1 pulse, 63 = 64 pulses (default: 0)
        Pulse length (0-3): 0 = 4 usec, 1 = 8 usec, 2 = 16 usec, 3 = 32 usec (default: 1)
        LED drive (0-3): 0 = 100 mA, 1 = 50 mA, 2 = 25 mA, 3 = 12.5 mA (default: 0)
        LED boost (0-3): 0 = 100%, 1 = 150%, 2 = 200%, 3 = 300% (default: 0)"""
        return (self._gpulse, self._gplen, self._gldrive, self._ledboost)

    @gesture_led_config.setter
    def gesture_led_config(self, led_config: Tuple[int, int, int, int]) -> None:
        """Tuple representing LED configuration for gesture measurements

        Pulse count (0-63): 0 = 1 pulse, 63 = 64 pulses (default: 0)
        Pulse length (0-3): 0 = 4 usec, 1 = 8 usec, 2 = 16 usec, 3 = 32 usec (default: 1)
        LED drive (0-3): 0 = 100 mA, 1 = 50 mA, 2 = 25 mA, 3 = 12.5 mA (default: 0)
        LED boost (0-3): 0 = 100%, 1 = 150%, 2 = 200%, 3 = 300% (default: 0)"""
        if 0 <= led_config[0] <= 63:
            self._gpulse = led_config[0]
        else:
            raise ValueError

        if 0 <= led_config[1] <= 3:
            self._gplen = led_config[1]
        else:
            raise ValueError

        if 0 <= led_config[2] <= 3:
            self._gldrive = led_config[2]
        else:
            raise ValueError

        if 0 <= led_config[3] <= 3:
            self._ledboost = led_config[3]
        else:
            raise ValueError

    # Proximity Sensor Configuration
    _ggain = RWBits(2, _APDS9960_GCONF2, 5)

    @property
    def gesture_gain(self) -> int:
        """Gesture sensor gain multipler

        Gesture Gain (0-3): 0 = 1x, 1 = 2x, 2 = 4x, 3 = 8x (default: 0)"""
        return self._ggain

    @gesture_gain.setter
    def gesture_gain(self, gain: int) -> None:
        """Gesture sensor gain multipler

        Gesture Gain (0-3): 0 = 1x, 1 = 2x, 2 = 4x, 3 = 8x (default: 0)"""
        if 0 <= gain <= 3:
            self._ggain = gain
        else:
            raise ValueError

    # _gdims = RWBits(2, _APDS9960_GCONF3, 0)
    # _goffset_u_sign = RWBit(_APDS9960_GOFFSET_U, 1)
    # _goffset_u = RWBits(7, _APDS9960_GOFFSET_U)
    # _goffset_d_sign = RWBit(_APDS9960_GOFFSET_D, 1)
    # _goffset_d = RWBits(7, _APDS9960_GOFFSET_D)
    # _goffset_l_sign = RWBit(_APDS9960_GOFFSET_L, 1)
    # _goffset_l = RWBits(7, _APDS9960_GOFFSET_L)
    # _goffset_r_sign = RWBit(_APDS9960_GOFFSET_R, 1)
    # _goffset_r = RWBits(7, _APDS9960_GOFFSET_R)

    # Gesture Interrupts and Interrupt Configuration
    gesture_interrupt = ROBit(_APDS9960_STATUS, 2)
    """Asserted when gesture FIFO threshold is met by sequential valid gesture measurements

    Cleared by full read of gesture FIFOs or manual gesture FIFO Clear"""
    enable_gesture_interrupt = RWBit(_APDS9960_GCONF4, 1)
    """If true, internal gesture interrupt asserts interrupt pin"""
    _gfifoth = RWBits(2, _APDS9960_GCONF1, 6)

    @property
    def gesture_fifo_threshold(self) -> int:
        """Minimum gesture FIFO depth to reach before triggering internal gesture interrupt

        FIFO Threshold (0-3): 0 = 1 dataset, 1 = 4 datasets, 2 = 8 datasets, 3 = 16 datasets"""
        return self._gfifoth

    @gesture_fifo_threshold.setter
    def gesture_fifo_threshold(self, threshold: int) -> None:
        """Minimum gesture FIFO depth to reach before triggering internal gesture interrupt

        FIFO Threshold (0-3): 0 = 1 dataset, 1 = 4 datasets, 2 = 8 datasets, 3 = 16 datasets"""
        if 0 <= threshold <= 3:
            self._gwtime = threshold
        else:
            raise ValueError

    # Color Data and Status
    @property
    def color_data(self) -> Tuple[int, int, int, int]:
        """Tuple containing r, g, b, c values"""
        return (
            self._color_data16(_APDS9960_CDATAL + 2),
            self._color_data16(_APDS9960_CDATAL + 4),
            self._color_data16(_APDS9960_CDATAL + 6),
            self._color_data16(_APDS9960_CDATAL),
        )

    color_valid = ROBit(_APDS9960_STATUS, 0)

    # Color Engine Configuration

    _atime = UnaryStruct(_APDS9960_ATIME, "<B")

    @property
    def color_integration_time(self) -> int:
        """Number of 2.78ms cycles to wait for ADC integration during color operations"""
        return 256 - self._atime

    @color_integration_time.setter
    def color_integration_time(self, integration_time: int) -> None:
        """Number of 2.78ms cycles to wait for ADC integration during color operations"""
        if 1 <= integration_time <= 256:
            self._atime = 256 - integration_time
        else:
            raise ValueError

    # _ailtl = UnaryStruct(_APDS_AILTL, "<B")
    # _ailth = UnaryStruct(_APDS_AILTH, "<B")
    # _aihtl = UnaryStruct(_APDS_AIHTL, "<B")
    # _alhth = UnaryStruct(_APDS_AIHTH, "<B")
    # _apers = RWBits(4, _APDS9960_PERS, 0)

    color_gain = RWBits(2, _APDS9960_CONTROL, 0)
    """Color gain value"""

    color_interrupt = ROBit(_APDS9960_STATUS, 4)
    """Asserted when color clear channel threshold is met or exceeded by sequential color results

    Cleared by 'clear_color_interrupt' or 'clear_all_interrupts'"""
    # color_saturation_interrupt = ROBit(_APDS9960_STATUS, 7)

    def clear_color_interrupt(self) -> None:
        """Clears internal color interrupt"""
        self._writecmdonly(_APDS9960_CICLEAR)

    @property
    def color_data_ready(self) -> int:
        """Color data ready flag.  zero if not ready, 1 is ready"""
        return self.color_valid

    # Gesture processing rotation
    @property
    def rotation(self) -> int:
        """Gesture rotation offset. Acceptable values are 0, 90, 180, 270."""
        return self._rotation

    @rotation.setter
    def rotation(self, new_rotation: int) -> None:
        if new_rotation in [0, 90, 180, 270]:
            self._rotation = new_rotation
        else:
            raise ValueError("Rotation value must be one of: 0, 90, 180, 270")

    def rotated_gesture(self, original_gesture: int) -> int:
        """Applies rotation offset to the given gesture direction and returns the result"""
        directions = [1, 4, 2, 3]
        new_index = (directions.index(original_gesture) + self._rotation // 90) % 4
        return directions[new_index]

    def gesture_string(self, blocking=False) -> str:
        """Gather and process gesture data, returns string representing gesture direction"""
        gesture_number = self.gesture(blocking)

        return _GESTURE_NAMES[gesture_number]

    def gesture(self, blocking=False) -> int:
        """Gather and process gesture data, returns int representing gesture direction

        Blocking: If true, wait for 'gesture_interrupt' before reading in gesture data.

        Return: 0 = None, 1 = Up, 2 = Down, 3 = Left, 4 = Right"""
        if blocking:
            # If we want to block until gesture data comes in we'll wait on our FIFO threshold
            while not self.gesture_interrupt:
                pass

        dataframe = self._get_gesture_data()

        if len(dataframe) > 0:
            processed_gesture = self._process_gesture_data(dataframe)
            if processed_gesture > 0:
                return self.rotated_gesture(processed_gesture)

        return 0

    def _get_gesture_data(self) -> List[Tuple[int, int, int, int]]:
        """Retrieves sequential gesture datasets from FIFO, if any are available"""
        dataframe = []

        # If FIFOs have overflowed we're already way too late, so clear those FIFOs and wait
        if self._gfov:
            self.clear_gesture_fifo()
            # Don't wait forever though, just enough to see if a gesture is happening
            wait_cycles = 0
            while not self.gesture_interrupt and wait_cycles <= 30:
                time.sleep(_CYCLE_TIME)
                wait_cycles += 1

        # Only start retrieval if there are datasets to retrieve
        dataset_count = self._gflvl
        if dataset_count > 0:
            if self.gesture_buffer is None:
                self.gesture_buffer = bytearray(129)

            buffer = self.gesture_buffer

            # Stack new gesture datasets into our dataframe
            # Also, keep stacking new datasets if they show up while we're reading in FIFO data
            while True:
                # Acquire all available data
                dataset_count = self._gflvl
                buffer[0] = _APDS9960_GFIFO_U
                with self.i2c_device as i2c:
                    i2c.write_then_readinto(
                        buffer,
                        buffer,
                        out_end=1,
                        in_start=1,
                        in_end=min(129, 1 + (dataset_count * 4)),
                    )

                idx = 0
                # Unpack data stream into more usable U/D/L/R dataset tuples
                for i in range(dataset_count):
                    rec = i + 1
                    idx = 1 + ((rec - 1) * 4)

                    dataset_tuple = (
                        buffer[idx],
                        buffer[idx + 1],
                        buffer[idx + 2],
                        buffer[idx + 3],
                    )

                    # Drop fully-saturated and fully-zero to conserve memory
                    # Low-pass filter to remove potentially spurious very-low-count entries
                    if self._filter_dataset(
                        dataset_tuple, self._data_gesture_high_pass_threshold
                    ):
                        dataframe.append(dataset_tuple)

                # Break out of our loop ASAP if we have way too many datasets
                if len(dataframe) > self._gesture_max_dataframes:
                    # print("Get: Halting, gflvl: {}, len: {}".format(self._gflvl, len(dataframe)))
                    break

                # Wait a very short time to see if new FIFO data has arrived before we drop out
                time.sleep(self._data_stream_persist_sleep)
                if self._gflvl == 0:
                    # print("Get: Halting, gflvl: {}, len: {}".format(self._gflvl, len(dataframe)))
                    break
                # else:
                # print("Get: Continuing, gflvl: {}, len: {}".format(self._gflvl, len(dataframe)))
        return dataframe

    # pylint: disable-msg=no-else-return, too-many-return-statements, too-many-branches
    def _process_gesture_data(self, dataframe: List[Tuple[int, int, int, int]]) -> int:
        """Processes gesture dataframes to determine gesture

        This assumes that the dataframe has already been high-pass filtered"""
        _gesture_delta_threshold = 30

        first_dataset = dataframe[0]
        last_dataset = dataframe[len(dataframe) - 1]

        # print("Process: f: {}, l: {}".format(first_dataset, last_dataset))

        ratios_first = self._dataset_ratios(first_dataset)
        ratios_last = self._dataset_ratios(last_dataset)

        delta_ud = ratios_last[0] - ratios_first[0]
        delta_lr = ratios_last[1] - ratios_first[1]

        state_ud = 0
        state_lr = 0

        if delta_ud >= _gesture_delta_threshold:
            state_ud = 1
        elif delta_ud <= -_gesture_delta_threshold:
            state_ud = -1

        if delta_lr >= _gesture_delta_threshold:
            state_lr = 1
        elif delta_lr <= -_gesture_delta_threshold:
            state_lr = -1

        # Easy cases
        if state_ud == -1 and state_lr == 0:
            return 1
        elif state_ud == 1 and state_lr == 0:
            return 2
        elif state_ud == 0 and state_lr == -1:
            return 3
        elif state_ud == 0 and state_lr == 1:
            return 4

        # Not so easy cases
        if state_ud == -1 and state_lr == 1:
            if abs(delta_ud) > abs(delta_lr):
                return 1
            else:
                return 4
        elif state_ud == 1 and state_lr == -1:
            if abs(delta_ud) > abs(delta_lr):
                return 2
            else:
                return 3
        elif state_ud == -1 and state_lr == -1:
            if abs(delta_ud) > abs(delta_lr):
                return 1
            else:
                return 3
        elif state_ud == 1 and state_lr == 1:
            if abs(delta_ud) > abs(delta_lr):
                return 2
            else:
                return 3

        return 0

    @staticmethod
    def _dataset_ratios(dataset: Tuple[int, int, int, int]) -> Tuple[int, int]:
        ratio_ud = ((dataset[0] - dataset[1]) * 100) // (dataset[0] + dataset[1])
        ratio_lr = ((dataset[2] - dataset[3]) * 100) // (dataset[2] + dataset[3])
        return ratio_ud, ratio_lr

    @staticmethod
    def _filter_dataset(dataset: Tuple[int, int, int, int], low_thresh: int) -> bool:
        if all(val == 255 for val in dataset):
            return False
        elif all(val == 0 for val in dataset):
            return False
        elif not all(val >= low_thresh for val in dataset):
            return False
        else:
            return True

    # method for reading and writing to I2C
    def _writecmdonly(self, command: int) -> None:
        """Writes a command and 0 bytes of data to the I2C device"""
        buf = self.msg_buffer
        buf[0] = command
        with self.i2c_device as i2c:
            i2c.write(buf, end=1)

    def _color_data16(self, command: int) -> int:
        """Sends a command and reads 2 bytes of data from the I2C device
        The returned data is low byte first followed by high byte"""
        buf = self.msg_buffer
        buf[0] = command
        with self.i2c_device as i2c:
            i2c.write_then_readinto(buf, buf, out_end=1)
        return buf[1] << 8 | buf[0]
