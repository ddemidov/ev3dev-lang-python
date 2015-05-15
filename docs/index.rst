.. ev3dev documentation master file, created by
   sphinx-quickstart on Wed May 13 14:22:19 2015.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. include:: ../README.rst

Module interface
----------------

.. toctree::
   :maxdepth: 2

.. autosummary::
    ev3dev.device
    ev3dev.motor
    ev3dev.dc_motor
    ev3dev.servo_motor
    ev3dev.medium_motor
    ev3dev.large_motor
    ev3dev.sensor
    ev3dev.i2c_sensor
    ev3dev.touch_sensor
    ev3dev.color_sensor
    ev3dev.ultrasonic_sensor
    ev3dev.gyro_sensor
    ev3dev.sound_sensor
    ev3dev.light_sensor
    ev3dev.infrared_sensor
    ev3dev.remote_control
    ev3dev.led
    ev3dev.power_supply
    ev3dev.button
    ev3dev.sound
    ev3dev.lcd
    ev3dev.LCD

.. automodule:: ev3dev

Generic device
^^^^^^^^^^^^^^

.. autoclass:: device
    :members:

Motors
^^^^^^

.. autoclass:: motor
    :members:

.. autoclass:: medium_motor
    :members:
    :show-inheritance:

.. autoclass:: large_motor
    :members:
    :show-inheritance:

.. autoclass:: dc_motor
    :members:

.. autoclass:: servo_motor
    :members:

.. autofunction:: steering

Sensors
^^^^^^^

.. autoclass:: sensor
    :members:

.. autoclass:: i2c_sensor
    :members:
    :show-inheritance:

.. autoclass:: touch_sensor
    :members:
    :show-inheritance:

.. autoclass:: color_sensor
    :members:
    :show-inheritance:

.. autoclass:: ultrasonic_sensor
    :members:
    :show-inheritance:

.. autoclass:: gyro_sensor
    :members:
    :show-inheritance:

.. autoclass:: sound_sensor
    :members:
    :show-inheritance:

.. autoclass:: light_sensor
    :members:
    :show-inheritance:

.. autoclass:: infrared_sensor
    :members:
    :show-inheritance:

.. autoclass:: remote_control
    :members:

Other
^^^^^

.. autoclass:: led
    :members:

.. autoclass:: power_supply
    :members:

.. autoclass:: button
    :members:

.. autoclass:: sound
    :members:

.. autoclass:: lcd
    :members:

.. autoclass:: LCD
    :members:
    :show-inheritance:

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

