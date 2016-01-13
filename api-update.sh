#!/bin/sh

if [ -n "$1" ]; then
    FILES_TO_CONVERT="$@"
else
    FILES_TO_CONVERT="$(find . -name '*.py')"
fi

for f in $FILES_TO_CONVERT; do
    perl -i -0 \
    -pe 's/color_sensor([.(])/ColorSensor$1/' \
    -pe 's/gyro_sensor([.(])/GyroSensor$1/' \
    -pe 's/i2c_sensor([.(])/I2cSensor$1/' \
    -pe 's/infrared_sensor([.(])/InfraredSensor$1/' \
    -pe 's/light_sensor([.(])/LightSensor$1/' \
    -pe 's/sound_sensor([.(])/SoundSensor$1/' \
    -pe 's/touch_sensor([.(])/TouchSensor$1/' \
    -pe 's/ultrasonic_sensor([.(])/UltraSonicSensor$1/' \
    -pe 's/sensor([.(])/Sensor$1/' \
\
    -pe 's/dc_motor([.(])/DCMotor$1/' \
    -pe 's/large_motor([.(])/LargeMotor$1/' \
    -pe 's/medium_motor([.(])/MediumMotor$1/' \
    -pe 's/servo_motor([.(])/ServoMotor$1/' \
    -pe 's/motor([.(])/Motor$1/' \
\
    -pe 's/device([.(])/Device$1/' \
    -pe 's/led([.(])/Led$1/' \
    -pe 's/lego_port([.(])/LegoPort$1/' \
    -pe 's/power_supply([.(])/PowerSupply$1/' \
    $f
done

# missing in pure-python: button, LCD/lcd, remote_control
# to be checked: new api in pure-python: Led_*
