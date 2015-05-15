#!/bin/bash

cp ../cpp/ev3dev.{cpp,h} ev3dev

# Create a binary egg
python setup.py bdist_egg -p linux-armv5tejl $*

rm ev3dev/ev3dev.{cpp,h}
