#!/bin/bash

cp ../cpp/ev3dev.{cpp,h} ev3dev

target="$1"
shift

case "${target}" in
src) python setup.py sdist $*
     ;;
ev3) python2 setup.py bdist_egg -p linux-armv5tejl $*
     python3 setup.py bdist_egg -p linux-armv5tejl $*
     ;;
rpi) python2 setup.py bdist_egg $*
     python3 setup.py bdist_egg $*
     ;;
esac

rm ev3dev/ev3dev.{cpp,h}
