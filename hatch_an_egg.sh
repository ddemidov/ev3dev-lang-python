#!/bin/bash

cp ../cpp/ev3dev.{cpp,h} .

if [ "$1" == "upload" ]; then
    # Create a binary egg and upload it to pypi
    python setup.py bdist_egg -p linux-armv5tejl upload
else
    # Install locally
    python setup.py install
fi

rm ev3dev.{cpp,h}
