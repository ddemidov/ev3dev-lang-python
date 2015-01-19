# Python language bindings for ev3dev

This is a python library implementing unified interface for [ev3dev][] devices.

## Installation

* Prerequisites:
```
apt-get install libboost-python-dev python-setuptools
```

* Now, the actual module installation should be as easy as
```
easy_install python-ev3dev
```
And it does work on the brickstrap image! But on the brick itself it results in an error (see #1). The following also gives an error, but seems to manage the installation nevertheless:
```
easy_install https://pypi.python.org/packages/2.7/p/python-ev3dev/python_ev3dev-0.0.1-py2.7-linux-armv7l.egg#md5=2f295fc519f7d03f23cebc3b36bb4444
```

## Usage

See the provided examples (`demo-*.py`).

[ev3dev]: http://ev3dev.org
