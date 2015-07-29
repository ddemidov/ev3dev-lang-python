from setuptools import setup, Extension
from spec_version import spec_version
from git_version import git_version
import os, sys, platform

pyver = sys.version_info

#----------------------------------------------------------------------------
def find_file(filename, search_dirs):
    for dirname in search_dirs:
        for root, dirs, files in os.walk(dirname):
            for f in files:
                if filename in f:
                    return dirname
            for d in dirs:
                if filename in d:
                    return dirname
            if filename in root:
                return dirname
    return False


#----------------------------------------------------------------------------
def boost_python_lib():
    library_dirs = [
            '/usr/local/lib64/',
            '/usr/lib64/',
            '/usr/local/lib/',
            '/usr/lib/',
            '/opt/local/lib/'
            ]

    boost_python = "boost_python-%s.%s" % (pyver[0], pyver[1])
    if find_file("lib" + boost_python, library_dirs):
        return boost_python

    if pyver >= (3, ):
        boost_python = "boost_python-py%s%s" % (pyver[0], pyver[1])
        if find_file("lib" + boost_python, library_dirs):
            return boost_python
        boost_python = "boost_python%s" % pyver[0]
        if find_file("lib" + boost_python, library_dirs):
            return boost_python

    return "boost_python"

# write version to version.py
open("ev3dev/version.py", "w").write("__version__='%s (%s)'\n" % (git_version(), spec_version))


#----------------------------------------------------------------------------
compile_args=['-O2', '-std=c++11']

if platform.machine() == 'armv6l':
    compile_args.append('-DEV3DEV_RPI')

setup(
        name='python-ev3dev',
        version=git_version(),
        description='Python language bindings for ev3dev',
        author='Denis Demidov',
        author_email='dennis.demidov@gmail.com',
        license='GPLv2',
        url='https://github.com/ddemidov/evdev-lang-python',
        include_package_data=True,
        packages=['ev3dev'],
        zip_safe=False,
        ext_modules=[
            Extension('ev3dev_ext',
                language='c++',
                sources=['ev3dev/pyev3dev.cpp', 'ev3dev/ev3dev.cpp'],
                include_dirs=['ev3dev'],
                libraries=[boost_python_lib()],
                extra_compile_args=compile_args
                )
            ]
)

