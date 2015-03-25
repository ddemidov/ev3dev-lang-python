from setuptools import setup, Extension
from subprocess import Popen, PIPE
import os, sys

pyver = sys.version_info

#----------------------------------------------------------------------------
# Get version string from git
#
# Author: Douglas Creager <dcreager@dcreager.net>
# http://dcreager.net/2010/02/10/setuptools-git-version-numbers/
#
# PEP 386 adaptation from
# https://gist.github.com/ilogue/2567778/f6661ea2c12c070851b2dfb4da8840a6641914bc
#----------------------------------------------------------------------------
def call_git_describe(abbrev=4):
    try:
        p = Popen(['git', 'describe', '--abbrev=%d' % abbrev],
                  stdout=PIPE, stderr=PIPE)
        p.stderr.close()
        line = p.stdout.readlines()[0]
        return line.strip().decode('utf8')

    except:
        return None


def pep386adapt(version):
    # adapt git-describe version to be in line with PEP 386
    parts = version.split('-')
    if len(parts) > 1:
        parts[-2] = 'post'+parts[-2]
        version = '.'.join(parts[:-1])
    return version


def get_git_version(abbrev=4):
    # Try to get the current version using "git describe".
    version = call_git_describe(abbrev)

    if version is None:
        raise ValueError("Cannot find the version number!")

    #adapt to PEP 386 compatible versioning scheme
    version = pep386adapt(version)

    # Finally, return the current version.
    return version


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


#----------------------------------------------------------------------------
setup(
        name='python-ev3dev',
        version=get_git_version(),
        description='Python language bindings for ev3dev',
        author='Denis Demidov',
        author_email='dennis.demidov@gmail.com',
        license='GPLv2',
        url='https://github.com/ddemidov/evdev-lang-python',
        include_package_data=True,
        packages=['ev3dev_utils'],
        ext_modules=[
            Extension('ev3dev',
                language='c++',
                sources=['pyev3dev.cpp', 'ev3dev.cpp'],
                include_dirs=['.'],
                libraries=[boost_python_lib()],
                extra_compile_args=['-O2', '-std=c++11']
                )
            ]
)

