cmake_minimum_required(VERSION 2.8)
project(pyev3dev)

find_package(PythonInterp)
find_package(PythonLibs ${PYTHON_VERSION_STRING})

if (PYTHON_VERSION_MAJOR EQUAL 3)
    set(BOOST_PYTHON_COMPONENT python3 CACHE STRING "Boost.Python library name")
else()
    set(BOOST_PYTHON_COMPONENT python CACHE STRING "Boost.Python library name")
endif()

find_package(Boost COMPONENTS ${BOOST_PYTHON_COMPONENT})

include_directories(
    ${Boost_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/../cpp
    )

add_library(ev3dev SHARED
    pyev3dev.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../cpp/ev3dev.cpp
    )
target_link_libraries(ev3dev ${Boost_LIBRARIES})
set_target_properties(ev3dev PROPERTIES PREFIX "" COMPILE_FLAGS "-std=c++11")

