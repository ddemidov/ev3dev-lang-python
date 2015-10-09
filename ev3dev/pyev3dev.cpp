#include <boost/python.hpp>
#include <boost/python/scope.hpp>
#include <boost/python/extract.hpp>
#include <boost/python/raw_function.hpp>
#include <boost/python/stl_iterator.hpp>
#include <boost/python/implicit.hpp>
#include <ev3dev.h>
#include <iostream>

//~autogen autogen-header
    // Sections of the following code were auto-generated based on spec v0.9.3-pre, rev 2. 
//~autogen

//---------------------------------------------------------------------------
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(device_get_attr_set_ovr, get_attr_set, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(sensor_value_ovr, value, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(sensor_float_value_ovr, float_value, 0, 1)

//---------------------------------------------------------------------------
// Connect generic device. Only port_name matching is supported.
// Need this function since there is no easy way to expose std::set to python
//---------------------------------------------------------------------------
bool device_connect(boost::python::tuple t, boost::python::dict d) {
    using namespace boost::python;

    std::map<std::string, std::set<std::string>> match;

    for(stl_input_iterator<tuple> arg(d.items()), end; arg != end; ++arg) {
        std::string key = extract<std::string>((*arg)[0]);
        extract<list> l((*arg)[1]);

        if (l.check()) {
            for(int i = 0, n = len(l()); i < n; ++i)
                match[key].insert(extract<std::string>(l()[i]));
        } else {
            match[key].insert(extract<std::string>((*arg)[1]));
        }

    }

    ev3dev::device& dev     = extract<ev3dev::device&>(t[0]);
    std::string     dir     = extract<std::string>(t[1]);
    std::string     pattern = extract<std::string>(t[2]);

    return dev.connect(dir, pattern, match);
}

//---------------------------------------------------------------------------
// Functions below allow python to iterate through std::set<std::string>
//---------------------------------------------------------------------------
typedef ev3dev::mode_set::const_iterator mode_set_iterator;

bool mode_set_contains(const ev3dev::mode_set *ms, const std::string &v) {
    return ms->count(v);
}

mode_set_iterator begin(const ev3dev::mode_set *ms) {
    return ms->cbegin();
}

mode_set_iterator end(const ev3dev::mode_set *ms) {
    return ms->cend();
}

//---------------------------------------------------------------------------
// The following functions are needed because Boost.Python has problems with
// methods imported from privately inherited classes.
//---------------------------------------------------------------------------
template <class Derived>
bool device_connected(const Derived *d) {
    return d->connected();
}

template <class Derived>
int device_device_index(const Derived *s) {
    return s->device_index();
}

//---------------------------------------------------------------------------
// Dummy getter method for write-only properties
//---------------------------------------------------------------------------
template <class T>
void no_getter(const T&) {
    throw std::runtime_error("Unreadable attribute");
}

//---------------------------------------------------------------------------
// Global interpreter lock holder
//---------------------------------------------------------------------------
struct HoldGIL {
    PyGILState_STATE state;
    HoldGIL() : state(PyGILState_Ensure()) {}
    ~HoldGIL() { PyGILState_Release(state); }
};

//---------------------------------------------------------------------------
// Remote control event processing functions
//---------------------------------------------------------------------------
void rc_on_red_up(ev3dev::remote_control *rc, PyObject *f) {
    Py_INCREF(f);
    rc->on_red_up = [f](bool pressed) {
        HoldGIL lock;
        boost::python::call<void>(f, pressed);
    };
}

void rc_on_red_down(ev3dev::remote_control *rc, PyObject *f) {
    Py_INCREF(f);
    rc->on_red_down = [f](bool pressed) {
        HoldGIL lock;
        boost::python::call<void>(f, pressed);
    };
}

void rc_on_blue_up(ev3dev::remote_control *rc, PyObject *f) {
    Py_INCREF(f);
    rc->on_blue_up = [f](bool pressed) {
        HoldGIL lock;
        boost::python::call<void>(f, pressed);
    };
}

void rc_on_blue_down(ev3dev::remote_control *rc, PyObject *f) {
    Py_INCREF(f);
    rc->on_blue_down = [f](bool pressed) {
        HoldGIL lock;
        boost::python::call<void>(f, pressed);
    };
}

void rc_on_beacon(ev3dev::remote_control *rc, PyObject *f) {
    Py_INCREF(f);
    rc->on_beacon = [f](bool pressed) {
        HoldGIL lock;
        boost::python::call<void>(f, pressed);
    };
}

void rc_on_state_change(ev3dev::remote_control *rc, PyObject *f) {
    Py_INCREF(f);
    rc->on_state_change = [f](int s) {
        HoldGIL lock;
        boost::python::call<void>(f, s);
    };
}

//---------------------------------------------------------------------------
// Button event processing functions
//---------------------------------------------------------------------------
void button_onclick(ev3dev::button *btn, PyObject *f) {
    Py_INCREF(f);
    btn->onclick = [f](bool pressed) {
        HoldGIL lock;
        boost::python::call<void>(f, pressed);
    };
}

//---------------------------------------------------------------------------
// Wraps LCD's frame buffer into Python object
//---------------------------------------------------------------------------
boost::python::object lcd_frame_buffer(ev3dev::lcd *lcd) {
    using namespace boost::python;

#if PY_MAJOR_VERSION < 3
    PyObject* py_buf = PyBuffer_FromReadWriteMemory(
            lcd->frame_buffer(),
            lcd->frame_buffer_size()
            );
#else
    PyObject* py_buf = PyMemoryView_FromMemory(
            reinterpret_cast<char*>(lcd->frame_buffer()),
            lcd->frame_buffer_size(),
            PyBUF_WRITE
            );
#endif

    return object(handle<>(py_buf));
}

//---------------------------------------------------------------------------
// Wrap char buffer returned by sensor::bin_data() into Python object
//---------------------------------------------------------------------------
boost::python::object sensor_bin_data(const ev3dev::sensor &s) {
  using namespace boost::python;

  const std::vector<char> &buf = s.bin_data();

#if PY_MAJOR_VERSION < 3
    PyObject* py_buf = PyBuffer_FromMemory(
            const_cast<char*>(buf.data()), buf.size()
            );
#else
    PyObject* py_buf = PyMemoryView_FromMemory(
            const_cast<char*>(buf.data()), buf.size(), PyBUF_READ
            );
#endif

    return object(handle<>(py_buf));
}

//---------------------------------------------------------------------------
// A return policy that just drops return value
//---------------------------------------------------------------------------
struct drop_return_value : boost::python::return_value_policy<boost::python::copy_const_reference>
{
    typedef boost::python::detail::return_none result_converter;
};

//---------------------------------------------------------------------------
// The module interface
//---------------------------------------------------------------------------
BOOST_PYTHON_MODULE(ev3dev_ext)
{
    using namespace boost::python;
    namespace ev3 = ev3dev;

    docstring_options options(true, true, false);

    PyEval_InitThreads();

    implicitly_convertible<float,int>();

    scope().attr("INPUT_AUTO") = ev3::INPUT_AUTO;
    scope().attr("INPUT_1")    = ev3::INPUT_1;
    scope().attr("INPUT_2")    = ev3::INPUT_2;
    scope().attr("INPUT_3")    = ev3::INPUT_3;
    scope().attr("INPUT_4")    = ev3::INPUT_4;

    scope().attr("OUTPUT_AUTO") = ev3::OUTPUT_AUTO;
    scope().attr("OUTPUT_A")    = ev3::OUTPUT_A;
    scope().attr("OUTPUT_B")    = ev3::OUTPUT_B;
    scope().attr("OUTPUT_C")    = ev3::OUTPUT_C;
    scope().attr("OUTPUT_D")    = ev3::OUTPUT_D;

    class_<ev3::mode_set>("mode_set", "List of strings")
        .def("__len__",      &ev3::mode_set::size)
        .def("__contains__", mode_set_contains)
        .def("__iter__",     iterator<ev3::mode_set>())
        .def("size",         &ev3::mode_set::size)
        .def("empty",        &ev3::mode_set::empty)
        .def("count",        &ev3::mode_set::count)
        ;

    //-----------------------------------------------------------------------
    // Generic device
    //-----------------------------------------------------------------------
    {
        class_<ev3::device>("device", "Generic device")
            .add_property("connected",    &ev3::device::connected)
            .add_property("device_index", &ev3::device::device_index)
            .def("connect", raw_function(device_connect),
                    "connect(path, pattern, attr1=val1, ...)\n\n"
                    "Tries to connect the device with the provided parameters\n"
                    "\n"
                    "Parameters:\n"
                    "\n"
                    "    path: path in sysfs to search device node under.\n"
                    "        Example: '/sys/class/tacho-motor/'\n"
                    "\n"
                    "    pattern: device name pattern.\n"
                    "        Example: 'motor'\n"
                    "\n"
                    "    keyword arguments: values of device attributes to match\n"
                    "        while looking for the device\n"
                    "\n"
                    "Example (connect large EV3 motor)::\n\n"
                    "    d = device()\n"
                    "    d.connect('/sys/class/tacho-motor/', 'motor', driver_name='lego-ev3-l-motor')\n"
                    )
            .def("get_attr_int",      &ev3::device::get_attr_int,
                    "Read integer attribute\n"
                )
            .def("set_attr_int",      &ev3::device::set_attr_int,
                    "Write integer attribute\n"
                )
            .def("get_attr_string",   &ev3::device::get_attr_string,
                    "Read string attribute\n"
                )
            .def("set_attr_string",   &ev3::device::set_attr_string,
                    "Write string attribute\n"
                )
            .def("get_attr_line",     &ev3::device::get_attr_line,
                    "Read line attribute\n"
                )
            .def("get_attr_set",      &ev3::device::get_attr_set,
                    device_get_attr_set_ovr(args("name"), "Read set attribute\n")
                )
            .def("get_attr_from_set", &ev3::device::get_attr_from_set,
                    "Read current mode from set attribute\n"
                )
            ;
    }

    //-----------------------------------------------------------------------
    // Sensors
    //-----------------------------------------------------------------------
    {
        scope s = class_<ev3::sensor>("sensor",
//~autogen python_generic-class-description classes.sensor>currentClass

                "The sensor class provides a uniform interface for using most of the\n"
                "sensors available for the EV3. The various underlying device drivers will\n"
                "create a `lego-sensor` device for interacting with the sensors.\n"
                "\n"
                "Sensors are primarily controlled by setting the `mode` and monitored by\n"
                "reading the `value<N>` attributes. Values can be converted to floating point\n"
                "if needed by `value<N>` / 10.0 ^ `decimals`.\n"
                "\n"
                "Since the name of the `sensor<N>` device node does not correspond to the port\n"
                "that a sensor is plugged in to, you must look at the `port_name` attribute if\n"
                "you need to know which port a sensor is plugged in to. However, if you don't\n"
                "have more than one sensor of each type, you can just look for a matching\n"
                "`driver_name`. Then it will not matter which port a sensor is plugged in to - your\n"
                "program will still work.\n"

//~autogen
                 , init<ev3::port_type>(args("port")))
            .add_property("connected", device_connected<ev3::sensor>)
            .add_property("device_index", device_device_index<ev3::sensor>)
            .def("value",        &ev3::sensor::value,
                    sensor_value_ovr(args("index=0"), "Reads unscaled sensor value"))
            .def("float_value",  &ev3::sensor::float_value,
                    sensor_float_value_ovr(args("index=0"), "Reads scaled sensor value"))
            .add_property("bin_data_format", &ev3::sensor::bin_data_format,
                    "Bin Data Format: read-only\n"
                    "Returns the format of the values in `bin_data` for the current mode.\n"
                    "Possible values are:\n"
                    "\n"
                    "   - `u8`: Unsigned 8-bit integer (byte)\n"
                    "   - `s8`: Signed 8-bit integer (sbyte)\n"
                    "   - `u16`: Unsigned 16-bit integer (ushort)\n"
                    "   - `s16`: Signed 16-bit integer (short)\n"
                    "   - `s16_be`: Signed 16-bit integer, big endian\n"
                    "   - `s32`: Signed 32-bit integer (int)\n"
                    "   - `float`: IEEE 754 32-bit floating point (float)\n"
                    )
            .add_property("bin_data_raw", sensor_bin_data,
                    "Bin Data: read-only\n"
                    "Returns the unscaled raw values in the `value<N>` attributes as raw byte\n"
                    "array. Use `bin_data_format`, `num_values` and the individual sensor\n"
                    "documentation to determine how to interpret the data.\n"
                    )
//~autogen python_generic-get-set classes.sensor>currentClass

            .add_property("command", no_getter<ev3::sensor>, make_function(&ev3::sensor::set_command, drop_return_value()),
                    "Command: write-only\n\n"
                    "Sends a command to the sensor.\n"
                    )
            .add_property("commands", &ev3::sensor::commands,
                    "Commands: read-only\n\n"
                    "Returns a list of the valid commands for the sensor.\n"
                    "Returns -EOPNOTSUPP if no commands are supported.\n"
                    )
            .add_property("decimals", &ev3::sensor::decimals,
                    "Decimals: read-only\n\n"
                    "Returns the number of decimal places for the values in the `value<N>`\n"
                    "attributes of the current mode.\n"
                    )
            .add_property("driver_name", &ev3::sensor::driver_name,
                    "Driver Name: read-only\n\n"
                    "Returns the name of the sensor device/driver. See the list of [supported\n"
                    "sensors] for a complete list of drivers.\n"
                    )
            .add_property("mode", &ev3::sensor::mode, make_function(&ev3::sensor::set_mode, drop_return_value()),
                    "Mode: read/write\n\n"
                    "Returns the current mode. Writing one of the values returned by `modes`\n"
                    "sets the sensor to that mode.\n"
                    )
            .add_property("modes", &ev3::sensor::modes,
                    "Modes: read-only\n\n"
                    "Returns a list of the valid modes for the sensor.\n"
                    )
            .add_property("num_values", &ev3::sensor::num_values,
                    "Num Values: read-only\n\n"
                    "Returns the number of `value<N>` attributes that will return a valid value\n"
                    "for the current mode.\n"
                    )
            .add_property("port_name", &ev3::sensor::port_name,
                    "Port Name: read-only\n\n"
                    "Returns the name of the port that the sensor is connected to, e.g. `ev3:in1`.\n"
                    "I2C sensors also include the I2C address (decimal), e.g. `ev3:in1:i2c8`.\n"
                    )
            .add_property("units", &ev3::sensor::units,
                    "Units: read-only\n\n"
                    "Returns the units of the measured value for the current mode. May return\n"
                    "empty string\n"
                    )

//~autogen
            ;

        s.attr("ev3_touch")      = ev3::sensor::ev3_touch;
        s.attr("ev3_color")      = ev3::sensor::ev3_color;
        s.attr("ev3_ultrasonic") = ev3::sensor::ev3_ultrasonic;
        s.attr("ev3_gyro")       = ev3::sensor::ev3_gyro;
        s.attr("ev3_infrared")   = ev3::sensor::ev3_infrared;

        s.attr("nxt_touch")      = ev3::sensor::nxt_touch;
        s.attr("nxt_light")      = ev3::sensor::nxt_light;
        s.attr("nxt_sound")      = ev3::sensor::nxt_sound;
        s.attr("nxt_ultrasonic") = ev3::sensor::nxt_ultrasonic;
        s.attr("nxt_i2c_sensor") = ev3::sensor::nxt_i2c_sensor;
        s.attr("nxt_analog")     = ev3::sensor::nxt_analog;
    }

    {
        class_<ev3::i2c_sensor, bases<ev3::sensor>>("i2c_sensor",
//~autogen python_generic-class-description classes.i2cSensor>currentClass

                "A generic interface to control I2C-type EV3 sensors.\n"

//~autogen
                 , init<>())
            .def(init<ev3::port_type>(args("port")))
            .def(init<ev3::port_type, ev3::address_type>(args("port", "address")))
//~autogen python_generic-get-set classes.i2cSensor>currentClass

            .add_property("fw_version", &ev3::i2c_sensor::fw_version,
                    "FW Version: read-only\n\n"
                    "Returns the firmware version of the sensor if available. Currently only\n"
                    "I2C/NXT sensors support this.\n"
                    )
            .add_property("poll_ms", &ev3::i2c_sensor::poll_ms, make_function(&ev3::i2c_sensor::set_poll_ms, drop_return_value()),
                    "Poll MS: read/write\n\n"
                    "Returns the polling period of the sensor in milliseconds. Writing sets the\n"
                    "polling period. Setting to 0 disables polling. Minimum value is hard\n"
                    "coded as 50 msec. Returns -EOPNOTSUPP if changing polling is not supported.\n"
                    "Currently only I2C/NXT sensors support changing the polling period.\n"
                    )

//~autogen
            ;
    }

    {
        class_<ev3::touch_sensor, bases<ev3::sensor>>("touch_sensor", "Touch sensor", init<>())
            .def(init<ev3::port_type>(args("port")))
            ;
    }

    {
        scope s = class_<ev3::color_sensor, bases<ev3::sensor>>(
                "color_sensor",
//~autogen python_generic-class-description classes.colorSensor>currentClass

                "LEGO EV3 color sensor.\n"

//~autogen
                 , init<>())
            .def(init<ev3::port_type>(args("port")))
            ;

//~autogen python_generic-property-value classes.colorSensor>currentClass

        s.attr("mode_col_reflect") = ev3::color_sensor::mode_col_reflect;
        s.attr("mode_col_ambient") = ev3::color_sensor::mode_col_ambient;
        s.attr("mode_col_color") = ev3::color_sensor::mode_col_color;
        s.attr("mode_ref_raw") = ev3::color_sensor::mode_ref_raw;
        s.attr("mode_rgb_raw") = ev3::color_sensor::mode_rgb_raw;

//~autogen
    }

    {
        scope s = class_<ev3::ultrasonic_sensor, bases<ev3::sensor>>(
                "ultrasonic_sensor",
//~autogen python_generic-class-description classes.ultrasonicSensor>currentClass

                "LEGO EV3 ultrasonic sensor.\n"

//~autogen
                , init<>())
            .def(init<ev3::port_type>(args("port")))
            ;

//~autogen python_generic-property-value classes.ultrasonicSensor>currentClass

        s.attr("mode_us_dist_cm") = ev3::ultrasonic_sensor::mode_us_dist_cm;
        s.attr("mode_us_dist_in") = ev3::ultrasonic_sensor::mode_us_dist_in;
        s.attr("mode_us_listen") = ev3::ultrasonic_sensor::mode_us_listen;
        s.attr("mode_us_si_cm") = ev3::ultrasonic_sensor::mode_us_si_cm;
        s.attr("mode_us_si_in") = ev3::ultrasonic_sensor::mode_us_si_in;

//~autogen
    }

    {
        scope s = class_<ev3::gyro_sensor, bases<ev3::sensor>>(
                "gyro_sensor",
//~autogen python_generic-class-description classes.gyroSensor>currentClass

                "LEGO EV3 gyro sensor.\n"

//~autogen
                , init<>())
            .def(init<ev3::port_type>(args("port")))
            ;

//~autogen python_generic-property-value classes.gyroSensor>currentClass

        s.attr("mode_gyro_ang") = ev3::gyro_sensor::mode_gyro_ang;
        s.attr("mode_gyro_rate") = ev3::gyro_sensor::mode_gyro_rate;
        s.attr("mode_gyro_fas") = ev3::gyro_sensor::mode_gyro_fas;
        s.attr("mode_gyro_g_a") = ev3::gyro_sensor::mode_gyro_g_a;
        s.attr("mode_gyro_cal") = ev3::gyro_sensor::mode_gyro_cal;

//~autogen
    }

    {
        scope s = class_<ev3::infrared_sensor, bases<ev3::sensor>>(
                "infrared_sensor",
//~autogen python_generic-class-description classes.infraredSensor>currentClass

                "LEGO EV3 infrared sensor.\n"

//~autogen
                , init<>())
            .def(init<ev3::port_type>(args("port")))
            ;

//~autogen python_generic-property-value classes.infraredSensor>currentClass

        s.attr("mode_ir_prox") = ev3::infrared_sensor::mode_ir_prox;
        s.attr("mode_ir_seek") = ev3::infrared_sensor::mode_ir_seek;
        s.attr("mode_ir_remote") = ev3::infrared_sensor::mode_ir_remote;
        s.attr("mode_ir_rem_a") = ev3::infrared_sensor::mode_ir_rem_a;
        s.attr("mode_ir_cal") = ev3::infrared_sensor::mode_ir_cal;

//~autogen
    }

    {
        scope s = class_<ev3::sound_sensor, bases<ev3::sensor>>(
                "sound_sensor",
//~autogen python_generic-class-description classes.soundSensor>currentClass

                "LEGO NXT Sound Sensor\n"

//~autogen
                , init<>())
            .def(init<ev3::port_type>(args("port")))
            ;

//~autogen python_generic-property-value classes.soundSensor>currentClass

        s.attr("mode_db") = ev3::sound_sensor::mode_db;
        s.attr("mode_dba") = ev3::sound_sensor::mode_dba;

//~autogen
    }

    {
        scope s = class_<ev3::light_sensor, bases<ev3::sensor>>(
                "light_sensor",
//~autogen python_generic-class-description classes.lightSensor>currentClass

                "LEGO NXT Light Sensor\n"

//~autogen
                , init<>())
            .def(init<ev3::port_type>(args("port")))
            ;

//~autogen python_generic-property-value classes.lightSensor>currentClass

        s.attr("mode_reflect") = ev3::light_sensor::mode_reflect;
        s.attr("mode_ambient") = ev3::light_sensor::mode_ambient;

//~autogen
    }

    //-----------------------------------------------------------------------
    // Motors
    //-----------------------------------------------------------------------
    {
        scope s = class_<ev3::motor>("motor",
//~autogen python_generic-class-description classes.motor>currentClass

                "The motor class provides a uniform interface for using motors with\n"
                "positional and directional feedback such as the EV3 and NXT motors.\n"
                "This feedback allows for precise control of the motors. This is the\n"
                "most common type of motor, so we just call it `motor`.\n"

//~autogen
                , init<ev3::port_type>(args("port")))
            .def(init<ev3::port_type, ev3::motor::motor_type>(args("port", "driver")))
            .add_property("connected",         device_connected<ev3::motor>)
            .add_property("device_index",      device_device_index<ev3::motor>)
//~autogen python_generic-get-set classes.motor>currentClass

            .add_property("command", no_getter<ev3::motor>, make_function(&ev3::motor::set_command, drop_return_value()),
                    "Command: write-only\n\n"
                    "Sends a command to the motor controller. See `commands` for a list of\n"
                    "possible values.\n"
                    )
            .add_property("commands", &ev3::motor::commands,
                    "Commands: read-only\n\n"
                    "Returns a list of commands that are supported by the motor\n"
                    "controller. Possible values are `run-forever`, `run-to-abs-pos`, `run-to-rel-pos`,\n"
                    "`run-timed`, `run-direct`, `stop` and `reset`. Not all commands may be supported.\n"
                    "`run-forever` will cause the motor to run until another command is sent.\n"
                    "`run-to-abs-pos` will run to an absolute position specified by `position_sp`\n"
                    "and then stop using the command specified in `stop_command`.\n"
                    "`run-to-rel-pos` will run to a position relative to the current `position` value.\n"
                    "The new position will be current `position` + `position_sp`. When the new\n"
                    "position is reached, the motor will stop using the command specified by `stop_command`.\n"
                    "`run-timed` will run the motor for the amount of time specified in `time_sp`\n"
                    "and then stop the motor using the command specified by `stop_command`.\n"
                    "`run-direct` will run the motor at the duty cycle specified by `duty_cycle_sp`.\n"
                    "Unlike other run commands, changing `duty_cycle_sp` while running *will*\n"
                    "take effect immediately.\n"
                    "`stop` will stop any of the run commands before they are complete using the\n"
                    "command specified by `stop_command`.\n"
                    "`reset` will reset all of the motor parameter attributes to their default value.\n"
                    "This will also have the effect of stopping the motor.\n"
                    )
            .add_property("count_per_rot", &ev3::motor::count_per_rot,
                    "Count Per Rot: read-only\n\n"
                    "Returns the number of tacho counts in one rotation of the motor. Tacho counts\n"
                    "are used by the position and speed attributes, so you can use this value\n"
                    "to convert rotations or degrees to tacho counts. In the case of linear\n"
                    "actuators, the units here will be counts per centimeter.\n"
                    )
            .add_property("driver_name", &ev3::motor::driver_name,
                    "Driver Name: read-only\n\n"
                    "Returns the name of the driver that provides this tacho motor device.\n"
                    )
            .add_property("duty_cycle", &ev3::motor::duty_cycle,
                    "Duty Cycle: read-only\n\n"
                    "Returns the current duty cycle of the motor. Units are percent. Values\n"
                    "are -100 to 100.\n"
                    )
            .add_property("duty_cycle_sp", &ev3::motor::duty_cycle_sp, make_function(&ev3::motor::set_duty_cycle_sp, drop_return_value()),
                    "Duty Cycle SP: read/write\n\n"
                    "Writing sets the duty cycle setpoint. Reading returns the current value.\n"
                    "Units are in percent. Valid values are -100 to 100. A negative value causes\n"
                    "the motor to rotate in reverse. This value is only used when `speed_regulation`\n"
                    "is off.\n"
                    )
            .add_property("encoder_polarity", &ev3::motor::encoder_polarity, make_function(&ev3::motor::set_encoder_polarity, drop_return_value()),
                    "Encoder Polarity: read/write\n\n"
                    "Sets the polarity of the rotary encoder. This is an advanced feature to all\n"
                    "use of motors that send inversed encoder signals to the EV3. This should\n"
                    "be set correctly by the driver of a device. It You only need to change this\n"
                    "value if you are using a unsupported device. Valid values are `normal` and\n"
                    "`inversed`.\n"
                    )
            .add_property("polarity", &ev3::motor::polarity, make_function(&ev3::motor::set_polarity, drop_return_value()),
                    "Polarity: read/write\n\n"
                    "Sets the polarity of the motor. With `normal` polarity, a positive duty\n"
                    "cycle will cause the motor to rotate clockwise. With `inversed` polarity,\n"
                    "a positive duty cycle will cause the motor to rotate counter-clockwise.\n"
                    "Valid values are `normal` and `inversed`.\n"
                    )
            .add_property("port_name", &ev3::motor::port_name,
                    "Port Name: read-only\n\n"
                    "Returns the name of the port that the motor is connected to.\n"
                    )
            .add_property("position", &ev3::motor::position, make_function(&ev3::motor::set_position, drop_return_value()),
                    "Position: read/write\n\n"
                    "Returns the current position of the motor in pulses of the rotary\n"
                    "encoder. When the motor rotates clockwise, the position will increase.\n"
                    "Likewise, rotating counter-clockwise causes the position to decrease.\n"
                    "Writing will set the position to that value.\n"
                    )
            .add_property("position_p", &ev3::motor::position_p, make_function(&ev3::motor::set_position_p, drop_return_value()),
                    "Position P: read/write\n\n"
                    "The proportional constant for the position PID.\n"
                    )
            .add_property("position_i", &ev3::motor::position_i, make_function(&ev3::motor::set_position_i, drop_return_value()),
                    "Position I: read/write\n\n"
                    "The integral constant for the position PID.\n"
                    )
            .add_property("position_d", &ev3::motor::position_d, make_function(&ev3::motor::set_position_d, drop_return_value()),
                    "Position D: read/write\n\n"
                    "The derivative constant for the position PID.\n"
                    )
            .add_property("position_sp", &ev3::motor::position_sp, make_function(&ev3::motor::set_position_sp, drop_return_value()),
                    "Position SP: read/write\n\n"
                    "Writing specifies the target position for the `run-to-abs-pos` and `run-to-rel-pos`\n"
                    "commands. Reading returns the current value. Units are in tacho counts. You\n"
                    "can use the value returned by `counts_per_rot` to convert tacho counts to/from\n"
                    "rotations or degrees.\n"
                    )
            .add_property("speed", &ev3::motor::speed,
                    "Speed: read-only\n\n"
                    "Returns the current motor speed in tacho counts per second. Not, this is\n"
                    "not necessarily degrees (although it is for LEGO motors). Use the `count_per_rot`\n"
                    "attribute to convert this value to RPM or deg/sec.\n"
                    )
            .add_property("speed_sp", &ev3::motor::speed_sp, make_function(&ev3::motor::set_speed_sp, drop_return_value()),
                    "Speed SP: read/write\n\n"
                    "Writing sets the target speed in tacho counts per second used when `speed_regulation`\n"
                    "is on. Reading returns the current value.  Use the `count_per_rot` attribute\n"
                    "to convert RPM or deg/sec to tacho counts per second.\n"
                    )
            .add_property("ramp_up_sp", &ev3::motor::ramp_up_sp, make_function(&ev3::motor::set_ramp_up_sp, drop_return_value()),
                    "Ramp Up SP: read/write\n\n"
                    "Writing sets the ramp up setpoint. Reading returns the current value. Units\n"
                    "are in milliseconds. When set to a value > 0, the motor will ramp the power\n"
                    "sent to the motor from 0 to 100% duty cycle over the span of this setpoint\n"
                    "when starting the motor. If the maximum duty cycle is limited by `duty_cycle_sp`\n"
                    "or speed regulation, the actual ramp time duration will be less than the setpoint.\n"
                    )
            .add_property("ramp_down_sp", &ev3::motor::ramp_down_sp, make_function(&ev3::motor::set_ramp_down_sp, drop_return_value()),
                    "Ramp Down SP: read/write\n\n"
                    "Writing sets the ramp down setpoint. Reading returns the current value. Units\n"
                    "are in milliseconds. When set to a value > 0, the motor will ramp the power\n"
                    "sent to the motor from 100% duty cycle down to 0 over the span of this setpoint\n"
                    "when stopping the motor. If the starting duty cycle is less than 100%, the\n"
                    "ramp time duration will be less than the full span of the setpoint.\n"
                    )
            .add_property("speed_regulation_enabled", &ev3::motor::speed_regulation_enabled, make_function(&ev3::motor::set_speed_regulation_enabled, drop_return_value()),
                    "Speed Regulation Enabled: read/write\n\n"
                    "Turns speed regulation on or off. If speed regulation is on, the motor\n"
                    "controller will vary the power supplied to the motor to try to maintain the\n"
                    "speed specified in `speed_sp`. If speed regulation is off, the controller\n"
                    "will use the power specified in `duty_cycle_sp`. Valid values are `on` and\n"
                    "`off`.\n"
                    )
            .add_property("speed_regulation_p", &ev3::motor::speed_regulation_p, make_function(&ev3::motor::set_speed_regulation_p, drop_return_value()),
                    "Speed Regulation P: read/write\n\n"
                    "The proportional constant for the speed regulation PID.\n"
                    )
            .add_property("speed_regulation_i", &ev3::motor::speed_regulation_i, make_function(&ev3::motor::set_speed_regulation_i, drop_return_value()),
                    "Speed Regulation I: read/write\n\n"
                    "The integral constant for the speed regulation PID.\n"
                    )
            .add_property("speed_regulation_d", &ev3::motor::speed_regulation_d, make_function(&ev3::motor::set_speed_regulation_d, drop_return_value()),
                    "Speed Regulation D: read/write\n\n"
                    "The derivative constant for the speed regulation PID.\n"
                    )
            .add_property("state", &ev3::motor::state,
                    "State: read-only\n\n"
                    "Reading returns a list of state flags. Possible flags are\n"
                    "`running`, `ramping` `holding` and `stalled`.\n"
                    )
            .add_property("stop_command", &ev3::motor::stop_command, make_function(&ev3::motor::set_stop_command, drop_return_value()),
                    "Stop Command: read/write\n\n"
                    "Reading returns the current stop command. Writing sets the stop command.\n"
                    "The value determines the motors behavior when `command` is set to `stop`.\n"
                    "Also, it determines the motors behavior when a run command completes. See\n"
                    "`stop_commands` for a list of possible values.\n"
                    )
            .add_property("stop_commands", &ev3::motor::stop_commands,
                    "Stop Commands: read-only\n\n"
                    "Returns a list of stop modes supported by the motor controller.\n"
                    "Possible values are `coast`, `brake` and `hold`. `coast` means that power will\n"
                    "be removed from the motor and it will freely coast to a stop. `brake` means\n"
                    "that power will be removed from the motor and a passive electrical load will\n"
                    "be placed on the motor. This is usually done by shorting the motor terminals\n"
                    "together. This load will absorb the energy from the rotation of the motors and\n"
                    "cause the motor to stop more quickly than coasting. `hold` does not remove\n"
                    "power from the motor. Instead it actively try to hold the motor at the current\n"
                    "position. If an external force tries to turn the motor, the motor will 'push\n"
                    "back' to maintain its position.\n"
                    )
            .add_property("time_sp", &ev3::motor::time_sp, make_function(&ev3::motor::set_time_sp, drop_return_value()),
                    "Time SP: read/write\n\n"
                    "Writing specifies the amount of time the motor will run when using the\n"
                    "`run-timed` command. Reading returns the current value. Units are in\n"
                    "milliseconds.\n"
                    )

//~autogen
            ;

        s.attr("motor_large")  = ev3::motor::motor_large;
        s.attr("motor_medium") = ev3::motor::motor_medium;

//~autogen python_generic-property-value classes.motor>currentClass

        s.attr("command_run_forever") = ev3::motor::command_run_forever;
        s.attr("command_run_to_abs_pos") = ev3::motor::command_run_to_abs_pos;
        s.attr("command_run_to_rel_pos") = ev3::motor::command_run_to_rel_pos;
        s.attr("command_run_timed") = ev3::motor::command_run_timed;
        s.attr("command_run_direct") = ev3::motor::command_run_direct;
        s.attr("command_stop") = ev3::motor::command_stop;
        s.attr("command_reset") = ev3::motor::command_reset;
        s.attr("encoder_polarity_normal") = ev3::motor::encoder_polarity_normal;
        s.attr("encoder_polarity_inversed") = ev3::motor::encoder_polarity_inversed;
        s.attr("polarity_normal") = ev3::motor::polarity_normal;
        s.attr("polarity_inversed") = ev3::motor::polarity_inversed;
        s.attr("speed_regulation_on") = ev3::motor::speed_regulation_on;
        s.attr("speed_regulation_off") = ev3::motor::speed_regulation_off;
        s.attr("stop_command_coast") = ev3::motor::stop_command_coast;
        s.attr("stop_command_brake") = ev3::motor::stop_command_brake;
        s.attr("stop_command_hold") = ev3::motor::stop_command_hold;

//~autogen
    }

    {
        class_<ev3::medium_motor, bases<ev3::motor>>("medium_motor", "EV3 medium motor", init<>())
            .def(init<ev3::port_type>(args("port")))
            ;
    }

    {
        class_<ev3::large_motor, bases<ev3::motor>>("large_motor", "EV3 large motor", init<>())
            .def(init<ev3::port_type>())
            ;
    }

    {
        scope s = class_<ev3::dc_motor>("dc_motor",
//~autogen python_generic-class-description classes.dcMotor>currentClass

                "The DC motor class provides a uniform interface for using regular DC motors\n"
                "with no fancy controls or feedback. This includes LEGO MINDSTORMS RCX motors\n"
                "and LEGO Power Functions motors.\n"

//~autogen
                , init<>())
            .def(init<ev3::port_type>(args("port")))
            .add_property("connected",    device_connected<ev3::dc_motor>)
            .add_property("device_index", device_device_index<ev3::dc_motor>)
//~autogen python_generic-get-set classes.dcMotor>currentClass

            .add_property("command", no_getter<ev3::dc_motor>, make_function(&ev3::dc_motor::set_command, drop_return_value()),
                    "Command: write-only\n\n"
                    "Sets the command for the motor. Possible values are `run-forever`, `run-timed` and\n"
                    "`stop`. Not all commands may be supported, so be sure to check the contents\n"
                    "of the `commands` attribute.\n"
                    )
            .add_property("commands", &ev3::dc_motor::commands,
                    "Commands: read-only\n\n"
                    "Returns a list of commands supported by the motor\n"
                    "controller.\n"
                    )
            .add_property("driver_name", &ev3::dc_motor::driver_name,
                    "Driver Name: read-only\n\n"
                    "Returns the name of the motor driver that loaded this device. See the list\n"
                    "of [supported devices] for a list of drivers.\n"
                    )
            .add_property("duty_cycle", &ev3::dc_motor::duty_cycle,
                    "Duty Cycle: read-only\n\n"
                    "Shows the current duty cycle of the PWM signal sent to the motor. Values\n"
                    "are -100 to 100 (-100% to 100%).\n"
                    )
            .add_property("duty_cycle_sp", &ev3::dc_motor::duty_cycle_sp, make_function(&ev3::dc_motor::set_duty_cycle_sp, drop_return_value()),
                    "Duty Cycle SP: read/write\n\n"
                    "Writing sets the duty cycle setpoint of the PWM signal sent to the motor.\n"
                    "Valid values are -100 to 100 (-100% to 100%). Reading returns the current\n"
                    "setpoint.\n"
                    )
            .add_property("polarity", &ev3::dc_motor::polarity, make_function(&ev3::dc_motor::set_polarity, drop_return_value()),
                    "Polarity: read/write\n\n"
                    "Sets the polarity of the motor. Valid values are `normal` and `inversed`.\n"
                    )
            .add_property("port_name", &ev3::dc_motor::port_name,
                    "Port Name: read-only\n\n"
                    "Returns the name of the port that the motor is connected to.\n"
                    )
            .add_property("ramp_down_sp", &ev3::dc_motor::ramp_down_sp, make_function(&ev3::dc_motor::set_ramp_down_sp, drop_return_value()),
                    "Ramp Down SP: read/write\n\n"
                    "Sets the time in milliseconds that it take the motor to ramp down from 100%\n"
                    "to 0%. Valid values are 0 to 10000 (10 seconds). Default is 0.\n"
                    )
            .add_property("ramp_up_sp", &ev3::dc_motor::ramp_up_sp, make_function(&ev3::dc_motor::set_ramp_up_sp, drop_return_value()),
                    "Ramp Up SP: read/write\n\n"
                    "Sets the time in milliseconds that it take the motor to up ramp from 0% to\n"
                    "100%. Valid values are 0 to 10000 (10 seconds). Default is 0.\n"
                    )
            .add_property("state", &ev3::dc_motor::state,
                    "State: read-only\n\n"
                    "Gets a list of flags indicating the motor status. Possible\n"
                    "flags are `running` and `ramping`. `running` indicates that the motor is\n"
                    "powered. `ramping` indicates that the motor has not yet reached the\n"
                    "`duty_cycle_sp`.\n"
                    )
            .add_property("stop_command", no_getter<ev3::dc_motor>, make_function(&ev3::dc_motor::set_stop_command, drop_return_value()),
                    "Stop Command: write-only\n\n"
                    "Sets the stop command that will be used when the motor stops. Read\n"
                    "`stop_commands` to get the list of valid values.\n"
                    )
            .add_property("stop_commands", &ev3::dc_motor::stop_commands,
                    "Stop Commands: read-only\n\n"
                    "Gets a list of stop commands. Valid values are `coast`\n"
                    "and `brake`.\n"
                    )

//~autogen
            ;

//~autogen python_generic-property-value classes.dcMotor>currentClass

        s.attr("command_run_forever") = ev3::dc_motor::command_run_forever;
        s.attr("command_run_timed") = ev3::dc_motor::command_run_timed;
        s.attr("command_run_direct") = ev3::dc_motor::command_run_direct;
        s.attr("command_stop") = ev3::dc_motor::command_stop;
        s.attr("polarity_normal") = ev3::dc_motor::polarity_normal;
        s.attr("polarity_inversed") = ev3::dc_motor::polarity_inversed;
        s.attr("stop_command_coast") = ev3::dc_motor::stop_command_coast;
        s.attr("stop_command_brake") = ev3::dc_motor::stop_command_brake;

//~autogen
    }

    {
        scope s = class_<ev3::servo_motor>("servo_motor",
//~autogen python_generic-class-description classes.servoMotor>currentClass

                "The servo motor class provides a uniform interface for using hobby type\n"
                "servo motors.\n"

//~autogen
                , init<>())
            .def(init<ev3::port_type>(args("port")))
            .add_property("connected",    device_connected<ev3::servo_motor>)
            .add_property("device_index", device_device_index<ev3::servo_motor>)
//~autogen python_generic-get-set classes.servoMotor>currentClass

            .add_property("command", no_getter<ev3::servo_motor>, make_function(&ev3::servo_motor::set_command, drop_return_value()),
                    "Command: write-only\n\n"
                    "Sets the command for the servo. Valid values are `run` and `float`. Setting\n"
                    "to `run` will cause the servo to be driven to the position_sp set in the\n"
                    "`position_sp` attribute. Setting to `float` will remove power from the motor.\n"
                    )
            .add_property("driver_name", &ev3::servo_motor::driver_name,
                    "Driver Name: read-only\n\n"
                    "Returns the name of the motor driver that loaded this device. See the list\n"
                    "of [supported devices] for a list of drivers.\n"
                    )
            .add_property("max_pulse_sp", &ev3::servo_motor::max_pulse_sp, make_function(&ev3::servo_motor::set_max_pulse_sp, drop_return_value()),
                    "Max Pulse SP: read/write\n\n"
                    "Used to set the pulse size in milliseconds for the signal that tells the\n"
                    "servo to drive to the maximum (clockwise) position_sp. Default value is 2400.\n"
                    "Valid values are 2300 to 2700. You must write to the position_sp attribute for\n"
                    "changes to this attribute to take effect.\n"
                    )
            .add_property("mid_pulse_sp", &ev3::servo_motor::mid_pulse_sp, make_function(&ev3::servo_motor::set_mid_pulse_sp, drop_return_value()),
                    "Mid Pulse SP: read/write\n\n"
                    "Used to set the pulse size in milliseconds for the signal that tells the\n"
                    "servo to drive to the mid position_sp. Default value is 1500. Valid\n"
                    "values are 1300 to 1700. For example, on a 180 degree servo, this would be\n"
                    "90 degrees. On continuous rotation servo, this is the 'neutral' position_sp\n"
                    "where the motor does not turn. You must write to the position_sp attribute for\n"
                    "changes to this attribute to take effect.\n"
                    )
            .add_property("min_pulse_sp", &ev3::servo_motor::min_pulse_sp, make_function(&ev3::servo_motor::set_min_pulse_sp, drop_return_value()),
                    "Min Pulse SP: read/write\n\n"
                    "Used to set the pulse size in milliseconds for the signal that tells the\n"
                    "servo to drive to the miniumum (counter-clockwise) position_sp. Default value\n"
                    "is 600. Valid values are 300 to 700. You must write to the position_sp\n"
                    "attribute for changes to this attribute to take effect.\n"
                    )
            .add_property("polarity", &ev3::servo_motor::polarity, make_function(&ev3::servo_motor::set_polarity, drop_return_value()),
                    "Polarity: read/write\n\n"
                    "Sets the polarity of the servo. Valid values are `normal` and `inversed`.\n"
                    "Setting the value to `inversed` will cause the position_sp value to be\n"
                    "inversed. i.e `-100` will correspond to `max_pulse_sp`, and `100` will\n"
                    "correspond to `min_pulse_sp`.\n"
                    )
            .add_property("port_name", &ev3::servo_motor::port_name,
                    "Port Name: read-only\n\n"
                    "Returns the name of the port that the motor is connected to.\n"
                    )
            .add_property("position_sp", &ev3::servo_motor::position_sp, make_function(&ev3::servo_motor::set_position_sp, drop_return_value()),
                    "Position SP: read/write\n\n"
                    "Reading returns the current position_sp of the servo. Writing instructs the\n"
                    "servo to move to the specified position_sp. Units are percent. Valid values\n"
                    "are -100 to 100 (-100% to 100%) where `-100` corresponds to `min_pulse_sp`,\n"
                    "`0` corresponds to `mid_pulse_sp` and `100` corresponds to `max_pulse_sp`.\n"
                    )
            .add_property("rate_sp", &ev3::servo_motor::rate_sp, make_function(&ev3::servo_motor::set_rate_sp, drop_return_value()),
                    "Rate SP: read/write\n\n"
                    "Sets the rate_sp at which the servo travels from 0 to 100.0% (half of the full\n"
                    "range of the servo). Units are in milliseconds. Example: Setting the rate_sp\n"
                    "to 1000 means that it will take a 180 degree servo 2 second to move from 0\n"
                    "to 180 degrees. Note: Some servo controllers may not support this in which\n"
                    "case reading and writing will fail with `-EOPNOTSUPP`. In continuous rotation\n"
                    "servos, this value will affect the rate_sp at which the speed ramps up or down.\n"
                    )
            .add_property("state", &ev3::servo_motor::state,
                    "State: read-only\n\n"
                    "Returns a list of flags indicating the state of the servo.\n"
                    "Possible values are:\n"
                    "* `running`: Indicates that the motor is powered.\n"
                    )

//~autogen
            ;

//~autogen python_generic-property-value classes.servoMotor>currentClass

        s.attr("command_run") = ev3::servo_motor::command_run;
        s.attr("command_float") = ev3::servo_motor::command_float;
        s.attr("polarity_normal") = ev3::servo_motor::polarity_normal;
        s.attr("polarity_inversed") = ev3::servo_motor::polarity_inversed;

//~autogen
    }

    //-----------------------------------------------------------------------
    // LED
    //-----------------------------------------------------------------------
    {
        scope s = class_<ev3::led>("led",
//~autogen python_generic-class-description classes.led>currentClass

                "Any device controlled by the generic LED driver.\n"
                "See https://www.kernel.org/doc/Documentation/leds/leds-class.txt\n"
                "for more details.\n"

//~autogen
                , init<std::string>(args("name")))
            .add_property("connected",      device_connected<ev3::led>)
            .def("on",             &ev3::led::on)
            .def("off",            &ev3::led::off)
            .def("flash",          &ev3::led::flash, args("on_ms", "off_ms"))
            .def("mix_colors",     &ev3::led::mix_colors, args("red", "green")).staticmethod("mix_colors")
            .def("all_off",        &ev3::led::all_off).staticmethod("all_off")
//~autogen python_led-color-methods

            .def("set_red", &ev3::led::set_red, args("intensity")).staticmethod("set_red")
            .def("red_on", &ev3::led::red_on).staticmethod("red_on")

            .def("set_green", &ev3::led::set_green, args("intensity")).staticmethod("set_green")
            .def("green_on", &ev3::led::green_on).staticmethod("green_on")

            .def("set_amber", &ev3::led::set_amber, args("intensity")).staticmethod("set_amber")
            .def("amber_on", &ev3::led::amber_on).staticmethod("amber_on")

            .def("set_orange", &ev3::led::set_orange, args("intensity")).staticmethod("set_orange")
            .def("orange_on", &ev3::led::orange_on).staticmethod("orange_on")

            .def("set_yellow", &ev3::led::set_yellow, args("intensity")).staticmethod("set_yellow")
            .def("yellow_on", &ev3::led::yellow_on).staticmethod("yellow_on")


//~autogen
//~autogen python_generic-get-set classes.led>currentClass

            .add_property("max_brightness", &ev3::led::max_brightness,
                    "Max Brightness: read-only\n\n"
                    "Returns the maximum allowable brightness value.\n"
                    )
            .add_property("brightness", &ev3::led::brightness, make_function(&ev3::led::set_brightness, drop_return_value()),
                    "Brightness: read/write\n\n"
                    "Sets the brightness level. Possible values are from 0 to `max_brightness`.\n"
                    )
            .add_property("triggers", &ev3::led::triggers,
                    "Triggers: read-only\n\n"
                    "Returns a list of available triggers.\n"
                    )
            .add_property("trigger", &ev3::led::trigger, make_function(&ev3::led::set_trigger, drop_return_value()),
                    "Trigger: read/write\n\n"
                    "Sets the led trigger. A trigger\n"
                    "is a kernel based source of led events. Triggers can either be simple or\n"
                    "complex. A simple trigger isn't configurable and is designed to slot into\n"
                    "existing subsystems with minimal additional code. Examples are the `ide-disk` and\n"
                    "`nand-disk` triggers.\n"
                    "\n"
                    "Complex triggers whilst available to all LEDs have LED specific\n"
                    "parameters and work on a per LED basis. The `timer` trigger is an example.\n"
                    "The `timer` trigger will periodically change the LED brightness between\n"
                    "0 and the current brightness setting. The `on` and `off` time can\n"
                    "be specified via `delay_{on,off}` attributes in milliseconds.\n"
                    "You can change the brightness value of a LED independently of the timer\n"
                    "trigger. However, if you set the brightness value to 0 it will\n"
                    "also disable the `timer` trigger.\n"
                    )
            .add_property("delay_on", &ev3::led::delay_on, make_function(&ev3::led::set_delay_on, drop_return_value()),
                    "Delay On: read/write\n\n"
                    "The `timer` trigger will periodically change the LED brightness between\n"
                    "0 and the current brightness setting. The `on` time can\n"
                    "be specified via `delay_on` attribute in milliseconds.\n"
                    )
            .add_property("delay_off", &ev3::led::delay_off, make_function(&ev3::led::set_delay_off, drop_return_value()),
                    "Delay Off: read/write\n\n"
                    "The `timer` trigger will periodically change the LED brightness between\n"
                    "0 and the current brightness setting. The `off` time can\n"
                    "be specified via `delay_off` attribute in milliseconds.\n"
                    )

//~autogen
            .add_property("brightness_pct", &ev3::led::brightness_pct, make_function(&ev3::led::set_brightness_pct, drop_return_value()),
                    "Brightness PCT: read/write\n\n"
                    "Sets the LED's brightness as a percentage (0-1) of the maximum.\n"
                    )
            ;

        s.attr("red_right")   = ev3::led::red_right;
        s.attr("red_left")    = ev3::led::red_left;
        s.attr("green_right") = ev3::led::green_right;
        s.attr("green_left")  = ev3::led::green_left;
    }

    //-----------------------------------------------------------------------
    // Power supply
    //-----------------------------------------------------------------------
    {
        scope s = class_<ev3::power_supply>("power_supply",
//~autogen python_generic-class-description classes.powerSupply>currentClass

                "A generic interface to read data from the system's power_supply class.\n"
                "Uses the built-in legoev3-battery if none is specified.\n"

//~autogen
                , init<std::string>(args("name")))
            .add_property("connected",        device_connected<ev3::power_supply>)
            .add_property("measured_amps",    &ev3::power_supply::measured_amps)
            .add_property("measured_volts",   &ev3::power_supply::measured_volts)
//~autogen python_generic-get-set classes.powerSupply>currentClass

            .add_property("measured_current", &ev3::power_supply::measured_current,
                    "Measured Current: read-only\n\n"
                    "The measured current that the battery is supplying (in microamps)\n"
                    )
            .add_property("measured_voltage", &ev3::power_supply::measured_voltage,
                    "Measured Voltage: read-only\n\n"
                    "The measured voltage that the battery is supplying (in microvolts)\n"
                    )
            .add_property("max_voltage", &ev3::power_supply::max_voltage,
                    "Max Voltage: read-only\n\n"
                    )
            .add_property("min_voltage", &ev3::power_supply::min_voltage,
                    "Min Voltage: read-only\n\n"
                    )
            .add_property("technology", &ev3::power_supply::technology,
                    "Technology: read-only\n\n"
                    )
            .add_property("type", &ev3::power_supply::type,
                    "Type: read-only\n\n"
                    )

//~autogen
            ;

        s.attr("battery") = ev3::power_supply::battery;
    }

    //-----------------------------------------------------------------------
    // Buttons
    //-----------------------------------------------------------------------
    {
        scope s = class_<ev3::button>("button", "EV3 buttons", init<int>(args("bit")))
            .add_property("pressed", &ev3::button::pressed, "Check if the button is pressed")
            .def("onclick", button_onclick, args("callable"),
                    "Set function to be called when the button is clicked."
                    )
            .def("process", &ev3::button::process,
                    "Checks if the button state has changed,\n"
                    "calls function in case it has.\n"
                    "Returns true if the state has changed since the last call."
                    )
            .def("process_all", &ev3::button::process_all,
                    "Calls process() for each of the EV3 buttons.\n"
                    "Returns true if any of the states have changed since the last call."
                    )
            .staticmethod("process_all")
            ;

        s.attr("back")  = ev3::button::back;
        s.attr("left")  = ev3::button::left;
        s.attr("right") = ev3::button::right;
        s.attr("up")    = ev3::button::up;
        s.attr("down")  = ev3::button::down;
        s.attr("enter") = ev3::button::enter;
    }

    //-----------------------------------------------------------------------
    // Sound
    //-----------------------------------------------------------------------
    class_<ev3::sound>("sound", "EV3 sound")
        .def("beep",       &ev3::sound::beep).staticmethod("beep")
        .def("tone",       &ev3::sound::tone).staticmethod("tone")
        .def("play",       (void (*)(const std::string&))&ev3::sound::play,  args("soundfile"))
        .def("play",       (void (*)(const std::string&, bool))&ev3::sound::play, args("soundfile", "bSynchronous"))
        .staticmethod("play")
        .def("speak",       (void (*)(const std::string&))&ev3::sound::speak,  args("text"))
        .def("speak",       (void (*)(const std::string&, bool))&ev3::sound::speak, args("text", "bSynchronous"))
        .staticmethod("speak")
        .add_static_property("volume", &ev3::sound::volume, &ev3::sound::set_volume)
        ;

    //-----------------------------------------------------------------------
    // LCD
    //-----------------------------------------------------------------------
    class_<ev3::lcd>("lcd", "EV3 LCD")
        .add_property("available",         &ev3::lcd::available)
        .add_property("resolution_x",      &ev3::lcd::resolution_x)
        .add_property("resolution_y",      &ev3::lcd::resolution_y)
        .add_property("bits_per_pixel",    &ev3::lcd::bits_per_pixel)
        .add_property("frame_buffer_size", &ev3::lcd::frame_buffer_size)
        .add_property("line_length",       &ev3::lcd::line_length)
        .add_property("frame_buffer",      lcd_frame_buffer)
        .def("fill",              &ev3::lcd::fill, args("pixel"))
        ;

    //-----------------------------------------------------------------------
    // Remote control
    //-----------------------------------------------------------------------
    {
        scope s = class_<ev3::remote_control>("remote_control", "EV3 remote control", init<>())
            .def(init<unsigned>(args("channel")))
            .def(init<ev3::infrared_sensor&>(args("ir_sensor")))
            .def(init<ev3::infrared_sensor&, unsigned>(args("ir_sensor", "channel")))
            .add_property("connected",    &ev3::remote_control::connected)
            .add_property("channel",      &ev3::remote_control::channel)
            .def("process", &ev3::remote_control::process)
            .def("on_red_up",       rc_on_red_up,       args("callable"))
            .def("on_red_down",     rc_on_red_down,     args("callable"))
            .def("on_blue_up",      rc_on_blue_up,      args("callable"))
            .def("on_blue_down",    rc_on_blue_down,    args("callable"))
            .def("on_beacon",       rc_on_beacon,       args("callable"))
            .def("on_state_change", rc_on_state_change, args("callable"))
            ;

        enum_<ev3::remote_control::buttons>("buttons")
            .value("red_up",    ev3::remote_control::red_up)
            .value("red_down",  ev3::remote_control::red_down)
            .value("blue_up",   ev3::remote_control::blue_up)
            .value("blue_down", ev3::remote_control::blue_down)
            .value("beacon",    ev3::remote_control::beacon)
            ;
    }

    //-----------------------------------------------------------------------
    // Lego Port
    //-----------------------------------------------------------------------
    {
        scope s = class_<ev3::lego_port>("lego_port",
//~autogen python_generic-class-description classes.legoPort>currentClass

                "The `lego-port` class provides an interface for working with input and\n"
                "output ports that are compatible with LEGO MINDSTORMS RCX/NXT/EV3, LEGO\n"
                "WeDo and LEGO Power Functions sensors and motors. Supported devices include\n"
                "the LEGO MINDSTORMS EV3 Intelligent Brick, the LEGO WeDo USB hub and\n"
                "various sensor multiplexers from 3rd party manufacturers.\n"
                "\n"
                "Some types of ports may have multiple modes of operation. For example, the\n"
                "input ports on the EV3 brick can communicate with sensors using UART, I2C\n"
                "or analog validate signals - but not all at the same time. Therefore there\n"
                "are multiple modes available to connect to the different types of sensors.\n"
                "\n"
                "In most cases, ports are able to automatically detect what type of sensor\n"
                "or motor is connected. In some cases though, this must be manually specified\n"
                "using the `mode` and `set_device` attributes. The `mode` attribute affects\n"
                "how the port communicates with the connected device. For example the input\n"
                "ports on the EV3 brick can communicate using UART, I2C or analog voltages,\n"
                "but not all at the same time, so the mode must be set to the one that is\n"
                "appropriate for the connected sensor. The `set_device` attribute is used to\n"
                "specify the exact type of sensor that is connected. Note: the mode must be\n"
                "correctly set before setting the sensor type.\n"
                "\n"
                "Ports can be found at `/sys/class/lego-port/port<N>` where `<N>` is\n"
                "incremented each time a new port is registered. Note: The number is not\n"
                "related to the actual port at all - use the `port_name` attribute to find\n"
                "a specific port.\n"

//~autogen
                , init<ev3::port_type>(args("port")))
            .add_property("connected",         device_connected<ev3::lego_port>)
            .add_property("device_index",      device_device_index<ev3::lego_port>)
//~autogen python_generic-get-set classes.legoPort>currentClass

            .add_property("driver_name", &ev3::lego_port::driver_name,
                    "Driver Name: read-only\n\n"
                    "Returns the name of the driver that loaded this device. You can find the\n"
                    "complete list of drivers in the [list of port drivers].\n"
                    )
            .add_property("modes", &ev3::lego_port::modes,
                    "Modes: read-only\n\n"
                    "Returns a list of the available modes of the port.\n"
                    )
            .add_property("mode", &ev3::lego_port::mode, make_function(&ev3::lego_port::set_mode, drop_return_value()),
                    "Mode: read/write\n\n"
                    "Reading returns the currently selected mode. Writing sets the mode.\n"
                    "Generally speaking when the mode changes any sensor or motor devices\n"
                    "associated with the port will be removed new ones loaded, however this\n"
                    "this will depend on the individual driver implementing this class.\n"
                    )
            .add_property("port_name", &ev3::lego_port::port_name,
                    "Port Name: read-only\n\n"
                    "Returns the name of the port. See individual driver documentation for\n"
                    "the name that will be returned.\n"
                    )
            .add_property("set_device", no_getter<ev3::lego_port>, make_function(&ev3::lego_port::set_set_device, drop_return_value()),
                    "Set Device: write-only\n\n"
                    "For modes that support it, writing the name of a driver will cause a new\n"
                    "device to be registered for that driver and attached to this port. For\n"
                    "example, since NXT/Analog sensors cannot be auto-detected, you must use\n"
                    "this attribute to load the correct driver. Returns -EOPNOTSUPP if setting a\n"
                    "device is not supported.\n"
                    )
            .add_property("status", &ev3::lego_port::status,
                    "Status: read-only\n\n"
                    "In most cases, reading status will return the same value as `mode`. In\n"
                    "cases where there is an `auto` mode additional values may be returned,\n"
                    "such as `no-device` or `error`. See individual port driver documentation\n"
                    "for the full list of possible values.\n"
                    )

//~autogen
            ;
    }
}
