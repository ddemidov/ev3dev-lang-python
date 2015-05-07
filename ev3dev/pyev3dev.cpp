#include <boost/python.hpp>
#include <boost/python/scope.hpp>
#include <boost/python/extract.hpp>
#include <boost/python/raw_function.hpp>
#include <boost/python/stl_iterator.hpp>
#include <ev3dev.h>
#include <iostream>

//~autogen autogen-header
    // Sections of the following code were auto-generated based on spec v0.9.2-pre, rev 3. 
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

    dev.connect(dir, pattern, match);

    return dev.connected();
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
// Remote control event processing functions
//---------------------------------------------------------------------------
struct HoldGIL {
    PyGILState_STATE state;
    HoldGIL() : state(PyGILState_Ensure()) {}
    ~HoldGIL() { PyGILState_Release(state); }
};

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
// The module interface
//---------------------------------------------------------------------------
BOOST_PYTHON_MODULE(ev3dev_ext)
{
    using namespace boost::python;
    namespace ev3 = ev3dev;

    PyEval_InitThreads();

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

    class_<ev3::mode_set>("mode_set")
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
        class_<ev3::device>("device")
            .def("connect", raw_function(device_connect))
            .add_property("connected",    &ev3::device::connected)
            .add_property("device_index", &ev3::device::device_index)
            .def("get_attr_int",    &ev3::device::get_attr_int)
            .def("set_attr_int",    &ev3::device::set_attr_int)
            .def("get_attr_string", &ev3::device::get_attr_string)
            .def("set_attr_string", &ev3::device::set_attr_string)
            .def("get_attr_line",   &ev3::device::get_attr_line)
            .def("get_attr_set",    &ev3::device::get_attr_set, device_get_attr_set_ovr())
            .def("get_attr_from_set", &ev3::device::get_attr_from_set)
            ;
    }

    //-----------------------------------------------------------------------
    // Sensors
    //-----------------------------------------------------------------------
    {
        scope s = class_<ev3::sensor>("sensor", init<ev3::port_type>())
            .add_property("connected", device_connected<ev3::sensor>)
            .add_property("device_index", device_device_index<ev3::sensor>)
            .def("value",        &ev3::sensor::value,       sensor_value_ovr())
            .def("float_value",  &ev3::sensor::float_value, sensor_float_value_ovr())
//~autogen python_generic-get-set classes.sensor>currentClass

            .add_property("command", no_getter<ev3::sensor>, &ev3::sensor::set_command)
            .add_property("commands", &ev3::sensor::commands)
            .add_property("decimals", &ev3::sensor::decimals)
            .add_property("driver_name", &ev3::sensor::driver_name)
            .add_property("mode", &ev3::sensor::mode, &ev3::sensor::set_mode)
            .add_property("modes", &ev3::sensor::modes)
            .add_property("num_values", &ev3::sensor::num_values)
            .add_property("port_name", &ev3::sensor::port_name)
            .add_property("units", &ev3::sensor::units)

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
    }

    {
        class_<ev3::i2c_sensor, bases<ev3::sensor>>("i2c_sensor", init<>())
            .def(init<ev3::port_type>())
            .def(init<ev3::port_type, ev3::address_type>())
//~autogen python_generic-get-set classes.i2cSensor>currentClass

            .add_property("fw_version", &ev3::i2c_sensor::fw_version)
            .add_property("poll_ms", &ev3::i2c_sensor::poll_ms, &ev3::i2c_sensor::set_poll_ms)

//~autogen
            ;
    }

    {
        class_<ev3::touch_sensor, bases<ev3::sensor>>("touch_sensor", init<>())
            .def(init<ev3::port_type>())
            ;
    }

    {
        scope s = class_<ev3::color_sensor, bases<ev3::sensor>>(
                "color_sensor", init<>())
            .def(init<ev3::port_type>())
            ;

//~autogen python_generic-property-value classes.colorSensor>currentClass

        s.attr("mode_col_reflect") = ev3::color_sensor::mode_col_reflect;
        s.attr("mode_col_ambient") = ev3::color_sensor::mode_col_ambient;
        s.attr("mode_col_color") = ev3::color_sensor::mode_col_color;
        s.attr("mode_ref_raw") = ev3::color_sensor::mode_ref_raw;
        s.attr("mode_rgb_raw") = ev3::color_sensor::mode_rgb_raw;
        s.attr("mode_col_cal") = ev3::color_sensor::mode_col_cal;

//~autogen
    }

    {
        scope s = class_<ev3::ultrasonic_sensor, bases<ev3::sensor>>(
                "ultrasonic_sensor", init<>()
                )
            .def(init<ev3::port_type>())
            ;

//~autogen python_generic-property-value classes.ultrasonicSensor>currentClass

        s.attr("mode_us_dist_cm") = ev3::ultrasonic_sensor::mode_us_dist_cm;
        s.attr("mode_us_dist_in") = ev3::ultrasonic_sensor::mode_us_dist_in;
        s.attr("mode_us_listen") = ev3::ultrasonic_sensor::mode_us_listen;
        s.attr("mode_us_si_cm") = ev3::ultrasonic_sensor::mode_us_si_cm;
        s.attr("mode_us_si_in") = ev3::ultrasonic_sensor::mode_us_si_in;
        s.attr("mode_us_dc_cm") = ev3::ultrasonic_sensor::mode_us_dc_cm;
        s.attr("mode_us_dc_in") = ev3::ultrasonic_sensor::mode_us_dc_in;

//~autogen
    }

    {
        scope s = class_<ev3::gyro_sensor, bases<ev3::sensor>>(
                "gyro_sensor", init<>()
                )
            .def(init<ev3::port_type>())
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
                "infrared_sensor", init<>()
                )
            .def(init<ev3::port_type>())
            ;

//~autogen python_generic-property-value classes.infraredSensor>currentClass

        s.attr("mode_ir_prox") = ev3::infrared_sensor::mode_ir_prox;
        s.attr("mode_ir_seek") = ev3::infrared_sensor::mode_ir_seek;
        s.attr("mode_ir_remote") = ev3::infrared_sensor::mode_ir_remote;
        s.attr("mode_ir_rem_a") = ev3::infrared_sensor::mode_ir_rem_a;
        s.attr("mode_ir_s_alt") = ev3::infrared_sensor::mode_ir_s_alt;
        s.attr("mode_ir_cal") = ev3::infrared_sensor::mode_ir_cal;

//~autogen
    }

    //-----------------------------------------------------------------------
    // Motors
    //-----------------------------------------------------------------------
    {
        scope s = class_<ev3::motor>("motor", init<ev3::port_type>())
            .def(init<ev3::port_type, ev3::motor::motor_type>())
            .add_property("connected",         device_connected<ev3::motor>)
            .add_property("device_index",      device_device_index<ev3::motor>)
//~autogen python_generic-get-set classes.motor>currentClass

            .add_property("command", no_getter<ev3::motor>, &ev3::motor::set_command)
            .add_property("commands", &ev3::motor::commands)
            .add_property("count_per_rot", &ev3::motor::count_per_rot)
            .add_property("driver_name", &ev3::motor::driver_name)
            .add_property("duty_cycle", &ev3::motor::duty_cycle)
            .add_property("duty_cycle_sp", &ev3::motor::duty_cycle_sp, &ev3::motor::set_duty_cycle_sp)
            .add_property("encoder_polarity", &ev3::motor::encoder_polarity, &ev3::motor::set_encoder_polarity)
            .add_property("polarity", &ev3::motor::polarity, &ev3::motor::set_polarity)
            .add_property("port_name", &ev3::motor::port_name)
            .add_property("position", &ev3::motor::position, &ev3::motor::set_position)
            .add_property("position_p", &ev3::motor::position_p, &ev3::motor::set_position_p)
            .add_property("position_i", &ev3::motor::position_i, &ev3::motor::set_position_i)
            .add_property("position_d", &ev3::motor::position_d, &ev3::motor::set_position_d)
            .add_property("position_sp", &ev3::motor::position_sp, &ev3::motor::set_position_sp)
            .add_property("speed", &ev3::motor::speed)
            .add_property("speed_sp", &ev3::motor::speed_sp, &ev3::motor::set_speed_sp)
            .add_property("ramp_up_sp", &ev3::motor::ramp_up_sp, &ev3::motor::set_ramp_up_sp)
            .add_property("ramp_down_sp", &ev3::motor::ramp_down_sp, &ev3::motor::set_ramp_down_sp)
            .add_property("speed_regulation_enabled", &ev3::motor::speed_regulation_enabled, &ev3::motor::set_speed_regulation_enabled)
            .add_property("speed_regulation_p", &ev3::motor::speed_regulation_p, &ev3::motor::set_speed_regulation_p)
            .add_property("speed_regulation_i", &ev3::motor::speed_regulation_i, &ev3::motor::set_speed_regulation_i)
            .add_property("speed_regulation_d", &ev3::motor::speed_regulation_d, &ev3::motor::set_speed_regulation_d)
            .add_property("state", &ev3::motor::state)
            .add_property("stop_command", &ev3::motor::stop_command, &ev3::motor::set_stop_command)
            .add_property("stop_commands", &ev3::motor::stop_commands)
            .add_property("time_sp", &ev3::motor::time_sp, &ev3::motor::set_time_sp)

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
        s.attr("encoder_polarity_inverted") = ev3::motor::encoder_polarity_inverted;
        s.attr("polarity_normal") = ev3::motor::polarity_normal;
        s.attr("polarity_inverted") = ev3::motor::polarity_inverted;
        s.attr("speed_regulation_on") = ev3::motor::speed_regulation_on;
        s.attr("speed_regulation_off") = ev3::motor::speed_regulation_off;
        s.attr("stop_command_coast") = ev3::motor::stop_command_coast;
        s.attr("stop_command_brake") = ev3::motor::stop_command_brake;
        s.attr("stop_command_hold") = ev3::motor::stop_command_hold;

//~autogen
    }

    {
        class_<ev3::medium_motor, bases<ev3::motor>>("medium_motor", init<>())
            .def(init<ev3::port_type>())
            ;
    }

    {
        class_<ev3::large_motor, bases<ev3::motor>>("large_motor", init<>())
            .def(init<ev3::port_type>())
            ;
    }

    {
        scope s = class_<ev3::dc_motor>("dc_motor", init<>())
            .def(init<ev3::port_type>())
            .add_property("connected",    device_connected<ev3::dc_motor>)
            .add_property("device_index", device_device_index<ev3::dc_motor>)
//~autogen python_generic-get-set classes.dcMotor>currentClass

            .add_property("command", no_getter<ev3::dc_motor>, &ev3::dc_motor::set_command)
            .add_property("commands", &ev3::dc_motor::commands)
            .add_property("driver_name", &ev3::dc_motor::driver_name)
            .add_property("duty_cycle", &ev3::dc_motor::duty_cycle)
            .add_property("duty_cycle_sp", &ev3::dc_motor::duty_cycle_sp, &ev3::dc_motor::set_duty_cycle_sp)
            .add_property("polarity", &ev3::dc_motor::polarity, &ev3::dc_motor::set_polarity)
            .add_property("port_name", &ev3::dc_motor::port_name)
            .add_property("ramp_down_sp", &ev3::dc_motor::ramp_down_sp, &ev3::dc_motor::set_ramp_down_sp)
            .add_property("ramp_up_sp", &ev3::dc_motor::ramp_up_sp, &ev3::dc_motor::set_ramp_up_sp)
            .add_property("state", &ev3::dc_motor::state)
            .add_property("stop_command", no_getter<ev3::dc_motor>, &ev3::dc_motor::set_stop_command)
            .add_property("stop_commands", &ev3::dc_motor::stop_commands)

//~autogen
            ;

//~autogen python_generic-property-value classes.dcMotor>currentClass

        s.attr("command_run_forever") = ev3::dc_motor::command_run_forever;
        s.attr("command_run_timed") = ev3::dc_motor::command_run_timed;
        s.attr("command_stop") = ev3::dc_motor::command_stop;
        s.attr("polarity_normal") = ev3::dc_motor::polarity_normal;
        s.attr("polarity_inverted") = ev3::dc_motor::polarity_inverted;
        s.attr("stop_command_coast") = ev3::dc_motor::stop_command_coast;
        s.attr("stop_command_brake") = ev3::dc_motor::stop_command_brake;

//~autogen
    }

    {
        scope s = class_<ev3::servo_motor>("servo_motor", init<>())
            .def(init<ev3::port_type>())
            .add_property("connected",    device_connected<ev3::servo_motor>)
            .add_property("device_index", device_device_index<ev3::servo_motor>)
//~autogen python_generic-get-set classes.servoMotor>currentClass

            .add_property("command", no_getter<ev3::servo_motor>, &ev3::servo_motor::set_command)
            .add_property("driver_name", &ev3::servo_motor::driver_name)
            .add_property("max_pulse_sp", &ev3::servo_motor::max_pulse_sp, &ev3::servo_motor::set_max_pulse_sp)
            .add_property("mid_pulse_sp", &ev3::servo_motor::mid_pulse_sp, &ev3::servo_motor::set_mid_pulse_sp)
            .add_property("min_pulse_sp", &ev3::servo_motor::min_pulse_sp, &ev3::servo_motor::set_min_pulse_sp)
            .add_property("polarity", &ev3::servo_motor::polarity, &ev3::servo_motor::set_polarity)
            .add_property("port_name", &ev3::servo_motor::port_name)
            .add_property("position_sp", &ev3::servo_motor::position_sp, &ev3::servo_motor::set_position_sp)
            .add_property("rate_sp", &ev3::servo_motor::rate_sp, &ev3::servo_motor::set_rate_sp)
            .add_property("state", &ev3::servo_motor::state)

//~autogen
            ;

//~autogen python_generic-property-value classes.servoMotor>currentClass

        s.attr("command_run") = ev3::servo_motor::command_run;
        s.attr("command_float") = ev3::servo_motor::command_float;
        s.attr("polarity_normal") = ev3::servo_motor::polarity_normal;
        s.attr("polarity_inverted") = ev3::servo_motor::polarity_inverted;

//~autogen
    }

    //-----------------------------------------------------------------------
    // LED
    //-----------------------------------------------------------------------
    {
        scope s = class_<ev3::led>("led", init<std::string>())
            .add_property("connected",      device_connected<ev3::led>)
            .def("on",             &ev3::led::on)
            .def("off",            &ev3::led::off)
            .def("flash",          &ev3::led::flash, args("interval_ms"))
            .def("set_on_delay",   &ev3::led::set_on_delay, args("ms"))
            .def("set_off_delay",  &ev3::led::set_off_delay, args("ms"))
            .def("triggers",       &ev3::led::triggers)
            .def("red_on",         &ev3::led::red_on).staticmethod("red_on")
            .def("red_off",        &ev3::led::red_off).staticmethod("red_off")
            .def("green_on",       &ev3::led::green_on).staticmethod("green_on")
            .def("green_off",      &ev3::led::green_off).staticmethod("green_off")
            .def("all_on",         &ev3::led::all_on).staticmethod("all_on")
            .def("all_off",        &ev3::led::all_off).staticmethod("all_off")
            .add_property("triggers", &ev3::led::triggers)
//~autogen python_generic-get-set classes.led>currentClass

            .add_property("max_brightness", &ev3::led::max_brightness)
            .add_property("brightness", &ev3::led::brightness, &ev3::led::set_brightness)
            .add_property("trigger", &ev3::led::trigger, &ev3::led::set_trigger)

//~autogen
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
        scope s = class_<ev3::power_supply>("power_supply", init<std::string>())
            .add_property("connected",        device_connected<ev3::power_supply>)
            .add_property("measured_amps",    &ev3::power_supply::measured_amps)
            .add_property("measured_volts",   &ev3::power_supply::measured_volts)
//~autogen python_generic-get-set classes.powerSupply>currentClass

            .add_property("measured_current", &ev3::power_supply::measured_current)
            .add_property("measured_voltage", &ev3::power_supply::measured_voltage)
            .add_property("max_voltage", &ev3::power_supply::max_voltage)
            .add_property("min_voltage", &ev3::power_supply::min_voltage)
            .add_property("technology", &ev3::power_supply::technology)
            .add_property("type", &ev3::power_supply::type)

//~autogen
            ;

        s.attr("battery") = ev3::power_supply::battery;
    }

    //-----------------------------------------------------------------------
    // Buttons
    //-----------------------------------------------------------------------
    {
        scope s = class_<ev3::button>("button", init<int>())
            .add_property("pressed", &ev3::button::pressed)
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
    class_<ev3::sound>("sound")
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
    class_<ev3::lcd>("lcd")
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
        scope s = class_<ev3::remote_control>("remote_control", init<>())
            .def(init<unsigned>())
            .def(init<ev3::infrared_sensor&>())
            .def(init<ev3::infrared_sensor&, unsigned>())
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
}
