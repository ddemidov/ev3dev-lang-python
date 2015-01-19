#include <boost/python.hpp>
#include <boost/python/scope.hpp>
#include <ev3dev.h>

//---------------------------------------------------------------------------
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(device_get_attr_set_ovr, get_attr_set, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(sensor_value_ovr, value, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(sensor_float_value_ovr, float_value, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(motor_run_ovr, run, 0, 1)

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
// Remote control event processing functions
//---------------------------------------------------------------------------
void rc_on_red_up(ev3dev::remote_control *rc, PyObject *f) {
    Py_INCREF(f);
    rc->on_red_up = [f](bool pressed) {
        PyGILState_STATE state = PyGILState_Ensure();
        boost::python::call<void>(f, pressed);
        PyGILState_Release(state);
    };
}

void rc_on_red_down(ev3dev::remote_control *rc, PyObject *f) {
    Py_INCREF(f);
    rc->on_red_down = [f](bool pressed) {
        PyGILState_STATE state = PyGILState_Ensure();
        boost::python::call<void>(f, pressed);
        PyGILState_Release(state);
    };
}

void rc_on_blue_up(ev3dev::remote_control *rc, PyObject *f) {
    Py_INCREF(f);
    rc->on_blue_up = [f](bool pressed) {
        PyGILState_STATE state = PyGILState_Ensure();
        boost::python::call<void>(f, pressed);
        PyGILState_Release(state);
    };
}

void rc_on_blue_down(ev3dev::remote_control *rc, PyObject *f) {
    Py_INCREF(f);
    rc->on_blue_down = [f](bool pressed) {
        PyGILState_STATE state = PyGILState_Ensure();
        boost::python::call<void>(f, pressed);
        PyGILState_Release(state);
    };
}

void rc_on_beacon(ev3dev::remote_control *rc, PyObject *f) {
    Py_INCREF(f);
    rc->on_beacon = [f](bool pressed) {
        PyGILState_STATE state = PyGILState_Ensure();
        boost::python::call<void>(f, pressed);
        PyGILState_Release(state);
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
BOOST_PYTHON_MODULE(ev3dev)
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
    class_<ev3::device>("device")
        .def("connect",           &ev3::device::connect, args("dir", "pattern", "match"))
        .def("connected",         &ev3::device::connected)
        .def("device_index",      &ev3::device::device_index)
        .def("get_attr_int",      &ev3::device::get_attr_int,      args("name"))
        .def("set_attr_int",      &ev3::device::set_attr_int,      args("name", "value"))
        .def("get_attr_string",   &ev3::device::get_attr_string,   args("name"))
        .def("set_attr_string",   &ev3::device::set_attr_string,   args("name", "value"))
        .def("get_attr_set",      &ev3::device::get_attr_set,      device_get_attr_set_ovr())
        .def("get_attr_from_set", &ev3::device::get_attr_from_set, args("name"))
        ;

    {
        scope s = class_<ev3::sensor>("sensor", init<ev3::port_type>())
            .def(init<ev3::port_type, const std::set<ev3::sensor::sensor_type>&>())
            .def("connected",    device_connected<ev3::sensor>)
            .def("device_index", device_device_index<ev3::sensor>)
            .def("port_name",    &ev3::sensor::port_name,   return_value_policy<copy_const_reference>())
            .def("type",         &ev3::sensor::type,        return_value_policy<copy_const_reference>())
            .def("type_name",    &ev3::sensor::type_name,   return_value_policy<copy_const_reference>())
            .def("units",        &ev3::sensor::units)
            .def("num_values",   &ev3::sensor::num_values)
            .def("dp",           &ev3::sensor::dp)
            .def("value",        &ev3::sensor::value,       sensor_value_ovr())
            .def("float_value",  &ev3::sensor::float_value, sensor_float_value_ovr())
            .def("modes",        &ev3::sensor::modes,       return_value_policy<copy_const_reference>())
            .add_property("mode",
                    make_function(&ev3::sensor::mode, return_value_policy<copy_const_reference>()),
                    &ev3::sensor::set_mode
                    )
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

    //-----------------------------------------------------------------------
    // Sensors
    //-----------------------------------------------------------------------
    class_<ev3::i2c_sensor, bases<ev3::sensor>>("i2c_sensor", init<>())
        .def(init<ev3::port_type>())
        .def(init<ev3::port_type, ev3::address_type>())
        .def("address",     &ev3::i2c_sensor::address)
        .def("fw_version",  &ev3::i2c_sensor::fw_version)
        .add_property("poll_ms", &ev3::i2c_sensor::poll_ms, &ev3::i2c_sensor::set_poll_ms)
        ;

    class_<ev3::touch_sensor, bases<ev3::sensor>>("touch_sensor", init<>())
        .def(init<ev3::port_type>())
        ;

    {
        scope s = class_<ev3::color_sensor, bases<ev3::sensor>>(
                "color_sensor", init<>())
            .def(init<ev3::port_type>())
            ;

        s.attr("mode_reflect") = ev3::color_sensor::mode_reflect;
        s.attr("mode_ambient") = ev3::color_sensor::mode_ambient;
        s.attr("mode_color")   = ev3::color_sensor::mode_color;
    }

    {
        scope s = class_<ev3::ultrasonic_sensor, bases<ev3::sensor>>(
                "ultrasonic_sensor", init<>()
                )
            .def(init<ev3::port_type>())
            ;

        s.attr("mode_dist_cm")   = ev3::ultrasonic_sensor::mode_dist_cm;
        s.attr("mode_dist_in")   = ev3::ultrasonic_sensor::mode_dist_in;
        s.attr("mode_listen")    = ev3::ultrasonic_sensor::mode_listen;
        s.attr("mode_single_cm") = ev3::ultrasonic_sensor::mode_single_cm;
        s.attr("mode_single_in") = ev3::ultrasonic_sensor::mode_single_in;
    }

    {
        scope s = class_<ev3::gyro_sensor, bases<ev3::sensor>>(
                "gyro_sensor", init<>()
                )
            .def(init<ev3::port_type>())
            ;

        s.attr("mode_angle")           = ev3::gyro_sensor::mode_angle;
        s.attr("mode_speed")           = ev3::gyro_sensor::mode_speed;
        s.attr("mode_angle_and_speed") = ev3::gyro_sensor::mode_angle_and_speed;
    }

    {
        scope s = class_<ev3::infrared_sensor, bases<ev3::sensor>>(
                "infrared_sensor", init<>()
                )
            .def(init<ev3::port_type>())
            ;

        s.attr("mode_proximity") = ev3::infrared_sensor::mode_proximity;
        s.attr("mode_ir_seeker") = ev3::infrared_sensor::mode_ir_seeker;
        s.attr("mode_ir_remote") = ev3::infrared_sensor::mode_ir_remote;
    }

    //-----------------------------------------------------------------------
    // Motors
    //-----------------------------------------------------------------------
    {
        scope s = class_<ev3::motor>("motor", init<ev3::port_type>())
            .def(init<ev3::port_type, ev3::motor::motor_type>())

            .def("connected",         device_connected<ev3::motor>)
            .def("device_index",      device_device_index<ev3::motor>)
            .def("port_name",         &ev3::motor::port_name)
            .def("type",              &ev3::motor::type)
            .def("run",               &ev3::motor::run, motor_run_ovr())
            .def("stop",              &ev3::motor::stop)
            .def("reset",             &ev3::motor::reset)
            .def("running",           &ev3::motor::running)
            .def("state",             &ev3::motor::state)
            .def("duty_cycle",        &ev3::motor::duty_cycle)
            .def("pulses_per_second", &ev3::motor::pulses_per_second)

            .add_property("position",                   &ev3::motor::position,                   &ev3::motor::set_position)
            .add_property("run_mode",                   &ev3::motor::run_mode,                   &ev3::motor::set_run_mode)
            .add_property("stop_mode",                  &ev3::motor::stop_mode,                  &ev3::motor::set_stop_mode)
            .add_property("regulation_mode",            &ev3::motor::regulation_mode,            &ev3::motor::set_regulation_mode)
            .add_property("position_mode",              &ev3::motor::position_mode,              &ev3::motor::set_position_mode)
            .add_property("duty_cycle_setpoint",        &ev3::motor::duty_cycle_setpoint,        &ev3::motor::set_duty_cycle_setpoint)
            .add_property("pulses_per_second_setpoint", &ev3::motor::pulses_per_second_setpoint, &ev3::motor::set_pulses_per_second_setpoint)
            .add_property("time_setpoint",              &ev3::motor::time_setpoint,              &ev3::motor::set_time_setpoint)
            .add_property("position_setpoint",          &ev3::motor::position_setpoint,          &ev3::motor::set_position_setpoint)
            .add_property("ramp_up",                    &ev3::motor::ramp_up,                    &ev3::motor::set_ramp_up)
            .add_property("ramp_down",                  &ev3::motor::ramp_down,                  &ev3::motor::set_ramp_down)
            .add_property("speed_regulation_p",         &ev3::motor::speed_regulation_p,         &ev3::motor::set_speed_regulation_p)
            .add_property("speed_regulation_i",         &ev3::motor::speed_regulation_i,         &ev3::motor::set_speed_regulation_i)
            .add_property("speed_regulation_d",         &ev3::motor::speed_regulation_d,         &ev3::motor::set_speed_regulation_d)
            .add_property("speed_regulation_k",         &ev3::motor::speed_regulation_k,         &ev3::motor::set_speed_regulation_k)
            ;

        s.attr("motor_large")  = ev3::motor::motor_large;
        s.attr("motor_medium") = ev3::motor::motor_medium;

        s.attr("mode_off") = ev3::motor::mode_off;
        s.attr("mode_on")  = ev3::motor::mode_on;

        s.attr("run_mode_forever")  = ev3::motor::run_mode_forever;
        s.attr("run_mode_time")     = ev3::motor::run_mode_time;
        s.attr("run_mode_position") = ev3::motor::run_mode_position;

        s.attr("stop_mode_coast") = ev3::motor::stop_mode_coast;
        s.attr("stop_mode_brake") = ev3::motor::stop_mode_brake;
        s.attr("stop_mode_hold")  = ev3::motor::stop_mode_hold;

        s.attr("position_mode_absolute") = ev3::motor::position_mode_absolute;
        s.attr("position_mode_relative") = ev3::motor::position_mode_relative;
    }

    class_<ev3::medium_motor, bases<ev3::motor>>("medium_motor", init<>())
        .def(init<ev3::port_type>())
        ;

    class_<ev3::large_motor, bases<ev3::motor>>("large_motor", init<>())
        .def(init<ev3::port_type>())
        ;

    {
        scope s = class_<ev3::dc_motor>("dc_motor", init<>())
            .def(init<ev3::port_type>())
            .def("connected",    device_connected<ev3::dc_motor>)
            .def("device_index", device_device_index<ev3::dc_motor>)
            .def("port_name",    &ev3::dc_motor::port_name)
            .def("type_name",    &ev3::dc_motor::type_name)
            .def("commands",     &ev3::dc_motor::command)

            .add_property("command",      &ev3::dc_motor::command,      &ev3::dc_motor::set_command)
            .add_property("duty_cycle",   &ev3::dc_motor::duty_cycle,   &ev3::dc_motor::set_duty_cycle)
            .add_property("ramp_down_ms", &ev3::dc_motor::ramp_down_ms, &ev3::dc_motor::set_ramp_down_ms)
            .add_property("ramp_up_ms",   &ev3::dc_motor::ramp_up_ms,   &ev3::dc_motor::set_ramp_up_ms)
            .add_property("polarity",     &ev3::dc_motor::polarity,     &ev3::dc_motor::set_polarity)
            ;

        s.attr("command_run")       = ev3::dc_motor::command_run;
        s.attr("command_brake")     = ev3::dc_motor::command_brake;
        s.attr("command_coast")     = ev3::dc_motor::command_coast;
        s.attr("polarity_normal")   = ev3::dc_motor::polarity_normal;
        s.attr("polarity_inverted") = ev3::dc_motor::polarity_inverted;
    }

    {
        scope s = class_<ev3::servo_motor>("servo_motor", init<>())
            .def(init<ev3::port_type>())
            .def("connected",    device_connected<ev3::servo_motor>)
            .def("device_index", device_device_index<ev3::servo_motor>)
            .def("port_name",    &ev3::servo_motor::port_name)
            .def("type_name",    &ev3::servo_motor::type_name)

            .add_property("command",      &ev3::servo_motor::command,      &ev3::servo_motor::set_command)
            .add_property("rate",         &ev3::servo_motor::rate,         &ev3::servo_motor::set_rate)
            .add_property("max_pulse_ms", &ev3::servo_motor::max_pulse_ms, &ev3::servo_motor::set_max_pulse_ms)
            .add_property("mid_pulse_ms", &ev3::servo_motor::mid_pulse_ms, &ev3::servo_motor::set_mid_pulse_ms)
            .add_property("min_pulse_ms", &ev3::servo_motor::min_pulse_ms, &ev3::servo_motor::set_min_pulse_ms)
            .add_property("polarity",     &ev3::servo_motor::polarity,     &ev3::servo_motor::set_polarity)
            ;

        s.attr("command_run")       = ev3::servo_motor::command_run;
        s.attr("command_float")     = ev3::servo_motor::command_float;
        s.attr("polarity_normal")   = ev3::servo_motor::polarity_normal;
        s.attr("polarity_inverted") = ev3::servo_motor::polarity_inverted;
    }

    //-----------------------------------------------------------------------
    // LED
    //-----------------------------------------------------------------------
    {
        scope s = class_<ev3::led>("led", init<std::string>())
            .def("connected",      device_connected<ev3::led>)
            .def("max_brightness", &ev3::led::max_brightness)
            .add_property("brightness", &ev3::led::brightness, &ev3::led::set_brightness)
            .def("on",             &ev3::led::on)
            .def("off",            &ev3::led::off)
            .def("flash",          &ev3::led::flash, args("interval_ms"))
            .def("set_on_delay",   &ev3::led::set_on_delay, args("ms"))
            .def("set_off_delay",  &ev3::led::set_off_delay, args("ms"))
            .def("triggers",       &ev3::led::triggers)
            .add_property("trigger", &ev3::led::trigger, &ev3::led::set_trigger)
            .def("red_on",         &ev3::led::red_on).staticmethod("red_on")
            .def("red_off",        &ev3::led::red_off).staticmethod("red_off")
            .def("green_on",       &ev3::led::green_on).staticmethod("green_on")
            .def("green_off",      &ev3::led::green_off).staticmethod("green_off")
            .def("all_on",         &ev3::led::all_on).staticmethod("all_on")
            .def("all_off",        &ev3::led::all_off).staticmethod("all_off")
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
            .def("connected",          device_connected<ev3::power_supply>)
            .def("current_now",        &ev3::power_supply::current_now)
            .def("current_amps",       &ev3::power_supply::current_amps)
            .def("current_max_design", &ev3::power_supply::current_max_design)
            .def("voltage_now",        &ev3::power_supply::voltage_now)
            .def("voltage_volts",      &ev3::power_supply::voltage_volts)
            .def("voltage_max_design", &ev3::power_supply::voltage_max_design)
            .def("technology",         &ev3::power_supply::technology)
            .def("type",               &ev3::power_supply::type)
            ;

        s.attr("battery") = ev3::power_supply::battery;
    }

    //-----------------------------------------------------------------------
    // Buttons
    //-----------------------------------------------------------------------
    {
        scope s = class_<ev3::button>("button", init<int>())
            .def("pressed", &ev3::button::pressed)
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
        .def("available",         &ev3::lcd::available)
        .def("resolution_x",      &ev3::lcd::resolution_x)
        .def("resolution_y",      &ev3::lcd::resolution_y)
        .def("bits_per_pixel",    &ev3::lcd::bits_per_pixel)
        .def("frame_buffer_size", &ev3::lcd::frame_buffer_size)
        .def("line_length",       &ev3::lcd::line_length)
        .def("frame_buffer",      lcd_frame_buffer)
        .def("fill",              &ev3::lcd::fill, args("pixel"))
        ;

    //-----------------------------------------------------------------------
    // Remote control
    //-----------------------------------------------------------------------
    class_<ev3::remote_control>("remote_control", init<>())
        .def(init<unsigned>())
        .def(init<ev3::infrared_sensor&>())
        .def(init<ev3::infrared_sensor&, unsigned>())
        .def("connected",    &ev3::remote_control::connected)
        .def("channel",      &ev3::remote_control::channel)
        .def("process",      &ev3::remote_control::process)
        .def("on_red_up",    rc_on_red_up,    args("callable"))
        .def("on_red_down",  rc_on_red_down,  args("callable"))
        .def("on_blue_up",   rc_on_blue_up,   args("callable"))
        .def("on_blue_down", rc_on_blue_down, args("callable"))
        .def("on_beacon",    rc_on_beacon,    args("callable"))
        ;
}
