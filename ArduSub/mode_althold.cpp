#include "Sub.h"


bool ModeAlthold::init(bool ignore_checks) {
    if(!sub.control_check_barometer()) {
        return false;
    }

    // initialize vertical maximum speeds and acceleration
    // sets the maximum speed up and down returned by position controller
    attitude_control->set_throttle_out(0.75, true, g.throttle_filt);
    position_control->init_z_controller();
    position_control->set_max_speed_accel_z(-sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    position_control->set_correction_speed_accel_z(-sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    attitude_control->relax_attitude_controllers();
    // initialise position and desired velocity
    float pos = sub.stopping_distance();
    float zero = 0;
    position_control->input_pos_vel_accel_z(pos, zero, zero);

    sub.last_pilot_heading = ahrs.yaw_sensor;
    sub.last_input_ms = AP_HAL::millis();
    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void ModeAlthold::run()
{
    if (disarmed(Number::MANUAL)){
        float pos = sub.stopping_distance();
        float zero = 0;
        position_control->input_pos_vel_accel_z(pos, zero, zero);
        return;
    }
    
    handle_attitude();
    control_depth();

    motors.set_forward(channel_forward->norm_input());
    motors.set_lateral(channel_lateral->norm_input());
}
