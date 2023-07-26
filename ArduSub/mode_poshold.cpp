// ArduSub position hold flight mode
// GPS required
// Jacob Walser August 2016

#include "Sub.h"

#if POSHOLD_ENABLED == ENABLED

// poshold_init - initialise PosHold controller
bool ModePoshold::init(bool ignore_checks)
{
    // fail to initialise PosHold mode if no GPS lock
    if (!sub.position_ok()) {
        return false;
    }

    // initialize vertical speeds and acceleration
    position_control->set_max_speed_accel_xy(sub.wp_nav.get_default_speed_xy(), sub.wp_nav.get_wp_acceleration());
    position_control->set_correction_speed_accel_xy(sub.wp_nav.get_default_speed_xy(), sub.wp_nav.get_wp_acceleration());
    position_control->set_max_speed_accel_z(-sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    position_control->set_correction_speed_accel_z(-sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise position and desired velocity
    position_control->init_xy_controller_stopping_point();
    // Stop all thrusters
    attitude_control->set_throttle_out(0.75,true,100.0);
    
    position_control->init_z_controller();

    // initialise position and desired velocity
    float pos = sub.stopping_distance();
    float zero = 0;
    position_control->input_pos_vel_accel_z(pos, zero, zero);

    sub.last_pilot_heading = ahrs.yaw_sensor;

    return true;
}

// poshold_run - runs the PosHold controller
// should be called at 100hz or more
void ModePoshold::run()
{
    if (disarmed(Number::GUIDED)){
        position_control->relax_velocity_controller_xy();
        position_control->relax_z_controller(0.5f);
        return;
    }

    ///////////////////////
    // update xy outputs //
    float pilot_lateral = channel_lateral->norm_input();
    float pilot_forward = channel_forward->norm_input();

    float lateral_out = 0;
    float forward_out = 0;

    if (sub.position_ok()) {
        // Allow pilot to reposition the sub
        if (fabsf(pilot_lateral) > 0.1 || fabsf(pilot_forward) > 0.1) {
            position_control->init_xy_controller_stopping_point();
        }
        sub.translate_pos_control_rp(lateral_out, forward_out);
        position_control->update_xy_controller();
    } else {
        position_control->init_xy_controller_stopping_point();
    }
    /////////////////////
    // Update attitude //
    handle_attitude();

    // Update z axis //
    control_depth();

    motors.set_forward(motors.get_forward() + forward_out + pilot_forward);
    motors.set_lateral(motors.get_lateral() + lateral_out + pilot_lateral);
}
#endif  // POSHOLD_ENABLED == ENABLED
