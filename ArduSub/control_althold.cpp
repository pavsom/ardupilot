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
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when not auto-armed (i.e. on the ground, pilot has never raised throttle)
        attitude_control->set_throttle_out(0.75,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        position_control->init_z_controller();
        // initialise position and desired velocity
        float pos = sub.stopping_distance();
        float zero = 0;
        position_control->input_pos_vel_accel_z(pos, zero, zero);
        sub.last_roll = 0;
        sub.last_pitch = 0;
        sub.last_pilot_heading = ahrs.yaw_sensor;
        return;
    }

    handle_attitude();
    control_depth();

    motors.set_forward(channel_forward->norm_input());
    motors.set_lateral(channel_lateral->norm_input());
}


/* void ModeAlthold::control_depth() {
    float target_climb_rate_cm_s = sub.get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate_cm_s = constrain_float(target_climb_rate_cm_s, -sub.get_pilot_speed_dn(), g.pilot_speed_up);

    // desired_climb_rate returns 0 when within the deadzone.
    //we allow full control to the pilot, but as soon as there's no input, we handle being at surface/bottom
    if (fabsf(target_climb_rate_cm_s) < 0.05f)  {
        if (sub.ap.at_surface) {
            position_control->set_pos_target_z_cm(MIN(position_control->get_pos_target_z_cm(), g.surface_depth - 5.0f)); // set target to 5 cm below surface level
        } else if (sub.ap.at_bottom) {
            position_control->set_pos_target_z_cm(MAX(inertial_nav.get_position_z_up_cm() + 10.0f, position_control->get_pos_target_z_cm())); // set target to 10 cm above bottom
        }
    }

    position_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate_cm_s);
    position_control->update_z_controller();

} */