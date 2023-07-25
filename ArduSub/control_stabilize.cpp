#include "Sub.h"


bool ModeStabilize::init(bool ignore_checks) {
    // set target altitude to zero for reporting
    position_control->set_pos_target_z_cm(0);
    sub.last_pilot_heading = ahrs.yaw_sensor;

    return true;
    return true;
}

void ModeStabilize::run()
{
    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        sub.last_pilot_heading = ahrs.yaw_sensor;
        sub.last_roll = 0;
        sub.last_pitch = 0;
        return;
    }
    
    uint32_t tnow = AP_HAL::millis();
    // initialize vertical speeds and acceleration
    position_control->set_max_speed_accel_z(-sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // get pilot desired lean angles/rates
    float target_roll, target_pitch, target_yaw;

    // Check if set_attitude_target_no_gps is valid
    if (tnow - sub.set_attitude_target_no_gps.last_message_ms < 5000) {
        Quaternion(
            sub.set_attitude_target_no_gps.packet.q
        ).to_euler(
            target_roll,
            target_pitch,
            target_yaw
        );
        target_roll = 100 * degrees(target_roll);
        target_pitch = 100 * degrees(target_pitch);
        target_yaw = 100 * degrees(target_yaw);
        sub.last_roll = target_roll;
        sub.last_pitch = target_pitch;
        sub.last_pilot_heading = target_yaw;
        attitude_control->input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, target_yaw, true);
        return;
    }
    // convert pilot input to lean angles
    // To-Do: convert sub.get_pilot_desired_lean_angles to return angles as floats
    // TODO2: move into mode.h
    sub.get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, attitude_control->get_althold_lean_angle_max_cd());

    // get pilot's desired yaw rate
    float yaw_input =  channel_yaw->pwm_to_angle_dz_trim(channel_yaw->get_dead_zone() * sub.gain, channel_yaw->get_radio_trim());
    target_yaw = sub.get_pilot_desired_yaw_rate(yaw_input);
    
    switch (g.control_frame) {
        case MAV_FRAME_BODY_FRD:
        {
          if (abs(target_roll) > 50 || abs(target_pitch) > 50 || abs(target_yaw) > 50) {
              sub.last_input_ms = tnow;
              attitude_control->input_rate_bf_roll_pitch_yaw(target_roll, target_pitch, target_yaw);
              Quaternion attitude_target = attitude_control->get_attitude_target_quat();
              sub.last_roll = degrees(attitude_target.get_euler_roll()) * 100;
              sub.last_pitch = degrees(attitude_target.get_euler_pitch()) * 100;
              sub.last_pilot_heading = degrees(attitude_target.get_euler_yaw()) * 100;
          } else {
              attitude_control->input_euler_angle_roll_pitch_yaw(sub.last_roll, sub.last_pitch, sub.last_pilot_heading, true);
          } 
        }
        break;
        default:
        {
            // call attitude controller
            if (!is_zero(target_yaw)) { // call attitude controller with rate yaw determined by pilot input
                attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(sub.last_roll, sub.last_pitch, target_yaw);
                sub.last_pilot_heading = ahrs.yaw_sensor;
                sub.last_pilot_yaw_input_ms = tnow; // time when pilot last changed heading

            } else { // hold current heading

                if (abs(target_roll) > 50 || abs(target_pitch) > 50 || abs(target_yaw) > 50) {
                    sub.last_roll = ahrs.roll_sensor;
                    sub.last_pitch = ahrs.pitch_sensor;
                    sub.last_pilot_heading = ahrs.yaw_sensor;
                    sub.last_input_ms = tnow;
                    attitude_control->input_rate_bf_roll_pitch_yaw(target_roll, target_pitch, target_yaw);

                // this check is required to prevent bounce back after very fast yaw maneuvers
                // the inertia of the vehicle causes the heading to move slightly past the point when pilot input actually stopped
                } else if (tnow < sub.last_pilot_yaw_input_ms + 250) { // give 250ms to slow down, then set target heading
                    target_yaw = 0; // Stop rotation on yaw axis

                    // call attitude controller with target yaw rate = 0 to decelerate on yaw axis
                    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(sub.last_roll, sub.last_pitch, target_yaw);
                    sub.last_pilot_heading = ahrs.yaw_sensor; // update heading to hold

                } else { // call attitude controller holding absolute absolute bearing
                    attitude_control->input_euler_angle_roll_pitch_yaw(sub.last_roll, sub.last_pitch, sub.last_pilot_heading, true);
                }
            }
        }
    }

    // output pilot's throttle
    attitude_control->set_throttle_out(channel_throttle->norm_input(), false, g.throttle_filt);

    //control_in is range -1000-1000
    //radio_in is raw pwm value
    motors.set_forward(channel_forward->norm_input());
    motors.set_lateral(channel_lateral->norm_input());
}
