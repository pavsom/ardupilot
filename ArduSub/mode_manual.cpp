#include "Sub.h"


bool ModeManual::init(bool ignore_checks) {
    // set target altitude to zero for reporting
    position_control->set_pos_target_z_cm(0);

    // attitude hold inputs become thrust inputs in manual mode
    // set to neutral to prevent chaotic behavior (esp. roll/pitch)
    sub.set_neutral_controls();

    return true;
}

// manual_run - runs the manual (passthrough) controller
// should be called at 100hz or more
void ModeManual::run()
{
    // if not armed set throttle to zero and exit immediately
    if (disarmed(Number::MANUAL)){
        return;
    }
    
    sub.motors.set_roll(channel_roll->norm_input());
    sub.motors.set_pitch(channel_pitch->norm_input());
    sub.motors.set_yaw(channel_yaw->norm_input() * g.acro_yaw_p / ACRO_YAW_P);
    sub.motors.set_throttle(channel_throttle->norm_input());
    sub.motors.set_forward(channel_forward->norm_input());
    sub.motors.set_lateral(channel_lateral->norm_input());
}
