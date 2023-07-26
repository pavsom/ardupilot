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
    if (disarmed(Number::MANUAL)){
        return;
    }
    
    handle_attitude();
    

    // output pilot's throttle
    attitude_control->set_throttle_out(channel_throttle->norm_input(), false, g.throttle_filt);

    //control_in is range -1000-1000
    //radio_in is raw pwm value
    motors.set_forward(channel_forward->norm_input());
    motors.set_lateral(channel_lateral->norm_input());
}
