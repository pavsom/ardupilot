#include "Sub.h"


/*
 * control_althold.pde - init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool Sub::althold_init()
{
    if(!control_check_barometer()) {
        return false;
    }

    // initialize vertical maximum speeds and acceleration
    // sets the maximum speed up and down returned by position controller
    attitude_control.set_throttle_out(0.75, true, 100.0);
    pos_control.init_z_controller();
    pos_control.set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control.set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    attitude_control.relax_attitude_controllers();
    // initialise position and desired velocity
    float pos = stopping_distance();
    float zero = 0;
    pos_control.input_pos_vel_accel_z(pos, zero, zero);

    if(prev_control_mode != control_mode_t::STABILIZE) {
        last_roll = 0;
        last_pitch = 0;
    }
    last_pilot_heading = ahrs.yaw_sensor;
    last_input_ms = AP_HAL::millis();
    
    return true;
}


float Sub::stopping_distance() {
    const float curr_pos_z = inertial_nav.get_position_neu_cm().z;
    float curr_vel_z = inertial_nav.get_velocity_neu_cms().z;
    float distance = - (curr_vel_z * curr_vel_z) / (2 * g.pilot_accel_z);
    return curr_pos_z  + distance;
}

void Sub::handle_attitude()
{
    //static uint32_t slowpoke = 0;
    uint32_t tnow = AP_HAL::millis();
    // initialize vertical speeds and acceleration
    pos_control.set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // get pilot desired lean angles/rates
    float target_roll, target_pitch, target_yaw;

    // Check if set_attitude_target_no_gps is valid
    if (tnow - sub.set_attitude_target_no_gps.last_message_ms < 5000) {
        Quaternion(
            set_attitude_target_no_gps.packet.q
        ).to_euler(
            target_roll,
            target_pitch,
            target_yaw
        );
        target_roll = 100 * degrees(target_roll);
        target_pitch = 100 * degrees(target_pitch);
        target_yaw = 100 * degrees(target_yaw);
        last_roll = target_roll;
        last_pitch = target_pitch;
        last_pilot_heading = target_yaw;
        attitude_control.input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, target_yaw, true);
        return;
    }
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, attitude_control.get_althold_lean_angle_max_cd());
    float yaw_input =  channel_yaw->pwm_to_angle_dz_trim(channel_yaw->get_dead_zone() * gain, channel_yaw->get_radio_trim());
    /* if (++slowpoke  > 200)
        printf("dz  %4d  ",channel_yaw->get_dead_zone());
    if (slowpoke  > 200)
        printf("gain  %4d  ",static_cast<int16_t>((gain)*100));
    if (slowpoke  > 200)
    /    printf("trim  %4d  ",channel_yaw->get_radio_trim());
    if (slowpoke  > 200)
        printf("yaw_input  %4d  ",static_cast<int16_t>(yaw_input)); */

    target_yaw = get_pilot_desired_yaw_rate(yaw_input);
    //if (slowpoke  > 200)
    //    printf("target_yaw  %4d  \n\r",static_cast<int32_t>((target_yaw)));
    // If we don't have a mavlink attitude target, we use the pilot's input instead
    switch (g.control_frame) {
        case MAV_FRAME_BODY_FRD:
        {
          if (abs(target_roll) > 50 || abs(target_pitch) > 50 || abs(target_yaw) > 50) {
              last_input_ms = tnow;
              attitude_control.input_rate_bf_roll_pitch_yaw(target_roll, target_pitch, target_yaw);
              Quaternion attitude_target = attitude_control.get_attitude_target_quat();
              last_roll = degrees(attitude_target.get_euler_roll()) * 100;
              last_pitch = degrees(attitude_target.get_euler_pitch()) * 100;
              last_pilot_heading = degrees(attitude_target.get_euler_yaw()) * 100;
          } else {
              attitude_control.input_euler_angle_roll_pitch_yaw(last_roll, last_pitch, last_pilot_heading, true);
          } 
        }
        break;
        default:
        {
            // call attitude controller
            if (!is_zero(target_yaw)) { // call attitude controller with rate yaw determined by pilot input
                attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(last_roll, last_pitch, target_yaw);
                last_pilot_heading = ahrs.yaw_sensor;
                last_pilot_yaw_input_ms = tnow; // time when pilot last changed heading

            } else { // hold current heading

                if (abs(target_roll) > 50 || abs(target_pitch) > 50 || abs(target_yaw) > 50) {
                    last_roll = ahrs.roll_sensor;
                    last_pitch = ahrs.pitch_sensor;
                    last_pilot_heading = ahrs.yaw_sensor;
                    last_input_ms = tnow;
                    attitude_control.input_rate_bf_roll_pitch_yaw(target_roll, target_pitch, target_yaw);

                // this check is required to prevent bounce back after very fast yaw maneuvers
                // the inertia of the vehicle causes the heading to move slightly past the point when pilot input actually stopped
                } else if (tnow < last_pilot_yaw_input_ms + 250) { // give 250ms to slow down, then set target heading
                    target_yaw = 0; // Stop rotation on yaw axis

                    // call attitude controller with target yaw rate = 0 to decelerate on yaw axis
                    attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(last_roll, last_pitch, target_yaw);
                    last_pilot_heading = ahrs.yaw_sensor; // update heading to hold

                } else { // call attitude controller holding absolute absolute bearing
                    attitude_control.input_euler_angle_roll_pitch_yaw(last_roll, last_pitch, last_pilot_heading, true);
                }
            }
        }
    }
    //if (slowpoke  > 200) slowpoke = 0;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void Sub::althold_run()
{

    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when not auto-armed (i.e. on the ground, pilot has never raised throttle)
        attitude_control.set_throttle_out(0.75,true,100.0);
        attitude_control.relax_attitude_controllers();
        //pos_control.relax_z_controller(motors.get_throttle_hover());
        pos_control.init_z_controller();
        // initialise position and desired velocity
        float pos = stopping_distance();
        float zero = 0;
        pos_control.input_pos_vel_accel_z(pos, zero, zero);
        last_roll = 0;
        last_pitch = 0;
        last_pilot_heading = ahrs.yaw_sensor;
        return;
    }

    handle_attitude();

    control_depth();
}

#define slowpokeRate 200
//#define zrateDebug
#define altitudeSimpleControl
void Sub::control_depth() {
#ifndef zrateDebug    
    static uint32_t slowpoke = 0;
    if (slowpoke  > slowpokeRate) slowpoke = 0;
    slowpoke++;
#endif    
    // We rotate the RC inputs to the earth frame to check if the user is giving an input that would change the depth.
    // Output the Z controller + pilot input to all motors.
#ifndef altitudeSimpleControl
    Vector3f earth_frame_rc_inputs = ahrs.get_rotation_body_to_ned() * Vector3f(-channel_forward->norm_input(), -channel_lateral->norm_input(), (2.0f*(-0.5f+channel_throttle->norm_input())));
    float target_climb_rate_cm_s = get_pilot_desired_climb_rate(500 + g.pilot_speed_up * earth_frame_rc_inputs.z);
#else    
    float target_climb_rate_cm_s = get_pilot_desired_climb_rate(500 + g.pilot_speed_up * (2.0f*(-0.5f+channel_throttle->norm_input())));
#endif    
#ifdef zrateDebug        
    if (slowpoke  > slowpokeRate){
        printf("depth  %4d  ",static_cast<int32_t>(barometer.get_altitude()*100));
        printf("Zrate  %4d  ",static_cast<int32_t>(target_climb_rate_cm_s));
    }
#endif


    bool surfacing = ap.at_surface || pos_control.get_pos_target_z_cm() > g.surface_depth;
    float upper_speed_limit = surfacing ? 0 : g.pilot_speed_up;
    float lower_speed_limit = ap.at_bottom ? 0 : -get_pilot_speed_dn();
    target_climb_rate_cm_s = constrain_float(target_climb_rate_cm_s, lower_speed_limit, upper_speed_limit);


#ifdef zrateDebug    
    if (slowpoke  > slowpokeRate)
        printf("surf=%d Z1  %4d  ",static_cast<int16_t>(g.surface_depth),static_cast<int16_t>((pos_control.get_pos_target_z_cm())));
    if (slowpoke  > slowpokeRate){
        printf("atSurf %s", ap.at_surface? "yes  " : "no  ");
    }
    if (slowpoke  > slowpokeRate)
        printf("upSpLim  %4d  ",static_cast<int32_t>(upper_speed_limit));
    if (slowpoke  > slowpokeRate)
        printf("loSpLim  %4d  ",static_cast<int32_t>(lower_speed_limit));
    if (slowpoke  > slowpokeRate)
        printf("Zrate  %4d  ",static_cast<int32_t>(target_climb_rate_cm_s));
#endif

    pos_control.set_pos_target_z_from_climb_rate_cm(target_climb_rate_cm_s);


#ifdef zrateDebug        
    if (slowpoke  > slowpokeRate)
        printf("Z2  %4d  ",static_cast<int16_t>((pos_control.get_pos_target_z_cm())));
#endif    


    if (surfacing) {
        pos_control.set_alt_target_with_slew(MIN(pos_control.get_pos_target_z_cm(), g.surface_depth - 5.0f)); // set target to 5 cm below surface level
    } else if (ap.at_bottom) {
        pos_control.set_alt_target_with_slew(MAX(inertial_nav.get_position_z_up_cm() + 10.0f, pos_control.get_pos_target_z_cm())); // set target to 10 cm above bottom
    }

#ifdef zrateDebug        
    if (slowpoke  > slowpokeRate)
        printf("Z3  %4d  ",static_cast<int16_t>((pos_control.get_pos_target_z_cm())));
#endif        

    pos_control.update_z_controller();
    // Read the output of the z controller and rotate it so it always points up
    Vector3f throttle_vehicle_frame = ahrs.get_rotation_body_to_ned().transposed() * Vector3f(0, 0, motors.get_throttle_in_bidirectional());
    //TODO: scale throttle with the ammount of thrusters in the given direction
    //float thrtl = motors.get_throttle_in_bidirectional();
    float raw_throttle_factor = (ahrs.get_rotation_body_to_ned() * Vector3f(0, 0, 1.0)).xy().length();
    motors.set_throttle(throttle_vehicle_frame.z + raw_throttle_factor * channel_throttle->norm_input());

#ifdef zrateDebug        
    if (slowpoke  > slowpokeRate)
        printf("throttle  %4d  \n\r",static_cast<int16_t>((throttle_vehicle_frame.z + raw_throttle_factor * channel_throttle->norm_input())*100));
#endif    

    motors.set_forward(-throttle_vehicle_frame.x + channel_forward->norm_input());
    motors.set_lateral(-throttle_vehicle_frame.y + channel_lateral->norm_input());  
    /* if (slowpoke  > slowpokeRate)
        printf("tthrl = %f; xx = %f; y = %f; z = %f; scale = %f; in = %f; thrt = %f; frw = %f; lat = %f\n\r",
            thrtl, throttle_vehicle_frame.x, throttle_vehicle_frame.y, throttle_vehicle_frame.z, raw_throttle_factor, channel_throttle->norm_input(), 
            motors.get_throttle_in_bidirectional(),motors.get_forward(),motors.get_lateral()); */
}