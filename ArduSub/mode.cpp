#include "Sub.h"

/*
  constructor for Mode object
 */
Mode::Mode(void) :
    g(sub.g),
    g2(sub.g2),
    inertial_nav(sub.inertial_nav),
    ahrs(sub.ahrs),
    motors(sub.motors),
    channel_roll(sub.channel_roll),
    channel_pitch(sub.channel_pitch),
    channel_throttle(sub.channel_throttle),
    channel_yaw(sub.channel_yaw),
    channel_forward(sub.channel_forward),
    channel_lateral(sub.channel_lateral),
    position_control(&sub.pos_control),
    attitude_control(&sub.attitude_control),
    G_Dt(sub.G_Dt)
{ };

// return the static controller object corresponding to supplied mode
Mode *Sub::mode_from_mode_num(const Mode::Number mode)
{
    Mode *ret = nullptr;

    switch (mode) {
    case Mode::Number::MANUAL:
        ret = &mode_manual;
        break;
    case Mode::Number::STABILIZE:
        ret = &mode_stabilize;
        break;
    case Mode::Number::ACRO:
        ret = &mode_acro;
        break;
    case Mode::Number::ALT_HOLD:
        ret = &mode_althold;
        break;
    case Mode::Number::POSHOLD:
        ret = &mode_poshold;
        break;
    case Mode::Number::AUTO:
        ret = &mode_auto;
        break;
    case Mode::Number::GUIDED:
        ret = &mode_guided;
        break;
    case Mode::Number::CIRCLE:
        ret = &mode_circle;
        break;
    case Mode::Number::SURFACE:
        ret = &mode_surface;
        break;
    case Mode::Number::MOTOR_DETECT:
        ret = &mode_motordetect;
        break;
    default:
        break;
    }

    return ret;
}


// set_mode - change flight mode and perform any necessary initialisation
// optional force parameter used to force the flight mode change (used only first time mode is set)
// returns true if mode was successfully set
// Some modes can always be set successfully but the return state of other flight modes should be checked and the caller should deal with failures appropriately
bool Sub::set_mode(Mode::Number mode, ModeReason reason)
{

    // return immediately if we are already in the desired mode
    if (mode == control_mode) {
        control_mode_reason = reason;
        return true;
    }

    Mode *new_flightmode = mode_from_mode_num((Mode::Number)mode);
    if (new_flightmode == nullptr) {
        notify_no_such_mode((uint8_t)mode);
        return false;
    }

    if (new_flightmode->requires_GPS() &&
        !sub.position_ok()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Mode change failed: %s requires position", new_flightmode->name());
        AP::logger().Write_Error(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(mode));
        return false;
    }

    // check for valid altitude if old mode did not require it but new one does
    // we only want to stop changing modes if it could make things worse
    if (!sub.control_check_barometer() && // maybe use ekf_alt_ok() instead?
        flightmode->has_manual_throttle() &&
        !new_flightmode->has_manual_throttle()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Mode change failed: %s need alt estimate", new_flightmode->name());
        AP::logger().Write_Error(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(mode));
        return false;
    }

    if (!new_flightmode->init(false)) {
        gcs().send_text(MAV_SEVERITY_WARNING,"Flight mode change failed %s", new_flightmode->name());
        AP::logger().Write_Error(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(mode));
        return false;
    }

    // perform any cleanup required by previous flight mode
    exit_mode(flightmode, new_flightmode);

    // store previous flight mode (only used by tradeheli's autorotation)
    prev_control_mode = control_mode;

    // update flight mode
    flightmode = new_flightmode;
    control_mode = mode;
    control_mode_reason = reason;
    logger.Write_Mode((uint8_t)control_mode, reason);
    gcs().send_message(MSG_HEARTBEAT);

    // update notify object
    notify_flight_mode();

    // return success
    return true;
}

// exit_mode - high level call to organise cleanup as a flight mode is exited
void Sub::exit_mode(Mode::Number old_control_mode, Mode::Number new_control_mode)
{
    // stop mission when we leave auto mode
    if (old_control_mode == Mode::Number::AUTO) {
        if (mission.state() == AP_Mission::MISSION_RUNNING) {
            mission.stop();
        }
#if HAL_MOUNT_ENABLED
        camera_mount.set_mode_to_default();
#endif  // HAL_MOUNT_ENABLED
    }
    if (old_control_mode != Mode::Number::ALT_HOLD &&
    new_control_mode == Mode::Number::STABILIZE){
        sub.last_roll = 0;
        sub.last_pitch = 0;
    }
    if (old_control_mode != Mode::Number::STABILIZE &&
    new_control_mode == Mode::Number::ALT_HOLD){
        sub.last_roll = 0;
        sub.last_pitch = 0;
    }
}

bool Sub::set_mode(const uint8_t new_mode, const ModeReason reason)
{
    hal.console->printf("setting mode MODE = %d\n\r", new_mode);
    static_assert(sizeof(Mode::Number) == sizeof(new_mode), "The new mode can't be mapped to the vehicles mode number");
    return sub.set_mode(static_cast<Mode::Number>(new_mode), reason);
}

// update_flight_mode - calls the appropriate attitude controllers based on flight mode
// called at 100hz or more
void Sub::update_flight_mode()
{
    flightmode->run();

    if (abs(motors.get_lateral()) < 0.1 && 
    abs(motors.get_forward()) > 0.2){
        ahrs.set_fly_forward(true);
    }else{
        ahrs.set_fly_forward(false);
    }
}

// exit_mode - high level call to organise cleanup as a flight mode is exited
void Sub::exit_mode(Mode *&old_flightmode, Mode *&new_flightmode){
#if HAL_MOUNT_ENABLED
        camera_mount.set_mode_to_default();
#endif  // HAL_MOUNT_ENABLED
}

// notify_flight_mode - sets notify object based on current flight mode.  Only used for OreoLED notify device
void Sub::notify_flight_mode()
{
    AP_Notify::flags.autopilot_mode = flightmode->is_autopilot();
    AP_Notify::flags.flight_mode = (uint8_t)control_mode;
    notify.set_flight_mode_str(flightmode->name4());
}

void Mode::handle_attitude(){
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
}

#define slowpokeRate 200
//#define zrateDebug
#define altitudeSimpleControl
void Mode::control_depth() {
#ifndef zrateDebug    
    static uint32_t slowpoke = 0;
    if (slowpoke  > slowpokeRate) slowpoke = 0;
    slowpoke++;
#endif    
    // We rotate the RC inputs to the earth frame to check if the user is giving an input that would change the depth.
    // Output the Z controller + pilot input to all motors.
#ifndef altitudeSimpleControl
    Vector3f earth_frame_rc_inputs = ahrs.get_rotation_body_to_ned() * Vector3f(-channel_forward->norm_input(), -channel_lateral->norm_input(), (2.0f*(-0.5f+channel_throttle->norm_input())));
    float target_climb_rate_cm_s = sub.get_pilot_desired_climb_rate(500 + g.pilot_speed_up * earth_frame_rc_inputs.z);
#else    
    float target_climb_rate_cm_s = sub.get_pilot_desired_climb_rate(500 + g.pilot_speed_up * (2.0f*(-0.5f+channel_throttle->norm_input())));
#endif    
#ifdef zrateDebug        
    if (slowpoke  > slowpokeRate){
        printf("depth  %4d  ",static_cast<int32_t>(barometer.get_altitude()*100));
        printf("Zrate  %4d  ",static_cast<int32_t>(target_climb_rate_cm_s));
    }
#endif

    bool burrowing = sub.ap.at_bottom || position_control->get_pos_target_z_cm() < (sub.depthTerrain + g.depth_surface);
    bool surfacing = sub.ap.at_surface || position_control->get_pos_target_z_cm() > g.depth_surface;
    float upper_speed_limit = surfacing ? 0 : g.pilot_speed_up;
    float lower_speed_limit = burrowing ? 0 : -sub.get_pilot_speed_dn();
    target_climb_rate_cm_s = constrain_float(target_climb_rate_cm_s, lower_speed_limit, upper_speed_limit);


#ifdef zrateDebug    
    if (slowpoke  > slowpokeRate)
        printf("surf=%d Z1  %4d  ",static_cast<int16_t>(g.depth_surface),static_cast<int16_t>((position_control->get_pos_target_z_cm())));
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

    position_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate_cm_s);


#ifdef zrateDebug        
    if (slowpoke  > slowpokeRate)
        printf("Z2  %4d  ",static_cast<int16_t>((position_control->get_pos_target_z_cm())));
#endif    


    if (surfacing) {
        position_control->set_alt_target_with_slew(MIN(position_control->get_pos_target_z_cm(), g.depth_surface - 5.0f)); // set target to 5 cm below surface level
    } else if (burrowing) {
        const float terrainDepth = sub.depthTerrain +10.0f;
        position_control->set_alt_target_with_slew(MAX(inertial_nav.get_position_z_up_cm() + 10.0f, MAX(terrainDepth, position_control->get_pos_target_z_cm()))); // set target to 10 cm above bottom
    }

#ifdef zrateDebug        
    if (slowpoke  > slowpokeRate)
        printf("Z3  %4d  ",static_cast<int16_t>((position_control->get_pos_target_z_cm())));
#endif        

    position_control->update_z_controller();
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

// get_pilot_desired_angle_rates - transform pilot's roll pitch and yaw input into a desired lean angle rates
// returns desired angle rates in centi-degrees-per-second
void Mode::get_pilot_desired_angle_rates(int16_t roll_in, int16_t pitch_in, int16_t yaw_in, float &roll_out, float &pitch_out, float &yaw_out)
{
    float rate_limit;
    Vector3f rate_ef_level, rate_bf_level, rate_bf_request;

    // apply circular limit to pitch and roll inputs
    float total_in = norm(pitch_in, roll_in);

    if (total_in > ROLL_PITCH_INPUT_MAX) {
        float ratio = (float)ROLL_PITCH_INPUT_MAX / total_in;
        roll_in *= ratio;
        pitch_in *= ratio;
    }

    // calculate roll, pitch rate requests
    if (g.acro_expo <= 0) {
        rate_bf_request.x = roll_in * g.acro_rp_p;
        rate_bf_request.y = pitch_in * g.acro_rp_p;
    } else {
        // expo variables
        float rp_in, rp_in3, rp_out;

        // range check expo
        if (g.acro_expo > 1.0f) {
            g.acro_expo.set(1.0f);
        }

        // roll expo
        rp_in = float(roll_in)/ROLL_PITCH_INPUT_MAX;
        rp_in3 = rp_in*rp_in*rp_in;
        rp_out = (g.acro_expo * rp_in3) + ((1 - g.acro_expo) * rp_in);
        rate_bf_request.x = ROLL_PITCH_INPUT_MAX * rp_out * g.acro_rp_p;

        // pitch expo
        rp_in = float(pitch_in)/ROLL_PITCH_INPUT_MAX;
        rp_in3 = rp_in*rp_in*rp_in;
        rp_out = (g.acro_expo * rp_in3) + ((1 - g.acro_expo) * rp_in);
        rate_bf_request.y = ROLL_PITCH_INPUT_MAX * rp_out * g.acro_rp_p;
    }

    // calculate yaw rate request
    rate_bf_request.z = yaw_in * g.acro_yaw_p;

    // calculate earth frame rate corrections to pull the vehicle back to level while in ACRO mode

    if (g.acro_trainer != ACRO_TRAINER_DISABLED) {
        // Calculate trainer mode earth frame rate command for roll
        int32_t roll_angle = wrap_180_cd(ahrs.roll_sensor);
        rate_ef_level.x = -constrain_int32(roll_angle, -ACRO_LEVEL_MAX_ANGLE, ACRO_LEVEL_MAX_ANGLE) * g.acro_balance_roll;

        // Calculate trainer mode earth frame rate command for pitch
        int32_t pitch_angle = wrap_180_cd(ahrs.pitch_sensor);
        rate_ef_level.y = -constrain_int32(pitch_angle, -ACRO_LEVEL_MAX_ANGLE, ACRO_LEVEL_MAX_ANGLE) * g.acro_balance_pitch;

        // Calculate trainer mode earth frame rate command for yaw
        rate_ef_level.z = 0;

        // Calculate angle limiting earth frame rate commands
        if (g.acro_trainer == ACRO_TRAINER_LIMITED) {
            if (roll_angle > sub.aparm.angle_max) {
                rate_ef_level.x -=  g.acro_balance_roll*(roll_angle-sub.aparm.angle_max);
            } else if (roll_angle < -sub.aparm.angle_max) {
                rate_ef_level.x -=  g.acro_balance_roll*(roll_angle+sub.aparm.angle_max);
            }

            if (pitch_angle > sub.aparm.angle_max) {
                rate_ef_level.y -=  g.acro_balance_pitch*(pitch_angle-sub.aparm.angle_max);
            } else if (pitch_angle < -sub.aparm.angle_max) {
                rate_ef_level.y -=  g.acro_balance_pitch*(pitch_angle+sub.aparm.angle_max);
            }
        }

        // convert earth-frame level rates to body-frame level rates
        attitude_control->euler_rate_to_ang_vel(attitude_control->get_att_target_euler_cd()*radians(0.01f), rate_ef_level, rate_bf_level);

        // combine earth frame rate corrections with rate requests
        if (g.acro_trainer == ACRO_TRAINER_LIMITED) {
            rate_bf_request.x += rate_bf_level.x;
            rate_bf_request.y += rate_bf_level.y;
            rate_bf_request.z += rate_bf_level.z;
        } else {
            float acro_level_mix = constrain_float(1-MAX(MAX(abs(roll_in), abs(pitch_in)), abs(yaw_in))/4500.0, 0, 1)*ahrs.cos_pitch();

            // Scale leveling rates by stick input
            rate_bf_level = rate_bf_level*acro_level_mix;

            // Calculate rate limit to prevent change of rate through inverted
            rate_limit = fabsf(fabsf(rate_bf_request.x)-fabsf(rate_bf_level.x));
            rate_bf_request.x += rate_bf_level.x;
            rate_bf_request.x = constrain_float(rate_bf_request.x, -rate_limit, rate_limit);

            // Calculate rate limit to prevent change of rate through inverted
            rate_limit = fabsf(fabsf(rate_bf_request.y)-fabsf(rate_bf_level.y));
            rate_bf_request.y += rate_bf_level.y;
            rate_bf_request.y = constrain_float(rate_bf_request.y, -rate_limit, rate_limit);

            // Calculate rate limit to prevent change of rate through inverted
            rate_limit = fabsf(fabsf(rate_bf_request.z)-fabsf(rate_bf_level.z));
            rate_bf_request.z += rate_bf_level.z;
            rate_bf_request.z = constrain_float(rate_bf_request.z, -rate_limit, rate_limit);
        }
    }

    // hand back rate request
    roll_out = rate_bf_request.x;
    pitch_out = rate_bf_request.y;
    yaw_out = rate_bf_request.z;
}


bool Mode::set_mode(Mode::Number mode, ModeReason reason)
{
    return sub.set_mode(mode, reason);
}

GCS_Sub &Mode::gcs()
{
    return sub.gcs();
}
