#include "Sub.h"

#include "RC_Channel.h"

// defining these two macros and including the RC_Channels_VarInfo
// header defines the parameter information common to all vehicle
// types
#define RC_CHANNELS_SUBCLASS RC_Channels_Sub
#define RC_CHANNEL_SUBCLASS RC_Channel_Sub

#include <RC_Channel/RC_Channels_VarInfo.h>

// note that this callback is not presently used on Plane:
int8_t RC_Channels_Sub::flight_mode_channel_number() const
{
    return 1; // sub does not have a flight mode channel
}

void RC_Channel_Sub::mode_switch_changed(modeswitch_pos_t new_pos)
{
    /* if (new_pos < 0 || (uint8_t)new_pos > copter.num_flight_modes) {
        // should not have been called
        return;
    }

    if (!copter.set_mode((Mode::Number)copter.flight_modes[new_pos].get(), ModeReason::RC_COMMAND)) {
        return;
    }

    if (!rc().find_channel_for_option(AUX_FUNC::SIMPLE_MODE) &&
        !rc().find_channel_for_option(AUX_FUNC::SUPERSIMPLE_MODE)) {
        // if none of the Aux Switches are set to Simple or Super Simple Mode then
        // set Simple Mode using stored parameters from EEPROM
        if (BIT_IS_SET(copter.g.super_simple, new_pos)) {
            copter.set_simple_mode(Copter::SimpleMode::SUPERSIMPLE);
        } else {
            copter.set_simple_mode(BIT_IS_SET(copter.g.simple_modes, new_pos) ? Copter::SimpleMode::SIMPLE : Copter::SimpleMode::NONE);
        }
    } */
}

bool RC_Channels_Sub::has_valid_input() const
{
    /* if (copter.failsafe.radio) {
        return false;
    }
    if (copter.failsafe.radio_counter != 0) {
        return false;
    } */
    return true;
}

// returns true if throttle arming checks should be run
bool RC_Channels_Sub::arming_check_throttle() const {
    return false;
    /* if ((copter.g.throttle_behavior & THR_BEHAVE_FEEDBACK_FROM_MID_STICK) != 0) {
        // center sprung throttle configured, dont run AP_Arming check
        // Copter already checks this case in its own arming checks
        return false;
    }
    return RC_Channels::arming_check_throttle(); */
}

RC_Channel * RC_Channels_Sub::get_arming_channel(void) const
{
    return sub.channel_yaw;
}


// init_aux_switch_function - initialize aux functions
void RC_Channel_Sub::init_aux_function(const aux_func_t ch_option, const AuxSwitchPos ch_flag)
{
    // init channel options
    switch(ch_option) {
    // the following functions do not need to be initialised:
    case AUX_FUNC::ALTHOLD:
    case AUX_FUNC::AUTO:
    case AUX_FUNC::AUTOTUNE:
    case AUX_FUNC::BRAKE:
    case AUX_FUNC::CIRCLE:
    case AUX_FUNC::DRIFT:
    case AUX_FUNC::FLIP:
    case AUX_FUNC::FLOWHOLD:
    case AUX_FUNC::FOLLOW:
    case AUX_FUNC::GUIDED:
    case AUX_FUNC::LAND:
    case AUX_FUNC::LOITER:
    case AUX_FUNC::PARACHUTE_RELEASE:
    case AUX_FUNC::POSHOLD:
    case AUX_FUNC::RESETTOARMEDYAW:
    case AUX_FUNC::RTL:
    case AUX_FUNC::SAVE_TRIM:
    case AUX_FUNC::SAVE_WP:
    case AUX_FUNC::SMART_RTL:
    case AUX_FUNC::STABILIZE:
    case AUX_FUNC::THROW:
    case AUX_FUNC::USER_FUNC1:
    case AUX_FUNC::USER_FUNC2:
    case AUX_FUNC::USER_FUNC3:
    case AUX_FUNC::WINCH_CONTROL:
    case AUX_FUNC::ZIGZAG:
    case AUX_FUNC::ZIGZAG_Auto:
    case AUX_FUNC::ZIGZAG_SaveWP:
    case AUX_FUNC::ACRO:
    case AUX_FUNC::AUTO_RTL:
    case AUX_FUNC::TURTLE:
    case AUX_FUNC::SIMPLE_HEADING_RESET:
    case AUX_FUNC::ARMDISARM_AIRMODE:
    case AUX_FUNC::TURBINE_START:
        break;
    case AUX_FUNC::ACRO_TRAINER:
    case AUX_FUNC::ATTCON_ACCEL_LIM:
    case AUX_FUNC::ATTCON_FEEDFWD:
    case AUX_FUNC::INVERTED:
    case AUX_FUNC::MOTOR_INTERLOCK:
    case AUX_FUNC::PARACHUTE_3POS:      // we trust the vehicle will be disarmed so even if switch is in release position the chute will not release
    case AUX_FUNC::PARACHUTE_ENABLE:
    case AUX_FUNC::PRECISION_LOITER:
    case AUX_FUNC::RANGEFINDER:
    case AUX_FUNC::SIMPLE_MODE:
    case AUX_FUNC::STANDBY:
    case AUX_FUNC::SUPERSIMPLE_MODE:
    case AUX_FUNC::SURFACE_TRACKING:
    case AUX_FUNC::WINCH_ENABLE:
    case AUX_FUNC::AIRMODE:
    case AUX_FUNC::FORCEFLYING:
    case AUX_FUNC::CUSTOM_CONTROLLER:
    case AUX_FUNC::WEATHER_VANE_ENABLE:
        run_aux_function(ch_option, ch_flag, AuxFuncTriggerSource::INIT);
        break;
    default:
        RC_Channel::init_aux_function(ch_option, ch_flag);
        break;
    }
}

// do_aux_function_change_mode - change mode based on an aux switch
// being moved
void RC_Channel_Sub::do_aux_function_change_mode(const control_mode_t mode,
                                                    const AuxSwitchPos ch_flag)
{
    /* switch(ch_flag) {
    case AuxSwitchPos::HIGH: {
        // engage mode (if not possible we remain in current flight mode)
        copter.set_mode(mode, ModeReason::RC_COMMAND);
        break;
    }
    default:
        // return to flight mode switch's flight mode if we are currently
        // in this mode
        if (copter.flightmode->mode_number() == mode) {
            rc().reset_mode_switch();
        }
    } */
}

// do_aux_function - implement the function invoked by auxiliary switches
bool RC_Channel_Sub::do_aux_function(const aux_func_t ch_option, const AuxSwitchPos ch_flag)
{
    switch(ch_option) {
        case AUX_FUNC::USER_FUNC1:
            switch (ch_flag) {
                case AuxSwitchPos::HIGH:
                    printf("user func1 HIGH\n\r");
                    break;
                case AuxSwitchPos::MIDDLE:
                    printf("user func1 MED\n\r");
                    break;
                case AuxSwitchPos::LOW:
                    printf("user func1 LOW\n\r");
                    break;
            }
            break;

        case AUX_FUNC::USER_FUNC2:
            switch (ch_flag) {
                case AuxSwitchPos::HIGH:
                    printf("user func2 HIGH\n\r");
                    break;
                case AuxSwitchPos::MIDDLE:
                    printf("user func2 MED\n\r");
                    break;
                case AuxSwitchPos::LOW:
                    printf("user func2 LOW\n\r");
                    break;
            }
            break;

        case AUX_FUNC::USER_FUNC3:
            switch (ch_flag) {
                case AuxSwitchPos::HIGH:
                    printf("user func3 HIGH\n\r");
                    break;
                case AuxSwitchPos::MIDDLE:
                    printf("user func3 MED\n\r");
                    break;
                case AuxSwitchPos::LOW:
                    printf("user func3 LOW\n\r");
                    break;
            }
            break;
        default:
            return true;// RC_Channel::do_aux_function(ch_option, ch_flag);
    }    
return true;
}

// change air-mode status
void RC_Channel_Sub::do_aux_function_change_air_mode(const AuxSwitchPos ch_flag)
{
    /* switch (ch_flag) {
    case AuxSwitchPos::HIGH:
        copter.air_mode = AirMode::AIRMODE_ENABLED;
        break;
    case AuxSwitchPos::MIDDLE:
        break;
    case AuxSwitchPos::LOW:
        copter.air_mode = AirMode::AIRMODE_DISABLED;
        break;
    } */
}


// change force flying status
void RC_Channel_Sub::do_aux_function_change_force_flying(const AuxSwitchPos ch_flag)
{
    /* switch (ch_flag) {
    case AuxSwitchPos::HIGH:
        copter.force_flying = true;
        break;
    case AuxSwitchPos::MIDDLE:
        break;
    case AuxSwitchPos::LOW:
        copter.force_flying = false;
        break;
    } */
}

