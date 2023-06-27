#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Motors/AP_Motors.h>

#ifndef AP_AIRSPEED_AVXL100_ENABLED
#define AP_AIRSPEED_AVXL100_ENABLED AP_AIRSPEED_BACKEND_DEFAULT_ENABLED
#endif

#if AP_AIRSPEED_AVXL100_ENABLED

#include <AP_HAL/AP_HAL.h>

#include "AP_Airspeed_Backend.h"

class AP_Airspeed_Avxl100 : public AP_Airspeed_Backend
{
public:
    AP_Airspeed_Avxl100(AP_Airspeed &frontend, uint8_t _instance);

    // probe and initialise the sensor
    bool init(void) override;

    // return the current differential_pressure in Pascal
    bool get_differential_pressure(float &pressure) override;

    // temperature not available via analog backend
    bool get_temperature(float &temperature) override { return false; }

    // true if sensor reads airspeed directly, not via pressue
    bool has_airspeed() override {return true;}

    // return airspeed in m/s if available
    bool get_airspeed(float& airspeed) override;

private:
#ifndef HAL_BUILD_AP_PERIPH
    AP_Motors6DOF* const motors = static_cast<AP_Motors6DOF*>(AP_Motors::get_singleton());
#endif    

    float bodyInertia = 0.6f;
    float forwardThrust = 0.0f;
    float rightThrust = 0.0f;

    float forwardVelRatio = 2.5f;
    float rightVelRatio = 0.5f;
    uint32_t lastEstimate = 0;
    uint32_t dT = 0;
    float forwardVelocity = 0.0f;
    float rightVelocity = 0.0f;

    float dF = 0.0f;
    float dR = 0.0f;
};

#endif  // AP_AIRSPEED_ANALOG_ENABLED
