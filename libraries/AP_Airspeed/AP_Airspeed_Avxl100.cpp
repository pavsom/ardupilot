/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 *   analog airspeed driver
 */

#include "AP_Airspeed_Avxl100.h"

#if AP_AIRSPEED_AVXL100_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_Airspeed.h"
#include <stdio.h>
extern const AP_HAL::HAL &hal;

// scaling for 3DR analog airspeed sensor
#define VOLTS_TO_PASCAL 819

AP_Airspeed_Avxl100::AP_Airspeed_Avxl100(AP_Airspeed &_frontend, uint8_t _instance) :
    AP_Airspeed_Backend(_frontend, _instance)
{
    
}

bool AP_Airspeed_Avxl100::init()
{
#ifndef HAL_BUILD_AP_PERIPH    
    gcs().send_text(MAV_SEVERITY_INFO, "Airspeed#%d avxl100 initialized id device", get_instance());
#endif    
    return true;
}

// read the airspeed sensor
bool AP_Airspeed_Avxl100::get_differential_pressure(float &pressure)
{
#ifndef HAL_BUILD_AP_PERIPH    
    gcs().send_text(MAV_SEVERITY_INFO, "Airspeed#%d avxl100 requested pressure", get_instance());
#endif    
    return false;
}

bool AP_Airspeed_Avxl100::get_airspeed(float& airspeed){
    uint32_t tnow = AP_HAL::millis();
#ifndef HAL_BUILD_AP_PERIPH    
    forwardThrust = forwardThrust * bodyInertia + motors->get_forward() * (1 - bodyInertia);
#endif    
    dT = (tnow - lastEstimate);
    lastEstimate = tnow;
    forwardVelocity = forwardThrust * forwardVelRatio;

    dF = forwardVelocity * dT;

    /* printf("fwRaw   %4d  ",static_cast<int32_t>((motors->get_forward())*100));
    printf("fw   %4d  ",static_cast<int32_t>((forwardThrust)*100));
    printf("dT   %4ld  ",dT);
    printf("fwVel   %4d  ",static_cast<int32_t>(forwardVelocity*100));
    printf("dist   %4d  ",static_cast<int32_t>(dF*100)); */
    airspeed = forwardVelocity;
    /* printf("roll  %4d  ",static_cast<int8_t>((motors->get_roll()+motors->get_roll_ff())*100));
    printf("pitch %4d  ",static_cast<int8_t>((motors->get_pitch()+motors->get_pitch_ff())*100)); 
    printf("yaw   %4d  ",static_cast<int8_t>((motors->get_yaw()+motors->get_yaw_ff())*100));
    printf("fwd   %4d  ",static_cast<int8_t>((motors->get_forward())*100));
    printf("strf  %4d  ",static_cast<int8_t>((motors->get_lateral())*100));
    printf("updw  %4d  \n\r",static_cast<int8_t>((motors->get_throttle_bidirectional())*100));
    gcs().send_text(MAV_SEVERITY_INFO, "Airspeed#%d avxl100 requested speed", get_instance()); */
    return true;
}

#endif  // AP_AIRSPEED_ANALOG_ENABLED
