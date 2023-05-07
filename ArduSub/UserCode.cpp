#include "Sub.h"

#ifdef USERHOOK_INIT
void Sub::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
void Sub::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Sub::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Sub::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Sub::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Sub::userhook_SuperSlowLoop()
{
    if (!ahrs.healthy())
    hal.console->printf("ahrs healthy %d\n\r",ahrs.healthy());  
    if (!ahrs.EKF3.healthy())
    hal.console->printf("ekf3 healthy %d\n\r",ahrs.EKF3.healthy());  
    if (ahrs.get_vehicle_class() != AP_AHRS::VehicleClass::SUBMARINE)
    hal.console->printf("ship class %d\n\r",(uint8_t)ahrs.get_vehicle_class());  
    if (!ahrs.have_inertial_nav())
        hal.console->printf("no inertial navigation\n\r");
    /* printf("RC input roll  = %3d  ",static_cast<int8_t>(channel_roll->norm_input()*100)); 
    printf("RC up      = %3d\n\r",static_cast<int8_t>(channel_throttle->norm_input()*100)); 
    
    printf("RC input pitch = %3d  ",static_cast<int8_t>(channel_pitch->norm_input()*100)); 
    printf("RC forward = %3d\n\r",static_cast<int8_t>(channel_forward->norm_input()*100)); 
    
    printf("RC input yaw   = %3d  ",static_cast<int8_t>(channel_yaw->norm_input()*100)); 
    printf("RC sttrafe = %3d\n\r\n\r",static_cast<int8_t>(channel_lateral->norm_input()*100));  */
    //hal.console->printf("posXYsource=%d\n\r", ahrs.EKF3.configuredToUseGPSForPosXY());
    //barometer.update();
    //hal.console->printf("\r\n%ld temperature = %.2f C\n\r",AP_HAL::micros()/1000000, barometer.get_temperature(1));
    // put your 1Hz code here
}
#endif
