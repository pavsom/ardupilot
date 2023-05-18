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
    printf("RC sttrafe = %3d\n\r\n\r",static_cast<int8_t>(channel_lateral->norm_input()*100));   */

    printf(" js0 %4d  ",static_cast<int8_t>(RC_Channels::rc_channel(0)->norm_input()*100)); 
    printf(" js1 %4d  ",static_cast<int8_t>(RC_Channels::rc_channel(1)->norm_input()*100)); 
    printf(" js2 %4d  ",static_cast<int8_t>(RC_Channels::rc_channel(2)->norm_input()*100)); 
    printf(" js3 %4d  ",static_cast<int8_t>(RC_Channels::rc_channel(3)->norm_input()*100)); 
    printf(" js4 %4d  ",static_cast<int8_t>(RC_Channels::rc_channel(4)->norm_input()*100)); 
    printf(" js5 %4d  ",static_cast<int8_t>(RC_Channels::rc_channel(5)->norm_input()*100)); 
    printf(" js6 %4d  ",static_cast<int8_t>(RC_Channels::rc_channel(6)->norm_input()*100)); 
    printf(" js7 %4d  ",static_cast<int8_t>(RC_Channels::rc_channel(7)->norm_input()*100)); 
    printf(" js8 %4d  ",static_cast<int8_t>(RC_Channels::rc_channel(8)->norm_input()*100)); 
    printf(" js9 %4d  ",static_cast<int8_t>(RC_Channels::rc_channel(9)->norm_input()*100)); 
    printf("js10 %4d  ",static_cast<int8_t>(RC_Channels::rc_channel(10)->norm_input()*100)); 
    printf("js11 %4d  \n\r",static_cast<int8_t>(RC_Channels::rc_channel(11)->norm_input()*100)); 

    printf("roll  %4d  ",static_cast<int8_t>((motors.get_roll()+motors.get_roll_ff())*100));
    printf("pitch %4d  ",static_cast<int8_t>((motors.get_pitch()+motors.get_pitch_ff())*100)); 
    printf("yaw   %4d  ",static_cast<int8_t>((motors.get_yaw()+motors.get_yaw_ff())*100));
    printf("fwd   %4d  ",static_cast<int8_t>((motors.get_forward())*100));
    printf("strf  %4d  ",static_cast<int8_t>((motors.get_lateral())*100));
    printf("updw  %4d  \n\r",static_cast<int8_t>((motors.get_throttle_bidirectional())*100));

    printf("sr0 %4d  ",SRV_Channels::srv_channel(0)->get_output_pwm()); 
    printf("sr1 %4d  ",SRV_Channels::srv_channel(1)->get_output_pwm());
    printf("sr2 %4d  ",SRV_Channels::srv_channel(2)->get_output_pwm());
    printf("sr3 %4d  ",SRV_Channels::srv_channel(3)->get_output_pwm());
    printf("sr4 %4d  ",SRV_Channels::srv_channel(4)->get_output_pwm());
    printf("sr5 %4d  ",SRV_Channels::srv_channel(5)->get_output_pwm());
    printf("sr6 %4d  ",SRV_Channels::srv_channel(6)->get_output_pwm());
    printf("sr7 %4d  \n\r\n\r",SRV_Channels::srv_channel(7)->get_output_pwm());
    

    
    //hal.console->printf("posXYsource=%d\n\r", ahrs.EKF3.configuredToUseGPSForPosXY());
    //barometer.update();
    //hal.console->printf("\r\n%ld temperature = %.2f C\n\r",AP_HAL::micros()/1000000, barometer.get_temperature(1));
    // put your 1Hz code here
}
#endif
