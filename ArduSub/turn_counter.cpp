// Code by Rustom Jehangir: rusty@bluerobotics.com

#include "Sub.h"
#include <stdio.h>
// Count total vehicle turns to avoid tangling tether

float bodyInertia = 0.6f;
float forwardThrust = 0.0f;
float rightThrust = 0.0f;
float upThrust = 0.0f;

float forwardVelRatio = 1.0f;
float rightVelRatio = 1.0f;
float upVelRatio = 1.0f;
uint32_t lastEstimate = 0;
uint32_t dT = 0;
float forwardVelocity = 0.0f;
float rightVelocity = 0.0f;
float upVelocity =  0.0f;
float dF = 0.0f;
float dR = 0.0f;
float trim = 0;

void Sub::update_turn_counter()
{
    // Determine state
    //AP::ahrs().writeExtNavVelData(vel_corrected, _frontend.get_vel_noise(), time_ms, _frontend.get_delay_ms());
    uint32_t tnow = AP_HAL::millis();
    if (!(static_cast<int32_t>(trim*100))){
        //trim = abs(1485 - (motors.get_pwm_output_max() + motors.get_pwm_output_min())/2);
        //trim = trim / ((motors.get_pwm_output_max() - motors.get_pwm_output_min())/2);
        
    }
    forwardThrust = forwardThrust * bodyInertia + (motors.get_forward() - trim) * (1 - bodyInertia);
    rightThrust = rightThrust * bodyInertia + (motors.get_lateral() - trim)  * (1 - bodyInertia);
    upThrust = upThrust * bodyInertia + (motors.get_throttle_in_bidirectional() - trim) * (1 - bodyInertia);
    //if (forwardThrust < -0.1) forwardThrust = 0;
    //if (rightThrust < -0.1) rightThrust = 0;
    //if (upThrust < -0.1) upThrust = 0;
    dT = (tnow - lastEstimate);
    lastEstimate = tnow;
    //printf("turn counter dT = %ld\n\r",dT);
    forwardVelocity = forwardThrust * forwardVelRatio;
    rightVelocity = rightThrust * rightVelRatio;
    upVelocity = upThrust * upVelRatio;
    dF = forwardVelocity * dT;

    /* printf("\n\rbodyFrame X=%+f; Y=%+f; Z=%+f--------\n\r",
    forwardThrust,rightThrust,upThrust); */

    Vector3f vel_corrected = ahrs.get_rotation_body_to_ned() * Vector3f(forwardVelocity, rightVelocity, --upVelocity); 
        /* printf("nedFrame1 X=%+f; Y=%+f; Z=%+f\n\r",
        vel_corrected.x,vel_corrected.y,vel_corrected.z); */

/*     Vector3f vel_corrected2 = ahrs.get_rotation_body_to_ned().transposed() * Vector3f(forwardVelocity, rightVelocity, upVelocity);
        printf("nedFrameT X=%+f; Y=%+f; Z=%+f\n\r",
        vel_corrected2.x,vel_corrected2.y,vel_corrected2.z);

    Vector3f vel_corrected3 = ahrs.get_rotation_body_to_ned() * Vector3f(forwardVelocity, rightVelocity, -upVelocity);
        printf("nedFrame3 X=%+f; Y=%+f; Z=%+f\n\r",
        vel_corrected3.x,vel_corrected3.y,vel_corrected3.z);
    
    Vector3f vel_corrected4 = ahrs.get_rotation_body_to_ned().transposed() * Vector3f(forwardVelocity, rightVelocity, -upVelocity);
        printf("nedFrame4 X=%+f; Y=%+f; Z=%+f\n\r",
        vel_corrected4.x,vel_corrected4.y,vel_corrected4.z); */

    vel_corrected.z = 0;
    AP::ahrs().writeExtNavVelData(vel_corrected, 0.3, tnow, 10);

    

    // 0: 0-90 deg, 1: 90-180 deg, 2: -180--90 deg, 3: -90--0 deg
    uint8_t turn_state;
    if (ahrs.yaw >= 0.0f && ahrs.yaw < radians(90)) {
        turn_state = 0;
    } else if (ahrs.yaw > radians(90)) {
        turn_state = 1;
    } else if (ahrs.yaw < -radians(90)) {
        turn_state = 2;
    } else {
        turn_state = 3;
    }

    // If yaw went from negative to positive (right turn)
    switch (last_turn_state) {
    case 0:
        if (turn_state == 1) {
            quarter_turn_count++;
        }
        if (turn_state == 3) {
            quarter_turn_count--;
        }
        break;
    case 1:
        if (turn_state == 2) {
            quarter_turn_count++;
        }
        if (turn_state == 0) {
            quarter_turn_count--;
        }
        break;
    case 2:
        if (turn_state == 3) {
            quarter_turn_count++;
        }
        if (turn_state == 1) {
            quarter_turn_count--;
        }
        break;
    case 3:
        if (turn_state == 0) {
            quarter_turn_count++;
        }
        if (turn_state == 2) {
            quarter_turn_count--;
        }
        break;
    }
    last_turn_state = turn_state;
}
