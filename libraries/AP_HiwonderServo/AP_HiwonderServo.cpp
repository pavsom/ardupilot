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
  implementation of Hiwonder Dynamixel 2.0 protocol for controlling servos

  Portions of this code are based on the dynamixel_sdk code:
  https://github.com/HIWONDER-GIT/DynamixelSDK
  which is under the following license:

* Copyright 2017 HIWONDER CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#include "AP_HiwonderServo.h"

#if AP_HIWONDERSERVO_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>
#include "AP_HiwonderServo_Device.h"
extern const AP_HAL::HAL& hal;

#define BROADCAST_ID 0xFE
#define MAX_ID 0xFD

#define HEADER0     0x55
#define HEADER1     0x55


#define HIWONDER_SRV_ID_DEFAULT 11
#define HIWONDER_SRV2_ID_DEFAULT 12
const AP_Param::GroupInfo AP_HiwonderServo::var_info[] = {

    // @Param: POSMIN
    // @DisplayName: Hiwonder servo position min
    // @Description: Position minimum at servo min value. This should be within the position control range of the servos, normally 0 to 4095
    // @Range: 0 4095
    // @User: Standard
    AP_GROUPINFO("POSMIN",  1, AP_HiwonderServo, pos_min, 0),

    // @Param: POSMAX
    // @DisplayName: Hiwonder servo position max
    // @Description: Position maximum at servo max value. This should be within the position control range of the servos, normally 0 to 4095
    // @Range: 0 4095
    // @User: Standard
    AP_GROUPINFO("POSMAX",  2, AP_HiwonderServo, pos_max, 4095),
    
    // @Param: POSMAX
    // @DisplayName: Hiwonder servo position max
    // @Description: Position maximum at servo max value. This should be within the position control range of the servos, normally 0 to 4095
    // @Range: 0 4095
    // @User: Standard
    AP_GROUPINFO("SRV1_ID",  3, AP_HiwonderServo, srv1ID, HIWONDER_SRV_ID_DEFAULT),

    // @Param: POSMAX
    // @DisplayName: Hiwonder servo position max
    // @Description: Position maximum at servo max value. This should be within the position control range of the servos, normally 0 to 4095
    // @Range: 0 4095
    // @User: Standard
    AP_GROUPINFO("SRV2_ID",  4, AP_HiwonderServo, srv2ID, HIWONDER_SRV_ID_DEFAULT+1),
    AP_GROUPEND
};

// constructor
AP_HiwonderServo::AP_HiwonderServo(void):
    rxQueue(16),
    txQueue(64)
{
    AP_Param::setup_object_defaults(this, var_info);
    for (uint8_t i = 0; i < AP_HIWONDER_SERVO_NUM; i++)
    {
        servo[i] = new AP_HiwonderServo_Device(i, this);
    }
    // set defaults from the parameter table
    
    servo[0]->setId(srv1ID);
    servo[1]->setId(srv2ID);
}

void AP_HiwonderServo::init(void)
{
    AP_SerialManager &serial_manager = AP::serialmanager();
    port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Hiwonder,0);
    if (port) {
        baudrate = serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Hiwonder, 0);
        us_per_byte = 10 * 1e6 / baudrate;
        us_gap = 4 * 1e6 / baudrate;
    }
    currentServo = servo[activeServo % AP_HIWONDER_SERVO_NUM];
}

/*
  send a protocol 2.0 packet
 */
bool AP_HiwonderServo::sendMessage(servoMessageItem& txItem)
{
    memcpy(&txBuf[2], &txItem.buf[1], sizeof(txItem) - 1);
    txBuf[0] = HEADER1;
    txBuf[1] = HEADER1;
    uint8_t length = txBuf[3];
    uint16_t crcTx = 0;
    for (uint8_t i = 0; i < length; i++){
        crcTx += txBuf[i+2];
    }
    crcTx = ~crcTx;

    txBuf[2+length] = crcTx & 0x00FF;

    port->write(txBuf, length+3);
    last_send_us = AP_HAL::micros();
    timeout = (txItem.item.withReply? 
        100000 : ((30) * us_per_byte + us_gap));
    replyPending = txItem.item.id;
    return txItem.item.withReply? false : true;
}

uint8_t AP_HiwonderServo::addressInvalid(uint8_t _instance){
    idToDetect++;
    for (uint8_t i = 0; i < AP_HIWONDER_SERVO_NUM; i++){
        if (i == _instance) continue;
        if (servo[i]->getId() == idToDetect){
            return 0xff;
        }
    }
    if (idToDetect >= 0xFE) return 0xff;
    return idToDetect;
}

/*
  use a broadcast ping to find attached servos
 */
/* bool AP_HiwonderServo::detect_servos(void)
{
    for (uint8_t i = 0; i < AP_HIWONDER_SERVO_NUM; i++){
        if (servo[i]->detected > 0) continue;
        if (servo[i]->timeoutCounts < 10){
            send_read(i, servo[i]->getId(), SERVO_ID_READ);
            return true;
        }
        idToDetect++;
        if (adressInvalid(i)) 
            return true;
        send_read(i, idToDetect, SERVO_ID_READ);
        return true;
    }
    return false;
} */

/*
  broadcast configure all servos
 */
/* bool AP_HiwonderServo::configure_servos(void)
{
    // disable torque control
    send_command(BROADCAST_ID, REG_TORQUE_ENABLE, 0, 1);

        // disable replies unless we read
    send_command(BROADCAST_ID, REG_STATUS_RETURN, STATUS_RETURN_READ, 1);

    // use position control mode
    send_command(BROADCAST_ID, REG_OPERATING_MODE, OPMODE_POS_CONTROL, 1);

    // enable torque control
    send_command(BROADCAST_ID, REG_TORQUE_ENABLE, 1, 1);
} */


uint8_t AP_HiwonderServo::decode(char c){
    rxBuf[rxReceived] = c;
    if (rxReceived < 2){
        crc = 0;
        rxReceived = (c != HEADER1)? 0 : rxReceived + 1;
        return 1;
    }else{
        crc += c;
        if (rxReceived++ >= 3 && 
            rxReceived > ((rxBuf[3] > 7)? 
                            0 : (rxBuf[3] + 2))){
            crc = ~crc;
            rxReceived = 0;
            return (!crc)? 0 : 3;
        }
        return 2;
        //rxReceived++;
    }
}
/*
  read response bytes
 */
bool AP_HiwonderServo::read_bytes(void)
{
    int16_t nbytes = port->available();
    if (!nbytes) return false;
    if (waitingReply()) last_send_us = AP_HAL::micros();
    while (nbytes-- > 0) {
        char c = port->read();
        uint8_t res = decode(c);
        if (res == 0) {
            servoMessageItem rxItem;
            memcpy(&rxItem.buf[1], &rxBuf[2], sizeof(rxItem)-1);
            rxQueue.push(rxItem);
        }
    }
    return false;
}

bool AP_HiwonderServo::addTxItem(servoMessageItem& txItem)
{
    return txQueue.push(txItem);
}

bool AP_HiwonderServo::waitingReply(){
    for (uint8_t i = 0; i < AP_HIWONDER_SERVO_NUM; i++){
        if (servo[i]->waitReply){
            return true;
        }
    }
    return false;
}

void AP_HiwonderServo::update()
{
    if (!initialised) {
        initialised = true;
        init();
        last_send_us = AP_HAL::micros();
        return;
    }
    if (port == nullptr) {
        return;
    }

    

    read_bytes();

    if (rxQueue.available()){
        servoMessageItem rxItem;
        if (!rxQueue.peek(rxItem)) return;
        for (uint8_t i = 0; i < AP_HIWONDER_SERVO_NUM; i++){
            if (servo[i]->handleMessage(rxItem)){
                if (servo[i]->getId() == replyPending) replyPending = -1;
                rxQueue.pop();
                break;
            }
        }
        return;
    }

    if (txQueue.available() && replyPending < 0){
        servoMessageItem txItem;
        while(txQueue.peek(txItem)){
            txQueue.pop();
            if (!sendMessage(txItem)){
                return;
            }
        }
    }
    
    uint32_t now = AP_HAL::micros();
    if (replyPending >= 0 && (now - last_send_us < timeout)) {
        // waiting for last send to complete
        return;
    }
    if (replyPending >= 0){ // timeout
        for (uint8_t i = 0; i < AP_HIWONDER_SERVO_NUM; i++){
            if (servo[i]->getId() == replyPending || 
                (!servo[i]->isDetected() && servo[i]->waitReply > 0)){
                servo[i]->noReply();
                replyPending = -1;
                break;
            }
        }
        replyPending = -1;
        rxReceived = 0;
    }

    currentServo->update(AP_HAL::millis());
    activeServo++;
    currentServo = servo[activeServo % AP_HIWONDER_SERVO_NUM];
    /* if (currentServo->isDetected())
        activeServo++;
    currentServo = servo[activeServo % AP_HIWONDER_SERVO_NUM];
    
    currentServo->detect();

    currentServo->configure(); */


    
    
    
    // loop for all 16 channels
    /* for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        if (((1U<<i) & servo_mask) == 0) {
            continue;
        }
        SRV_Channel *c = SRV_Channels::srv_channel(i);
        if (c == nullptr) {
            continue;
        }
        const uint16_t pwm = c->get_output_pwm();
        const uint16_t min = c->get_output_min();
        const uint16_t max = c->get_output_max();
        float v = float(pwm - min) / (max - min);
        uint32_t value = pos_min + v * (pos_max - pos_min);
        send_command(1,i+1, REG_GOAL_POSITION, (uint8_t)value);
    } */
}

namespace AP {

AP_HiwonderServo *hiwonder()
{
    return AP_HiwonderServo::get_singleton();
}

};
#endif  // AP_HIWONDERSERVO_ENABLED
