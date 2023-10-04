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

#include "AP_HiwonderServo_Device.h"

#if AP_HIWONDERSERVO_ENABLED


extern const AP_HAL::HAL& hal;

#define BROADCAST_ID 0xFE
#define MAX_ID 0xFD


#define SERVO_MOVE_TIME_WRITE           0x1//7
#define SERVO_MOVE_TIME_READ            0x2//3
#define SERVO_MOVE_TIME_WAIT_WRITE      0x7//7
#define SERVO_MOVE_TIME_WAIT_READ       0x8//3
#define SERVO_MOVE_START                0xB//3
#define SERVO_MOVE_STOP                 0xC//3
#define SERVO_ID_WRITE                  0xD//4
#define SERVO_ID_READ                   0xE//3
#define SERVO_ANGLE_OFFSET_ADJUST       0x11//4
#define SERVO_ANGLE_OFFSET_WRITE        0x12//3
#define SERVO_ANGLE_OFFSET_READ         0x13//3
#define SERVO_ANGLE_LIMIT_WRITE         0x14//7
#define SERVO_ANGLE_LIMIT_READ          0x15//3
#define SERVO_VIN_LIMIT_WRITE           0x16//7
#define SERVO_VIN_LIMIT_READ            0x17//3
#define SERVO_TEMP_MAX_LIMIT_WRITE      0x18//4
#define SERVO_TEMP_MAX_LIMIT_READ       0x19//3
#define SERVO_TEMP_READ                 0x1A//3
#define SERVO_VIN_READ                  0x1B//3
#define SERVO_POS_READ                  0x1C//3
#define SERVO_OR_MOTOR_MODE_WRITE       0x1D//7
#define SERVO_OR_MOTOR_MODE_READ        0x1E//3
#define SERVO_LOAD_OR_UNLOAD_WRITE      0x1F//4
#define SERVO_LOAD_OR_UNLOAD_READ       0x20//3
#define SERVO_LED_CTRL_WRITE            0x21//4
#define SERVO_LED_CTRL_READ             0x22//3
#define SERVO_LED_ERROR_WRITE           0x23//4
#define SERVO_LED_ERROR_READ            0x24//3

// 2.0 protocol packet offsets
#define HEADER0     0x55
#define HEADER1     0x55

/* Macro for Control Table Value */
#define DXL_MAKEWORD(a, b)  ((uint16_t)(((uint8_t)(((uint64_t)(a)) & 0xff)) | ((uint16_t)((uint8_t)(((uint64_t)(b)) & 0xff))) << 8))
#define DXL_MAKEDWORD(a, b) ((uint32_t)(((uint16_t)(((uint64_t)(a)) & 0xffff)) | ((uint32_t)((uint16_t)(((uint64_t)(b)) & 0xffff))) << 16))
#define DXL_LOWORD(l)       ((uint16_t)(((uint64_t)(l)) & 0xffff))
#define DXL_HIWORD(l)       ((uint16_t)((((uint64_t)(l)) >> 16) & 0xffff))
#define DXL_LOBYTE(w)       ((uint8_t)(((uint64_t)(w)) & 0xff))
#define DXL_HIBYTE(w)       ((uint8_t)((((uint64_t)(w)) >> 8) & 0xff))




AP_HiwonderServo_Device::AP_HiwonderServo_Device(uint8_t _instance, AP_HiwonderServo* _serialDriver)
{
    instance = _instance;
    id = HIWONDER_SRV_ID_DEFAULT + instance;
    detected = 0;
    serialDriver = _serialDriver;
}

bool AP_HiwonderServo_Device::handleMessage(servoMessageItem& rxItem){
    if (!repliesToReceive) return false;
    if (detected > 0 && rxItem.item.id != id) return false;
    repliesToReceive--;
    if (state == State::READ_CONFIG_WAIT){
        state = detected < 2? State::CONFIG : State::IDLE;
    } 
    timeoutCounts = 0;
    switch (rxItem.item.cmd)
    {
    case SERVO_MOVE_TIME_READ:
        params.angleSet = DXL_MAKEWORD(rxItem.item.data[0],rxItem.item.data[1]);
        params.moveTime = DXL_MAKEWORD(rxItem.item.data[2],rxItem.item.data[3]);
        break;
    case SERVO_MOVE_TIME_WAIT_READ:
        params.angleWaitSet = DXL_MAKEWORD(rxItem.item.data[0],rxItem.item.data[1]);
        params.moveWaitTime = DXL_MAKEWORD(rxItem.item.data[2],rxItem.item.data[3]);
        break;
    case SERVO_ID_READ:
        if (rxItem.item.data[0] == id){
            if (state == State::DETECT){
                detected = 1;
                state = State::READ_CONFIG;
            }
            break;
        }
        changeAddress(rxItem.item.data[0]);
        break;
    case SERVO_ANGLE_OFFSET_READ:
        params.angleOffset = (rxItem.item.data[0] & 0x80)? 
            -(int8_t)(rxItem.item.data[0] & 0x7F) : rxItem.item.data[0] & 0x7F;
        break;
    case SERVO_ANGLE_LIMIT_READ:
        params.angleMinimum = DXL_MAKEWORD(rxItem.item.data[0],rxItem.item.data[1]);
        params.angleMaximum = DXL_MAKEWORD(rxItem.item.data[2],rxItem.item.data[3]);
        break;
    case SERVO_VIN_LIMIT_READ:
        params.voltageMinimum = DXL_MAKEWORD(rxItem.item.data[0],rxItem.item.data[1]);
        params.voltageMaximum = DXL_MAKEWORD(rxItem.item.data[2],rxItem.item.data[3]);
        break;
    case SERVO_TEMP_MAX_LIMIT_READ:
        params.temperatureLimit = rxItem.item.data[0];
        break;
    case SERVO_TEMP_READ:
        params.temperatureCurrent = rxItem.item.data[0];
        break;
    case SERVO_VIN_READ:
        params.voltageCurrent = DXL_MAKEWORD(rxItem.item.data[0],rxItem.item.data[1]);
        break;
    case SERVO_POS_READ:
        positionCurrent = DXL_MAKEWORD(rxItem.item.data[0],rxItem.item.data[1]);
        break;
    case SERVO_OR_MOTOR_MODE_READ:
        params.mode = rxItem.item.data[0];
        params.speed = DXL_MAKEWORD(rxItem.item.data[2],rxItem.item.data[3]);
        break;
    case SERVO_LOAD_OR_UNLOAD_READ:
        params.powerOn = rxItem.item.data[0];
        break;
    case SERVO_LED_CTRL_READ:
        params.ctrlLed = rxItem.item.data[0];
        break;
    case SERVO_LED_ERROR_READ:
        params.errorBits = rxItem.item.data[0];
        break;
    default:
        break;
    }
    return true;
}

bool AP_HiwonderServo_Device::timeout(int16_t _id){
    if (!repliesToReceive) return false;
    if (_id != id && detected > 0) return false;
    repliesToReceive--;
    if (timeoutCounts < 200) timeoutCounts++;
    else state = State::DETECT;
    return true;
}

bool AP_HiwonderServo_Device::detect(){
    if (detected > 0) return true;
    if (timeoutCounts < 10){
        send_read(SERVO_ID_READ);
        return true;
    }
    uint8_t address = serialDriver->addressInvalid(instance);
    if (address == 0xFF) return false;
    send_read(SERVO_ID_READ, address);
    return true;
}

void AP_HiwonderServo_Device::update(uint32_t _time)
{
    timeCurrent = _time;
    switch (servo)
    {
    case Servo::IDLE:
    if (!repliesToReceive){
        send_read(SERVO_TEMP_READ);
        send_read(SERVO_LED_ERROR_READ);
    }
        break;
    case Servo::MOVING:
        if (timeCurrent - timeLastMove > positionTime && inPosition()){
                servo = Servo::IDLE;
                send_command(SERVO_LOAD_OR_UNLOAD_WRITE, 0);
        }
        break;
    default:
        break;
    }
    switch (state)
    {
    case State::DETECT:
        detected = 0;
        detect();
        break;
    case State::CONFIG:
        sendConfig();
        detected = 2;
        state = State::WAIT;
        timeLastMove = timeCurrent;
        break;
    case State::IDLE:
        if (servo == Servo::IDLE){
            state = State::WAIT;
        }
        break;
    case State::READ_CONFIG:
        readAll();
        state = State::READ_CONFIG_WAIT;
        break;
    case State::READ_CONFIG_WAIT:
        break;
    case State::WAIT:
        //if (timeCurrent - timeLastMove > 400)
            state = State::MOVE_MAX;
        break;
    case State::MOVE_MAX:
        setPosition(1000, 0);
        state = State::MOVE_MIN;
        break;
    case State::MOVE_MIN:
        if (servo == Servo::IDLE){
            setPosition(0, 1000);
            state = State::MOVE_MID;
        }
        break;
    case State::MOVE_MID:
        if (servo == Servo::IDLE){
            setPosition(500, 100);
            state = State::IDLE;
        }
        break;
    default:
        break;
    }
}

bool AP_HiwonderServo_Device::inPosition(){
    if (abs(positionCurrent - positionSet) < 10) return true;
    if (!repliesToReceive){
        if (params.angleSet != positionSet){
            send_command(SERVO_MOVE_TIME_WRITE,(uint16_t)positionSet,(uint16_t)positionTime);
        }
        send_read(SERVO_POS_READ);
        send_read(SERVO_MOVE_TIME_READ);
    }
    return false;
}


void AP_HiwonderServo_Device::setPosition(int16_t _position){
    /* positionSet = _position;
    positionTime = 0;
    timeLastMove = timeCurrent;
    servo = Servo::MOVING; */
    send_command(SERVO_MOVE_TIME_WRITE,(uint16_t)_position,(uint16_t)0);
    //send_read(SERVO_MOVE_TIME_READ);
}

void AP_HiwonderServo_Device::setPosition(int16_t _position, uint16_t _time){
    positionSet = _position;
    positionTime = _time;
    timeLastMove = timeCurrent;
    servo = Servo::MOVING;
    send_command(SERVO_MOVE_TIME_WRITE,(uint16_t)_position,(uint16_t)_time);
    send_read(SERVO_MOVE_TIME_READ);
}

void AP_HiwonderServo_Device::sendConfig()
{
    send_command(SERVO_ANGLE_OFFSET_ADJUST,(uint8_t)0);
    send_command(SERVO_ANGLE_LIMIT_WRITE,(uint16_t)0,(uint16_t)1000);
    send_command(SERVO_LOAD_OR_UNLOAD_WRITE,(uint8_t)1);

    send_command(SERVO_OR_MOTOR_MODE_WRITE,(uint16_t)0,(uint16_t)0);
    send_command(SERVO_TEMP_MAX_LIMIT_WRITE,(uint8_t)85);
    send_command(SERVO_LED_CTRL_WRITE,(uint8_t)1);
    send_command(SERVO_LED_ERROR_WRITE,(uint8_t)7);
    
}

void AP_HiwonderServo_Device::readAll()
{
    send_read(SERVO_MOVE_TIME_READ);
    //send_read(SERVO_MOVE_TIME_WAIT_READ); // bugged response with 55 55 id ff
    send_read(SERVO_ANGLE_OFFSET_READ);
    send_read(SERVO_ANGLE_LIMIT_READ);
    send_read(SERVO_VIN_LIMIT_READ);
    send_read(SERVO_TEMP_MAX_LIMIT_READ);
    send_read(SERVO_TEMP_READ);
    send_read(SERVO_VIN_READ);
    send_read(SERVO_POS_READ);
    send_read(SERVO_OR_MOTOR_MODE_READ);
    send_read(SERVO_LOAD_OR_UNLOAD_READ);
    send_read(SERVO_LED_CTRL_READ);
    send_read(SERVO_LED_ERROR_READ);
}

void AP_HiwonderServo_Device::send_read(uint8_t cmd)
{
    /* repliesToReceive++;
    servoMessageItem txItem;
    txItem.item.id = id;
    txItem.item.cmd = cmd;
    txItem.item.length = 3;
    txItem.item.withReply = 1;
    serialDriver->addTxItem(txItem); */
    send_read(cmd, id);
}

void AP_HiwonderServo_Device::send_read(uint8_t cmd, uint8_t _id)
{
    repliesToReceive++;
    servoMessageItem txItem;
    txItem.item.id = _id;
    txItem.item.cmd = cmd;
    txItem.item.length = 3;
    txItem.item.withReply = 1;
    serialDriver->addTxItem(txItem);
}

void AP_HiwonderServo_Device::changeAddress(uint8_t idOld){
    servoMessageItem txItem;
    txItem.item.id = idOld;
    txItem.item.cmd = SERVO_ID_WRITE;
    txItem.item.length = 4;
    txItem.item.data[0] = id;
    txItem.item.withReply = 0;
    serialDriver->addTxItem(txItem);
}

void AP_HiwonderServo_Device::send_command(uint8_t cmd)
{
    servoMessageItem txItem;
    txItem.item.id = id;
    txItem.item.cmd = cmd;
    txItem.item.length = 3;
    txItem.item.withReply = 0;
    serialDriver->addTxItem(txItem);
}

void AP_HiwonderServo_Device::send_command(uint8_t cmd, uint8_t param1)
{
    servoMessageItem txItem;
    txItem.item.id = id;
    txItem.item.cmd = cmd;
    txItem.item.length = 4;
    txItem.item.data[0] = param1;
    txItem.item.withReply = 0;
    serialDriver->addTxItem(txItem);
}

void AP_HiwonderServo_Device::send_command(uint8_t cmd, uint16_t param1, uint16_t param2)
{
    servoMessageItem txItem;
    txItem.item.id = id;
    txItem.item.cmd = cmd;
    txItem.item.length = 7;
    txItem.item.data[0] = DXL_LOBYTE(param1);
    txItem.item.data[1] = DXL_HIBYTE(param1);
    txItem.item.data[2] = DXL_LOBYTE(param2);
    txItem.item.data[3] = DXL_HIBYTE(param2);
    txItem.item.withReply = 0;
    serialDriver->addTxItem(txItem);
}
#endif  // AP_HIWONDERSERVO_ENABLED
