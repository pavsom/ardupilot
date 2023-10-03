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
 */

#pragma once

#include "AP_HiwonderServo.h"
#if AP_HIWONDERSERVO_ENABLED

//class AP_HiwonderServo;
#define HIWONDER_SRV_ID_DEFAULT 11
#define HIWONDER_SRV2_ID_DEFAULT 12

class AP_HiwonderServo_Device {
  enum class State{
    DETECT,
    CONFIG,
    IDLE,
    READ_CONFIG,
    READ_CONFIG_WAIT,
    WAIT,
    MOVE_MAX,
    MOVE_MIN,
    MOVE_MID
  };
  enum class Servo{
    IDLE,
    MOVING
  };
public:
  AP_HiwonderServo_Device(uint8_t _instance, AP_HiwonderServo* serialDriver);
  
  bool handleMessage(servoMessageItem& rxItem);
  bool noReply();
  uint8_t waitReply = 0;
  uint8_t getId(){return id;};
  uint8_t isDetected(){return detected > 0;};
  void setId(uint8_t _id){id = _id;};
  bool inPosition();
  uint8_t timeoutCounts = 0;
  bool detect();
  bool configure();
  void update(uint32_t _time);
  State state = State::DETECT;
  Servo servo = Servo::IDLE;
private:
  void setPosition(int16_t _position);
  void setPosition(int16_t _position, uint16_t _time);
  void readAll();
  void sendConfig();
  void send_read(uint8_t cmd);
  void send_read(uint8_t cmd, uint8_t _id);
  void send_command(uint8_t cmd);
  void changeAddress(uint8_t idOld);
  void send_command(uint8_t cmd, uint8_t param1);
  void send_command(uint8_t cmd, uint16_t param1, uint16_t param2);
  uint8_t id;
  uint8_t instance;
  AP_HiwonderServo* serialDriver;
  uint8_t detected = 0;
  // stats
  int16_t positionCurrent;
  int16_t positionSet;
  uint16_t positionTime;
  uint32_t timeCurrent;
  uint32_t timeLastMove;
  struct servoParameters{
    uint32_t posMax;
    uint32_t posMin;

    uint16_t angleSet;
    uint16_t moveTime;
    
    uint16_t angleWaitSet;
    uint16_t moveWaitTime;

    int8_t angleOffset;
    uint16_t angleMinimum;
    uint16_t angleMaximum;
    uint16_t voltageMinimum;
    uint16_t voltageMaximum;
    uint8_t temperatureLimit;
    uint8_t temperatureCurrent;
    uint16_t voltageCurrent;

    uint8_t mode;
    uint16_t speed;
    uint8_t powerOn;
    uint8_t ctrlLed;
    uint8_t errorBits;
  } params;

};


#endif  // AP_HIWONDERSERVO_ENABLED
