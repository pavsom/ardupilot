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
    MOVE_MID,
    MOVE1,
    MOVE2,
    MOVE3,
    MOVE4
  };
  enum class Servo{
    UNKNOWN,
    IDLE,
    MOVING
  };
  const float angleToStepsRatio= 4.1666666666666f;
  const uint16_t moveTimeMAx = 30000;
public:
  AP_HiwonderServo_Device(uint8_t _instance, AP_HiwonderServo* serialDriver);
  
  // -------------     coms   ---------- //
  bool handleMessage(servoMessageItem& rxItem);
  bool timeout(int16_t _id);
  bool waitReply(){return (repliesToReceive != 0);};
  // -------------    local   ---------- //
  void setId(uint8_t _id){id = _id;};
  uint8_t getId(){return id;};
  void update(uint32_t _time);

  const float maxSpeed = 240;
  /* uint8_t isDetected(){return detected > 0;}; */
private:
  // -------------     coms   ---------- //
  AP_HiwonderServo* serialDriver;
  uint8_t repliesToReceive = 0;
  uint8_t instance;
  uint8_t timeoutCounts = 0;
  uint32_t lastRead = 0;
  bool didWeSendIt(uint8_t _id);
  uint32_t moveResendDelay = 0;
  //uint32_t 
  // -------------    local   ---------- //
  uint8_t id;
  uint8_t detected = 0;
  uint32_t timeCurrent;
  bool detect();
  State state = State::DETECT;
  uint8_t detectId = 0;
  // ------------- position  ---------- //
  int16_t positionCurrent;

  int16_t positionSet;
  uint16_t timeSet;

  int16_t positionNeeded;
  uint16_t timeNeeded;
  
  uint32_t timeLastMove;
  Servo servo = Servo::UNKNOWN;
  // -------------  move  ---------- //
  inline void start();
  inline bool inPosition();
  
  void setDegree(float angle, float degreePerSecond);
  void moveDegree(float angle, float degreePerSecond);
  // ------------- comand ---------- //
  void sendConfig();
  inline void send_command(uint8_t cmd);
  inline void changeAddress(uint8_t idOld);
  inline void send_command(uint8_t cmd, uint8_t param1);
  inline void send_command(uint8_t cmd, uint16_t param1, uint16_t param2);

 
  // ------------- read ------------ //
  void readAll();
  inline void send_read(uint8_t cmd);
  inline void send_read(uint8_t cmd, uint8_t _id);
  // ------------ utils ------------ //
  inline float degreeToPosition(float angle);
  inline float positionToDegree(int16_t _position);
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
