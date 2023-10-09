#pragma once


#include <AP_HAL/AP_HAL_Boards.h>
#ifndef AP_HIWONDERSERVO_ENABLED
#define AP_HIWONDERSERVO_ENABLED 0
#endif

struct messageStructure{
  uint8_t withReply;
  uint8_t id;
  uint8_t length;
  uint8_t cmd;
  uint8_t data[7];
};
union servoMessageItem{
  messageStructure item;
  uint8_t buf[11];
};
