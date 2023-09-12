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
#pragma once

#include "AP_Notify_config.h"

#if AP_NOTIFY_NEOPIXEL_ENABLED

#include "RGBLed.h"
#include "SerialLED.h"
#include <AP_Common/AP_Common.h>

class NeoPixel: public SerialLED {
public:
    NeoPixel();

    uint16_t init_ports() override;


    void rgb_set_id(uint8_t r, uint8_t g, uint8_t b, uint8_t id) override;

    virtual void update() override;
private:
    uint16_t num_leds;
    struct RGB {
        uint8_t b;
        uint8_t r;
        uint8_t g;
    };
    RGB **rgb;
    uint16_t enable_mask;
    bool needUpdate = false;
};

#endif  // AP_NOTIFY_NEOPIXEL_ENABLED
