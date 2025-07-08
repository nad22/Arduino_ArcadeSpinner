/*  Gamepad.h
 *   
 *  Based on the advanced HID library for Arduino: 
 *  https://github.com/NicoHood/HID
 *  Copyright (c) 2014-2015 NicoHood
 * 
 *  Copyright (c) 2020 Mikael Norrgård <http://daemonbite.com>
 *
 *  GNU GENERAL PUBLIC LICENSE
 *  Version 3, 29 June 2007
 *  
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *  
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *  
 */

#pragma once

#include <Arduino.h>
#include "HID.h"

extern const char* gp_serial;

typedef struct {
  union 
  {
    struct {
      uint16_t b0:  1;
      uint16_t b1:  1;
      uint16_t b2:  1;
      uint16_t b3:  1;
      uint16_t b4:  1;
      uint16_t b5:  1;
      uint16_t b6:  1;
      uint16_t b7:  1;
      uint16_t b8:  1;
      uint16_t b9:  1;
      uint16_t b10: 1;
      uint16_t b11: 1;
      uint16_t b12: 1;
      uint16_t b13: 1;
      uint16_t b14: 1;
      uint16_t b15: 1;
    };
    uint16_t buttons;
  };

  uint8_t hat;
  int8_t xAxis;
  int8_t yAxis;
  int8_t spinner;
  uint8_t paddle;

} GamepadReport;



class Gamepad_ : public PluggableUSBModule
{  
  private:
    uint8_t reportId;

  protected:
    int getInterface(uint8_t* interfaceCount);
    int getDescriptor(USBSetup& setup);
    bool setup(USBSetup& setup);
    uint8_t getShortName(char *name);

    uint8_t epType[1];
    uint8_t protocol;
    uint8_t idle;
    
  public:
    GamepadReport _GamepadReport;
    Gamepad_(void);
    void reset(void);
    void send();
};
