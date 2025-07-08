/*  Gamepad.cpp
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

#include "Gamepad.h"

static const uint8_t _hidReportDescriptor[] PROGMEM = {
  0x05, 0x01,       // USAGE_PAGE (Generic Desktop)
  0x09, 0x05,       // USAGE (Game Pad)
  0xA1, 0x01,       // COLLECTION (Application)
    0xA1, 0x00,     // COLLECTION (Physical)

      // Buttons 1-12 (12 bits)
      0x05, 0x09,   // USAGE_PAGE (Button)
      0x19, 0x01,   // USAGE_MINIMUM (Button 1)
      0x29, 0x0C,   // USAGE_MAXIMUM (Button 12)
      0x15, 0x00,   // LOGICAL_MINIMUM (0)
      0x25, 0x01,   // LOGICAL_MAXIMUM (1)
      0x75, 0x01,   // REPORT_SIZE (1 bit)
      0x95, 0x0C,   // REPORT_COUNT (12)
      0x81, 0x02,   // INPUT (Data,Var,Abs)

      // Padding 4 bits (to fill to 16 bits for buttons field)
      0x75, 0x04,   // REPORT_SIZE (4 bits)
      0x95, 0x01,   // REPORT_COUNT (1)
      0x81, 0x03,   // INPUT (Const,Var,Abs)

      // Hat switch (4 bits)
      0x05, 0x01,   // USAGE_PAGE (Generic Desktop)
      0x09, 0x39,   // USAGE (Hat switch)
      0x15, 0x00,   // LOGICAL_MINIMUM (0)
      0x25, 0x07,   // LOGICAL_MAXIMUM (7)
      0x35, 0x00,   // PHYSICAL_MINIMUM (0)
      0x46, 0x3B, 0x01, // PHYSICAL_MAXIMUM (315)
      0x65, 0x14,   // UNIT (Eng Rot: Degrees)
      0x75, 0x04,   // REPORT_SIZE (4 bits)
      0x95, 0x01,   // REPORT_COUNT (1)
      0x81, 0x42,   // INPUT (Data,Var,Abs,Null State)

      // Padding 4 bits (to fill 1 byte)
      0x75, 0x04,   // REPORT_SIZE (4 bits)
      0x95, 0x01,   // REPORT_COUNT (1)
      0x81, 0x03,   // INPUT (Const,Var,Abs)

      // X Axis
      0x05, 0x01,
      0x09, 0x30,
      0x15, 0x80,
      0x25, 0x7F,
      0x75, 0x08,
      0x95, 0x01,
      0x81, 0x02,

      // Y Axis
      0x09, 0x31,
      0x15, 0x80,
      0x25, 0x7F,
      0x75, 0x08,
      0x95, 0x01,
      0x81, 0x02,

      // Spinner (Dial)
      0x09, 0x37,
      0x15, 0x80,
      0x25, 0x7F,
      0x75, 0x08,
      0x95, 0x01,
      0x81, 0x06,

       0x09, 0x38,                   // USAGE (Wheel)
      0x15, 0x00,                   // LOGICAL_MINIMUM (0)
      0x26, 0xFF, 0x00,             // LOGICAL_MAXIMUM (255)
      0x95, 0x01,                   // REPORT_COUNT (1)
      0x75, 0x08,                   // REPORT_SIZE (8)
      0x81, 0x02,                   // INPUT (Data,Var,Abs)

    0xC0,           // END_COLLECTION
  0xC0              // END_COLLECTION
};


Gamepad_::Gamepad_(void) : PluggableUSBModule(1, 1, epType), protocol(HID_REPORT_PROTOCOL), idle(1)
{
  epType[0] = EP_TYPE_INTERRUPT_IN;
  PluggableUSB().plug(this);
}

int Gamepad_::getInterface(uint8_t* interfaceCount)
{
  *interfaceCount += 1; // uses 1
  HIDDescriptor hidInterface = {
    D_INTERFACE(pluggedInterface, 1, USB_DEVICE_CLASS_HUMAN_INTERFACE, HID_SUBCLASS_NONE, HID_PROTOCOL_NONE),
    D_HIDREPORT(sizeof(_hidReportDescriptor)),
    D_ENDPOINT(USB_ENDPOINT_IN(pluggedEndpoint), USB_ENDPOINT_TYPE_INTERRUPT, USB_EP_SIZE, 0x01)
  };
  return USB_SendControl(0, &hidInterface, sizeof(hidInterface));
}

int Gamepad_::getDescriptor(USBSetup& setup)
{
  // Check if this is a HID Class Descriptor request
  if (setup.bmRequestType != REQUEST_DEVICETOHOST_STANDARD_INTERFACE) { return 0; }
  if (setup.wValueH != HID_REPORT_DESCRIPTOR_TYPE) { return 0; }

  // In a HID Class Descriptor wIndex cointains the interface number
  if (setup.wIndex != pluggedInterface) { return 0; }

  // Reset the protocol on reenumeration. Normally the host should not assume the state of the protocol
  // due to the USB specs, but Windows and Linux just assumes its in report mode.
  protocol = HID_REPORT_PROTOCOL;

  return USB_SendControl(TRANSFER_PGM, _hidReportDescriptor, sizeof(_hidReportDescriptor));
}

bool Gamepad_::setup(USBSetup& setup)
{
  if (pluggedInterface != setup.wIndex) {
    return false;
  }

  uint8_t request = setup.bRequest;
  uint8_t requestType = setup.bmRequestType;

  if (requestType == REQUEST_DEVICETOHOST_CLASS_INTERFACE)
  {
    if (request == HID_GET_REPORT) {
      // TODO: HID_GetReport();
      return true;
    }
    if (request == HID_GET_PROTOCOL) {
      // TODO: Send8(protocol);
      return true;
    }
  }

  if (requestType == REQUEST_HOSTTODEVICE_CLASS_INTERFACE)
  {
    if (request == HID_SET_PROTOCOL) {
      protocol = setup.wValueL;
      return true;
    }
    if (request == HID_SET_IDLE) {
      idle = setup.wValueL;
      return true;
    }
    if (request == HID_SET_REPORT)
    {
    }
  }

  return false;
}

void Gamepad_::reset()
{
  _GamepadReport.paddle = 0;
  _GamepadReport.spinner = 0;
  _GamepadReport.buttons = 0;
  _GamepadReport.xAxis = 0;
  _GamepadReport.yAxis = 0;
  this->send();
}


void Gamepad_::send() 
{
  USB_Send(pluggedEndpoint | TRANSFER_RELEASE, &_GamepadReport, sizeof(GamepadReport));
}

uint8_t Gamepad_::getShortName(char *name)
{
  if(!next) 
  {
    strcpy(name, gp_serial);
    return strlen(name);
  }
  return 0;
}
