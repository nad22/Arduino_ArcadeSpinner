/*  
 *  Arduino USB Arcade Spinner
 *  (C) Wilfried JEANNIARD [https://github.com/willoucom]
 *  
 *  Based on project by Alexey Melnikov [https://github.com/MiSTer-devel/Retro-Controllers-USB-MiSTer/blob/master/PaddleTwoControllersUSB/PaddleTwoControllersUSB.ino]
 *  Based on project by Mikael Norrgård <mick@daemonbite.com>
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

///////////////// Customizable settings /////////////////////////
// For debug (check serial monitor)
// #define DEBUG

// Spinner pulses per revolution
#define SPINNER_PPR 600

// Spinner/Mouse sensitivity
// 1 is more sensitive 
// 999 is less sensitive
int spinner_sensitivity = 15;
int mouse_sensitivity = 5;
int8_t spinner_axis_value = 0;
int8_t spinner_axis_value_temp = 0;
const int deadzone = 15;


/////////////////////////////////////////////////////////////////

// Pins used by encoder
#define pinA 3
#define pinB 2
// Pins used by buttons
#define Button0 5 // Left button
#define Button1 4 // Right button
#define Button2 15 // Small Left: Select/Coin
#define Button3 14 // Small Right: Start

////////////////////////////////////////////////////////

// ID for special support in MiSTer 
// ATT: 20 chars max (including NULL at the end) according to Arduino source code.
// Additionally serial number is used to differentiate arduino projects to have different button maps!
const char *gp_serial = "MiSTer-A1 Spinner";

#include <Mouse.h>
#include "Gamepad.h"

// Create Gamepad
Gamepad_ Gamepad;
GamepadReport rep;

// Default virtual spinner position
int16_t drvpos = 0;
// Default real spinner position
int16_t r_drvpos = 0;
// Default virtual mouse position
int16_t m_drvpos = 0;

// Variables for paddle_emu
#define SP_MAX ((SPINNER_PPR*4*270UL)/360)
int32_t sp_clamp = SP_MAX/2;

// For emulation
bool mouse_emu = 0;
bool paddle_emu = 0;
bool mr_spinner_emu = 0;
bool gamepad_emu = 0;

// Interrupt pins of Rotary Encoder
void drv_proc()
{
  static int8_t prev = drvpos;
  int8_t a = digitalRead(pinA);
  int8_t b = digitalRead(pinB);

  int8_t spval = (b << 1) | (b^a);
  int8_t diff = (prev - spval)&3;

  if(diff == 1) 
  {
    r_drvpos += 1;
    if(sp_clamp < SP_MAX) sp_clamp++;
  }
  if(diff == 3) 
  {
    r_drvpos -= 1;
    if(sp_clamp > 0) sp_clamp--;
  }

  drvpos = r_drvpos / spinner_sensitivity;
  m_drvpos = r_drvpos / mouse_sensitivity;
  prev = spval;
}

// Run at startup
void setup()
{
  #ifdef DEBUG
    Serial.begin(9600);
  #endif
  Gamepad.reset();

  // Encoder
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  // Init encoder reading
  drv_proc();
  // Attach interrupt to each pin of the encoder
  attachInterrupt(digitalPinToInterrupt(pinA), drv_proc, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB), drv_proc, CHANGE);

  // Initialize Button Pins
  pinMode(Button0, INPUT_PULLUP);
  pinMode(Button1, INPUT_PULLUP);
  pinMode(Button2, INPUT_PULLUP);
  pinMode(Button3, INPUT_PULLUP);

  // Enable mouse emulation
  if (!digitalRead(Button0)) {
    mouse_emu = !mouse_emu;
    Mouse.begin();
  } 
  // Enable paddle emulation
  if (!digitalRead(Button1)) {
    paddle_emu = !paddle_emu;
  }
  // Spinner only (AKA mr.Spinner mode)
  if (!digitalRead(Button2)) {
    // Announce the device as mr.Spinner (more explanations in the readme file)
    mr_spinner_emu = !mr_spinner_emu;
    gp_serial = "MiSTer-S1 Spinner";
  }

  if (!digitalRead(Button3)) {
    gamepad_emu = !gamepad_emu;
    gp_serial = "MiSTer-GPad";
  }
}

// Main loop
void loop()
{
  // Maus-Sensitivität per Button3 (--) und Button2 (++) anpassen
  static uint32_t lastSensChange = 0;
  uint32_t now = millis();
  if (mouse_emu && now - lastSensChange > 200) {
    if (!digitalRead(Button3)) { // Jetzt Button3 = empfindlicher
      if (mouse_sensitivity > 1) {
        mouse_sensitivity-=10;
        lastSensChange = now;
      }
    }
    if (!digitalRead(Button2)) { // Jetzt Button2 = weniger empfindlich
      if (mouse_sensitivity < 100) {
        mouse_sensitivity+=10;
        lastSensChange = now;
      }
    }
  }
  // Spinner-Sensitivität per Button3 (--) und Button2 (++) anpassen im mr-spinner Modus
  static uint32_t lastSpinnerChange = 0;
  if (mr_spinner_emu && millis() - lastSpinnerChange > 200) {
    if (!digitalRead(Button3)) { // Button3 = empfindlicher
      if (spinner_sensitivity > 1) {
        spinner_sensitivity-=10;
        lastSpinnerChange = millis();
      }
    }
    if (!digitalRead(Button2)) { // Button2 = weniger empfindlich
      if (spinner_sensitivity < 100) {
        spinner_sensitivity+=10;
        lastSpinnerChange = millis();
      }
    }
  }

 
  // Default Spinner/Paddle position;
  rep.paddle = 0;
  rep.spinner = 0;
  rep.xAxis = 0;
  rep.yAxis = 0;
  
  // Buttons
  if (mr_spinner_emu) {
    rep.b0 = !digitalRead(Button0) || !digitalRead(Button1);
  } else {
    rep.b0 = !digitalRead(Button0);
    rep.b1 = !digitalRead(Button1);
    rep.b2 = !digitalRead(Button2);
    rep.b3 = !digitalRead(Button3);
    }

  // spinner rotation
  static uint16_t prev = 0;
  int16_t val = ((int16_t)(drvpos - prev));
  if(val>127) val = 127; else if(val<-127) val = -127;
  rep.spinner = val;
  prev += val;

  // Paddle Emulation
  if(paddle_emu) {
    rep.paddle = ((sp_clamp*255)/SP_MAX);
    rep.spinner = 0;
    rep.xAxis = 0;
    rep.yAxis = 0;
  }

  // Mouse Emulation
  if(mouse_emu) {
    static uint16_t m_prev = 0;
    int16_t val = ((int16_t)(m_drvpos - m_prev));
    if(val>127) val = 127; else if(val<-127) val = -127;
    m_prev += val;
    Mouse.move(val, 0);
    rep.spinner = 0;
    rep.xAxis = 0;
    rep.yAxis = 0;
    
    // Maus-Buttons
    if (!digitalRead(Button0)) {
      Mouse.press(MOUSE_LEFT);
    } else {
      Mouse.release(MOUSE_LEFT);
    }

    if (!digitalRead(Button1)) {
      Mouse.press(MOUSE_RIGHT);
    } else {
      Mouse.release(MOUSE_RIGHT);
    }
  }

// Gamepad-Emulation (Joystick-Modus)
 if (gamepad_emu) {
  rep.b0 = !digitalRead(Button0);
  rep.b1 = !digitalRead(Button1);
  rep.b2 = !digitalRead(Button2);
  rep.b3 = !digitalRead(Button3);
  rep.spinner = 0;
  rep.paddle = 0;

  // Spinner → X-Achse
  static int16_t prev_drvpos = 0;
  int16_t delta = drvpos - prev_drvpos;
  prev_drvpos = drvpos;

  if (delta > 0 && spinner_axis_value < 127) {
    spinner_axis_value += 3;  // langsam steigern
  } else if (delta < 0 && spinner_axis_value > -127) {
    spinner_axis_value -= 3;  // langsam senken
  }


  if (spinner_axis_value > -deadzone && spinner_axis_value < deadzone) {
    spinner_axis_value_temp = 0;
  }
  else
  {
      spinner_axis_value_temp=spinner_axis_value;
  }

  rep.xAxis = spinner_axis_value_temp;
  rep.yAxis = 0;

  if (memcmp(&Gamepad._GamepadReport, &rep, sizeof(GamepadReport))) {
    Gamepad._GamepadReport = rep;
    Gamepad.send();
  }
  return;
}


  // Only report controller state if it has changed
  if (memcmp(&Gamepad._GamepadReport, &rep, sizeof(GamepadReport)))
  {
    #ifdef DEBUG
      // Very verbose debug
      Serial.print(gp_serial); Serial.print(" ");
      Serial.print(drvpos); Serial.print(" ");
      Serial.print(mouse_emu); Serial.print(" ");
      Serial.print(paddle_emu); Serial.print(" ");
      Serial.print(rep.spinner); Serial.print(" ");
      Serial.print(rep.paddle); Serial.print(" ");
      Serial.print(rep.b0); Serial.print(" ");
      Serial.print(rep.b1); Serial.print(" ");
      Serial.print(rep.b2); Serial.print(" ");
      Serial.println(rep.b3);
    #endif
    // Send Gamepad changes
    Gamepad._GamepadReport = rep;
    Gamepad.send();
  }
}
