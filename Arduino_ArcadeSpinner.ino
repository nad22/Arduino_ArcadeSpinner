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
 #define DEBUG

// Spinner pulses per revolution
#define SPINNER_PPR 600

int spinner_sensitivity = 15;
int mouse_sensitivity = 5;
int wheel_sensitivity = 3;
float paddle_sensitivity = 1.5f;  // Startwert
int8_t spinner_axis_value = 0;
int8_t spinner_axis_value_temp = 0;
const int deadzone = 15;

bool wheelSensChangedInc = false;  // für SELECT + R erhöhen
bool wheelSensChangedDec = false;  // für SELECT + L verringern
bool mouseSensChangedInc = false;  // für SELECT + R erhöhen
bool mouseSensChangedDec = false;  // für SELECT + L verringern
bool paddleSensChangedInc = false;  // für SELECT + R erhöhen
bool paddleSensChangedDec = false;  // für SELECT + L verringern



/////////////////////////////////////////////////////////////////

// Pins used by encoder
#define pinA 3
#define pinB 2
// Pins used by buttons
#define Button0 5 //Button 0
#define Button1 4 //Button 1
#define Button2 15 //Button 2
#define Button3 14 //Button 3
#define ButtonL 10 //Button L
#define ButtonR 16 //Button R
#define ButtonStart A0 //Button START
#define ButtonSelect A1 //Button SELECT
#define OledSCK A2
#define OledSDA A3
#define ButtonUp 6 // DPAD UP
#define ButtonDown 7 // DPAD DOWN
#define ButtonLeft 8 // DPAD LEFT
#define ButtonRight 9 // DPAD RIGHT

////////////////////////////////////////////////////////

// ID for special support in MiSTer 
// ATT: 20 chars max (including NULL at the end) according to Arduino source code.
// Additionally serial number is used to differentiate arduino projects to have different button maps!
const char *gp_serial = "MiSTer-GPad";

String modeStr="";


#include <Mouse.h>
#include "Gamepad.h"
#include <U8g2lib.h>
//#include <SoftwareWire.h>


// I2C-Verbindung über SoftwareWire
//SoftwareWire myWire(OledSDA, OledSCK); // SDA, SCL

// OLED-Objekt (verwende dein Display-Modell)
U8G2_SH1106_128X64_NONAME_F_SW_I2C oled(U8G2_R0, OledSCK, OledSDA, U8X8_PIN_NONE);


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
bool gamepad_emu = 1;


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

  modeStr="Joystick / Wheel";

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
  pinMode(ButtonUp, INPUT_PULLUP);
  pinMode(ButtonDown, INPUT_PULLUP);
  pinMode(ButtonLeft, INPUT_PULLUP);
  pinMode(ButtonRight, INPUT_PULLUP);
  pinMode(ButtonL, INPUT_PULLUP);
  pinMode(ButtonR, INPUT_PULLUP);
  pinMode(ButtonStart, INPUT_PULLUP);
  pinMode(ButtonSelect, INPUT_PULLUP);

  // Enable mouse emulation
  if (!digitalRead(Button0)) {
    mouse_emu = !mouse_emu;
    gamepad_emu = !gamepad_emu;
    Mouse.begin();
    modeStr="Mouse Mode";
  } 
  // Enable paddle emulation
  if (!digitalRead(Button1)) {
    paddle_emu = !paddle_emu;
    gamepad_emu = !gamepad_emu;
    //gp_serial = "MiSTer-S1 Spinner";
    modeStr="Paddle Mode";
  }
  // Spinner only (AKA mr.Spinner mode)
  if (!digitalRead(Button2)) {
    // Announce the device as mr.Spinner (more explanations in the readme file)
    mr_spinner_emu = !mr_spinner_emu;
    gamepad_emu = !gamepad_emu;
    gp_serial = "MiSTer-S1 Spinner";
    modeStr="Mr. Spinner Mode";
  }

  if (!digitalRead(Button3)) {
    gamepad_emu = !gamepad_emu;
    gp_serial = "MiSTer-A1 Spinner";
  }

  oled.begin();
  oled.setBusClock(100000); // I2C auf 100 kHz setzen für mehr Stabilität
  oled.clearBuffer();
  drawDisplay();
}

// Main loop
void loop()
{

     
  //static GamepadReport rep = {}; // Struktur für aktuellen Zustand

  // ---- Reset aller Werte zu Beginn ----
  //memset(&rep, 0, sizeof(GamepadReport));
  

  // Maus-Sensitivität per Button3 (--) und Button2 (++) anpassen
  static uint32_t lastSensChange = 0;
  uint32_t now = millis();
  
  static uint32_t lastMouseIncChange = 0;
  static uint32_t lastMouseDecChange = 0;
  const uint16_t mouseChangeDelay = 150;

  if (mouse_emu) {
    uint32_t now = millis();
  
    // Erhöhen mit SELECT + R
    if (!digitalRead(ButtonSelect) && !digitalRead(ButtonR)) {
      if (!mouseSensChangedInc && (now - lastMouseIncChange > mouseChangeDelay)) {
        if (mouse_sensitivity < 100) {
          mouse_sensitivity += 1;
          drawDisplay();
        }
        mouseSensChangedInc = true;
        lastMouseIncChange = now;
      }
    } else {
      mouseSensChangedInc = false;
    }
  
    // Verringern mit SELECT + L
    if (!digitalRead(ButtonSelect) && !digitalRead(ButtonL)) {
      if (!mouseSensChangedDec && (now - lastMouseDecChange > mouseChangeDelay)) {
        if (mouse_sensitivity > 1) {
          mouse_sensitivity -= 1;
          drawDisplay();
        }
        mouseSensChangedDec = true;
        lastMouseDecChange = now;
      }
    } else {
      mouseSensChangedDec = false;
    }
  }

  static uint32_t lastSpinnerIncChange = 0;
  static uint32_t lastSpinnerDecChange = 0;
  const uint16_t spinnerChangeDelay = 150;
  
  if (mr_spinner_emu) {
      
    // Erhöhen mit Button2 (weniger empfindlich)
    if (!digitalRead(ButtonSelect) && !digitalRead(ButtonR)) {
      if (!wheelSensChangedInc && (now - lastSpinnerIncChange > spinnerChangeDelay)) {
        if (spinner_sensitivity < 100) {
          spinner_sensitivity += 10;
          drawDisplay();
        }
        wheelSensChangedInc = true;
        lastSpinnerIncChange = now;
      }
    } else {
      wheelSensChangedInc = false;
    }
  
    // Verringern mit Button3 (empfindlicher)
    if (!digitalRead(ButtonSelect) && !digitalRead(ButtonL)) {
      if (!wheelSensChangedDec && (now - lastSpinnerDecChange > spinnerChangeDelay)) {
        if (spinner_sensitivity > 1) {
          spinner_sensitivity -= 10;
          drawDisplay();
        }
        wheelSensChangedDec = true;
        lastSpinnerDecChange = now;
      }
    } else {
      wheelSensChangedDec = false;
    }
  }


 
  // Default Spinner/Paddle/HAT position;
  rep.paddle = 0;
  rep.spinner = 0;
  rep.xAxis = 0;
  rep.yAxis = 0;
  rep.hat = 8; // Neutralposition

  
  // Buttons
  if (mr_spinner_emu || paddle_emu) {
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


  static uint32_t lastPaddleChange = 0;
  const uint16_t paddleChangeDelay = 150;
  
  if (paddle_emu) {
    uint32_t now = millis();
  
    // Erhöhen mit SELECT + R
    if (!digitalRead(ButtonSelect) && !digitalRead(ButtonR)) {
      if (!paddleSensChangedInc && (now - lastPaddleChange > paddleChangeDelay)) {
        if (paddle_sensitivity < 5.0f) {
          paddle_sensitivity += 0.5f;
          drawDisplay();
        }
        lastPaddleChange = now;
        paddleSensChangedInc = true;
      }
    } else {
      paddleSensChangedInc = false;
    }
  
    // Verringern mit SELECT + L
    if (!digitalRead(ButtonSelect) && !digitalRead(ButtonL)) {
      if (!paddleSensChangedDec && (now - lastPaddleChange > paddleChangeDelay)) {
        if (paddle_sensitivity > 1.0f) {
          paddle_sensitivity -= 0.5f;
          drawDisplay();
        }
        lastPaddleChange = now;
        paddleSensChangedDec = true;
      }
    } else {
      paddleSensChangedDec = false;
    }
  }



  // Paddle Emulation
  if(paddle_emu) {
    rep.paddle = constrain(int(((float(sp_clamp) * 255.0f) / (float(SP_MAX) / paddle_sensitivity)) + 0.5f), 0, 255);
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

 // Erhöhen mit SELECT + R
  if (!digitalRead(ButtonSelect) && !digitalRead(ButtonR)) {
    if (!wheelSensChangedInc) {
      wheel_sensitivity += 1;
      if (wheel_sensitivity > 10) wheel_sensitivity = 10; // obere Grenze
      wheelSensChangedInc = true;
      drawDisplay();
    }
  } else {
    wheelSensChangedInc = false;
  }

  // Verringern mit SELECT + L
  if (!digitalRead(ButtonSelect) && !digitalRead(ButtonL)) {
    if (!wheelSensChangedDec) {
      if (wheel_sensitivity > 1) {
        wheel_sensitivity -= 1;
      }
      wheelSensChangedDec = true;
      drawDisplay();
    }
  } else {
    wheelSensChangedDec = false;
  }

  
    uint8_t hat = 8; // Neutral = 8
    
    bool up    = !digitalRead(ButtonUp);
    bool down  = !digitalRead(ButtonDown);
    bool left  = !digitalRead(ButtonLeft);
    bool right = !digitalRead(ButtonRight);
    
    if (up && !down) {
      if (right && !left) {
        hat = 1; // oben rechts
      } else if (left && !right) {
        hat = 7; // oben links
      } else {
        hat = 0; // oben
      }
    } else if (down && !up) {
      if (right && !left) {
        hat = 3; // unten rechts
      } else if (left && !right) {
        hat = 5; // unten links
      } else {
        hat = 4; // unten
      }
    } else if (left && !right) {
      hat = 6; // links
    } else if (right && !left) {
      hat = 2; // rechts
    } else {
      hat = 8; // neutral
    }
    
    // Dann im Report setzen:
    rep.hat = hat;
     
    rep.buttons = 0;
    if (!digitalRead(Button0)) rep.buttons |= (1 << 0);
    if (!digitalRead(Button1)) rep.buttons |= (1 << 1);
    if (!digitalRead(Button2)) rep.buttons |= (1 << 2);
    if (!digitalRead(Button3)) rep.buttons |= (1 << 3);
    if (!digitalRead(ButtonL)) rep.buttons |= (1 << 4);
    if (!digitalRead(ButtonR)) rep.buttons |= (1 << 5);
    if (!digitalRead(ButtonSelect)) rep.buttons |= (1 << 6);
    if (!digitalRead(ButtonStart)) rep.buttons |= (1 << 7);


  
  
    rep.spinner = 0;
    rep.paddle = 0;
  
    // Spinner → X-Achse
    static int16_t prev_drvpos = 0;
    int16_t delta = drvpos - prev_drvpos;
    prev_drvpos = drvpos;
  
    if (delta > 0) {
    spinner_axis_value = min(spinner_axis_value + wheel_sensitivity, 127);
    } else if (delta < 0) {
      spinner_axis_value = max(spinner_axis_value - wheel_sensitivity, -127);
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

  /* if (memcmp(&Gamepad._GamepadReport, &rep, sizeof(GamepadReport))) {
    Gamepad._GamepadReport = rep;
    Gamepad.send();
  } 
  return; */
}

    // Jetzt Buttons filtern, wenn SELECT + R gedrückt (Button 6 und 5)
  if (!digitalRead(ButtonSelect) && !digitalRead(ButtonR)) {
    rep.buttons &= ~( (1 << 5) | (1 << 6) ); // clear bits für R und SELECT
  }

  // Wenn SELECT + L gedrückt (Button 6 und 4)
  if (!digitalRead(ButtonSelect) && !digitalRead(ButtonL)) {
    rep.buttons &= ~( (1 << 4) | (1 << 6) ); // clear bits für L und SELECT
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
     Serial.print("Buttons Raw: ");
Serial.println(rep.buttons, BIN);


Serial.print(" sp_clamp: "); Serial.print(sp_clamp);
Serial.print(" rep.paddle: "); Serial.print(rep.paddle);
Serial.print(" SP_MAX: "); Serial.print(SP_MAX);
Serial.print(" Sens: "); Serial.print(paddle_sensitivity);
Serial.print(" scaled: ");
Serial.println((sp_clamp * 255.0f) / (SP_MAX / paddle_sensitivity));


    #endif

    
    // Send Gamepad changes
    Gamepad._GamepadReport = rep;
    Gamepad.send();
  }
}


void drawDisplay() {
  oled.clearBuffer();
  oled.setFont(u8g2_font_sonicmania_te);
  oled.drawStr(4, 14, "THE ARCADER");
  oled.drawHLine(0, 18, 128);

  oled.setFont(u8g2_font_helvB08_tf);

  oled.drawStr(6, 32, ("MODE: "));
  oled.setFont(u8g2_font_helvB08_tf);
  oled.drawStr(9, 42, (modeStr).c_str());
  
  oled.drawHLine(0, 46, 128);
  // Bei bestimmten Modi Sensitivität anzeigen
  if (mr_spinner_emu) {
    char sensStr[20];
    sprintf(sensStr, "Spinner Speed: %d", spinner_sensitivity);
    oled.drawStr(6, 57, sensStr);
  }else if (mouse_emu) {
    char sensStr[20];
    sprintf(sensStr, "Mouse Speed: %d", mouse_sensitivity);
    oled.drawStr(6, 60, sensStr);
  }else if (paddle_emu) {
    char sensStr[20];
    sprintf(sensStr, "Paddle Speed: %d", (int)(paddle_sensitivity * 2));
    oled.drawStr(6, 60, sensStr);
  } else {   
    char sensStr[20];
    sprintf(sensStr, "Wheel Speed: %d", wheel_sensitivity);
    oled.drawStr(6, 60, sensStr);
    }
  oled.sendBuffer();
}
