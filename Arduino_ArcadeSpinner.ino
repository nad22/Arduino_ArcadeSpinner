/*  
 *  Arduino USB Arcade Spinner - Enhanced Fork
 *  (C) pukepals.com - This is a fork of the original project
 *  
 *  Original project by Wilfried JEANNIARD [https://github.com/willoucom]
 *  Based on project by Alexey Melnikov [https://github.com/MiSTer-devel/Retro-Controllers-USB-MiSTer/blob/master/PaddleTwoControllersUSB/PaddleTwoControllersUSB.ino]
 *  Based on project by Mikael Norrgård <mick@daemonbite.com>
 *  
 *  DESCRIPTION:
 *  This project implements a USB arcade spinner controller using an Arduino Leonardo.
 *  It supports multiple operation modes including gamepad, mouse, paddle, and dedicated
 *  spinner modes. The device features an OLED display for mode selection and real-time
 *  sensitivity adjustment, making it perfect for retro gaming and arcade emulation.
 *  
 *  HARDWARE REQUIREMENTS:
 *  - Arduino Leonardo (or compatible board with USB HID support)
 *  - Rotary encoder (600 PPR recommended for best performance)
 *  - 8-12 buttons for arcade controls
 *  - SH1106 128x64 OLED display (I2C)
 *  - Pull-up resistors for buttons (if not using internal pull-ups)
 *  
 *  FEATURES:
 *  - Gamepad Mode: Standard USB gamepad with analog stick and 8 buttons
 *  - Mouse Mode: Mouse emulation with left/right click support
 *  - Paddle Mode: Analog paddle controller for Arkanoid-style games
 *  - Mr. Spinner Mode: Specialized mode for MiSTer FPGA compatibility
 *  - Real-time sensitivity adjustment via button combinations
 *  - OLED display showing current mode and settings
 *  - Configurable deadzone for analog inputs
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

///////////////// CUSTOMIZABLE SETTINGS /////////////////////////

// Enable debug output to serial monitor for troubleshooting
// Comment out to disable debug messages for production use
#define DEBUG

// Rotary encoder specifications
#define SPINNER_PPR 600  // Pulses Per Revolution of the rotary encoder

// Sensitivity settings for different operation modes
// Higher values = less sensitive (more encoder movement required)
// Lower values = more sensitive (less encoder movement required)
int spinner_sensitivity = 15;    // Sensitivity for Mr. Spinner mode (1-100)
int mouse_sensitivity = 5;       // Sensitivity for mouse movement (1-100)
int wheel_sensitivity = 3;       // Sensitivity for gamepad wheel mode (1-10)
float paddle_sensitivity = 1.5f; // Sensitivity for paddle mode (1.0-5.0)

// Analog input processing variables
int8_t spinner_axis_value = 0;      // Current analog axis value
int8_t spinner_axis_value_temp = 0; // Temporary value for deadzone processing
const int deadzone = 15;            // Deadzone threshold to prevent jitter (0-50)

// Sensitivity adjustment state tracking
// These prevent multiple rapid changes when holding button combinations
bool wheelSensChangedInc = false;   // SELECT + R button combination state
bool wheelSensChangedDec = false;   // SELECT + L button combination state  
bool mouseSensChangedInc = false;   // Mouse sensitivity increase state
bool mouseSensChangedDec = false;   // Mouse sensitivity decrease state
bool paddleSensChangedInc = false;  // Paddle sensitivity increase state
bool paddleSensChangedDec = false;  // Paddle sensitivity decrease state



/////////////////////////////////////////////////////////////////

///////////////// HARDWARE PIN ASSIGNMENTS ////////////////////

// Rotary encoder pins (must support interrupts on Leonardo)
#define pinA 3              // Encoder channel A (interrupt pin)
#define pinB 2              // Encoder channel B (interrupt pin)

// Main arcade buttons
#define Button0 5           // Primary button (left/button 1)
#define Button1 4           // Secondary button (right/button 2) 
#define Button2 15          // Auxiliary button 3
#define Button3 14          // Auxiliary button 4

// Shoulder/trigger buttons
#define ButtonL 10          // Left shoulder button
#define ButtonR 16          // Right shoulder button

// System control buttons
#define ButtonStart A0      // Start button
#define ButtonSelect A1     // Select button

// OLED display I2C pins (SH1106 128x64)
#define OledSCK A2          // I2C clock line (SCL)
#define OledSDA A3          // I2C data line (SDA)

// D-Pad (directional pad) buttons
#define ButtonUp 6          // D-Pad up direction
#define ButtonDown 7        // D-Pad down direction  
#define ButtonLeft 8        // D-Pad left direction
#define ButtonRight 9       // D-Pad right direction

/////////////////////////////////////////////////////////////////

///////////////// DEVICE IDENTIFICATION & GLOBAL VARIABLES ///////////////////

// Device serial identifier for MiSTer FPGA compatibility
// Maximum 20 characters including NULL terminator
// Used to differentiate Arduino projects for different button mappings
const char *gp_serial = "MiSTer-GPad";

// Current operation mode display string
String modeStr = "";

// Required libraries
#include <Mouse.h>      // Arduino Mouse library for mouse emulation
#include "Gamepad.h"    // Custom gamepad library for USB HID
#include <U8g2lib.h>    // U8g2 library for OLED display control

// Function declarations
void drawDisplay();     // Display current mode and settings on OLED

// OLED display object (SH1106 128x64 with software I2C)
U8G2_SH1106_128X64_NONAME_F_SW_I2C oled(U8G2_R0, OledSCK, OledSDA, U8X8_PIN_NONE);

// USB Gamepad object and report structure
Gamepad_ Gamepad;       // Main gamepad interface
GamepadReport rep;      // Current gamepad state report

// Spinner position tracking variables
int16_t drvpos = 0;     // Virtual spinner position for gamepad/paddle modes
int16_t r_drvpos = 0;   // Raw spinner position from encoder
int16_t m_drvpos = 0;   // Mouse-specific position tracking

// Paddle emulation constants and variables
#define SP_MAX ((SPINNER_PPR*4*270UL)/360)  // Maximum paddle range (270 degrees)
int32_t sp_clamp = SP_MAX/2;                // Current paddle position (clamped)

// Operation mode flags (only one should be true at a time)
bool mouse_emu = 0;      // Mouse emulation mode
bool paddle_emu = 0;     // Paddle emulation mode  
bool mr_spinner_emu = 0; // Mr. Spinner mode (MiSTer specific)
bool gamepad_emu = 1;    // Gamepad mode (default)

///////////////// ROTARY ENCODER INTERRUPT HANDLER ///////////////////

/**
 * Rotary encoder interrupt service routine
 * This function is called whenever the encoder pins change state
 * It tracks rotation direction and updates position counters
 * Uses quadrature decoding to determine rotation direction
 */
void drv_proc()
{
  static int8_t prev = drvpos;
  int8_t a = digitalRead(pinA);  // Read encoder channel A
  int8_t b = digitalRead(pinB);  // Read encoder channel B

  // Quadrature decoding: combine channels A and B into a state value
  int8_t spval = (b << 1) | (b^a);
  int8_t diff = (prev - spval) & 3;

  // Determine rotation direction and update counters
  if(diff == 1) 
  {
    r_drvpos += 1;                          // Clockwise rotation
    if(sp_clamp < SP_MAX) sp_clamp++;       // Update paddle position (clamped)
  }
  if(diff == 3) 
  {
    r_drvpos -= 1;                          // Counter-clockwise rotation
    if(sp_clamp > 0) sp_clamp--;            // Update paddle position (clamped)
  }

  // Apply sensitivity scaling for different modes
  drvpos = r_drvpos / spinner_sensitivity;   // Gamepad/paddle mode position
  m_drvpos = r_drvpos / mouse_sensitivity;   // Mouse mode position
  prev = spval;                              // Store previous state
}

///////////////// ARDUINO SETUP FUNCTION ///////////////////

/**
 * Arduino setup function - runs once at startup
 * Initializes hardware, sets pin modes, configures interrupts,
 * detects operation mode based on button states at startup,
 * and initializes the OLED display
 */
void setup()
{
  // Initialize serial communication for debugging (if enabled)
  #ifdef DEBUG
    Serial.begin(9600);
  #endif
  
  // Reset gamepad to known state
  Gamepad.reset();

  // Set default mode
  modeStr = "Joystick / Wheel";

  // Configure rotary encoder pins with internal pull-up resistors
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  
  // Initialize encoder reading and attach interrupts
  drv_proc();  // Get initial encoder state
  attachInterrupt(digitalPinToInterrupt(pinA), drv_proc, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB), drv_proc, CHANGE);

  // Configure all button pins with internal pull-up resistors
  // Buttons are active LOW (pressed = 0, released = 1)
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

  // MODE SELECTION: Check which button is held during startup to determine operation mode
  
  // Mouse Mode: Hold Button0 during startup
  if (!digitalRead(Button0)) {
    mouse_emu = !mouse_emu;
    gamepad_emu = !gamepad_emu;
    Mouse.begin();                    // Initialize USB mouse functionality
    modeStr = "Mouse Mode";
  } 
  
  // Paddle Mode: Hold Button1 during startup  
  if (!digitalRead(Button1)) {
    paddle_emu = !paddle_emu;
    gamepad_emu = !gamepad_emu;
    modeStr = "Paddle Mode";
  }
  
  // Mr. Spinner Mode: Hold Button2 during startup
  if (!digitalRead(Button2)) {
    mr_spinner_emu = !mr_spinner_emu;
    gamepad_emu = !gamepad_emu;
    gp_serial = "MiSTer-S1 Spinner";  // Change device ID for MiSTer compatibility
    modeStr = "Mr. Spinner Mode";
  }

  // Alternative Gamepad Mode: Hold Button3 during startup
  if (!digitalRead(Button3)) {
    gamepad_emu = !gamepad_emu;
    gp_serial = "MiSTer-A1 Spinner";  // Alternative device ID
  }

  // Initialize OLED display
  oled.begin();
  oled.setBusClock(100000);           // Set I2C clock to 100kHz for stability
  oled.clearBuffer();                 // Clear display buffer
  drawDisplay();                      // Show initial mode information
}

///////////////// MAIN PROGRAM LOOP ///////////////////

/**
 * Arduino main loop function - runs continuously
 * Handles input processing, mode-specific behavior,
 * sensitivity adjustments, and USB HID communication
 */
void loop()
{
  // Reset gamepad report structure to default values
  rep.paddle = 0;     // Paddle position (0-255)
  rep.spinner = 0;    // Spinner delta movement (-127 to +127)
  rep.xAxis = 0;      // Analog X-axis value (-127 to +127)
  rep.yAxis = 0;      // Analog Y-axis value (-127 to +127)
  rep.hat = 8;        // D-pad neutral position (8 = center)

  ///////////////// MOUSE MODE SENSITIVITY ADJUSTMENT ///////////////////
  
  // Timing variables for sensitivity changes to prevent rapid adjustments
  static uint32_t lastMouseIncChange = 0;
  static uint32_t lastMouseDecChange = 0;
  const uint16_t mouseChangeDelay = 150;  // Minimum time between changes (ms)

  if (mouse_emu) {
    uint32_t now = millis();
  
    // Increase mouse sensitivity: SELECT + R buttons
    if (!digitalRead(ButtonSelect) && !digitalRead(ButtonR)) {
      if (!mouseSensChangedInc && (now - lastMouseIncChange > mouseChangeDelay)) {
        if (mouse_sensitivity < 100) {
          mouse_sensitivity += 1;
          drawDisplay();              // Update display with new value
        }
        mouseSensChangedInc = true;
        lastMouseIncChange = now;
      }
    } else {
      mouseSensChangedInc = false;
    }
  
    // Decrease mouse sensitivity: SELECT + L buttons
    if (!digitalRead(ButtonSelect) && !digitalRead(ButtonL)) {
      if (!mouseSensChangedDec && (now - lastMouseDecChange > mouseChangeDelay)) {
        if (mouse_sensitivity > 1) {
          mouse_sensitivity -= 1;
          drawDisplay();              // Update display with new value
        }
        mouseSensChangedDec = true;
        lastMouseDecChange = now;
      }
    } else {
      mouseSensChangedDec = false;
    }
  }

  ///////////////// MR. SPINNER MODE SENSITIVITY ADJUSTMENT ///////////////////
  
  static uint32_t lastSpinnerIncChange = 0;
  static uint32_t lastSpinnerDecChange = 0;
  const uint16_t spinnerChangeDelay = 150;  // Minimum time between changes (ms)
  
  if (mr_spinner_emu) {
    uint32_t now = millis();
      
    // Increase spinner sensitivity (less sensitive): SELECT + R buttons
    if (!digitalRead(ButtonSelect) && !digitalRead(ButtonR)) {
      if (!wheelSensChangedInc && (now - lastSpinnerIncChange > spinnerChangeDelay)) {
        if (spinner_sensitivity < 100) {
          spinner_sensitivity += 10;   // Larger increments for spinner mode
          drawDisplay();
        }
        wheelSensChangedInc = true;
        lastSpinnerIncChange = now;
      }
    } else {
      wheelSensChangedInc = false;
    }
  
    // Decrease spinner sensitivity (more sensitive): SELECT + L buttons
    if (!digitalRead(ButtonSelect) && !digitalRead(ButtonL)) {
      if (!wheelSensChangedDec && (now - lastSpinnerDecChange > spinnerChangeDelay)) {
        if (spinner_sensitivity > 1) {
          spinner_sensitivity -= 10;   // Larger decrements for spinner mode
          drawDisplay();
        }
        wheelSensChangedDec = true;
        lastSpinnerDecChange = now;
      }
    } else {
      wheelSensChangedDec = false;
    }
  }

  ///////////////// BUTTON STATE PROCESSING ///////////////////
  
  // Button handling differs between operation modes
  if (mr_spinner_emu || paddle_emu) {
    // In spinner/paddle modes, combine main buttons into single button
    rep.b0 = !digitalRead(Button0) || !digitalRead(Button1);
  } else {
    // In gamepad mode, map each button individually
    rep.b0 = !digitalRead(Button0);
    rep.b1 = !digitalRead(Button1);
    rep.b2 = !digitalRead(Button2);
    rep.b3 = !digitalRead(Button3);  
  }

  ///////////////// SPINNER ROTATION PROCESSING ///////////////////
  
  // Calculate spinner movement delta since last update
  static uint16_t prev = 0;
  int16_t val = ((int16_t)(drvpos - prev));
  
  // Clamp movement to prevent overflow in USB HID report
  if(val > 127) val = 127; 
  else if(val < -127) val = -127;
  
  rep.spinner = val;    // Set spinner delta movement
  prev += val;          // Update previous position tracker

  ///////////////// PADDLE MODE SENSITIVITY ADJUSTMENT ///////////////////
  
  static uint32_t lastPaddleChange = 0;
  const uint16_t paddleChangeDelay = 150;  // Minimum time between changes (ms)
  
  if (paddle_emu) {
    uint32_t now = millis();
  
    // Increase paddle sensitivity: SELECT + R buttons
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


///////////////// OLED DISPLAY FUNCTION ///////////////////

/**
 * Updates the OLED display with current mode and sensitivity information
 * Shows device title, current operation mode, and mode-specific settings
 * Called whenever mode changes or sensitivity values are adjusted
 */
void drawDisplay() {
  oled.clearBuffer();                                    // Clear display buffer
  
  // Draw title header
  oled.setFont(u8g2_font_sonicmania_te);                // Large font for title
  oled.drawStr(4, 14, "THE ARCADER");                   // Device title
  oled.drawHLine(0, 18, 128);                           // Horizontal separator line

  // Draw current mode information  
  oled.setFont(u8g2_font_helvB08_tf);                   // Standard font for content
  oled.drawStr(6, 32, "MODE: ");                        // Mode label
  oled.drawStr(9, 42, (modeStr).c_str());               // Current mode string
  
  oled.drawHLine(0, 46, 128);                           // Another separator line
  
  // Display sensitivity settings based on current operation mode
  if (mr_spinner_emu) {
    char sensStr[20];
    sprintf(sensStr, "Spinner Speed: %d", spinner_sensitivity);
    oled.drawStr(6, 57, sensStr);
  } else if (mouse_emu) {
    char sensStr[20];
    sprintf(sensStr, "Mouse Speed: %d", mouse_sensitivity);
    oled.drawStr(6, 60, sensStr);
  } else if (paddle_emu) {
    char sensStr[20];
    sprintf(sensStr, "Paddle Speed: %d", (int)(paddle_sensitivity * 2));
    oled.drawStr(6, 60, sensStr);
  } else {   
    // Default gamepad mode
    char sensStr[20];
    sprintf(sensStr, "Wheel Speed: %d", wheel_sensitivity);
    oled.drawStr(6, 60, sensStr);
  }
  
  oled.sendBuffer();                                     // Send buffer to display
}
