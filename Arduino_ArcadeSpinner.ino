/*  
 *  Arduino USB Arcade Spinner
 *  (C) Wilfried JEANNIARD [https://github.com/willoucom]
 *  
 *  Based on projects by:
 *    - Alexey Melnikov [https://github.com/MiSTer-devel/Retro-Controllers-USB-MiSTer/blob/master/PaddleTwoControllersUSB/PaddleTwoControllersUSB.ino]
 *    - Mikael Norrgård <mick@daemonbite.com>
 *  
 *  License: GNU GENERAL PUBLIC LICENSE v3.0
 *  https://www.gnu.org/licenses/
 */

/////////////////////// Configurable Settings ///////////////////////

#define DEBUG                      // Enable serial debug output

#define SPINNER_PPR 600            // Spinner pulses per revolution

int spinner_sensitivity = 15;
int mouse_sensitivity = 5;
int wheel_sensitivity = 3;
float paddle_sensitivity = 2.0f;

int8_t spinner_axis_value = 0;
int8_t spinner_axis_value_temp = 0;

const int deadzone = 15;           // Deadzone for analog X-axis

// Flags to track sensitivity changes
bool wheelSensChangedInc = false;
bool wheelSensChangedDec = false;
bool mouseSensChangedInc = false;
bool mouseSensChangedDec = false;
bool paddleSensChangedInc = false;
bool paddleSensChangedDec = false;

/////////////////////////////////////////////////////////////////////

/////////////////////// Pin Definitions /////////////////////////////

// Rotary Encoder
#define pinA 3
#define pinB 2

// Action Buttons
#define Button0      5
#define Button1      4
#define Button2     15
#define Button3     14
#define ButtonL     10
#define ButtonR     16
#define ButtonStart A0
#define ButtonSelect A1

// OLED I2C pins
#define OledSCK A2
#define OledSDA A3

// D-Pad Buttons
#define ButtonUp     6
#define ButtonDown   7
#define ButtonLeft   8
#define ButtonRight  9

/////////////////////////////////////////////////////////////////////

const char *gp_serial = "MiSTer-GPad";   // USB identifier (max 20 chars)

String modeStr = "";

#include <Mouse.h>
#include "Gamepad.h"
#include <U8g2lib.h>

// OLED display instance
U8G2_SH1106_128X64_NONAME_F_SW_I2C oled(U8G2_R0, OledSCK, OledSDA, U8X8_PIN_NONE);

// Gamepad and report objects
Gamepad_ Gamepad;
GamepadReport rep;

// Spinner & Paddle tracking
int16_t drvpos = 0;
int16_t r_drvpos = 0;
int16_t m_drvpos = 0;

#define SP_MAX ((SPINNER_PPR * 4 * 270UL) / 360)
int32_t sp_clamp = SP_MAX / 2;

// Mode Flags
bool mouse_emu = 0;
bool paddle_emu = 0;
bool mr_spinner_emu = 0;
bool gamepad_emu = 1;

/////////////////////////////////////////////////////////////////////
// Rotary encoder interrupt handler
/////////////////////////////////////////////////////////////////////

void drv_proc() {
  static int8_t prev = drvpos;
  int8_t a = digitalRead(pinA);
  int8_t b = digitalRead(pinB);

  int8_t spval = (b << 1) | (b ^ a);
  int8_t diff = (prev - spval) & 3;

  if (diff == 1) {
    r_drvpos += 1;
    if (sp_clamp < SP_MAX) sp_clamp++;
  } else if (diff == 3) {
    r_drvpos -= 1;
    if (sp_clamp > 0) sp_clamp--;
  }

  drvpos = r_drvpos / spinner_sensitivity;
  m_drvpos = r_drvpos / mouse_sensitivity;
  prev = spval;
}

/////////////////////////////////////////////////////////////////////
// Setup function (runs once on startup)
/////////////////////////////////////////////////////////////////////

void setup() {
  #ifdef DEBUG
    Serial.begin(9600);
  #endif

  Gamepad.reset();
  modeStr = "Joystick / Wheel";

  // Encoder pin setup
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  drv_proc();  // Init encoder values

  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(pinA), drv_proc, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB), drv_proc, CHANGE);

  // Button inputs
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

  // Determine initial mode based on held buttons
  if (!digitalRead(Button0)) {
    mouse_emu = !mouse_emu;
    gamepad_emu = !gamepad_emu;
    Mouse.begin();
    modeStr = "Mouse Mode";
  } 
  if (!digitalRead(Button1)) {
    paddle_emu = !paddle_emu;
    gamepad_emu = !gamepad_emu;
    modeStr = "Paddle Mode";
  }
  if (!digitalRead(Button2)) {
    mr_spinner_emu = !mr_spinner_emu;
    gamepad_emu = !gamepad_emu;
    gp_serial = "MiSTer-S1 Spinner";
    modeStr = "Mr. Spinner Mode";
  }
  if (!digitalRead(Button3)) {
    gamepad_emu = !gamepad_emu;
    gp_serial = "MiSTer-A1 Spinner";
  }

  // OLED initialization
  oled.begin();
  oled.setBusClock(100000);  // Set I2C to 100 kHz
  oled.clearBuffer();
  drawDisplay();
}
/////////////////////////////////////////////////////////////////////
// Main loop (runs continuously)
/////////////////////////////////////////////////////////////////////

void loop() {
  // Spinner value smoothing
  spinner_axis_value_temp = constrain(drvpos, -127, 127);

  // Display update
  drawDisplay();

  // Read analog paddle input
  int analogX = analogRead(A3);
  analogX = map(analogX, 0, 1023, 0, 255);

  // Deadzone adjustment
  if (abs(analogX - 127) <= deadzone) {
    analogX = 127;
  }

  // Handle paddle emulation mode
  if (paddle_emu) {
    analogX = analogRead(A3);
    analogX = map(analogX, 0, 1023, 0, 255);
    Gamepad.write(analogX, 127, 0, 0, 0, 0, 0);
    delay(5);
    return;
  }

  // Handle mouse emulation mode
  if (mouse_emu) {
    Mouse.move(m_drvpos, 0, 0);
    m_drvpos = 0;
    delay(5);
    return;
  }

  // Handle Mr. Spinner mode
  if (mr_spinner_emu) {
    spinner_axis_value = (int8_t)constrain(spinner_axis_value_temp, -127, 127);
    Gamepad.write(127, 127, spinner_axis_value, 0, 0, 0, 0);
    delay(5);
    return;
  }

  // Default Gamepad (Joystick/Wheel) mode
  spinner_axis_value = (int8_t)constrain(spinner_axis_value_temp, -127, 127);
  Gamepad.write(
    127,                           // X
    127,                           // Y
    0,                             // Z
    spinner_axis_value,           // Rx (spinner)
    0,                             // Ry
    0,                             // Rz
    0                              // Slider
  );

  // Read and report button states
  rep.btn = 0;
  rep.btn |= !digitalRead(Button0)    << 0;
  rep.btn |= !digitalRead(Button1)    << 1;
  rep.btn |= !digitalRead(Button2)    << 2;
  rep.btn |= !digitalRead(Button3)    << 3;
  rep.btn |= !digitalRead(ButtonL)    << 4;
  rep.btn |= !digitalRead(ButtonR)    << 5;
  rep.btn |= !digitalRead(ButtonStart)<< 6;
  rep.btn |= !digitalRead(ButtonSelect)<<7;

  rep.hat = HAT_CENTERED;
  if (!digitalRead(ButtonUp))    rep.hat = HAT_UP;
  if (!digitalRead(ButtonDown))  rep.hat = HAT_DOWN;
  if (!digitalRead(ButtonLeft))  rep.hat = HAT_LEFT;
  if (!digitalRead(ButtonRight)) rep.hat = HAT_RIGHT;

  Gamepad.write(&rep);
  delay(5);
}

/////////////////////////////////////////////////////////////////////
// Display output function
/////////////////////////////////////////////////////////////////////

void drawDisplay() {
  oled.clearBuffer();
  oled.setFont(u8g2_font_5x7_tf);
  oled.setCursor(0, 10);

  oled.print("Mode: ");
  oled.println(modeStr);

  oled.print("Sensitivity: ");
  if (mouse_emu) {
    oled.print(mouse_sensitivity);
  } else if (paddle_emu) {
    oled.print(paddle_sensitivity);
  } else {
    oled.print(spinner_sensitivity);
  }

  oled.setCursor(0, 30);
  oled.print("Raw Encoder: ");
  oled.println(r_drvpos);

  oled.setCursor(0, 40);
  oled.print("Smoothed: ");
  oled.println(spinner_axis_value);

  oled.setCursor(0, 50);
  oled.print("Analog X: ");
  oled.println(analogRead(A3));

  oled.sendBuffer();  // Display everything
}
