#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "os.h"

// ═══════════════════════════════════════════
//  JOYSTICK PINS
// ═══════════════════════════════════════════
#define JOY1_X  32  // swapped
#define JOY1_Y  35   // swapped
#define JOY2_X  34   // swapped
#define JOY2_Y  39   // swapped

// ═══════════════════════════════════════════
//  CONFIRM BUTTON
// ═══════════════════════════════════════════
#define JOY_BTN  4    // GPIO4 — joystick click

// ═══════════════════════════════════════════
//  TOUCH PINS
// ═══════════════════════════════════════════
#define TOUCH_STATE  27
#define TOUCH_ADC    33

// ═══════════════════════════════════════════
//  PCF8574
// ═══════════════════════════════════════════
#define PCF_ADDR  0x20   // A0+A1+A2 = GND
#define PCF_INT   13     // INT pin → GPIO12

// PCF button bit positions (active LOW → inverted)
#define PCF_BTN_ARM    0b00000001  // P0
#define PCF_BTN_DISARM 0b00000010  // P1
#define PCF_BTN_MODE   0b00000100  // P2
#define PCF_BTN_3      0b00001000  // P3
#define PCF_BTN_4      0b00010000  // P4
#define PCF_BTN_5      0b00100000  // P5
#define PCF_BTN_6      0b01000000  // P6
#define PCF_BTN_7      0b10000000  // P7

// ═══════════════════════════════════════════
//  JOYSTICK THRESHOLDS
// ═══════════════════════════════════════════
#define JOY1_X_CENTER  1900
#define JOY1_Y_CENTER  1870
#define JOY2_X_CENTER  2000
#define JOY2_Y_CENTER  1940
#define JOY_DEADZONE   200

// ═══════════════════════════════════════════
//  JOYSTICK DIRECTIONS
// ═══════════════════════════════════════════
enum JoyDir {
  JOY_NONE = 0,
  JOY_UP,
  JOY_DOWN,
  JOY_LEFT,
  JOY_RIGHT,
  JOY_CLICK
};

// ═══════════════════════════════════════════
//  INPUT STATE
// ═══════════════════════════════════════════
struct InputState {
  JoyDir    joy;
  JoyDir    lastJoy;
  bool      btnPressed;
  bool      btnLast;
  uint16_t  touchX;
  uint16_t  touchY;
  bool      touched;
  uint32_t  lastJoyTime;
  uint8_t   pcfButtons;      // current button state
  uint8_t   pcfButtonsLast;  // previous button state
};

// ═══════════════════════════════════════════
//  FUNCTIONS
// ═══════════════════════════════════════════
void     inputInit();
void     inputLoop();
JoyDir   readJoystick();
bool     readButton();
bool     readTouch(uint16_t& x, uint16_t& y);
uint8_t  readPCF();

extern InputState inputState;
