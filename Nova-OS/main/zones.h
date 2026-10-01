#pragma once
#include <Arduino.h>
#include "os.h"
#include "graphics.h"

#define MAX_ZONES   12
#define MAX_DRAWS   12

#define DRAW_HBAR    0
#define DRAW_VBAR    1
#define DRAW_CIRCLE  2
#define DRAW_TEXT    3
#define DRAW_RECT    4
#define DRAW_HORIZON 5   
#define DRAW_SIGNAL_BARS 6

#define DCOL_GREEN    0
#define DCOL_RED      1
#define DCOL_YELLOW   2
#define DCOL_CYAN     3
#define DCOL_WHITE    4
#define DCOL_ORANGE   5
#define DCOL_DYNAMIC  6

struct Zone {
  uint16_t  x, y, w, h;
  ScreenID  target;
  uint8_t   actionID;
};

struct DrawZone {
  char     id[20];
  uint8_t  type;
  uint16_t x, y;
  uint16_t w, h;
  uint8_t  colorID;
  uint16_t dynColor;
  uint8_t  value;
  uint8_t  lastValue;
  char     text[8];
  char     lastText[8];
  uint8_t  textSize;
  int16_t  rollTenths;        // ← NEW
  int16_t  pitchTenths;       // ← NEW
  int16_t  lastRollTenths;    // ← NEW
  int16_t  lastPitchTenths;   // ← NEW
};

struct ZoneTable {
  Zone     zones[MAX_ZONES];
  uint8_t  count;
  int8_t   selected;
  DrawZone draws[MAX_DRAWS];
  uint8_t  drawCount;
};

void     zonesInit();
bool     zonesLoad(ScreenID id);
void     zonesClear();

ScreenID zonesCheckTouch(uint16_t x, uint16_t y);

void     zonesJoyUp();
void     zonesJoyDown();
ScreenID zonesJoyClick();

void     zonesDrawHighlight();

void      zonesRenderDraws();
void      zonesSetValue(const char* id, uint8_t value);
void      zonesSetColor(const char* id, uint16_t color);
void      zonesSetText(const char* id, const char* text);
void      zonesSetHorizon(const char* id, int16_t rollTenths, int16_t pitchTenths);  // ← NEW
DrawZone* zonesGetDraw(const char* id);
void zonesForceRedrawAll();

extern ZoneTable zoneTable;