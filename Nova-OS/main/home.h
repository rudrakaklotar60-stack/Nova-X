#pragma once
#include "os.h"
#include "input.h"
#include "graphics.h"
#include "drone.h"
#include "zones.h"

// ═══════════════════════════════════════════
//  HOME SCREEN MENU ITEMS
// ═══════════════════════════════════════════
enum HomeItem {
  HOME_FLY = 0,
  HOME_LOGS,
  HOME_SETTINGS,
  HOME_ABOUT,
  HOME_COUNT
};

static const uint8_t homeItemRow[HOME_COUNT] = {0, 0, 1, 1};
static const uint8_t homeItemCol[HOME_COUNT] = {0, 1, 0, 1};

static const char* homeItemNames[HOME_COUNT] = {
  "Fly", "Logs", "Settings", "About"
};

static const ScreenID homeItemTarget[HOME_COUNT] = {
  SCR_DISCONNECTED,
  SCR_LOGS_EMPTY,
  SCR_SETTINGS_MENU,
  SCR_ABOUT
};

// ═══════════════════════════════════════════
//  HIGHLIGHT BOX POSITIONS
//  Adjust to match your home.raw layout
//{FLY, LOGS, SETTINGS, ABOUT} ═══════════════════════════════════════════
static const uint16_t homeHighlightX[HOME_COUNT] = { 8,  85,  8,  85};
static const uint16_t homeHighlightY[HOME_COUNT] = {85,  85, 151, 151};
static const uint16_t homeHighlightW[HOME_COUNT] = {70,  70,  80,  80};
static const uint16_t homeHighlightH[HOME_COUNT] = {70,  70,  80,  80};

// Position of "Selected: ___" text (right panel)
#define HOME_SEL_TEXT_X  213
#define HOME_SEL_TEXT_Y  113

// ═══════════════════════════════════════════
//  STATE — defined once in os.cpp
// ═══════════════════════════════════════════
extern HomeItem selectedItem;
extern HomeItem lastDrawnItem;

// ═══════════════════════════════════════════
//  DRAW HIGHLIGHT
// ═══════════════════════════════════════════
static void drawHomeHighlight(HomeItem item, bool selected)
{
  uint16_t color = selected ? CYAN : BLACK;
  uint16_t x = homeHighlightX[item];
  uint16_t y = homeHighlightY[item];
  uint16_t w = homeHighlightW[item];
  uint16_t h = homeHighlightH[item];

  if(w == 0 || h == 0) return;
  drawRect(x, y, w, h, color);
}

// ═══════════════════════════════════════════
//  HOME INIT
// ═══════════════════════════════════════════
inline void homeInit()
{
  selectedItem  = HOME_FLY;
  lastDrawnItem = (HomeItem)255;
}

// ═══════════════════════════════════════════
//  HOME DRAW
// ═══════════════════════════════════════════
inline void homeDraw()
{
  if(lastDrawnItem != (HomeItem)255)
    drawHomeHighlight(lastDrawnItem, false);

  drawHomeHighlight(selectedItem, true);

  fillRect(HOME_SEL_TEXT_X - 30, HOME_SEL_TEXT_Y, 140, 35, BLACK);

  // Adjust X per item to center text visually
  int16_t textX = HOME_SEL_TEXT_X;
  if(selectedItem == HOME_SETTINGS) textX = HOME_SEL_TEXT_X - 23;
  else if(selectedItem == HOME_FLY) textX = HOME_SEL_TEXT_X + 10;

  printTextF(textX, HOME_SEL_TEXT_Y,
             homeItemNames[selectedItem], GREEN, FONT_LARGE);

  lastDrawnItem = selectedItem;
  zonesRenderDraws();
}

// ═══════════════════════════════════════════
//  BATTERY UPDATE
// ═══════════════════════════════════════════
static uint8_t smoothedBatPct = 255;

static void homeUpdateBattery()
{
  uint16_t mv = readBatteryVoltageMv();
  novaX.remote.remoteBatV = mv / 1000.0f;

  uint8_t rawPct = batteryPercent(mv);

  if(smoothedBatPct == 255) {
    smoothedBatPct = rawPct;
  } else {
    if(rawPct > smoothedBatPct) smoothedBatPct++;
    else if(rawPct < smoothedBatPct) smoothedBatPct--;
  }

  novaX.remote.remoteBatPct = smoothedBatPct;

  zonesSetValue("home_battery", smoothedBatPct);

  uint16_t col = smoothedBatPct > 50 ? GREEN :
                 smoothedBatPct > 20 ? YELLOW : RED;
  zonesSetColor("home_battery", col);
  zonesSetColor("home_battery_text", col);

  char buf[8];
  snprintf(buf, sizeof(buf), "%d%%", smoothedBatPct);
  zonesSetText("home_battery_text", buf);
}

// ═══════════════════════════════════════════
//  HOME LOOP — call every frame from main loop
// ═══════════════════════════════════════════
static uint32_t lastHomeBattMs = 0;

inline void homeLoop()
{
  if(osState.current != SCR_HOME) return;

  uint32_t now = millis();
  if(now - lastHomeBattMs < 500) return;
  lastHomeBattMs = now;

  homeUpdateBattery();
  zonesRenderDraws();
}

// ═══════════════════════════════════════════
//  JOYSTICK HANDLER
// ═══════════════════════════════════════════
inline void onJoystick(JoyDir dir)
{
  if(osState.current != SCR_HOME) return;

  uint8_t row = homeItemRow[selectedItem];
  uint8_t col = homeItemCol[selectedItem];

  switch(dir)
  {
    case JOY_UP:    if(row > 0) row--; break;
    case JOY_DOWN:  if(row < 1) row++; break;
    case JOY_LEFT:  if(col > 0) col--; break;
    case JOY_RIGHT: if(col < 1) col++; break;
    case JOY_CLICK:
      osGoto(homeItemTarget[selectedItem]);
      return;
    default:
      return;
  }

  for(uint8_t i = 0; i < HOME_COUNT; i++) {
    if(homeItemRow[i] == row && homeItemCol[i] == col) {
      selectedItem = (HomeItem)i;
      break;
    }
  }

  homeDraw();
}

// ═══════════════════════════════════════════
//  TOUCH HANDLER
// ═══════════════════════════════════════════
inline void onTouch(uint16_t x, uint16_t y)
{
  if(osState.current != SCR_HOME) return;

  for(uint8_t i = 0; i < HOME_COUNT; i++) {
    uint16_t hx = homeHighlightX[i];
    uint16_t hy = homeHighlightY[i];
    uint16_t hw = homeHighlightW[i];
    uint16_t hh = homeHighlightH[i];

    if(hw == 0 || hh == 0) continue;

    if(x >= hx && x <= hx+hw && y >= hy && y <= hy+hh) {
      selectedItem = (HomeItem)i;
      homeDraw();
      delay(100);
      osGoto(homeItemTarget[i]);
      return;
    }
  }
}