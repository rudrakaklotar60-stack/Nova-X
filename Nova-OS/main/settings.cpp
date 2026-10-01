#include "settings.h"
#include "drone.h"
#include "os.h"
#include "zones.h"
#include "storage.h"
#include "graphics.h"
#include "input.h"
#include "nrf.h"

// ═══════════════════════════════════════════
//  REFRESH RATE
// ═══════════════════════════════════════════
#define SETTINGS_REFRESH_MS  150

static uint32_t lastDrawMs = 0;

// ═══════════════════════════════════════════
//  HELPER — check if current screen is
//  inside the settings branch
// ═══════════════════════════════════════════
static bool onSettingsScreen()
{
  ScreenID s = osState.current;
  return (s == SCR_SETTINGS_MENU        ||
          s == SCR_DISP_SETTINGS        ||
          s == SCR_CTRL_MENU            ||
          s == SCR_CTRL_JOY_MODE        ||
          s == SCR_CTRL_DEADZONE        ||
          s == SCR_CTRL_THROTTLE_CURVE  ||
          s == SCR_CTRL_CALIBRATION     ||
          s == SCR_CTRL_CAL_DONE        ||
          s == SCR_RADIO_SETTINGS       ||
          s == SCR_SOUND_INFO           ||
          s == SCR_PID_MENU             ||
          s == SCR_PID_ROLL             ||
          s == SCR_PID_PITCH            ||
          s == SCR_PID_YAW              ||
          s == SCR_PID_THR              ||
          s == SCR_PID_LIVE);
}

// ═══════════════════════════════════════════
//  SHOW SAVED OVERLAY
// ═══════════════════════════════════════════
static void showSaved()
{
  osShowOverlay(SCR_OVL_SAVED, 110, 100);
  delay(1000);
  osHideOverlay();
}

// ════════════════════════════════════════════
//  DISPLAY SETTINGS
// ════════════════════════════════════════════
static void drawDisplaySettings()
{
  zonesSetValue("bright_bar", novaX.disp.brightness);
  zonesSetColor("bright_bar", CYAN);

  DrawZone* toDZ = zonesGetDraw("timeout_val");
  if(toDZ) {
    fillRect(toDZ->x, toDZ->y, toDZ->w, toDZ->h, BLACK);
    String txt = novaX.disp.timeoutSec == 0 ? "Never" : String(novaX.disp.timeoutSec) + "s";
    printTextF(toDZ->x, toDZ->y, txt, WHITE, FONT_SMALL);
  }
}

static void handleDisplayAction(uint8_t actionID)
{
  switch(actionID) {
    case 1: if(novaX.disp.brightness > 5) novaX.disp.brightness -= 5; break;
    case 2: if(novaX.disp.brightness < 100) novaX.disp.brightness += 5; break;
    case 3: if(novaX.disp.timeoutSec > 0) novaX.disp.timeoutSec -= 5; break;
    case 4: if(novaX.disp.timeoutSec < 120) novaX.disp.timeoutSec += 5; break;
  }
}

// ════════════════════════════════════════════
//  CONTROLS — JOYSTICK MODE (4-option grid, preview-only)
// ════════════════════════════════════════════
static uint8_t joyModePreview = 0;
static bool    joyModeInited  = false;

static const uint16_t modeBoxX[4] = {40,  170, 40,  170};
static const uint16_t modeBoxY[4] = {70,  70,  140, 140};
#define MODE_BOX_W 100
#define MODE_BOX_H 50

static void drawModeBox(uint8_t i, bool selected)
{
  uint16_t color = selected ? CYAN : BLACK;
  drawRect(modeBoxX[i], modeBoxY[i], MODE_BOX_W, MODE_BOX_H, color);
}

static void drawJoyMode()
{
  if(!joyModeInited) {
    joyModePreview = (uint8_t)novaX.joy.mode;
    joyModeInited  = true;
    for(uint8_t i = 0; i < 4; i++) drawModeBox(i, i == joyModePreview);
  }
}

static void handleJoyModeAction(uint8_t actionID)
{
  switch(actionID) {
    case 1: case 2: case 3: case 4: {
      uint8_t newSel = actionID - 1;
      if(newSel != joyModePreview) {
        drawModeBox(joyModePreview, false);
        joyModePreview = newSel;
        drawModeBox(joyModePreview, true);
      }
      break;
    }
  }
}

// ════════════════════════════════════════════
//  CONTROLS — DEADZONE
// ════════════════════════════════════════════
static void drawDeadzone()
{
  zonesSetValue("dz_bar", map(novaX.joy.deadzone, 0, 15, 0, 100));
  zonesSetColor("dz_bar", CYAN);

  DrawZone* valDZ = zonesGetDraw("dz_val");
  if(valDZ) {
    fillRect(valDZ->x, valDZ->y, valDZ->w, valDZ->h, BLACK);
    printTextF(valDZ->x, valDZ->y, String(novaX.joy.deadzone) + "%", WHITE, FONT_SMALL);
  }
}

static void handleDeadzoneAction(uint8_t actionID)
{
  switch(actionID) {
    case 1: if(novaX.joy.deadzone > 0) novaX.joy.deadzone--; break;
    case 2: if(novaX.joy.deadzone < 15) novaX.joy.deadzone++; break;
  }
}

// ════════════════════════════════════════════
//  CONTROLS — THROTTLE CURVE (3-option grid, preview-only)
// ════════════════════════════════════════════
static uint8_t curvePreview = 0;
static bool    curveInited  = false;

static const uint16_t curveBoxX[3] = {30, 130, 230};
static const uint16_t curveBoxY[3] = {100, 100, 100};
#define CURVE_BOX_W 80
#define CURVE_BOX_H 50

static void drawCurveBox(uint8_t i, bool selected)
{
  uint16_t color = selected ? CYAN : BLACK;
  drawRect(curveBoxX[i], curveBoxY[i], CURVE_BOX_W, CURVE_BOX_H, color);
}

static void drawThrottleCurve()
{
  if(!curveInited) {
    curvePreview = novaX.joy.throttleCurve;
    curveInited  = true;
    for(uint8_t i = 0; i < 3; i++) drawCurveBox(i, i == curvePreview);
  }
}

static void handleThrottleAction(uint8_t actionID)
{
  switch(actionID) {
    case 1: case 2: case 3: {
      uint8_t newSel = actionID - 1;
      if(newSel != curvePreview) {
        drawCurveBox(curvePreview, false);
        curvePreview = newSel;
        drawCurveBox(curvePreview, true);
      }
      break;
    }
  }
}

// ════════════════════════════════════════════
//  CONTROLS — JOYSTICK CALIBRATION
// ════════════════════════════════════════════
static bool calActive   = false;
static uint8_t calPhase = 0;

static void drawCalibration()
{
  DrawZone* j1DZ = zonesGetDraw("joy1_dot");
  DrawZone* j2DZ = zonesGetDraw("joy2_dot");

  if(j1DZ) {
    uint16_t dotX = map(novaX.remote.joy1X,
                        novaX.joy.joy1XMin, novaX.joy.joy1XMax,
                        j1DZ->x, j1DZ->x + j1DZ->w);
    uint16_t dotY = map(novaX.remote.joy1Y,
                        novaX.joy.joy1YMin, novaX.joy.joy1YMax,
                        j1DZ->y, j1DZ->y + j1DZ->h);

    // Erase full zone
    fillRect(j1DZ->x, j1DZ->y, j1DZ->w, j1DZ->h, BLACK);

    // Clamp crosshair arms to stay inside zone
    uint16_t lx0 = max((int)j1DZ->x,           (int)dotX - 5);
    uint16_t lx1 = min((int)j1DZ->x + j1DZ->w, (int)dotX + 5);
    uint16_t ly0 = max((int)j1DZ->y,           (int)dotY - 5);
    uint16_t ly1 = min((int)j1DZ->y + j1DZ->h, (int)dotY + 5);

    drawLine(lx0, dotY, lx1, dotY, CYAN);  // horizontal arm
    drawLine(dotX, ly0, dotX, ly1, CYAN);  // vertical arm
  }

  if(j2DZ) {
    uint16_t dotX = map(novaX.remote.joy2X,
                        novaX.joy.joy2XMin, novaX.joy.joy2XMax,
                        j2DZ->x, j2DZ->x + j2DZ->w);
    uint16_t dotY = map(novaX.remote.joy2Y,
                        novaX.joy.joy2YMin, novaX.joy.joy2YMax,
                        j2DZ->y, j2DZ->y + j2DZ->h);

    fillRect(j2DZ->x, j2DZ->y, j2DZ->w, j2DZ->h, BLACK);

    uint16_t lx0 = max((int)j2DZ->x,           (int)dotX - 5);
    uint16_t lx1 = min((int)j2DZ->x + j2DZ->w, (int)dotX + 5);
    uint16_t ly0 = max((int)j2DZ->y,           (int)dotY - 5);
    uint16_t ly1 = min((int)j2DZ->y + j2DZ->h, (int)dotY + 5);

    drawLine(lx0, dotY, lx1, dotY, CYAN);
    drawLine(dotX, ly0, dotX, ly1, CYAN);
  }

  DrawZone* stDZ = zonesGetDraw("cal_status");
  if(stDZ) {
    fillRect(stDZ->x, stDZ->y, stDZ->w, stDZ->h, BLACK);
    String msg = calPhase == 0 ? "Press Start" :
                 calPhase == 1 ? "Move to extremes, then press" :
                                 "Center sticks, then press";
    printTextF(stDZ->x, stDZ->y, msg, YELLOW, FONT_SMALL);
  }
}

static void handleCalibrationAction(uint8_t actionID)
{
  if(actionID != 1) return;

  switch(calPhase)
  {
    case 0:
      calPhase = 1;
      novaX.joy.joy1XMin = novaX.joy.joy1XCenter;
      novaX.joy.joy1XMax = novaX.joy.joy1XCenter;
      novaX.joy.joy1YMin = novaX.joy.joy1YCenter;
      novaX.joy.joy1YMax = novaX.joy.joy1YCenter;
      novaX.joy.joy2XMin = novaX.joy.joy2XCenter;
      novaX.joy.joy2XMax = novaX.joy.joy2XCenter;
      novaX.joy.joy2YMin = novaX.joy.joy2YCenter;
      novaX.joy.joy2YMax = novaX.joy.joy2YCenter;
      break;

    case 1:
      calPhase = 2;
      break;

    case 2:
      novaX.joy.joy1XCenter = novaX.remote.joy1X;
      novaX.joy.joy1YCenter = novaX.remote.joy1Y;
      novaX.joy.joy2XCenter = novaX.remote.joy2X;
      novaX.joy.joy2YCenter = novaX.remote.joy2Y;
      calPhase = 0;
      saveJoystick();
      osGoto(SCR_CTRL_CAL_DONE);
      break;
  }
}

static void updateCalMinMax()
{
  if(calPhase != 1) return;

  uint16_t x1 = novaX.remote.joy1X;
  uint16_t y1 = novaX.remote.joy1Y;
  uint16_t x2 = novaX.remote.joy2X;
  uint16_t y2 = novaX.remote.joy2Y;

  if(x1 < novaX.joy.joy1XMin) novaX.joy.joy1XMin = x1;
  if(x1 > novaX.joy.joy1XMax) novaX.joy.joy1XMax = x1;
  if(y1 < novaX.joy.joy1YMin) novaX.joy.joy1YMin = y1;
  if(y1 > novaX.joy.joy1YMax) novaX.joy.joy1YMax = y1;
  if(x2 < novaX.joy.joy2XMin) novaX.joy.joy2XMin = x2;
  if(x2 > novaX.joy.joy2XMax) novaX.joy.joy2XMax = x2;
  if(y2 < novaX.joy.joy2YMin) novaX.joy.joy2YMin = y2;
  if(y2 > novaX.joy.joy2YMax) novaX.joy.joy2YMax = y2;
}

// ════════════════════════════════════════════
//  RADIO SETTINGS
// ════════════════════════════════════════════
static void drawRadioSettings()
{
  DrawZone* chDZ  = zonesGetDraw("radio_ch");
  DrawZone* pwDZ  = zonesGetDraw("radio_pw");

  const char* powers[] = {"MIN","LOW","HIGH","MAX"};

  if(chDZ) {
    fillRect(chDZ->x, chDZ->y, chDZ->w, chDZ->h, BLACK);
    printTextF(chDZ->x, chDZ->y, String(novaX.radio.channel), WHITE, FONT_SMALL);
  }
  if(pwDZ) {
    fillRect(pwDZ->x, pwDZ->y, pwDZ->w, pwDZ->h, BLACK);
    printTextF(pwDZ->x, pwDZ->y, powers[novaX.radio.txPower], WHITE, FONT_SMALL);
  }
}

static void handleRadioAction(uint8_t actionID)
{
  switch(actionID) {
    case 1: if(novaX.radio.channel > 0) novaX.radio.channel--; break;
    case 2: if(novaX.radio.channel < 125) novaX.radio.channel++; break;
    case 3: novaX.radio.txPower = (novaX.radio.txPower + 1) % 4; break;
  }
}

// ════════════════════════════════════════════
//  PID TUNING — generic for roll/pitch/yaw
// ════════════════════════════════════════════
static PIDValues* getActivePID()
{
  switch(osState.current) {
    case SCR_PID_ROLL:  return &novaX.pid.roll;
    case SCR_PID_PITCH: return &novaX.pid.pitch;
    case SCR_PID_YAW:   return &novaX.pid.yaw;
    default:            return nullptr;
  }
}

static uint8_t pidSelected = 0;

static void drawPIDScreen()
{
  PIDValues* pid = getActivePID();

  if(osState.current == SCR_PID_THR) {
    DrawZone* minDZ = zonesGetDraw("thr_min_bar");
    DrawZone* maxDZ = zonesGetDraw("thr_max_bar");
    DrawZone* minVDZ= zonesGetDraw("thr_min_val");
    DrawZone* maxVDZ= zonesGetDraw("thr_max_val");

    if(minDZ) {
      zonesSetValue("thr_min_bar", map(novaX.pid.thrMin, 0, 1000, 0, 100));
      zonesSetColor("thr_min_bar", YELLOW);
    }
    if(maxDZ) {
      zonesSetValue("thr_max_bar", map(novaX.pid.thrMax, 0, 1000, 0, 100));
      zonesSetColor("thr_max_bar", YELLOW);
    }
    if(minVDZ) {
      fillRect(minVDZ->x, minVDZ->y, minVDZ->w, minVDZ->h, BLACK);
      printTextF(minVDZ->x, minVDZ->y, String((int)novaX.pid.thrMin), WHITE, FONT_SMALL);
    }
    if(maxVDZ) {
      fillRect(maxVDZ->x, maxVDZ->y, maxVDZ->w, maxVDZ->h, BLACK);
      printTextF(maxVDZ->x, maxVDZ->y, String((int)novaX.pid.thrMax), WHITE, FONT_SMALL);
    }
    return;
  }

  if(!pid) return;

  zonesSetValue("p_bar", map(pid->p * 100, 0, 500, 0, 100));
  zonesSetColor("p_bar", YELLOW);

  zonesSetValue("i_bar", map(pid->i * 1000, 0, 200, 0, 100));
  zonesSetColor("i_bar", RED);

  zonesSetValue("d_bar", map(pid->d * 100, 0, 100, 0, 100));
  zonesSetColor("d_bar", CYAN);

  DrawZone* pVDZ = zonesGetDraw("p_val");
  DrawZone* iVDZ = zonesGetDraw("i_val");
  DrawZone* dVDZ = zonesGetDraw("d_val");

  if(pVDZ) {
    fillRect(pVDZ->x, pVDZ->y, pVDZ->w, pVDZ->h, BLACK);
    printTextF(pVDZ->x, pVDZ->y, String(pid->p, 2), YELLOW, FONT_SMALL);
  }
  if(iVDZ) {
    fillRect(iVDZ->x, iVDZ->y, iVDZ->w, iVDZ->h, BLACK);
    printTextF(iVDZ->x, iVDZ->y, String(pid->i, 3), RED, FONT_SMALL);
  }
  if(dVDZ) {
    fillRect(dVDZ->x, dVDZ->y, dVDZ->w, dVDZ->h, BLACK);
    printTextF(dVDZ->x, dVDZ->y, String(pid->d, 2), CYAN, FONT_SMALL);
  }

  DrawZone* selDZ = zonesGetDraw("pid_sel_label");
  if(selDZ) {
    fillRect(selDZ->x, selDZ->y, selDZ->w, selDZ->h, BLACK);
    const char* names[] = {"P", "I", "D"};
    printTextF(selDZ->x, selDZ->y, names[pidSelected], WHITE, FONT_LARGE);
  }
}

static void handlePIDAction(uint8_t actionID)
{
  PIDValues* pid = getActivePID();

  if(osState.current == SCR_PID_THR) {
    switch(actionID) {
      case 1: if(novaX.pid.thrMin > 0) novaX.pid.thrMin -= 10; break;
      case 2: if(novaX.pid.thrMin < novaX.pid.thrMax - 10) novaX.pid.thrMin += 10; break;
      case 3: if(novaX.pid.thrMax > novaX.pid.thrMin + 10) novaX.pid.thrMax -= 10; break;
      case 4: if(novaX.pid.thrMax < 1000) novaX.pid.thrMax += 10; break;
    }
    return;
  }

  if(!pid) return;

  switch(actionID) {
    case 1: pidSelected = 0; break;
    case 2: pidSelected = 1; break;
    case 3: pidSelected = 2; break;

    case 4:
      if(pidSelected == 0 && pid->p > 0.01f)       pid->p -= 0.01f;
      else if(pidSelected == 1 && pid->i > 0.001f) pid->i -= 0.001f;
      else if(pidSelected == 2 && pid->d > 0.01f)  pid->d -= 0.01f;
      break;

    case 5:
      if(pidSelected == 0)      pid->p += 0.01f;
      else if(pidSelected == 1) pid->i += 0.001f;
      else if(pidSelected == 2) pid->d += 0.01f;
      break;
  }
}

// ════════════════════════════════════════════
//  DISPATCH
// ════════════════════════════════════════════
static void drawCurrentSettings()
{
  switch(osState.current) {
    case SCR_DISP_SETTINGS:          drawDisplaySettings(); break;
    case SCR_CTRL_JOY_MODE:          drawJoyMode(); break;
    case SCR_CTRL_DEADZONE:          drawDeadzone(); break;
    case SCR_CTRL_THROTTLE_CURVE:    drawThrottleCurve(); break;
    case SCR_CTRL_CALIBRATION:       updateCalMinMax(); drawCalibration(); break;
    case SCR_RADIO_SETTINGS:         drawRadioSettings(); break;
    case SCR_PID_ROLL:
    case SCR_PID_PITCH:
    case SCR_PID_YAW:
    case SCR_PID_THR:                drawPIDScreen(); break;
    default: break;
  }
}

static void handleAction(uint8_t actionID)
{
  if(actionID == 0) return;

  switch(osState.current) {
    case SCR_DISP_SETTINGS:          handleDisplayAction(actionID); break;
    case SCR_CTRL_JOY_MODE:          handleJoyModeAction(actionID); break;
    case SCR_CTRL_DEADZONE:          handleDeadzoneAction(actionID); break;
    case SCR_CTRL_THROTTLE_CURVE:    handleThrottleAction(actionID); break;
    case SCR_CTRL_CALIBRATION:       handleCalibrationAction(actionID); break;
    case SCR_RADIO_SETTINGS:         handleRadioAction(actionID); break;
    case SCR_PID_ROLL:
    case SCR_PID_PITCH:
    case SCR_PID_YAW:
    case SCR_PID_THR:                handlePIDAction(actionID); break;
    default: break;
  }
}

// ════════════════════════════════════════════
//  SAVE CURRENT SCREEN
// ════════════════════════════════════════════
void settingsSaveCurrent()
{
  switch(osState.current) {
    case SCR_DISP_SETTINGS:
      saveDisplay();
      showSaved();
      break;

    case SCR_CTRL_JOY_MODE:
      novaX.joy.mode = (JoyMode)joyModePreview;
      saveJoystick();
      showSaved();
      break;

    case SCR_CTRL_DEADZONE:
      saveJoystick();
      showSaved();
      break;

    case SCR_CTRL_THROTTLE_CURVE:
      novaX.joy.throttleCurve = curvePreview;
      saveJoystick();
      showSaved();
      break;

    case SCR_RADIO_SETTINGS:
      saveRadio();
      nrfInit();
      showSaved();
      break;

    case SCR_PID_ROLL:
    case SCR_PID_PITCH:
    case SCR_PID_YAW:
    case SCR_PID_THR:
      savePID();
      showSaved();
      break;

    default:
      break;
  }
}

// ════════════════════════════════════════════
//  SETTINGS INIT
// ════════════════════════════════════════════
void settingsInit()
{
  calPhase      = 0;
  calActive     = false;
  pidSelected   = 0;
  joyModeInited = false;
  curveInited   = false;
  lastDrawMs    = 0;
}

// ════════════════════════════════════════════
//  SETTINGS LOOP
// ════════════════════════════════════════════
void settingsLoop()
{
  if(!onSettingsScreen()) return;

  uint32_t now = millis();
  if(now - lastDrawMs < SETTINGS_REFRESH_MS) return;
  lastDrawMs = now;

  zonesRenderDraws();
  drawCurrentSettings();
}

// ════════════════════════════════════════════
//  ACTION HANDLER
// ════════════════════════════════════════════
void settingsHandleAction(uint8_t actionID)
{
  handleAction(actionID);
}