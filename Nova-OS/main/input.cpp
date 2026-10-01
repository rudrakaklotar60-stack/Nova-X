#include "input.h"
#include "os.h"
#include "zones.h"
#include "drone.h"
#include "home.h"
#include "settings.h"
#include "logs.h"
#include "nrf.h"
#include <Wire.h>

InputState inputState;

void inputInit()
{
  pinMode(JOY_BTN, INPUT_PULLUP);
  pinMode(TOUCH_STATE, INPUT);
  pinMode(PCF_INT, INPUT_PULLUP);

  Wire.begin(21, 22);
  Wire.beginTransmission(PCF_ADDR);
  Wire.write(0xFF);
  Wire.endTransmission();

  inputState.joy             = JOY_NONE;
  inputState.lastJoy         = JOY_NONE;
  inputState.btnPressed      = false;
  inputState.btnLast         = false;
  inputState.touched         = false;
  inputState.lastJoyTime     = 0;
  inputState.pcfButtons      = 0x00;
  inputState.pcfButtonsLast  = 0x00;

  Serial.println("Input OK");
}

JoyDir readJoystick()
{
  int x1 = analogRead(JOY1_X);
  int y1 = analogRead(JOY1_Y);
  int x2 = analogRead(JOY2_X);
  int y2 = analogRead(JOY2_Y);

  y1 = 4095 - y1;
  y2 = 4095 - y2;

  int dx1 = abs(x1 - JOY1_X_CENTER);
  int dy1 = abs(y1 -  JOY1_Y_CENTER);
  int dx2 = abs(x2 - JOY2_X_CENTER);
  int dy2 = abs(y2 - JOY2_Y_CENTER);

  int x, y, xCenter, yCenter;
  if(dx1 > dx2) { x = x1; xCenter = JOY1_X_CENTER; }
  else          { x = x2; xCenter = JOY2_X_CENTER; }

  if(dy1 > dy2) { y = y1; yCenter = 4095 - JOY1_Y_CENTER; }
  else          { y = y2; yCenter = 4095 - JOY2_Y_CENTER; }

  bool xMoved = abs(x - xCenter) > JOY_DEADZONE;
  bool yMoved = abs(y - yCenter) > JOY_DEADZONE;

  if(!xMoved && !yMoved) return JOY_NONE;

  if(abs(x - xCenter) > abs(y - yCenter))
    return (x < xCenter) ? JOY_LEFT : JOY_RIGHT;
  else
    return (y < yCenter) ? JOY_UP : JOY_DOWN;
}

bool readButton()
{
  return (digitalRead(JOY_BTN) == LOW);
}

bool readTouch(uint16_t& x, uint16_t& y)
{
  if(digitalRead(TOUCH_STATE) == LOW) return false;
  x = SCREEN_W / 2;
  y = SCREEN_H / 2;
  return true;
}

uint8_t readPCF()
{
  Wire.requestFrom(PCF_ADDR, 1);
  if(Wire.available())
    return Wire.read();
  return 0xFF;
}

// ═══════════════════════════════════════════
//  HANDLE PCF8574 BUTTONS
//  — debounced, and ARM/DISARM now sets local
//    INTENT only (commandedFlightMode), never
//    the confirmed novaX.drone.* state directly
// ═══════════════════════════════════════════
static void handlePCFButtons()
{
  static uint32_t lastPcfMs = 0;
  if(digitalRead(PCF_INT) != LOW) return;
  if(millis() - lastPcfMs < 50) return;   // debounce
  lastPcfMs = millis();

  uint8_t raw  = readPCF();
  uint8_t btns = ~raw;

  uint8_t changed = btns ^ inputState.pcfButtonsLast;
  uint8_t pressed = changed & btns;

  inputState.pcfButtonsLast = btns;
  inputState.pcfButtons     = btns;

  if(pressed)
  {
    osState.lastInputTime = millis();
    novaX.remote.buttons  = pressed;

    // ARM/DISARM — single button pure toggle, intent only
    if(pressed & PCF_BTN_ARM) {
      if(novaX.remote.commandedFlightMode == MODE_ARMED) {
        novaX.remote.commandedFlightMode = MODE_DISARMED;
        novaX.remote.throttle = 0;
        Serial.println("BTN: DISARM commanded");
      } else {
        novaX.remote.commandedFlightMode = MODE_ARMED;
        nrfSetPanic(false);
        Serial.println("BTN: ARM commanded");
      }
    }

    // PANIC — zero throttle instantly
    if(pressed & PCF_BTN_DISARM) {
      nrfSetPanic(true);
      novaX.remote.commandedFlightMode = MODE_DISARMED;
      Serial.println("BTN: PANIC");
    }

    if(pressed & PCF_BTN_MODE)   Serial.println("BTN: MODE");
    if(pressed & PCF_BTN_4) {
      Serial.println("BTN: BACK");
      osBackStack();
    }
    if(pressed & PCF_BTN_5) {
      Serial.println("BTN: SAVE");
      settingsSaveCurrent();
    }
  }
}

static void dispatchAction(uint8_t actionID)
{
  if(actionID == 0) return;

  ScreenID s = osState.current;

  if(s == SCR_SETTINGS_MENU    ||
     s == SCR_DISP_SETTINGS    ||
     s == SCR_CTRL_MENU        ||
     s == SCR_CTRL_JOY_MODE    ||
     s == SCR_CTRL_DEADZONE    ||
     s == SCR_CTRL_THROTTLE_CURVE ||
     s == SCR_CTRL_CALIBRATION ||
     s == SCR_CTRL_CAL_DONE    ||
     s == SCR_RADIO_SETTINGS   ||
     s == SCR_SOUND_INFO       ||
     s == SCR_PID_MENU         ||
     s == SCR_PID_ROLL         ||
     s == SCR_PID_PITCH        ||
     s == SCR_PID_YAW          ||
     s == SCR_PID_THR          ||
     s == SCR_PID_LIVE)
  {
    settingsHandleAction(actionID);
    return;
  }

  if(s == SCR_LOGS_EMPTY         ||
     s == SCR_LOGS_LIST          ||
     s == SCR_LOG_OVERVIEW       ||
     s == SCR_LOG_ALT_GRAPH      ||
     s == SCR_LOG_BAT_GRAPH      ||
     s == SCR_LOG_ATT_GRAPH      ||
     s == SCR_LOG_EVENTS         ||
     s == SCR_LOG_DELETE_CONFIRM ||
     s == SCR_LOG_EXPORT)
  {
    logsHandleAction(actionID);
    return;
  }
}

static void handleJoy(JoyDir dir)
{
  osState.lastInputTime = millis();

  if(osState.current == SCR_HOME) {
    onJoystick(dir);
    return;
  }

  switch(dir) {
    case JOY_UP:
    case JOY_LEFT:
      zonesJoyUp();
      break;

    case JOY_DOWN:
    case JOY_RIGHT:
      zonesJoyDown();
      break;

    case JOY_CLICK: {
      if(zoneTable.selected >= 0 &&
         zoneTable.selected < zoneTable.count)
      {
        uint8_t actionID = zoneTable.zones[zoneTable.selected].actionID;

        if(actionID > 0)
          dispatchAction(actionID);
        else {
          ScreenID next = zonesJoyClick();
          if(next != osState.current)
            osGoto(next);
        }
      }
      break;
    }

    default: break;
  }
}

static void handleTouch(uint16_t x, uint16_t y)
{
  osState.lastInputTime = millis();

  if(osState.current == SCR_HOME) {
    onTouch(x, y);
    return;
  }

  for(uint8_t i = 0; i < zoneTable.count; i++) {
    Zone& z = zoneTable.zones[i];
    if(x >= z.x && x <= z.x + z.w &&
       y >= z.y && y <= z.y + z.h)
    {
      zoneTable.selected = i;
      zonesDrawHighlight();

      if(z.actionID > 0)
        dispatchAction(z.actionID);
      else if(z.target != osState.current)
        osGoto(z.target);
      return;
    }
  }
}

void inputLoop()
{
  uint32_t now = millis();

  if(now - inputState.lastJoyTime > 200)
  {
    JoyDir dir = readJoystick();

    if(dir != JOY_NONE && dir != inputState.lastJoy)
    {
      inputState.joy         = dir;
      inputState.lastJoy     = dir;
      inputState.lastJoyTime = now;
      handleJoy(dir);
    }
    else if(dir == JOY_NONE)
    {
      inputState.lastJoy = JOY_NONE;
    }
  }

  bool btn = readButton();
  if(btn && !inputState.btnLast)
    handleJoy(JOY_CLICK);
  inputState.btnLast = btn;

  handlePCFButtons();

  uint16_t tx, ty;
  if(readTouch(tx, ty)) {
    if(!inputState.touched) {
      inputState.touched = true;
      inputState.touchX  = tx;
      inputState.touchY  = ty;
      handleTouch(tx, ty);
    }
  } else {
    inputState.touched = false;
  }
}