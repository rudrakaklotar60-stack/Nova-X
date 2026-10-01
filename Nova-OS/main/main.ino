#include <SPI.h>
#include <SD_MMC.h>
#include <RF24.h>
#include <Wire.h>
#include <Preferences.h>

#include "graphics.h"
#include "os.h"
#include "input.h"
#include "zones.h"
#include "drone.h"
#include "nrf.h"
#include "storage.h"
#include "home.h"
#include "hud.h"
#include "settings.h"
#include "logs.h"
#include "about.h"

// ═══════════════════════════════════════════
//  ANIMATION SKIP PIN
//  Slide switch: OPEN=play anim, CLOSED=skip
// ═══════════════════════════════════════════
#define ANIM_SKIP_PIN  26

// ═══════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════
void setup()
{
  btStop();
  Serial.begin(115200);
  delay(1000);
  Serial.println("Nova-X OS 2.0 Starting...");

  // ── Pin setup ────────────────────────────
  pinMode(ANIM_SKIP_PIN, INPUT_PULLUP);
  pinMode(27, OUTPUT); digitalWrite(27, LOW);  // touch CS
  pinMode(16, OUTPUT); digitalWrite(16, LOW);  // NRF CE
  pinMode(17, OUTPUT); digitalWrite(17, HIGH); // NRF CSN
  Serial.println("Pins OK");

  // ── Display bus ──────────────────────────
  initBus();
  Serial.println("Bus OK");

  // ── SD card ──────────────────────────────
  initSD();
Serial.println("SD OK");

  // ── Display ──────────────────────────────
  initDisplay();
  fillScreen(BLACK);
  Serial.println("Display OK");

  // ── Boot animation ───────────────────────
  if(digitalRead(ANIM_SKIP_PIN) == HIGH) {
    Serial.println("Playing boot animation...");
    playAnimation("/anim.dlt");
  } else {
    Serial.println("Animation skipped");
  }

  // ── Load defaults + settings from SD ─────
  novaXDefaults();
  storageInit();
  Serial.println("Settings OK");

  // ── NRF ──────────────────────────────────
  nrfInit();
  Serial.println("NRF OK");

  // ── Input ────────────────────────────────
  inputInit();
  Serial.println("Input OK");

  // ── OS ───────────────────────────────────
  osInit();
  settingsInit();
  logsInit();   // loads log list, redirects if empty
  aboutInit();
  Serial.println("OS OK");

  // ── Home screen ──────────────────────────
  osGoto(SCR_HOME);
  homeInit();
  osRedraw();
  homeDraw();

  Serial.println("Nova-X Ready! ");
}

// ═══════════════════════════════════════════
//  WHICH BRANCH IS ACTIVE?
// ═══════════════════════════════════════════
static bool isHUDScreen()
{
  ScreenID s = osState.current;
  return (s == SCR_DISCONNECTED      ||
          s == SCR_CONNECTED_INFO    ||
          s == SCR_HUD_POSITION_HOLD ||
          s == SCR_HUD_ACTIVE_FLIGHT);
}

static bool isSettingsScreen()
{
  ScreenID s = osState.current;
  return (s == SCR_SETTINGS_MENU    ||
          s == SCR_DISP_SETTINGS        ||
          s == SCR_DISP_SETTINGS  ||
          s == SCR_DISP_SETTINGS     ||
          s == SCR_CTRL_MENU        ||
          s == SCR_CTRL_JOY_MODE    ||
          s == SCR_CTRL_DEADZONE    ||
          s == SCR_CTRL_THROTTLE_CURVE    ||
          s == SCR_CTRL_CALIBRATION ||
          s == SCR_CTRL_CAL_DONE    ||
          s == SCR_RADIO_SETTINGS       ||
          s == SCR_SOUND_INFO       ||
          s == SCR_PID_MENU         ||
          s == SCR_PID_ROLL         ||
          s == SCR_PID_PITCH        ||
          s == SCR_PID_YAW          ||
          s == SCR_PID_THR          ||
          s == SCR_PID_LIVE);
}

static bool isLogsScreen()
{
  ScreenID s = osState.current;
  return (s == SCR_LOGS_EMPTY         ||
          s == SCR_LOGS_LIST          ||
          s == SCR_LOG_OVERVIEW       ||
          s == SCR_LOG_ALT_GRAPH      ||
          s == SCR_LOG_BAT_GRAPH      ||
          s == SCR_LOG_ATT_GRAPH      ||
          s == SCR_LOG_EVENTS         ||
          s == SCR_LOG_DELETE_CONFIRM ||
          s == SCR_LOG_EXPORT         ||
          s == SCR_STATS_DASHBOARD);
}

static bool isAboutScreen()
{
  ScreenID s = osState.current;
  return (s == SCR_ABOUT       ||
          s == SCR_CREDITS     ||
          s == SCR_HW_INFO     ||
          s == SCR_FLASH_STATS ||
          s == SCR_LICENSES);
}
// ═══════════════════════════════════════════
//  INCREMENTAL THROTTLE
// ═══════════════════════════════════════════


// ═══════════════════════════════════════════
//  RAW JOYSTICK CENTERS (measured from your hardware)
//  These are the actual ADC readings at rest position
// ═══════════════════════════════════════════
#define J1X_CENTER  1900
#define J1Y_CENTER  1870
#define J2X_CENTER  2000
#define J2Y_CENTER  1940
#define JOY_DZ      200    // deadzone in raw ADC units — increase if drifting

// ═══════════════════════════════════════════
//  INCREMENTAL THROTTLE
// ═══════════════════════════════════════════
#define THROTTLE_UP_RATE    240.0f    // units/sec at full deflection — same as before
#define THROTTLE_DOWN_RATE  2000.0f   // ← NEW — 3x faster falloff when decreasing

static void updateThrottle()
{
  if(!novaX.drone.isArmed) {
    novaX.remote.throttle = 0;
    return;
  }

  static uint32_t lastThrMs = 0;
  static uint32_t dbgMs = 0;
  uint32_t now = millis();

  if(now - dbgMs > 500) {
    dbgMs = now;
    Serial.print("J1Y="); Serial.print(analogRead(JOY1_Y));
    Serial.print(" center="); Serial.print(J1Y_CENTER);
    Serial.print(" thr="); Serial.println(novaX.remote.throttle);
  }

  uint32_t dt  = now - lastThrMs;
  if(dt < 10) return;
  lastThrMs = now;

  int raw = analogRead(JOY1_Y);
  int deflection = raw - J1Y_CENTER;

  if(abs(deflection) <= JOY_DZ) return;

  int def = deflection > 0 ? deflection - JOY_DZ : deflection + JOY_DZ;
  int maxDef = deflection > 0 ? (4095 - J1Y_CENTER - JOY_DZ)
                               : (J1Y_CENTER - 0 - JOY_DZ);
  if(maxDef <= 0) return;

  float ratio = constrain(abs(def) / (float)maxDef, 0.0f, 1.0f);

  float rate = (deflection > 0) ? THROTTLE_UP_RATE : THROTTLE_DOWN_RATE;   // ← NEW
  float delta = ratio * rate * (dt / 1000.0f);                             // ← changed: was hardcoded 80.0f

  float newThr = (float)novaX.remote.throttle;
  if(deflection > 0) newThr += delta;
  else               newThr -= delta;

  novaX.remote.throttle = (int16_t)constrain(newThr, 0.0f, 1000.0f);
}
// ═══════════════════════════════════════════
//  JOYSTICK READING
// ═══════════════════════════════════════════
static void updateJoysticks()
{
  // Read raw values — store for calibration screen display
  novaX.remote.joy1X = analogRead(JOY1_X);
  novaX.remote.joy1Y = analogRead(JOY1_Y);
  novaX.remote.joy2X = analogRead(JOY2_X);
  novaX.remote.joy2Y = analogRead(JOY2_Y);

  // YAW — Joy1 X
  int yawRaw = novaX.remote.joy1X - J1X_CENTER;
  if(abs(yawRaw) <= JOY_DZ) novaX.remote.yaw = 0;
  else novaX.remote.yaw = constrain(
    map(yawRaw, -(4095-J1X_CENTER), (4095-J1X_CENTER), -500, 500),
    -500, 500);

  // PITCH — Joy2 Y (inverted: push up = positive pitch)
  int pitchRaw = -(novaX.remote.joy2Y - J2Y_CENTER);  // negate for inversion
  if(abs(pitchRaw) <= JOY_DZ) novaX.remote.pitch = 0;
  else novaX.remote.pitch = constrain(
    map(pitchRaw, -(4095-J2Y_CENTER), (4095-J2Y_CENTER), -500, 500),
    -500, 500);

  // ROLL — Joy2 X
  int rollRaw = novaX.remote.joy2X - J2X_CENTER;
  if(abs(rollRaw) <= JOY_DZ) novaX.remote.roll = 0;
  else novaX.remote.roll = constrain(
    map(rollRaw, -(4095-J2X_CENTER), (4095-J2X_CENTER), -500, 500),
    -500, 500);

  // THROTTLE — separate incremental logic
  updateThrottle();
}
// ═══════════════════════════════════════════
//  DISPLAY TIMEOUT
// ═══════════════════════════════════════════
static bool displayOn = true;

static void checkDisplayTimeout()
{
  if(novaX.disp.timeoutSec == 0) return; // never timeout

  uint32_t now = millis();
  uint32_t timeout = novaX.disp.timeoutSec * 1000UL;

  if(now - osState.lastInputTime > timeout && displayOn) {
    // Turn off backlight (not implemented yet)
    // digitalWrite(TFT_BL, LOW);
    displayOn = false;
  }

  if(now - osState.lastInputTime < timeout && !displayOn) {
    // Turn backlight back on
    // digitalWrite(TFT_BL, HIGH);
    displayOn = true;
    osState.needsRedraw = true;
  }
}
static void updateRemoteBattery()
{
  static uint32_t lastBattMs = 0;
  uint32_t now = millis();
  if(now - lastBattMs < 1000) return;
  lastBattMs = now;

  uint16_t mv = readBatteryVoltageMv();
  novaX.remote.remoteBatV   = mv / 1000.0f;
  novaX.remote.remoteBatPct = batteryPercent(mv);
}
// ═══════════════════════════════════════════
//  MAIN LOOP
// ═══════════════════════════════════════════
void loop()
{
  // Always update joysticks
  updateJoysticks();
  updateRemoteBattery(); 
  nrfLoop();  
  // Display timeout
  checkDisplayTimeout();

  // OS core — redraws + input handling
  osLoop();
  homeLoop();

  // Branch-specific loops
  if(isHUDScreen())      hudLoop();
  if(isSettingsScreen()) settingsLoop();
  if(isLogsScreen())     logsLoop();
  if(isAboutScreen())    aboutLoop();

  // Write flight log samples during flight
  if(novaX.drone.isArmed)
    logsWriteSample();
}