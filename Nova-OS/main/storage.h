#pragma once
#include <Arduino.h>
#include "drone.h"

// ═══════════════════════════════════════════
//  SD CARD PATHS
// ═══════════════════════════════════════════
#define PATH_PID      "/config/pid.json"
#define PATH_JOY      "/config/joystick.json"
#define PATH_DISP     "/config/display.json"
#define PATH_RADIO    "/config/radio.json"
#define PATH_LIMITS   "/config/limits.json"
#define PATH_LOGS_DIR "/logs"

// ═══════════════════════════════════════════
//  FUNCTIONS
// ═══════════════════════════════════════════

// Init — load all settings on boot
void storageInit();

// Individual save/load
bool savePID();
bool loadPID();
bool saveJoystick();
bool loadJoystick();
bool saveDisplay();
bool loadDisplay();
bool saveRadio();
bool loadRadio();
bool saveLimits();
bool loadLimits();

// Save/load everything at once
bool loadAllSettings();
bool saveAllSettings();
