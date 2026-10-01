#pragma once
#include <Arduino.h>
#include "graphics.h"

// ═══════════════════════════════════════════
//  SCREEN IDs — matches exactly what user
//  designed in PixelLab
// ═══════════════════════════════════════════
enum ScreenID {

  // BOOT
  SCR_BOOT_SPLASH = 0,
  SCR_BOOT_ERROR,

  // HOME
  SCR_HOME,

  // FLY — CONNECTION
  SCR_DISCONNECTED,
  SCR_CONNECTED_INFO,      // found + getting info combined

  // FLY — HUD
//  SCR_HUD_DISARMED,
//  SCR_HUD_ARMED,
  SCR_HUD_POSITION_HOLD,
  SCR_HUD_ACTIVE_FLIGHT,

  // FLY — AUTO TUNE
  SCR_AUTOTUNE_HOME,
  SCR_AUTOTUNE_PROGRESS,
  SCR_AUTOTUNE_RESULTS,
  SCR_AUTOTUNE_FAILED,

  // FLY — PRE FLIGHT
  SCR_CHECKLIST,
  SCR_PROPS_CHECK,
  SCR_ALL_CLEAR,

  // LOGS
  SCR_LOGS_EMPTY,
  SCR_LOGS_LIST,
  SCR_LOG_OVERVIEW,
  SCR_LOG_ALT_GRAPH,
  SCR_LOG_BAT_GRAPH,
  SCR_LOG_ATT_GRAPH,
  SCR_LOG_EVENTS,
  SCR_LOG_DELETE_CONFIRM,
  SCR_LOG_EXPORT,
  SCR_LOG_RAW_DATA,
  SCR_STATS_DASHBOARD,

  // SETTINGS
  SCR_SETTINGS_MENU,
  SCR_DISP_SETTINGS,       // ONE screen: brightness + timeout combined
  SCR_CTRL_MENU,
  SCR_CTRL_JOY_MODE,
  SCR_CTRL_DEADZONE,
  SCR_CTRL_THROTTLE_CURVE,
  SCR_CTRL_CALIBRATION,
  SCR_CTRL_CAL_DONE,
  SCR_RADIO_SETTINGS,
  SCR_SOUND_INFO,
  SCR_PID_MENU,
  SCR_PID_ROLL,
  SCR_PID_PITCH,
  SCR_PID_YAW,
  SCR_PID_THR,             // throttle min/max
  SCR_PID_LIVE,

  // ABOUT
  SCR_ABOUT,
  SCR_CREDITS,
  SCR_HW_INFO,
  SCR_FLASH_STATS,
  SCR_LICENSES,

  // CONTROLLER TEST
  SCR_TEST_HOME,
  SCR_TEST_JOYSTICK,
  SCR_TEST_BUTTON,

  // LOCK
  SCR_LOCK_PIN,
  SCR_LOCK_WRONG_PIN,

  // NOTIFICATIONS
  SCR_NOTIF_CENTER,
  SCR_NOTIF_DETAIL,
  SCR_NOTIF_ACHIEVEMENT,

  // ═══════════════════════════════════════
  //  OVERLAYS (drawn on top of any screen)
  // ═══════════════════════════════════════
  SCR_OVL_CONNECTION_FAILED,
  SCR_OVL_RECONNECTING,
  SCR_OVL_WEAK_SIGNAL,
  SCR_OVL_SIGNAL_LOST,
  SCR_OVL_SIGNAL_RESTORED,
  SCR_OVL_DRONE_BAT_LOW,
  SCR_OVL_REMOTE_BAT_LOW,
  SCR_OVL_BOTH_BAT_LOW,
  SCR_OVL_SENSOR_ERROR,
  SCR_OVL_MODE_ACTIVE,
  SCR_OVL_MODE_HOLD,
  SCR_OVL_FAIL_AUTO_LAND,
  SCR_OVL_FAIL_MOTOR_CUTOFF,
  SCR_OVL_FAIL_SAFE_LAND,
  SCR_OVL_SAVED,
  SCR_OVL_SUCCESS,
  SCR_OVL_ERROR,
  SCR_OVL_WARNING,
  SCR_OVL_LOADING,
  SCR_OVL_CONFIRM,
  SCR_OVL_TOAST,

  // OBSTACLE POPUPS (15 combinations)
  SCR_OBS_FRONT,
  SCR_OBS_BACK,
  SCR_OBS_LEFT,
  SCR_OBS_RIGHT,
  SCR_OBS_TOP,
  SCR_OBS_FRONT_LEFT,
  SCR_OBS_FRONT_RIGHT,
  SCR_OBS_BACK_LEFT,
  SCR_OBS_BACK_RIGHT,
  SCR_OBS_FRONT_BACK,
  SCR_OBS_LEFT_RIGHT,
  SCR_OBS_FRONT_LEFT_RIGHT,
  SCR_OBS_BACK_LEFT_RIGHT,
  SCR_OBS_FRONT_BACK_LEFT,
  SCR_OBS_ALL,

  SCR_COUNT  // total — always last!
};

// ═══════════════════════════════════════════
//  OS STATE
// ═══════════════════════════════════════════
struct OSState {
  ScreenID  current;
  ScreenID  previous;
  bool      needsRedraw;
  bool      overlayActive;
  ScreenID  overlayID;
  uint32_t  lastInputTime;
};

// ═══════════════════════════════════════════
//  OS FUNCTIONS
// ═══════════════════════════════════════════
#define MAX_NAV_STACK  16

void osBackStack();
void osInit();
void osLoop();
void osGoto(ScreenID id);
void osBack();
void osShowOverlay(ScreenID id, uint16_t x=60, uint16_t y=70);
void osHideOverlay();
void osRedraw();
void osLoadBackground(ScreenID id);

extern OSState osState;
