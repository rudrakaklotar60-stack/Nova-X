#include "os.h"
#include "graphics.h"
#include "input.h"
#include "zones.h"
#include "home.h"
#include "hud.h"
#include <SD_MMC.h>

OSState osState;
HomeItem selectedItem = HOME_FLY;
HomeItem lastDrawnItem = (HomeItem)255;

// ═══════════════════════════════════════════
//  SCREEN → SD FILE MAP
//  Must match ScreenID enum order exactly!
// ═══════════════════════════════════════════
static const char* screenFiles[SCR_COUNT] = {
  "/screens/boot_splash.raw",         // SCR_BOOT_SPLASH
  "/screens/boot_error.raw",          // SCR_BOOT_ERROR
  "/screens/home.raw",                // SCR_HOME
  "/screens/disconnected.raw",        // SCR_DISCONNECTED
  "/screens/connected_info.raw",      // SCR_CONNECTED_INFO
//  "/screens/hud_disarmed.raw",        // SCR_HUD_DISARMED
//  "/screens/hud_armed.raw",           // SCR_HUD_ARMED
  "/screens/hud_poshold.raw",         // SCR_HUD_POSITION_HOLD
  "/screens/hud_active_flight.raw",         // SCR_HUD_AUTO_LANDING
  "/screens/autotune_home.raw",       // SCR_AUTOTUNE_HOME
  "/screens/autotune_progress.raw",   // SCR_AUTOTUNE_PROGRESS
  "/screens/autotune_results.raw",    // SCR_AUTOTUNE_RESULTS
  "/screens/autotune_failed.raw",     // SCR_AUTOTUNE_FAILED
  "/screens/checklist.raw",           // SCR_CHECKLIST
  "/screens/props_check.raw",         // SCR_PROPS_CHECK
  "/screens/all_clear.raw",           // SCR_ALL_CLEAR
  "/screens/logs_empty.raw",          // SCR_LOGS_EMPTY
  "/screens/logs_list.raw",           // SCR_LOGS_LIST
  "/screens/log_overview.raw",        // SCR_LOG_OVERVIEW
  "/screens/log_alt_graph.raw",       // SCR_LOG_ALT_GRAPH
  "/screens/log_bat_graph.raw",       // SCR_LOG_BAT_GRAPH
  "/screens/log_att_graph.raw",       // SCR_LOG_ATT_GRAPH
  "/screens/log_events.raw",          // SCR_LOG_EVENTS
  "/screens/log_delete.raw",          // SCR_LOG_DELETE_CONFIRM
  "/screens/log_export.raw",          // SCR_LOG_EXPORT
  "/screens/log_raw_data.raw",        // SCR_LOG_RAW_DATA
  "/screens/stats_dash.raw",          // SCR_STATS_DASHBOARD
  "/screens/settings.raw",            // SCR_SETTINGS_MENU
  "/screens/disp_set.raw",       // SCR_DISP_SETTINGS
  "/screens/ctrl_menu.raw",           // SCR_CTRL_MENU
  "/screens/ctrl_joymode.raw",        // SCR_CTRL_JOY_MODE
  "/screens/ctrl_deadzone.raw",       // SCR_CTRL_DEADZONE
  "/screens/ctrl_throttle.raw",       // SCR_CTRL_THROTTLE_CURVE
  "/screens/ctrl_cal.raw",            // SCR_CTRL_CALIBRATION
  "/screens/ctrl_cal_done.raw",       // SCR_CTRL_CAL_DONE
  "/screens/radio_menu.raw",      // SCR_RADIO_SETTINGS
  "/screens/sound_info.raw",          // SCR_SOUND_INFO
  "/screens/pid_menu.raw",            // SCR_PID_MENU
  "/screens/pid_roll.raw",            // SCR_PID_ROLL
  "/screens/pid_pitch.raw",           // SCR_PID_PITCH
  "/screens/pid_yaw.raw",             // SCR_PID_YAW
  "/screens/pid_thr.raw",             // SCR_PID_THR
  "/screens/pid_live.raw",            // SCR_PID_LIVE
  "/screens/about.raw",               // SCR_ABOUT
  "/screens/credits.raw",             // SCR_CREDITS
  "/screens/hw_info.raw",             // SCR_HW_INFO
  "/screens/flash_stats.raw",         // SCR_FLASH_STATS
  "/screens/licenses.raw",            // SCR_LICENSES
  "/screens/test_home.raw",           // SCR_TEST_HOME
  "/screens/test_joystick.raw",       // SCR_TEST_JOYSTICK
  "/screens/test_button.raw",         // SCR_TEST_BUTTON
  "/screens/lock_pin.raw",            // SCR_LOCK_PIN
  "/screens/lock_wrong.raw",          // SCR_LOCK_WRONG_PIN
  "/screens/notif_center.raw",        // SCR_NOTIF_CENTER
  "/screens/notif_detail.raw",        // SCR_NOTIF_DETAIL
  "/screens/notif_achieve.raw",       // SCR_NOTIF_ACHIEVEMENT
  "/screens/ovl_conn_fail.raw",       // SCR_OVL_CONNECTION_FAILED
  "/screens/ovl_reconnect.raw",       // SCR_OVL_RECONNECTING
  "/screens/ovl_weak_sig.raw",        // SCR_OVL_WEAK_SIGNAL
  "/screens/ovl_sig_lost.raw",        // SCR_OVL_SIGNAL_LOST
  "/screens/ovl_sig_resto.raw",       // SCR_OVL_SIGNAL_RESTORED
  "/screens/ovl_drone_bat.raw",       // SCR_OVL_DRONE_BAT_LOW
  "/screens/ovl_remote_bat.raw",      // SCR_OVL_REMOTE_BAT_LOW
  "/screens/ovl_both_bat.raw",        // SCR_OVL_BOTH_BAT_LOW
  "/screens/ovl_sensor.raw",          // SCR_OVL_SENSOR_ERROR
  "/screens/ovl_mode_act.raw",        // SCR_OVL_MODE_ACTIVE
  "/screens/ovl_mode_hold.raw",       // SCR_OVL_MODE_HOLD
  "/screens/ovl_fail_land.raw",       // SCR_OVL_FAIL_AUTO_LAND
  "/screens/ovl_fail_cut.raw",        // SCR_OVL_FAIL_MOTOR_CUTOFF
  "/screens/ovl_fail_safe.raw",       // SCR_OVL_FAIL_SAFE_LAND
  "/screens/ovl_saved.raw",           // SCR_OVL_SAVED
  "/screens/ovl_success.raw",         // SCR_OVL_SUCCESS
  "/screens/ovl_error.raw",           // SCR_OVL_ERROR
  "/screens/ovl_warning.raw",         // SCR_OVL_WARNING
  "/screens/ovl_loading.raw",         // SCR_OVL_LOADING
  "/screens/ovl_confirm.raw",         // SCR_OVL_CONFIRM
  "/screens/ovl_toast.raw",           // SCR_OVL_TOAST
  "/screens/obs_front.raw",           // SCR_OBS_FRONT
  "/screens/obs_back.raw",            // SCR_OBS_BACK
  "/screens/obs_left.raw",            // SCR_OBS_LEFT
  "/screens/obs_right.raw",           // SCR_OBS_RIGHT
  "/screens/obs_top.raw",             // SCR_OBS_TOP
  "/screens/obs_front_left.raw",      // SCR_OBS_FRONT_LEFT
  "/screens/obs_front_right.raw",     // SCR_OBS_FRONT_RIGHT
  "/screens/obs_back_left.raw",       // SCR_OBS_BACK_LEFT
  "/screens/obs_back_right.raw",      // SCR_OBS_BACK_RIGHT
  "/screens/obs_front_back.raw",      // SCR_OBS_FRONT_BACK
  "/screens/obs_left_right.raw",      // SCR_OBS_LEFT_RIGHT
  "/screens/obs_flr.raw",             // SCR_OBS_FRONT_LEFT_RIGHT
  "/screens/obs_blr.raw",             // SCR_OBS_BACK_LEFT_RIGHT
  "/screens/obs_fbl.raw",             // SCR_OBS_FRONT_BACK_LEFT
  "/screens/obs_all.raw",             // SCR_OBS_ALL
};

// ═══════════════════════════════════════════
//  LOAD BACKGROUND FROM SD
// ═══════════════════════════════════════════
void osLoadBackground(ScreenID id)
{
  if(id >= SCR_COUNT) {
    fillScreen(BLACK);
    return;
  }

  const char* file = screenFiles[id];
  if(!file || file[0] == '\0') {
    fillScreen(BLACK);
    return;
  }
  
  if(!SD_MMC.exists(file)) {
    Serial.print("Screen missing: ");
    Serial.println(file);
    fillScreen(BLACK);
    return;
  }
  
  drawImageSD(file, 0, 0);
}

// ═══════════════════════════════════════════
//  GO TO SCREEN
// ═══════════════════════════════════════════
static ScreenID navStack[MAX_NAV_STACK];
static uint8_t  navStackTop = 0;

// ═══════════════════════════════════════════
//  GO TO SCREEN
// ═══════════════════════════════════════════
void osGoto(ScreenID id)
{
  // Push current screen onto stack before navigating away
  if(navStackTop < MAX_NAV_STACK) {
    navStack[navStackTop] = osState.current;
    navStackTop++;
  }

  osState.previous      = osState.current;
  osState.current       = id;
  osState.needsRedraw   = true;
  osState.overlayActive = false;
  zonesLoad(id);
  if(id == SCR_DISCONNECTED) hudInit();
}

// ═══════════════════════════════════════════
//  GO BACK (one level, from stack)
// ═══════════════════════════════════════════
void osBackStack()
{
  if(navStackTop == 0) {
    osGoto(SCR_HOME);   // nothing left — fall back to home
    return;
  }

  navStackTop--;
  ScreenID prev = navStack[navStackTop];

  osState.previous      = osState.current;
  osState.current       = prev;
  osState.needsRedraw   = true;
  osState.overlayActive = false;
  zonesLoad(prev);
  // NOTE: doesn't call osGoto() — avoids re-pushing this screen onto the stack
}

// ═══════════════════════════════════════════
//  GO BACK
// ═══════════════════════════════════════════
void osBack()
{
  osGoto(osState.previous);
}

// ═══════════════════════════════════════════
//  OVERLAY
// ═══════════════════════════════════════════
void osShowOverlay(ScreenID id, uint16_t x, uint16_t y)
{
  if(id >= SCR_COUNT) return;

  osState.overlayActive = true;
  osState.overlayID     = id;
  drawImageSD(screenFiles[id], x, y);
}

void osHideOverlay()
{
  osState.overlayActive = false;
  osState.needsRedraw   = true;
}

// ═══════════════════════════════════════════
//  REDRAW
// ═══════════════════════════════════════════
void osRedraw()
{
  osLoadBackground(osState.current);
  zonesForceRedrawAll(); 
  if(osState.overlayActive && osState.overlayID < SCR_COUNT)
    drawImageSD(screenFiles[osState.overlayID], 60, 70);
    
  zonesDrawHighlight();

  // Screen-specific post-background draws
  if(osState.current == SCR_HOME) homeDraw();

  osState.needsRedraw = false;
}

// ═══════════════════════════════════════════
//  OS INIT
// ═══════════════════════════════════════════
void osInit()
{
  osState.current       = SCR_HOME;
  osState.previous      = SCR_HOME;
  osState.needsRedraw   = true;
  osState.overlayActive = false;
  osState.lastInputTime = millis();
  zonesInit();
}

// ═══════════════════════════════════════════
//  OS LOOP
// ═══════════════════════════════════════════
void osLoop()
{
  if(osState.needsRedraw)
    osRedraw();
  inputLoop();
}