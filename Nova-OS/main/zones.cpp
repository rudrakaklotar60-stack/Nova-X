#include "zones.h"
#include "graphics.h"
#include <SD_MMC.h>
#include <ArduinoJson.h>
#include <math.h>

ZoneTable zoneTable;
extern volatile bool sdBusy;

static const char* screenNames[SCR_COUNT] = {
  "SCR_BOOT_SPLASH", "SCR_BOOT_ERROR", "SCR_HOME", "SCR_DISCONNECTED",
  "SCR_CONNECTED_INFO", "SCR_HUD_POSITION_HOLD", "SCR_HUD_ACTIVE_FLIGHT",
  "SCR_AUTOTUNE_HOME", "SCR_AUTOTUNE_PROGRESS", "SCR_AUTOTUNE_RESULTS",
  "SCR_AUTOTUNE_FAILED", "SCR_CHECKLIST", "SCR_PROPS_CHECK", "SCR_ALL_CLEAR",
  "SCR_LOGS_EMPTY", "SCR_LOGS_LIST", "SCR_LOG_OVERVIEW", "SCR_LOG_ALT_GRAPH",
  "SCR_LOG_BAT_GRAPH", "SCR_LOG_ATT_GRAPH", "SCR_LOG_EVENTS",
  "SCR_LOG_DELETE_CONFIRM", "SCR_LOG_EXPORT", "SCR_LOG_RAW_DATA",
  "SCR_STATS_DASHBOARD", "SCR_SETTINGS_MENU", "SCR_DISP_SETTINGS",
  "SCR_CTRL_MENU", "SCR_CTRL_JOY_MODE", "SCR_CTRL_DEADZONE",
  "SCR_CTRL_THROTTLE_CURVE", "SCR_CTRL_CALIBRATION", "SCR_CTRL_CAL_DONE",
  "SCR_RADIO_SETTINGS", "SCR_SOUND_INFO", "SCR_PID_MENU", "SCR_PID_ROLL",
  "SCR_PID_PITCH", "SCR_PID_YAW", "SCR_PID_THR", "SCR_PID_LIVE",
  "SCR_ABOUT", "SCR_CREDITS", "SCR_HW_INFO", "SCR_FLASH_STATS", "SCR_LICENSES",
  "SCR_TEST_HOME", "SCR_TEST_JOYSTICK", "SCR_TEST_BUTTON", "SCR_LOCK_PIN",
  "SCR_LOCK_WRONG_PIN", "SCR_NOTIF_CENTER", "SCR_NOTIF_DETAIL",
  "SCR_NOTIF_ACHIEVEMENT", "SCR_OVL_CONNECTION_FAILED", "SCR_OVL_RECONNECTING",
  "SCR_OVL_WEAK_SIGNAL", "SCR_OVL_SIGNAL_LOST", "SCR_OVL_SIGNAL_RESTORED",
  "SCR_OVL_DRONE_BAT_LOW", "SCR_OVL_REMOTE_BAT_LOW", "SCR_OVL_BOTH_BAT_LOW",
  "SCR_OVL_SENSOR_ERROR", "SCR_OVL_MODE_ACTIVE", "SCR_OVL_MODE_HOLD",
  "SCR_OVL_FAIL_AUTO_LAND", "SCR_OVL_FAIL_MOTOR_CUTOFF", "SCR_OVL_FAIL_SAFE_LAND",
  "SCR_OVL_SAVED", "SCR_OVL_SUCCESS", "SCR_OVL_ERROR", "SCR_OVL_WARNING",
  "SCR_OVL_LOADING", "SCR_OVL_CONFIRM", "SCR_OVL_TOAST", "SCR_OBS_FRONT",
  "SCR_OBS_BACK", "SCR_OBS_LEFT", "SCR_OBS_RIGHT", "SCR_OBS_TOP",
  "SCR_OBS_FRONT_LEFT", "SCR_OBS_FRONT_RIGHT", "SCR_OBS_BACK_LEFT",
  "SCR_OBS_BACK_RIGHT", "SCR_OBS_FRONT_BACK", "SCR_OBS_LEFT_RIGHT",
  "SCR_OBS_FRONT_LEFT_RIGHT", "SCR_OBS_BACK_LEFT_RIGHT",
  "SCR_OBS_FRONT_BACK_LEFT", "SCR_OBS_ALL",
};

static const char* configFiles[SCR_COUNT] = {
  "/config/boot_splash.json", "/config/boot_error.json", "/config/home.json",
  "/config/disconnected.json", "/config/connected_info.json",
  "/config/hud_poshold.json", "/config/hud_active_flight.json",
  "/config/autotune_home.json", "/config/autotune_progress.json",
  "/config/autotune_results.json", "/config/autotune_failed.json",
  "/config/checklist.json", "/config/props_check.json", "/config/all_clear.json",
  "/config/logs_empty.json", "/config/logs_list.json", "/config/log_overview.json",
  "/config/log_alt_graph.json", "/config/log_bat_graph.json",
  "/config/log_att_graph.json", "/config/log_events.json",
  "/config/log_delete.json", "/config/log_export.json", "/config/log_raw_data.json",
  "/config/stats_dash.json", "/config/settings.json", "/config/disp_settings.json",
  "/config/ctrl_menu.json", "/config/ctrl_joymode.json", "/config/ctrl_deadzone.json",
  "/config/ctrl_throttle.json", "/config/ctrl_cal.json", "/config/ctrl_cal_done.json",
  "/config/radio_settings.json", "/config/sound_info.json", "/config/pid_menu.json",
  "/config/pid_roll.json", "/config/pid_pitch.json", "/config/pid_yaw.json",
  "/config/pid_thr.json", "/config/pid_live.json", "/config/about.json",
  "/config/credits.json", "/config/hw_info.json", "/config/flash_stats.json",
  "/config/licenses.json", "/config/test_home.json", "/config/test_joystick.json",
  "/config/test_button.json", "/config/lock_pin.json", "/config/lock_wrong.json",
  "/config/notif_center.json", "/config/notif_detail.json", "/config/notif_achieve.json",
  "/config/ovl_conn_fail.json", "/config/ovl_reconnect.json", "/config/ovl_weak_sig.json",
  "/config/ovl_sig_lost.json", "/config/ovl_sig_resto.json", "/config/ovl_drone_bat.json",
  "/config/ovl_remote_bat.json", "/config/ovl_both_bat.json", "/config/ovl_sensor.json",
  "/config/ovl_mode_act.json", "/config/ovl_mode_hold.json", "/config/ovl_fail_land.json",
  "/config/ovl_fail_cut.json", "/config/ovl_fail_safe.json", "/config/ovl_saved.json",
  "/config/ovl_success.json", "/config/ovl_error.json", "/config/ovl_warning.json",
  "/config/ovl_loading.json", "/config/ovl_confirm.json", "/config/ovl_toast.json",
  "/config/obs_front.json", "/config/obs_back.json", "/config/obs_left.json",
  "/config/obs_right.json", "/config/obs_top.json", "/config/obs_front_left.json",
  "/config/obs_front_right.json", "/config/obs_back_left.json",
  "/config/obs_back_right.json", "/config/obs_front_back.json",
  "/config/obs_left_right.json", "/config/obs_flr.json", "/config/obs_blr.json",
  "/config/obs_fbl.json", "/config/obs_all.json",
};

static uint8_t colorNameToID(const char* name)
{
  if(strcmp(name, "GREEN")   == 0) return DCOL_GREEN;
  if(strcmp(name, "RED")     == 0) return DCOL_RED;
  if(strcmp(name, "YELLOW")  == 0) return DCOL_YELLOW;
  if(strcmp(name, "CYAN")    == 0) return DCOL_CYAN;
  if(strcmp(name, "WHITE")   == 0) return DCOL_WHITE;
  if(strcmp(name, "ORANGE")  == 0) return DCOL_ORANGE;
  if(strcmp(name, "dynamic") == 0) return DCOL_DYNAMIC;
  return DCOL_WHITE;
}

static uint8_t drawTypeToID(const char* name)
{
  if(strcmp(name, "hbar")    == 0) return DRAW_HBAR;
  if(strcmp(name, "vbar")    == 0) return DRAW_VBAR;
  if(strcmp(name, "circle")  == 0) return DRAW_CIRCLE;
  if(strcmp(name, "text")    == 0) return DRAW_TEXT;
  if(strcmp(name, "horizon") == 0) return DRAW_HORIZON;
  if(strcmp(name, "signal_bars") == 0) return DRAW_SIGNAL_BARS;
  return DRAW_RECT;
}

static uint16_t resolveColor(uint8_t colorID, uint16_t dynColor)
{
  switch(colorID) {
    case DCOL_GREEN:   return GREEN;
    case DCOL_RED:     return RED;
    case DCOL_YELLOW:  return YELLOW;
    case DCOL_CYAN:    return CYAN;
    case DCOL_WHITE:   return WHITE;
    case DCOL_ORANGE:  return ORANGE;
    case DCOL_DYNAMIC: return dynColor;
    default:           return WHITE;
  }
}

static ScreenID nameToID(const char* name)
{
  for(int i = 0; i < SCR_COUNT; i++)
    if(strcmp(screenNames[i], name) == 0)
      return (ScreenID)i;
  return SCR_HOME;
}

void zonesInit()  { zonesClear(); }

void zonesClear()
{
  zoneTable.count     = 0;
  zoneTable.selected  = 0;
  zoneTable.drawCount = 0;
}

bool zonesLoad(ScreenID id)
{
  zonesClear();

  if(id >= SCR_COUNT) return false;

  const char* path = configFiles[id];
  if(!path || path[0] == '\0') return false;

  sdBusy = true;

  File f = SD_MMC.open(path, FILE_READ);
  if(!f) {
    Serial.print("No config: "); Serial.println(path);
    sdBusy = false;
    return false;
  }

  StaticJsonDocument<2048> doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();

  sdBusy = false;

  if(err) {
    Serial.print("JSON err: "); Serial.println(err.c_str());
    return false;
  }

  JsonArray zones = doc["zones"];
  uint8_t zCount = 0;
  for(JsonObject z : zones) {
    if(zCount >= MAX_ZONES) break;
    zoneTable.zones[zCount].x        = z["x"] | 0;
    zoneTable.zones[zCount].y        = z["y"] | 0;
    zoneTable.zones[zCount].w        = z["w"] | 0;
    zoneTable.zones[zCount].h        = z["h"] | 0;
    zoneTable.zones[zCount].actionID = z["action"] | 0;
    const char* target = z["target"] | "SCR_HOME";
    zoneTable.zones[zCount].target   = nameToID(target);
    zCount++;
  }
  zoneTable.count    = zCount;
  zoneTable.selected = 0;

  JsonArray draws = doc["draws"];
  uint8_t dCount = 0;
  for(JsonObject d : draws) {
    if(dCount >= MAX_DRAWS) break;
    const char* did   = d["id"]    | "element";
    const char* dtype = d["type"]  | "rect";
    const char* dcol  = d["color"] | "WHITE";

    strncpy(zoneTable.draws[dCount].id, did, 19);
    zoneTable.draws[dCount].id[19]    = '\0';
    zoneTable.draws[dCount].type      = drawTypeToID(dtype);
    zoneTable.draws[dCount].x         = d["x"] | 0;
    zoneTable.draws[dCount].y         = d["y"] | 0;
    zoneTable.draws[dCount].w         = d["w"] | 0;
    zoneTable.draws[dCount].h         = d["h"] | 0;
    zoneTable.draws[dCount].colorID   = colorNameToID(dcol);
    zoneTable.draws[dCount].dynColor  = WHITE;
    zoneTable.draws[dCount].value     = 0;
    zoneTable.draws[dCount].lastValue = 255;
    zoneTable.draws[dCount].text[0]      = '\0';
    zoneTable.draws[dCount].lastText[0]  = '\0';
    zoneTable.draws[dCount].textSize     = d["size"] | 1;
    zoneTable.draws[dCount].rollTenths      = 0;
    zoneTable.draws[dCount].pitchTenths     = 0;
    zoneTable.draws[dCount].lastRollTenths  = 9999;
    zoneTable.draws[dCount].lastPitchTenths = 9999;
    dCount++;
  }
  zoneTable.drawCount = dCount;

  Serial.print("Loaded: "); Serial.print(zCount);
  Serial.print(" zones, ");  Serial.print(dCount);
  Serial.println(" draws");
  return true;
}

// ═══════════════════════════════════════════
//  ARTIFICIAL HORIZON
// ═══════════════════════════════════════════
struct LadderRung { int8_t deg; bool major; };
static const LadderRung rungs[] = {
  { 20, true }, { 15, false }, { 10, true }, { 5, false },
  { -5, false }, { -10, true }, { -15, false }, { -20, true }
};
#define RUNG_COUNT (sizeof(rungs) / sizeof(LadderRung))

#define LADDER_MAJOR_HALF_WIDTH_PX 35   // longer rungs — 10°, 20°, etc.
#define LADDER_MINOR_HALF_WIDTH_PX 18   // shorter rungs — 5°, 15°, etc.

#define HORIZON_PX_PER_DEG_PITCH  2.0f
#define HORIZON_DEADBAND_TENTHS   2


static void drawPitchLadder(DrawZone& d, int cx, int trueCenterY, float sinT, float cosT, float slope)
{
  int top = d.y, bottom = d.y + d.h - 1;
  int left = d.x, right = d.x + d.w - 1;

  for(uint8_t r = 0; r < RUNG_COUNT; r++)
  {
    int halfW = rungs[r].major ? LADDER_MAJOR_HALF_WIDTH_PX : LADDER_MINOR_HALF_WIDTH_PX;

    // FIXED offset from center — no longer shifts with pitch, only rotates with roll
    float R = -rungs[r].deg * HORIZON_PX_PER_DEG_PITCH;
    float rCx = cx - R * sinT;
    float rCy = trueCenterY + R * cosT;

    int x0 = constrain((int)(rCx - halfW), left, right);
    int x1 = constrain((int)(rCx + halfW), left, right);

    int y0 = (int)(rCy + (x0 - rCx) * slope);
    int y1 = (int)(rCy + (x1 - rCx) * slope);

    if(y0 < top || y0 > bottom) continue;
    if(y1 < top || y1 > bottom) continue;

    drawLine(x0, y0, x1, y1, WHITE);

    if(rungs[r].major)
    {
      char buf[4];
      snprintf(buf, sizeof(buf), "%d", abs(rungs[r].deg));
      int textW  = textWidthF(buf, FONT_SMALL);
      int labelX = x0 - textW - 4;
      int labelY = (int)rCy - 6;
      if(labelX >= d.x)
        printTextF(labelX, labelY, buf, WHITE, FONT_SMALL);
    }
  }
}

static void drawArtificialHorizon(DrawZone& d)
{
  int16_t dRoll  = abs(d.rollTenths  - d.lastRollTenths);
  int16_t dPitch = abs(d.pitchTenths - d.lastPitchTenths);
  if(dRoll < HORIZON_DEADBAND_TENTHS && dPitch < HORIZON_DEADBAND_TENTHS) return;

  d.lastRollTenths  = d.rollTenths;
  d.lastPitchTenths = d.pitchTenths;

  float rollDeg  = d.rollTenths  / 10.0f;
  float pitchDeg = d.pitchTenths / 10.0f;
  float rollRad  = rollDeg * PI / 180.0f;

  float sinT  = sin(rollRad);
  float cosT  = cos(rollRad);
  float slope = tan(rollRad);   // every line — horizon and every rung — shares this one tilt

  static const uint16_t SKY_COLOR    = 0x34BF;
  static const uint16_t GROUND_COLOR = 0x9A45;

  int cx = d.x + d.w / 2;
  int trueCenterY = d.y + d.h / 2;   // ← the ONE fixed pivot — never shifts, regardless of pitch
  int top = d.y, bottom = d.y + d.h - 1;

  float horizonR = -pitchDeg * HORIZON_PX_PER_DEG_PITCH;
  float hCx = cx - horizonR * sinT;
  float hCy = trueCenterY + horizonR * cosT;

  for(int col = 0; col < d.w; col++)
  {
    int x = d.x + col;
    int lineY = constrain((int)(hCy + (x - hCx) * slope), top, bottom);

    if(lineY > top)
      fillRect(x, top, 1, lineY - top, SKY_COLOR);
    if(lineY <= bottom)
      fillRect(x, lineY, 1, bottom - lineY + 1, GROUND_COLOR);
  }

  int hy0 = constrain((int)(hCy + (d.x - hCx) * slope), top, bottom);
  int hy1 = constrain((int)(hCy + (d.x + d.w - 1 - hCx) * slope), top, bottom);
  drawLine(d.x, hy0, d.x + d.w - 1, hy1, WHITE);

  drawPitchLadder(d, cx, trueCenterY, sinT, cosT, slope);
}
#define SIGNAL_BAR_COUNT   5
#define SIGNAL_BAR_GAP     2

static void drawSignalBars(DrawZone& d)
{
  if(d.value == d.lastValue) return;
  d.lastValue = d.value;

  fillRect(d.x, d.y, d.w, d.h, BLACK);   // clear whole widget first

  uint8_t barsLit = d.value;   // 0-5, set directly from RSSI% in steps of 20
  int barW = (d.w - (SIGNAL_BAR_GAP * (SIGNAL_BAR_COUNT - 1))) / SIGNAL_BAR_COUNT;

  for(uint8_t i = 0; i < SIGNAL_BAR_COUNT; i++)
  {
    int barH = (d.h * (i + 1)) / SIGNAL_BAR_COUNT;   // increasing height, left to right
    int barX = d.x + i * (barW + SIGNAL_BAR_GAP);
    int barY = d.y + d.h - barH;                     // bottom-aligned, like phone signal bars

    uint16_t color = (i < barsLit) ? GREEN : GRAY;    // lit vs unlit
    fillRect(barX, barY, barW, barH, color);
  }
}
void zonesRenderDraws()
{
  for(uint8_t i = 0; i < zoneTable.drawCount; i++) {
    DrawZone& d = zoneTable.draws[i];
    uint16_t col = resolveColor(d.colorID, d.dynColor);

    switch(d.type) {
      case DRAW_HBAR: {
        if(d.value == d.lastValue) break;
        fillRect(d.x, d.y, d.w, d.h, BLACK);
        uint16_t fillW = (d.w * d.value) / 100;
        fillRect(d.x, d.y, fillW, d.h, col);
        d.lastValue = d.value;
        break;
      }
      case DRAW_VBAR: {
        if(d.value == d.lastValue) break;
        fillRect(d.x, d.y, d.w, d.h, BLACK);
        uint16_t fillH = (d.h * d.value) / 100;
        fillRect(d.x, d.y + d.h - fillH, d.w, fillH, col);
        d.lastValue = d.value;
        break;
      }
      case DRAW_CIRCLE: {
        if(d.value == d.lastValue) break;
        fillCircle(d.x, d.y, d.w, BLACK);
        fillCircle(d.x, d.y, d.w, col);
        d.lastValue = d.value;
        break;
      }
      case DRAW_RECT: {
        if(d.value == d.lastValue) break;
        fillRect(d.x, d.y, d.w, d.h, col);
        d.lastValue = d.value;
        break;
      }
      case DRAW_TEXT: {
        if(strcmp(d.text, d.lastText) == 0) break;
        fillRect(d.x, d.y, d.w, d.h, BLACK);
        printText(d.x, d.y, d.text, col, d.textSize);
        strncpy(d.lastText, d.text, 7);
        d.lastText[7] = '\0';
        break;
      }
      case DRAW_HORIZON: {
        drawArtificialHorizon(d);
        break;
      }
      case DRAW_SIGNAL_BARS: {
  drawSignalBars(d);
  break;
    }
    }
  }
}
void zonesForceRedrawAll()
{
  for(uint8_t i = 0; i < zoneTable.drawCount; i++) {
    DrawZone& d = zoneTable.draws[i];
    d.lastValue       = 255;    // guaranteed to differ from any real 0-100 value
    d.lastText[0]      = '\0';  // guaranteed to differ from any real text string
    d.lastRollTenths   = 9999;
    d.lastPitchTenths  = 9999;
  }
}
void zonesSetValue(const char* id, uint8_t value)
{
  DrawZone* d = zonesGetDraw(id);
  if(d) d->value = value;
}

void zonesSetColor(const char* id, uint16_t color)
{
  DrawZone* d = zonesGetDraw(id);
  if(d) { d->colorID = DCOL_DYNAMIC; d->dynColor = color; }
}



void zonesSetHorizon(const char* id, int16_t rollTenths, int16_t pitchTenths)
{
  DrawZone* d = zonesGetDraw(id);
  if(d) { d->rollTenths = rollTenths; d->pitchTenths = pitchTenths; }
}

DrawZone* zonesGetDraw(const char* id)
{
  for(uint8_t i = 0; i < zoneTable.drawCount; i++)
    if(strcmp(zoneTable.draws[i].id, id) == 0)
      return &zoneTable.draws[i];
  return nullptr;
}

void zonesDrawHighlight()
{
  if(zoneTable.count == 0) return;
  if(zoneTable.selected < 0) return;
  Zone& z = zoneTable.zones[zoneTable.selected];
  drawRect(z.x, z.y, z.w, z.h, CYAN);
}

static void zonesClearHighlight(int8_t idx)
{
  if(idx < 0 || idx >= zoneTable.count) return;
  Zone& z = zoneTable.zones[idx];
  drawRect(z.x, z.y, z.w, z.h, BLACK);
}

void zonesJoyUp()
{
  if(zoneTable.count == 0) return;
  zonesClearHighlight(zoneTable.selected);
  zoneTable.selected--;
  if(zoneTable.selected < 0)
    zoneTable.selected = zoneTable.count - 1;
  zonesDrawHighlight();
}

void zonesJoyDown()
{
  if(zoneTable.count == 0) return;
  zonesClearHighlight(zoneTable.selected);
  zoneTable.selected++;
  if(zoneTable.selected >= zoneTable.count)
    zoneTable.selected = 0;
  zonesDrawHighlight();
}

ScreenID zonesJoyClick()
{
  if(zoneTable.count == 0)   return osState.current;
  if(zoneTable.selected < 0) return osState.current;
  return zoneTable.zones[zoneTable.selected].target;
}

ScreenID zonesCheckTouch(uint16_t x, uint16_t y)
{
  for(uint8_t i = 0; i < zoneTable.count; i++) {
    Zone& z = zoneTable.zones[i];
    if(x >= z.x && x <= z.x + z.w &&
       y >= z.y && y <= z.y + z.h)
    {
      zonesClearHighlight(zoneTable.selected);
      zoneTable.selected = i;
      zonesDrawHighlight();
      return z.target;
    }
  }
  return osState.current;
}

void zonesSetText(const char* id, const char* text)
{
  for(uint8_t i = 0; i < zoneTable.drawCount; i++) {
    if(strcmp(zoneTable.draws[i].id, id) == 0) {
      strncpy(zoneTable.draws[i].text, text, 7);
      zoneTable.draws[i].text[7] = '\0';
      return;
    }
  }
}