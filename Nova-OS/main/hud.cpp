#include "hud.h"
#include "drone.h"
#include "os.h"
#include "zones.h"
#include "nrf.h"
#include "graphics.h"

static ScreenID flightModeToScreen(FlightMode mode);

static ConnState  lastConnState  = (ConnState)-1;
static FlightMode lastFlightMode = (FlightMode)-1;
static bool       lastArmed      = false;
static uint8_t    lastObstacle   = 0xFF;
static ScreenID   activeHUDScreen = SCR_DISCONNECTED;
static uint32_t   connectedSinceMs = 0;
static bool       showingConnInfo  = false;

static uint8_t lastDroneBattPct  = 0xFF;
static uint8_t lastRemoteBattPct = 0xFF;

static void handleConnectionFlow()
{
  ConnState cs = novaX.drone.connState;

  if(cs != lastConnState)
  {
    lastConnState = cs;

    switch(cs)
    {
      case CONN_DISCONNECTED:
      case CONN_SEARCHING:
        showingConnInfo  = false;
        connectedSinceMs = 0;
        if(osState.current != SCR_DISCONNECTED)
          osGoto(SCR_DISCONNECTED);
        break;

      case CONN_CONNECTED:
        showingConnInfo = true;
        osGoto(SCR_CONNECTED_INFO);
        osRedraw();
        connectedSinceMs = millis();
        break;

      case CONN_LOST:
        showingConnInfo = false;
        osShowOverlay(SCR_OVL_SIGNAL_LOST, 60, 70);
        break;

      case CONN_RECONNECTING:
        showingConnInfo = false;
        osShowOverlay(SCR_OVL_RECONNECTING, 60, 70);
        break;

      default:
        break;
    }
  }

  if(showingConnInfo && (millis() - connectedSinceMs > 1000))
  {
    showingConnInfo = false;
    ScreenID hud    = flightModeToScreen(novaX.drone.flightMode);
    activeHUDScreen = hud;
    osGoto(hud);
  }
}

#define ARM_INDICATOR_X  300
#define ARM_INDICATOR_Y   15
#define ARM_INDICATOR_R   10

static void drawArmIndicator()
{
  bool armed = novaX.drone.isArmed;
  if(armed == lastArmed) return;
  lastArmed = armed;

  uint16_t color = armed ? GREEN : RED;
  fillCircle(ARM_INDICATOR_X, ARM_INDICATOR_Y, ARM_INDICATOR_R, color);
  drawCircle(ARM_INDICATOR_X, ARM_INDICATOR_Y, ARM_INDICATOR_R, WHITE);
}

static ScreenID flightModeToScreen(FlightMode mode)
{
  switch(mode) {
    case MODE_POSITION_HOLD: return SCR_HUD_POSITION_HOLD;
    case MODE_ACTIVE_FLIGHT: return SCR_HUD_ACTIVE_FLIGHT;
    default:                 return SCR_HUD_ACTIVE_FLIGHT;
  }
}

static void checkFlightModeChange()
{
  FlightMode fm = novaX.drone.flightMode;
  if(fm == lastFlightMode) return;
  lastFlightMode = fm;

  ScreenID newHUD = flightModeToScreen(fm);
  if(newHUD != activeHUDScreen) {
    activeHUDScreen = newHUD;
    osGoto(newHUD);
  }
}

static void checkOverlays()
{
  if(novaX.drone.connState == CONN_CONNECTED &&
     lastConnState == CONN_LOST)
  {
    osHideOverlay();
    osShowOverlay(SCR_OVL_SIGNAL_RESTORED, 60, 70);
  }

  if(novaX.drone.droneBatPct < 20 &&
     novaX.drone.connState == CONN_CONNECTED)
  {
    osShowOverlay(SCR_OVL_DRONE_BAT_LOW, 60, 70);
  }

  if(novaX.remote.remoteBatPct < 20)
  {
    osShowOverlay(SCR_OVL_REMOTE_BAT_LOW, 60, 70);
  }

  if(novaX.drone.obstacleFlags != lastObstacle)
  {
    lastObstacle = novaX.drone.obstacleFlags;
    if(novaX.drone.obstacleFlags != 0)
      osShowOverlay(SCR_OVL_WARNING, 60, 70);
    else
      osHideOverlay();
  }
}

// ═══════════════════════════════════════════
//  LIVE TELEMETRY — routed through the JSON
//  draw-zone system (zones.cpp), not hardcoded
// ═══════════════════════════════════════════
void hudDrawTelemetry()
{
  ScreenID s = osState.current;
  if(s != SCR_HUD_POSITION_HOLD && s != SCR_HUD_ACTIVE_FLIGHT) return;

  char buf[8];

  static uint8_t lastRssiBars = 0xFF;
uint8_t rssiBars = novaX.drone.rssi / 20;   // 0-100% → 0-5 bars
if(rssiBars > 5) rssiBars = 5;

if(rssiBars != lastRssiBars) {
  lastRssiBars = rssiBars;
  zonesSetValue("rssi_value", rssiBars);
}

  snprintf(buf, sizeof(buf), "%ldcm", novaX.drone.altitudeMm / 10);
  zonesSetText("altitude_value", buf);

  snprintf(buf, sizeof(buf), "%.1f", novaX.drone.roll / 10.0f);
  zonesSetText("roll_value", buf);

  snprintf(buf, sizeof(buf), "%.1f", novaX.drone.pitch / 10.0f);
  zonesSetText("pitch_value", buf);

  if(novaX.drone.droneBatPct != lastDroneBattPct) {
    lastDroneBattPct = novaX.drone.droneBatPct;
    zonesSetValue("drone_battery", lastDroneBattPct);
    zonesSetColor("drone_battery", (lastDroneBattPct > 50) ? GREEN : (lastDroneBattPct > 20) ? YELLOW : RED);
  }

  if(novaX.remote.remoteBatPct != lastRemoteBattPct) {
    lastRemoteBattPct = novaX.remote.remoteBatPct;
    zonesSetValue("remote_battery", lastRemoteBattPct);
    zonesSetColor("remote_battery", (lastRemoteBattPct > 50) ? GREEN : (lastRemoteBattPct > 20) ? YELLOW : RED);
  }

  zonesSetHorizon("horizon", novaX.drone.roll, novaX.drone.pitch);
}

void hudInit()
{
  lastConnState    = (ConnState)-1;
  lastFlightMode   = (FlightMode)-1;
  lastArmed        = !novaX.drone.isArmed;
  lastObstacle     = 0xFF;
  activeHUDScreen  = SCR_DISCONNECTED;
  showingConnInfo  = false;
  connectedSinceMs = 0;
  lastDroneBattPct  = 0xFF;
  lastRemoteBattPct = 0xFF;

  novaX.drone.connState = CONN_SEARCHING;
}

void hudLoop()
{
  handleConnectionFlow();

  if(novaX.drone.connState == CONN_CONNECTED && !showingConnInfo)
  {
    checkFlightModeChange();
    checkOverlays();
    drawArmIndicator();
    hudDrawTelemetry();
    zonesRenderDraws();
  }
}