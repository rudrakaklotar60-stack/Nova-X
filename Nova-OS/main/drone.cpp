#include "drone.h"
#include "os.h"

NovaXState novaX;

void novaXDefaults()
{
  novaX.drone.connState    = CONN_DISCONNECTED;
  novaX.drone.rssi         = 0;
  novaX.drone.flightMode   = MODE_DISARMED;
  novaX.drone.isArmed      = false;
  novaX.drone.failsafe     = FAIL_NONE;
  novaX.drone.altitudeMm   = 0;
  novaX.drone.roll         = 0;
  novaX.drone.pitch        = 0;
  novaX.drone.yaw          = 0;
  novaX.drone.droneBatPct  = 0;
  novaX.drone.droneBatV    = 0.0f;
  novaX.drone.obstacleFlags = OBS_NONE;
  novaX.drone.lastPacketMs = 0;
  novaX.drone.signalLostMs = 0;
  novaX.drone.tofFront     = 9999;
  novaX.drone.tofBack      = 9999;
  novaX.drone.tofLeft      = 9999;
  novaX.drone.tofRight     = 9999;
  novaX.drone.tofTop       = 9999;

  novaX.remote.throttle    = 0;
  novaX.remote.roll        = 0;
  novaX.remote.pitch       = 0;
  novaX.remote.yaw         = 0;
  novaX.remote.remoteBatPct = 0;
  novaX.remote.remoteBatV  = 0.0f;
  novaX.remote.buttons     = 0;
  novaX.remote.commandedFlightMode = MODE_DISARMED;   // ← NEW

  novaX.pid.roll.p  = 1.2f;
  novaX.pid.roll.i  = 0.04f;
  novaX.pid.roll.d  = 0.18f;
  novaX.pid.pitch.p = 1.2f;
  novaX.pid.pitch.i = 0.04f;
  novaX.pid.pitch.d = 0.18f;
  novaX.pid.yaw.p   = 2.0f;
  novaX.pid.yaw.i   = 0.02f;
  novaX.pid.yaw.d   = 0.0f;
  novaX.pid.thrMin  = 100.0f;
  novaX.pid.thrMax  = 1000.0f;

  novaX.joy.mode           = JOYMODE_2;
  novaX.joy.deadzone       = 5;
  novaX.joy.throttleCurve  = 0;
  novaX.joy.joy1XMin       = 1200;
  novaX.joy.joy1XMax       = 2200;
  novaX.joy.joy1XCenter    = 1700;
  novaX.joy.joy1YMin       = 1200;
  novaX.joy.joy1YMax       = 2200;
  novaX.joy.joy1YCenter    = 1700;
  novaX.joy.joy2XMin       = 1200;
  novaX.joy.joy2XMax       = 2200;
  novaX.joy.joy2XCenter    = 1700;
  novaX.joy.joy2YMin       = 1200;
  novaX.joy.joy2YMax       = 2200;
  novaX.joy.joy2YCenter    = 1700;

  novaX.disp.brightness  = 80;
  novaX.disp.timeoutSec  = 0;

  novaX.radio.channel  = 108;
  novaX.radio.txPower  = 3;
  novaX.radio.dataRate = 0;

  novaX.limits.maxAltitudeCm   = 3000;
  novaX.limits.maxSpeedCmS     = 200;
  novaX.limits.returnAltitudeCm = 150;
  novaX.limits.brakeCm         = 30;
}

ScreenID obstacleToScreen(uint8_t flags)
{
  if(flags == OBS_NONE)                          return osState.current;
  if(flags == OBS_FRONT)                         return SCR_OBS_FRONT;
  if(flags == OBS_BACK)                          return SCR_OBS_BACK;
  if(flags == OBS_LEFT)                          return SCR_OBS_LEFT;
  if(flags == OBS_RIGHT)                         return SCR_OBS_RIGHT;
  if(flags == OBS_TOP)                           return SCR_OBS_TOP;
  if(flags == (OBS_FRONT | OBS_LEFT))            return SCR_OBS_FRONT_LEFT;
  if(flags == (OBS_FRONT | OBS_RIGHT))           return SCR_OBS_FRONT_RIGHT;
  if(flags == (OBS_BACK  | OBS_LEFT))            return SCR_OBS_BACK_LEFT;
  if(flags == (OBS_BACK  | OBS_RIGHT))           return SCR_OBS_BACK_RIGHT;
  if(flags == (OBS_FRONT | OBS_BACK))            return SCR_OBS_FRONT_BACK;
  if(flags == (OBS_LEFT  | OBS_RIGHT))           return SCR_OBS_LEFT_RIGHT;
  if(flags == (OBS_FRONT | OBS_LEFT | OBS_RIGHT))return SCR_OBS_FRONT_LEFT_RIGHT;
  if(flags == (OBS_BACK  | OBS_LEFT | OBS_RIGHT))return SCR_OBS_BACK_LEFT_RIGHT;
  if(flags == (OBS_FRONT | OBS_BACK | OBS_LEFT)) return SCR_OBS_FRONT_BACK_LEFT;
  return SCR_OBS_ALL;
}

struct BatteryPoint { uint16_t mv; uint8_t pct; };

static const BatteryPoint battCurve[] = {
  {3300,   0}, {3500,   5}, {3600,  10}, {3650,  15}, {3700,  25},
  {3740,  35}, {3770,  45}, {3800,  55}, {3840,  65}, {3880,  75},
  {3920,  83}, {3960,  88}, {4000,  92}, {4050,  95}, {4100,  97},
  {4150,  99}, {4170, 100}
};
#define BATT_CURVE_LEN (sizeof(battCurve) / sizeof(BatteryPoint))

uint8_t batteryPercent(uint16_t mv)
{
  if(mv <= battCurve[0].mv) return 0;
  if(mv >= battCurve[BATT_CURVE_LEN-1].mv) return 100;

  for(uint8_t i = 0; i < BATT_CURVE_LEN - 1; i++)
  {
    uint16_t v0 = battCurve[i].mv;
    uint16_t v1 = battCurve[i+1].mv;
    if(mv >= v0 && mv <= v1)
    {
      uint8_t p0 = battCurve[i].pct;
      uint8_t p1 = battCurve[i+1].pct;
      float t = (float)(mv - v0) / (float)(v1 - v0);
      return p0 + (uint8_t)(t * (p1 - p0));
    }
  }
  return 0;
}

struct CalPoint { uint16_t raw; uint16_t actual; };

static const CalPoint calTable[] = {
  {1765, 3900}, {1780, 3930}, {1820, 3940}, {1840, 3950}, {1850, 3970},
  {1860, 3980}, {1870, 4020}, {1885, 4040}, {1900, 4060}, {1910, 4090},
  {1920, 4120}, {1930, 4170},
};
#define CAL_TABLE_LEN (sizeof(calTable) / sizeof(CalPoint))

static uint16_t calibrateBatteryMv(uint16_t rawMv)
{
  if(rawMv <= calTable[0].raw) {
    float slope = (float)(calTable[1].actual - calTable[0].actual) /
                  (float)(calTable[1].raw    - calTable[0].raw);
    int32_t result = calTable[0].actual - (int32_t)((calTable[0].raw - rawMv) * slope);
    return (uint16_t)max((int32_t)0, result);
  }

  if(rawMv >= calTable[CAL_TABLE_LEN-1].raw)
  {
    float slope = (float)(calTable[CAL_TABLE_LEN-1].actual - calTable[CAL_TABLE_LEN-2].actual) /
                  (float)(calTable[CAL_TABLE_LEN-1].raw    - calTable[CAL_TABLE_LEN-2].raw);
    return calTable[CAL_TABLE_LEN-1].actual + (uint16_t)((rawMv - calTable[CAL_TABLE_LEN-1].raw) * slope);
  }

  for(uint8_t i = 0; i < CAL_TABLE_LEN - 1; i++)
  {
    uint16_t r0 = calTable[i].raw,    r1 = calTable[i+1].raw;
    uint16_t a0 = calTable[i].actual, a1 = calTable[i+1].actual;
    if(rawMv >= r0 && rawMv <= r1)
    {
      float t = (float)(rawMv - r0) / (float)(r1 - r0);
      return a0 + (uint16_t)(t * (a1 - a0));
    }
  }
  return rawMv * 2;
}

#define BATT_ADC_PIN  36
#define ADC_SAMPLES   8

uint16_t readBatteryVoltageMv()
{
  uint32_t sum = 0;
  for(uint8_t i = 0; i < ADC_SAMPLES; i++) {
    sum += analogRead(BATT_ADC_PIN);
    delayMicroseconds(100);
  }
  uint16_t raw = sum / ADC_SAMPLES;
  float adcVoltage = (raw / 4095.0) * 3300.0;
  return calibrateBatteryMv((uint16_t)adcVoltage);
}