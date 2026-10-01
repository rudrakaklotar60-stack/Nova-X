#pragma once
#include <Arduino.h>
#include "os.h"
uint8_t  batteryPercent(uint16_t mv);
uint16_t readBatteryVoltageMv();

enum FlightMode {
  MODE_DISARMED = 0,
  MODE_ARMED,
  MODE_ACTIVE_FLIGHT,
  MODE_POSITION_HOLD,
  MODE_AUTO_LANDING,
  MODE_EMERGENCY_STOP
};

enum ConnState {
  CONN_DISCONNECTED = 0,
  CONN_SEARCHING,
  CONN_CONNECTING,
  CONN_GETTING_INFO,
  CONN_CONNECTED,
  CONN_LOST,
  CONN_RECONNECTING
};

enum FailsafeState {
  FAIL_NONE = 0,
  FAIL_SIGNAL_LOST,
  FAIL_DRONE_BAT_LOW,
  FAIL_REMOTE_BAT_LOW,
  FAIL_SENSOR_ERROR,
  FAIL_MOTOR_CUTOFF
};

#define OBS_NONE   0b00000000
#define OBS_FRONT  0b00000001
#define OBS_BACK   0b00000010
#define OBS_LEFT   0b00000100
#define OBS_RIGHT  0b00001000
#define OBS_TOP    0b00010000

struct PIDValues {
  float p;
  float i;
  float d;
};

struct PIDConfig {
  PIDValues roll;
  PIDValues pitch;
  PIDValues yaw;
  float     thrMin;
  float     thrMax;
};

enum JoyMode {
  JOYMODE_1 = 0,
  JOYMODE_2,
  JOYMODE_3,
  JOYMODE_4
};

struct JoyConfig {
  JoyMode  mode;
  uint8_t  deadzone;
  uint8_t  throttleCurve;
  uint16_t joy1XMin, joy1XMax, joy1XCenter;
  uint16_t joy1YMin, joy1YMax, joy1YCenter;
  uint16_t joy2XMin, joy2XMax, joy2XCenter;
  uint16_t joy2YMin, joy2YMax, joy2YCenter;
};

struct DispConfig {
  uint8_t brightness;
  uint8_t timeoutSec;
};

struct RadioConfig {
  uint8_t channel;
  uint8_t txPower;
  uint8_t dataRate;
};

struct FlightLimits {
  uint16_t maxAltitudeCm;
  uint16_t maxSpeedCmS;
  uint16_t returnAltitudeCm;
  uint16_t brakeCm;
};

struct DroneData {
  ConnState    connState;
  uint8_t      rssi;
  FlightMode   flightMode;
  bool         isArmed;
  FailsafeState failsafe;
  int32_t      altitudeMm;
  int16_t      roll;
  int16_t      pitch;
  int16_t      yaw;
  uint16_t     tofFront;
  uint16_t     tofBack;
  uint16_t     tofLeft;
  uint16_t     tofRight;
  uint16_t     tofTop;
  uint8_t      droneBatPct;
  float        droneBatV;
  uint8_t      obstacleFlags;
  uint32_t     lastPacketMs;
  uint32_t     signalLostMs;
};

struct RemoteData {
  uint16_t joy1X, joy1Y;
  uint16_t joy2X, joy2Y;

  int16_t  throttle;
  int16_t  roll;
  int16_t  pitch;
  int16_t  yaw;

  uint8_t  remoteBatPct;
  float    remoteBatV;

  uint8_t  buttons;

  FlightMode commandedFlightMode;   // ← NEW: local intent only — never touched by telemetry
};

struct NovaXState {
  DroneData    drone;
  RemoteData   remote;
  PIDConfig    pid;
  JoyConfig    joy;
  DispConfig   disp;
  RadioConfig  radio;
  FlightLimits limits;
};

extern NovaXState novaX;

inline uint8_t batVtoPercent(float v) {
  if(v >= 4.20f) return 100;
  if(v <= 3.30f) return 0;
  return (uint8_t)((v - 3.30f) / (4.20f - 3.30f) * 100.0f);
}

ScreenID obstacleToScreen(uint8_t flags);
void novaXDefaults();