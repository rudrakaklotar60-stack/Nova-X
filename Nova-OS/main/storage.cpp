#include "storage.h"
#include <Preferences.h>

static Preferences prefs;

// ═══════════════════════════════════════════
//  STORAGE INIT
// ═══════════════════════════════════════════
void storageInit()
{
  
  if(!loadAllSettings()) {
    Serial.println("Settings not found — using defaults");
    novaXDefaults();
    saveAllSettings();   // safe to write — NVS, not SD
  } else {
    Serial.println("Settings loaded OK");
  }
}

// ═══════════════════════════════════════════
//  PID
// ═══════════════════════════════════════════
bool savePID()
{
  prefs.begin("pid", false);

  prefs.putFloat("rollP", novaX.pid.roll.p);
  prefs.putFloat("rollI", novaX.pid.roll.i);
  prefs.putFloat("rollD", novaX.pid.roll.d);

  prefs.putFloat("pitchP", novaX.pid.pitch.p);
  prefs.putFloat("pitchI", novaX.pid.pitch.i);
  prefs.putFloat("pitchD", novaX.pid.pitch.d);

  prefs.putFloat("yawP", novaX.pid.yaw.p);
  prefs.putFloat("yawI", novaX.pid.yaw.i);
  prefs.putFloat("yawD", novaX.pid.yaw.d);

  prefs.putFloat("thrMin", novaX.pid.thrMin);
  prefs.putFloat("thrMax", novaX.pid.thrMax);

  prefs.end();
  Serial.println("PID saved");
  return true;
}

bool loadPID()
{
  prefs.begin("pid", true);

  if(!prefs.isKey("rollP")) { prefs.end(); return false; }

  novaX.pid.roll.p  = prefs.getFloat("rollP",  1.2f);
  novaX.pid.roll.i  = prefs.getFloat("rollI",  0.04f);
  novaX.pid.roll.d  = prefs.getFloat("rollD",  0.18f);

  novaX.pid.pitch.p = prefs.getFloat("pitchP", 1.2f);
  novaX.pid.pitch.i = prefs.getFloat("pitchI", 0.04f);
  novaX.pid.pitch.d = prefs.getFloat("pitchD", 0.18f);

  novaX.pid.yaw.p   = prefs.getFloat("yawP",   2.0f);
  novaX.pid.yaw.i   = prefs.getFloat("yawI",   0.02f);
  novaX.pid.yaw.d   = prefs.getFloat("yawD",   0.0f);

  novaX.pid.thrMin  = prefs.getFloat("thrMin", 100.0f);
  novaX.pid.thrMax  = prefs.getFloat("thrMax", 1000.0f);

  prefs.end();
  return true;
}

// ═══════════════════════════════════════════
//  JOYSTICK
// ═══════════════════════════════════════════
bool saveJoystick()
{
  prefs.begin("joy", false);

  prefs.putUChar("mode",          (uint8_t)novaX.joy.mode);
  prefs.putUChar("deadzone",      novaX.joy.deadzone);
  prefs.putUChar("throttleCurve", novaX.joy.throttleCurve);

  prefs.putUShort("j1xMin",    novaX.joy.joy1XMin);
  prefs.putUShort("j1xMax",    novaX.joy.joy1XMax);
  prefs.putUShort("j1xCenter", novaX.joy.joy1XCenter);
  prefs.putUShort("j1yMin",    novaX.joy.joy1YMin);
  prefs.putUShort("j1yMax",    novaX.joy.joy1YMax);
  prefs.putUShort("j1yCenter", novaX.joy.joy1YCenter);

  prefs.putUShort("j2xMin",    novaX.joy.joy2XMin);
  prefs.putUShort("j2xMax",    novaX.joy.joy2XMax);
  prefs.putUShort("j2xCenter", novaX.joy.joy2XCenter);
  prefs.putUShort("j2yMin",    novaX.joy.joy2YMin);
  prefs.putUShort("j2yMax",    novaX.joy.joy2YMax);
  prefs.putUShort("j2yCenter", novaX.joy.joy2YCenter);

  prefs.end();
  Serial.println("Joystick saved");
  return true;
}

bool loadJoystick()
{
  prefs.begin("joy", true);
  if(!prefs.isKey("mode")) { prefs.end(); return false; }

  novaX.joy.mode          = (JoyMode)prefs.getUChar("mode", 0);
  novaX.joy.deadzone      = prefs.getUChar("deadzone", 5);
  novaX.joy.throttleCurve = prefs.getUChar("throttleCurve", 0);

  // Joy1 X — not inverted
  novaX.joy.joy1XMin      = prefs.getUShort("j1xMin",    0);
  novaX.joy.joy1XMax      = prefs.getUShort("j1xMax",    4095);
  novaX.joy.joy1XCenter   = prefs.getUShort("j1xCenter", 1900);

  // Joy1 Y — raw value (inversion handled at read time in updateThrottle)
  novaX.joy.joy1YMin      = prefs.getUShort("j1yMin",    0);
  novaX.joy.joy1YMax      = prefs.getUShort("j1yMax",    4095);
  novaX.joy.joy1YCenter   = prefs.getUShort("j1yCenter", 1870);

  // Joy2 X — not inverted
  novaX.joy.joy2XMin      = prefs.getUShort("j2xMin",    0);
  novaX.joy.joy2XMax      = prefs.getUShort("j2xMax",    4095);
  novaX.joy.joy2XCenter   = prefs.getUShort("j2xCenter", 2000);

  // Joy2 Y — raw value (inversion handled at read time in updateJoysticks)
  novaX.joy.joy2YMin      = prefs.getUShort("j2yMin",    0);
  novaX.joy.joy2YMax      = prefs.getUShort("j2yMax",    4095);
  novaX.joy.joy2YCenter   = prefs.getUShort("j2yCenter", 1940);

  prefs.end();
  return true;
}
// ═══════════════════════════════════════════
//  DISPLAY
// ═══════════════════════════════════════════
bool saveDisplay()
{
  prefs.begin("disp", false);
  prefs.putUChar("brightness", novaX.disp.brightness);
  prefs.putUChar("timeout",    novaX.disp.timeoutSec);
  prefs.end();
  return true;
}

bool loadDisplay()
{
  prefs.begin("disp", true);
  if(!prefs.isKey("brightness")) { prefs.end(); return false; }

  novaX.disp.brightness = prefs.getUChar("brightness", 80);
  novaX.disp.timeoutSec = prefs.getUChar("timeout", 0);

  prefs.end();
  return true;
}

// ═══════════════════════════════════════════
//  RADIO
// ═══════════════════════════════════════════
bool saveRadio()
{
  prefs.begin("radio", false);
  prefs.putUChar("channel",  novaX.radio.channel);
  prefs.putUChar("txPower",  novaX.radio.txPower);
  prefs.putUChar("dataRate", novaX.radio.dataRate);
  prefs.end();
  return true;
}

bool loadRadio()
{
  prefs.begin("radio", true);
  if(!prefs.isKey("channel")) { prefs.end(); return false; }

  novaX.radio.channel  = prefs.getUChar("channel", 108);
  novaX.radio.txPower  = prefs.getUChar("txPower", 3);
  novaX.radio.dataRate = prefs.getUChar("dataRate", 0);

  prefs.end();
  return true;
}

// ═══════════════════════════════════════════
//  FLIGHT LIMITS
// ═══════════════════════════════════════════
bool saveLimits()
{
  prefs.begin("limits", false);
  prefs.putUShort("maxAlt",    novaX.limits.maxAltitudeCm);
  prefs.putUShort("maxSpeed",  novaX.limits.maxSpeedCmS);
  prefs.putUShort("retAlt",    novaX.limits.returnAltitudeCm);
  prefs.putUShort("brakeDist", novaX.limits.brakeCm);
  prefs.end();
  return true;
}

bool loadLimits()
{
  prefs.begin("limits", true);
  if(!prefs.isKey("maxAlt")) { prefs.end(); return false; }

  novaX.limits.maxAltitudeCm    = prefs.getUShort("maxAlt", 3000);
  novaX.limits.maxSpeedCmS      = prefs.getUShort("maxSpeed", 200);
  novaX.limits.returnAltitudeCm = prefs.getUShort("retAlt", 150);
  novaX.limits.brakeCm          = prefs.getUShort("brakeDist", 30);

  prefs.end();
  return true;
}

// ═══════════════════════════════════════════
//  LOAD ALL
// ═══════════════════════════════════════════
bool loadAllSettings()
{
  bool ok = true;
  ok &= loadPID();
  ok &= loadJoystick();
  ok &= loadDisplay();
  ok &= loadRadio();
  ok &= loadLimits();
  return ok;
}

// ═══════════════════════════════════════════
//  SAVE ALL
// ═══════════════════════════════════════════
bool saveAllSettings()
{
  bool ok = true;
  ok &= savePID();
  ok &= saveJoystick();
  ok &= saveDisplay();
  ok &= saveRadio();
  ok &= saveLimits();
  return ok;
}