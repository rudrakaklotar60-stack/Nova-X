#include <Wire.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

const int PIN_TL = 5;
const int PIN_BL = 6;
const int PIN_BR = 9;
const int PIN_TR = 10;

#define PM1_RX_PIN 2
#define PM1_TX_PIN 3
SoftwareSerial pm1Link(PM1_RX_PIN, PM1_TX_PIN);

#define UART_FRAME_SIZE 13
uint8_t frameBuf[UART_FRAME_SIZE];
uint8_t frameIdx = 0;

uint32_t lastUartMs = 0;
#define UART_TIMEOUT_MS 300
bool armed = false;
bool g_calibrateRequested = false;
uint8_t g_avoidFlags = 0;

#define TELEM_FRAME_SIZE 7
uint32_t lastTelemMs = 0;

const int MPU_ADDR = 0x68;
float gyroX, gyroY, gyroZ;
int16_t accX_raw, accY_raw, accZ_raw;

// Calibrated via diagnostic sketch — unchanged since last calibration
float gyroX_offset = -137.67;
float gyroY_offset = 146.83;
float gyroZ_offset = -78.59;

float accelXOffset = -17;
float accelYOffset = -307;

// ── Accelerometer low-pass filter — tames vibration noise before it reaches angle calc ──
#define ACCEL_LPF_ALPHA 0.98f
static float filtAccX = 0, filtAccY = 0, filtAccZ = 0;
static bool accFilterInit = false;
bool obstacleAvoidanceEnabled = false;

static float angleRoll = 0, anglePitch = 0;
#define COMP_ALPHA 0.98f

uint32_t i2cFailCount = 0;
uint32_t lastFailPrintMs = 0;

#define MAX_TILT_ANGLE     10.0
#define ANGLE_P_GAIN        3.0
#define AVOID_RETREAT_ANGLE 3.0
float pitchTrimDeg = 4.2;
float rollTrimDeg  = -1.5;

float pid_p_gain_roll = 3, pid_i_gain_roll = 0.015, pid_d_gain_roll = 15.0;
float pid_p_gain_pitch = pid_p_gain_roll, pid_i_gain_pitch = pid_i_gain_roll, pid_d_gain_pitch = pid_d_gain_roll;
float pid_p_gain_yaw = 15.0, pid_i_gain_yaw = 0.1, pid_d_gain_yaw = 10.0;

float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;

// ── D-term filtering — smooths just the derivative term, roll/pitch only ──
static float dTermRollFiltered = 0, dTermPitchFiltered = 0;

int receiver_throttle = 0;
int receiver_roll  = 1500;
int receiver_pitch = 1500;
int receiver_yaw   = 1500;

unsigned long loop_timer;

#define CONFIG_FRAME_SIZE 47
uint8_t configBuf[CONFIG_FRAME_SIZE];
uint8_t configIdx = 0;
bool inConfigFrame = false;

void applyConfigFrame()
{
  if(armed) {
    Serial.println(F("Config update ignored — drone is armed"));
    return;
  }
  memcpy(&pid_p_gain_roll,  &configBuf[1],  4);
  memcpy(&pid_i_gain_roll,  &configBuf[5],  4);
  memcpy(&pid_d_gain_roll,  &configBuf[9],  4);
  memcpy(&pid_p_gain_pitch, &configBuf[13], 4);
  memcpy(&pid_i_gain_pitch, &configBuf[17], 4);
  memcpy(&pid_d_gain_pitch, &configBuf[21], 4);
  memcpy(&pid_p_gain_yaw,   &configBuf[25], 4);
  memcpy(&pid_i_gain_yaw,   &configBuf[29], 4);
  memcpy(&pid_d_gain_yaw,   &configBuf[33], 4);
  memcpy(&pitchTrimDeg,     &configBuf[37], 4);
  memcpy(&rollTrimDeg,      &configBuf[41], 4);
  saveConfigToEEPROM();
  Serial.println(F("Config updated from remote + saved to EEPROM"));
}

uint8_t calcChecksum(uint8_t* data, uint8_t from, uint8_t to)
{
  uint8_t cs = 0;
  for(uint8_t i = from; i <= to; i++) cs ^= data[i];
  return cs;
}
#define CAL_SAMPLE_COUNT 150

void runGyroCalibration()
{
  Serial.println(F("Calibrating — keep the drone still and level..."));

  long sumAX = 0, sumAY = 0, sumAZ = 0;
  long sumGX = 0, sumGY = 0, sumGZ = 0;
  uint16_t got = 0;

  for(uint16_t i = 0; i < CAL_SAMPLE_COUNT; i++) {
    if(readMPU()) {
      sumAX += accX_raw; sumAY += accY_raw; sumAZ += accZ_raw;
      sumGX += gyroX;    sumGY += gyroY;    sumGZ += gyroZ;
      got++;
    }
    delay(20);
  }

  if(got < CAL_SAMPLE_COUNT / 2) {
    Serial.println(F("Calibration failed — too many I2C read errors."));
    return;
  }

  gyroX_offset = sumGX / (float)got;
  gyroY_offset = sumGY / (float)got;
  gyroZ_offset = sumGZ / (float)got;
  accelXOffset = sumAX / (float)got;
  accelYOffset = sumAY / (float)got;

  angleRoll  = 0;
  anglePitch = 0;
  accFilterInit = false;

  Serial.println(F("Calibration done. New offsets:"));
  Serial.print(F("gyroX_offset=")); Serial.println(gyroX_offset);
  Serial.print(F("gyroY_offset=")); Serial.println(gyroY_offset);
  Serial.print(F("gyroZ_offset=")); Serial.println(gyroZ_offset);
  Serial.print(F("accelXOffset=")); Serial.println(accelXOffset);
  Serial.print(F("accelYOffset=")); Serial.println(accelYOffset);
}

#define EEPROM_MAGIC 0xA6

struct PersistedConfig {
  uint8_t magic;
  float   pP_roll, pI_roll, pD_roll;
  float   pP_pitch, pI_pitch, pD_pitch;
  float   pP_yaw, pI_yaw, pD_yaw;
  float   pitchTrim, rollTrim;
};

void saveConfigToEEPROM()
{
  PersistedConfig cfg;
  cfg.magic     = EEPROM_MAGIC;
  cfg.pP_roll   = pid_p_gain_roll;   cfg.pI_roll  = pid_i_gain_roll;   cfg.pD_roll  = pid_d_gain_roll;
  cfg.pP_pitch  = pid_p_gain_pitch;  cfg.pI_pitch = pid_i_gain_pitch;  cfg.pD_pitch = pid_d_gain_pitch;
  cfg.pP_yaw    = pid_p_gain_yaw;    cfg.pI_yaw   = pid_i_gain_yaw;    cfg.pD_yaw   = pid_d_gain_yaw;
  cfg.pitchTrim = pitchTrimDeg;      cfg.rollTrim = rollTrimDeg;
  EEPROM.put(0, cfg);
}

void loadConfigFromEEPROM()
{
  PersistedConfig cfg;
  EEPROM.get(0, cfg);
  if(cfg.magic != EEPROM_MAGIC) {
    Serial.println(F("No saved config — using defaults"));
    return;
  }
  pid_p_gain_roll = cfg.pP_roll;   pid_i_gain_roll = cfg.pI_roll;   pid_d_gain_roll = cfg.pD_roll;
  pid_p_gain_pitch = cfg.pP_pitch; pid_i_gain_pitch = cfg.pI_pitch; pid_d_gain_pitch = cfg.pD_pitch;
  pid_p_gain_yaw = cfg.pP_yaw;     pid_i_gain_yaw = cfg.pI_yaw;     pid_d_gain_yaw = cfg.pD_yaw;
  pitchTrimDeg = cfg.pitchTrim;    rollTrimDeg = cfg.rollTrim;
  Serial.println(F("Loaded saved PID/trim config from EEPROM"));
}

void readUartFromPM1()
{
  while(pm1Link.available())
  {
    uint8_t b = pm1Link.read();

    if(inConfigFrame) {
      configBuf[configIdx++] = b;
      if(configIdx >= CONFIG_FRAME_SIZE) {
        inConfigFrame = false;
        configIdx = 0;
        if(configBuf[CONFIG_FRAME_SIZE - 1] == 0xFD) {
          uint8_t cs = calcChecksum(configBuf, 1, CONFIG_FRAME_SIZE - 3);
          if(cs == configBuf[CONFIG_FRAME_SIZE - 2]) applyConfigFrame();
        }
      }
      continue;
    }

    if(frameIdx == 0) {
      if(b == 0xFC) { inConfigFrame = true; configIdx = 0; configBuf[configIdx++] = b; continue; }
      if(b != 0xFF) continue;
    }

    frameBuf[frameIdx++] = b;

    if(frameIdx >= UART_FRAME_SIZE) {
      frameIdx = 0;
      if(frameBuf[12] != 0xFE) continue;
      uint8_t cs = calcChecksum(frameBuf, 1, 10);
      if(cs != frameBuf[11]) continue;

      int16_t thr   = (int16_t)((frameBuf[1] << 8) | frameBuf[2]);
      int16_t roll  = (int16_t)((frameBuf[3] << 8) | frameBuf[4]);
      int16_t pitch = (int16_t)((frameBuf[5] << 8) | frameBuf[6]);
      int16_t yaw   = (int16_t)((frameBuf[7] << 8) | frameBuf[8]);
      bool    isArmed = (frameBuf[9] & 0x01) != 0;
      bool    calRequest = (frameBuf[9] & 0x02) != 0;
      g_avoidFlags = frameBuf[10];

      receiver_throttle = map(constrain(thr, 0, 1000), 0, 1000, 0, 255);
      receiver_roll  = 1500 + constrain(roll,  -500, 500);
      receiver_pitch = 1500 + constrain(pitch, -500, 500);
      receiver_yaw   = 1500 + constrain(yaw,   -500, 500);
      armed = isArmed;
      g_calibrateRequested = calRequest;
      lastUartMs = millis();
    }
  }
}

// ── DRIFT BRAKE — opposes horizontal acceleration when roll/pitch stick is centered ──
#define ACCEL_BRAKE_GAIN  0.002f   // START LOW — see tuning notes below
#define ACCEL_BRAKE_MAX   5.0f     // degrees — safety clamp, not the full ±10° range

float computeDriftBrakeRoll()
{
  float accYRel = filtAccY - accelYOffset;         // ≈0 when level & not accelerating
  return constrain(-ACCEL_BRAKE_GAIN * accYRel, -ACCEL_BRAKE_MAX, ACCEL_BRAKE_MAX);
}

float computeDriftBrakePitch()
{
  float accXRel = filtAccX - accelXOffset;
  return constrain(ACCEL_BRAKE_GAIN * accXRel, -ACCEL_BRAKE_MAX, ACCEL_BRAKE_MAX);  // sign matches accPitch's existing negation
}

bool readMPU()
{
  for(uint8_t attempt = 0; attempt < 3; attempt++)
  {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    if(Wire.endTransmission(false) != 0) continue;

    uint8_t got = Wire.requestFrom(MPU_ADDR, 14, true);
    if(got < 14) continue;

    accX_raw = (Wire.read() << 8) | Wire.read();
    accY_raw = (Wire.read() << 8) | Wire.read();
    accZ_raw = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read();
    gyroX    = (Wire.read() << 8) | Wire.read();
    gyroY    = (Wire.read() << 8) | Wire.read();
    gyroZ    = (Wire.read() << 8) | Wire.read();
    return true;
  }
  return false;
}

void updateAttitudeEstimate()
{
  if(!accFilterInit) {
    filtAccX = accX_raw; filtAccY = accY_raw; filtAccZ = accZ_raw;
    accFilterInit = true;
  }
  filtAccX = ACCEL_LPF_ALPHA * filtAccX + (1.0f - ACCEL_LPF_ALPHA) * accX_raw;
  filtAccY = ACCEL_LPF_ALPHA * filtAccY + (1.0f - ACCEL_LPF_ALPHA) * accY_raw;
  filtAccZ = ACCEL_LPF_ALPHA * filtAccZ + (1.0f - ACCEL_LPF_ALPHA) * accZ_raw;

  float accMagnitude = sqrt(filtAccX*filtAccX + filtAccY*filtAccY + filtAccZ*filtAccZ);
  const float dt = 0.004;
  bool accelTrustworthy = (accMagnitude > 14000 && accMagnitude < 19000);

  if(accelTrustworthy) {
    float accRoll  = atan2((filtAccY - accelYOffset), filtAccZ) * 180.0 / PI;
    float accPitch = atan2(-(filtAccX - accelXOffset), sqrt(filtAccY*filtAccY + filtAccZ*filtAccZ)) * 180.0 / PI;

    angleRoll  = COMP_ALPHA * (angleRoll  + gyro_roll_input  * dt) + (1.0f - COMP_ALPHA) * accRoll;
    anglePitch = COMP_ALPHA * (anglePitch + gyro_pitch_input * dt) + (1.0f - COMP_ALPHA) * accPitch;
  } else {
    angleRoll  += gyro_roll_input  * dt;
    anglePitch += gyro_pitch_input * dt;
  }

  angleRoll  = constrain(angleRoll,  -60.0f, 60.0f);
  anglePitch = constrain(anglePitch, -60.0f, 60.0f);
}

void sendTelemetryToPM1()
{
  if(millis() - lastTelemMs < 100) return;
  lastTelemMs = millis();

  int16_t rollTx  = (int16_t)(angleRoll  * 10);
  int16_t pitchTx = (int16_t)(anglePitch * 10);

  uint8_t frame[TELEM_FRAME_SIZE];
  frame[0] = 0xFA;
  frame[1] = (rollTx >> 8) & 0xFF;   frame[2] = rollTx & 0xFF;
  frame[3] = (pitchTx >> 8) & 0xFF;  frame[4] = pitchTx & 0xFF;
  frame[5] = calcChecksum(frame, 1, 4);
  frame[6] = 0xFB;

  pm1Link.write(frame, TELEM_FRAME_SIZE);
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("PM2 - PID/Motor board, UART control mode"));

  pm1Link.begin(9600);

  pinMode(PIN_TL, OUTPUT);
  pinMode(PIN_TR, OUTPUT);
  pinMode(PIN_BR, OUTPUT);
  pinMode(PIN_BL, OUTPUT);
  analogWrite(PIN_TL, 0);
  analogWrite(PIN_TR, 0);
  analogWrite(PIN_BR, 0);
  analogWrite(PIN_BL, 0);

  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00);   // wake up
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); Wire.write(0x08);   // gyro range ±500°/s
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);  Wire.write(0x04);  // ← NEW — internal DLPF, ~44Hz bandwidth
  Wire.endTransmission(true);
  loadConfigFromEEPROM();
  Serial.println(F("Waiting for PM1 commands..."));
  delay(1000);
  loop_timer = micros();

  loadConfigFromEEPROM();
  Serial.print(F("Loaded: P_roll=")); Serial.print(pid_p_gain_roll);
  Serial.print(F(" I_roll="));  Serial.print(pid_i_gain_roll);
  Serial.print(F(" D_roll="));  Serial.print(pid_d_gain_roll);
  Serial.print(F(" P_pitch=")); Serial.print(pid_p_gain_pitch);
  Serial.print(F(" I_pitch=")); Serial.print(pid_i_gain_pitch);
  Serial.print(F(" D_pitch=")); Serial.print(pid_d_gain_pitch);
  Serial.print(F(" P_yaw="));   Serial.print(pid_p_gain_yaw);
  Serial.print(F(" I_yaw="));   Serial.print(pid_i_gain_yaw);
  Serial.print(F(" D_yaw="));   Serial.print(pid_d_gain_yaw);
  Serial.print(F(" pitchTrim=")); Serial.print(pitchTrimDeg);
  Serial.print(F(" rollTrim="));  Serial.println(rollTrimDeg);
}

void loop() {
  readUartFromPM1();

  static bool calReqLast = false;
  if(g_calibrateRequested && !calReqLast && !armed) {
    runGyroCalibration();
  }
  calReqLast = g_calibrateRequested;

  if(millis() - lastUartMs > UART_TIMEOUT_MS) {
    armed = false;
    receiver_throttle = 0;
  }

  bool mpuOk = readMPU();

  if(mpuOk) {
    gyro_roll_input  = (gyroX - gyroX_offset) / 65.5;
    gyro_pitch_input = (gyroY - gyroY_offset) / 65.5;
    gyro_yaw_input   = -1.0 * ((gyroZ - gyroZ_offset) / 65.5);
    updateAttitudeEstimate();
  } else {
    i2cFailCount++;
  }

  if(millis() - lastFailPrintMs > 1000) {
    lastFailPrintMs = millis();
    Serial.print(F("I2C fails/sec=")); Serial.println(i2cFailCount);
    i2cFailCount = 0;
  }

  float rollAngleTarget = 0;
  if (receiver_roll > 1508) rollAngleTarget = ((receiver_roll - 1508) / 500.0) * MAX_TILT_ANGLE;
  else if (receiver_roll < 1492) rollAngleTarget = ((receiver_roll - 1492) / 500.0) * MAX_TILT_ANGLE;

  float pitchAngleTarget = 0;
  if (receiver_pitch > 1508) pitchAngleTarget = ((receiver_pitch - 1508) / 500.0) * MAX_TILT_ANGLE;
  else if (receiver_pitch < 1492) pitchAngleTarget = ((receiver_pitch - 1492) / 500.0) * MAX_TILT_ANGLE;

  if (rollAngleTarget == 0)  rollAngleTarget  += computeDriftBrakeRoll();
  if (pitchAngleTarget == 0) pitchAngleTarget += computeDriftBrakePitch();

  pitchAngleTarget += pitchTrimDeg;
  rollAngleTarget  += rollTrimDeg;

  // ── OBSTACLE AVOIDANCE ──
  if(obstacleAvoidanceEnabled){
    bool obsFront = (g_avoidFlags & 0b0001);
    bool obsBack  = (g_avoidFlags & 0b0010);
    bool obsLeft  = (g_avoidFlags & 0b0100);
    bool obsRight = (g_avoidFlags & 0b1000);

    if(obsFront && obsBack) {
      pitchAngleTarget = 0;
    } else if(obsFront) {
      pitchAngleTarget = -AVOID_RETREAT_ANGLE;
    } else if(obsBack) {
      pitchAngleTarget = AVOID_RETREAT_ANGLE;
    }

    if(obsLeft && obsRight) {
      rollAngleTarget = 0;
    } else if(obsRight) {
      rollAngleTarget = -AVOID_RETREAT_ANGLE;
    } else if(obsLeft) {
      rollAngleTarget = AVOID_RETREAT_ANGLE;
    }
  }

  pid_roll_setpoint  = (rollAngleTarget  - angleRoll)  * ANGLE_P_GAIN;
  pid_pitch_setpoint = (pitchAngleTarget - anglePitch) * ANGLE_P_GAIN;

  pid_yaw_setpoint = 0;
  if (receiver_throttle > 10) {
    if (receiver_yaw > 1508) pid_yaw_setpoint = (receiver_yaw - 1508) / 3.0;
    else if (receiver_yaw < 1492) pid_yaw_setpoint = (receiver_yaw - 1492) / 3.0;
  }

  calculate_pid();

  int esc_TL_pwm = 0, esc_TR_pwm = 0, esc_BR_pwm = 0, esc_BL_pwm = 0;

  if (armed && receiver_throttle > 10) {
    esc_TL_pwm = receiver_throttle + pid_output_pitch - pid_output_roll + pid_output_yaw;
    esc_TR_pwm = receiver_throttle + pid_output_pitch + pid_output_roll - pid_output_yaw;
    esc_BR_pwm = receiver_throttle - pid_output_pitch + pid_output_roll + pid_output_yaw;
    esc_BL_pwm = receiver_throttle - pid_output_pitch - pid_output_roll - pid_output_yaw;

    esc_TL_pwm = constrain(esc_TL_pwm, 0, 255);
    esc_TR_pwm = constrain(esc_TR_pwm, 0, 255);
    esc_BR_pwm = constrain(esc_BR_pwm, 0, 255);
    esc_BL_pwm = constrain(esc_BL_pwm, 0, 255);
  }

  analogWrite(PIN_TL, esc_TL_pwm);
  analogWrite(PIN_TR, esc_TR_pwm);
  analogWrite(PIN_BR, esc_BR_pwm);
  analogWrite(PIN_BL, esc_BL_pwm);

  sendTelemetryToPM1();
  while(micros() - loop_timer < 4000);
  loop_timer = micros();
}

void calculate_pid() {
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  pid_i_mem_roll = constrain(pid_i_mem_roll, -100, 100);
  float dTermRollRaw = pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  dTermRollFiltered = 0.7f * dTermRollFiltered + 0.3f * dTermRollRaw;
  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + dTermRollFiltered;
  pid_output_roll = constrain(pid_output_roll, -100, 100);
  pid_last_roll_d_error = pid_error_temp;

  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  pid_i_mem_pitch = constrain(pid_i_mem_pitch, -100, 100);
  float dTermPitchRaw = pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  dTermPitchFiltered = 0.7f * dTermPitchFiltered + 0.3f * dTermPitchRaw;
  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + dTermPitchFiltered;
  pid_output_pitch = constrain(pid_output_pitch, -100, 100);
  pid_last_pitch_d_error = pid_error_temp;

  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  pid_i_mem_yaw = constrain(pid_i_mem_yaw, -100, 100);
  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  pid_output_yaw = constrain(pid_output_yaw, -100, 100);
  pid_last_yaw_d_error = pid_error_temp;
}