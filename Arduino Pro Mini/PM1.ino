#include <SPI.h>
#include <RF24.h>
#include <SoftwareSerial.h>
#include <VL53L0X.h>
#include <avr/wdt.h>

// ═══════════════════════════════════════════
//  NRF24
// ═══════════════════════════════════════════
#define NRF_CE   9
#define NRF_CSN  10
RF24 radio(NRF_CE, NRF_CSN);

#define PACKET_SIZE  32
#define DATA_BYTES   28

const uint8_t RECV_PIPE[6] = "NovaX";
const uint8_t SEND_PIPE[6] = "evaXN";

uint8_t rxPacket[PACKET_SIZE];
uint8_t txPacket[PACKET_SIZE];
uint8_t replyCounter = 0;

#define PKT_REMOTE_TO_DRONE 0x01
#define PKT_DRONE_TO_REMOTE 0x02
#define FLAG_ARMED 0x01
#define PKT_CONFIG_UPDATE 0x03

#define RADIO_STUCK_MS 3000

// ═══════════════════════════════════════════
//  UART LINK TO PM2
// ═══════════════════════════════════════════
#define PM2_RX_PIN 2
#define PM2_TX_PIN 3
SoftwareSerial pm2Link(PM2_RX_PIN, PM2_TX_PIN);

#define UART_FRAME_SIZE 13
uint8_t uartFrame[UART_FRAME_SIZE];

#define TELEM_FRAME_SIZE 7
uint8_t telemBuf[TELEM_FRAME_SIZE];
uint8_t telemIdx = 0;
int16_t g_droneRoll = 0, g_dronePitch = 0;

int16_t g_throttle = 0, g_roll = 0, g_pitch = 0, g_yaw = 0;
uint8_t g_flightMode = 0;
uint8_t g_buttons = 0;

uint32_t lastGoodPacketMs = 0;
#define SIGNAL_LOST_MS 500

static uint8_t lastRssiPct = 100;

// ═══════════════════════════════════════════
//  BATTERY
// ═══════════════════════════════════════════
#define BATT_ADC_PIN A0
#define BATT_SAMPLES 8
uint8_t droneBatPct = 0;

// ═══════════════════════════════════════════
//  LEDS — pin 5 (PWM), freed by moving XSHUT_BACK to A1
// ═══════════════════════════════════════════
#define LED_PIN 5

enum LedMode { LED_OFF = 0, LED_ON, LED_BLINK, LED_FADE, LED_STROBE, LED_BREATHE, LED_HEARTBEAT };
uint8_t g_ledMode = LED_OFF;

uint8_t ledBrightnessPresets[7] = {20, 60, 100, 140, 180, 220, 255};   // LOW/MED/HIGH — 200/255 ≈ 78%, never full 100%
uint8_t g_ledBrightnessIdx = 1;

#define LED_BLINK_PERIOD_MS 500
#define LED_FADE_PERIOD_MS  2000

#define PCF_BTN_LED_MODE   0b01000000
#define PCF_BTN_LED_BRIGHT 0b10000000
#define PCF_BTN_CALIBRATE  0b00001000 

void updateLED()
{
  uint8_t peak = ledBrightnessPresets[g_ledBrightnessIdx];

  switch(g_ledMode)
  {
    case LED_OFF:
      analogWrite(LED_PIN, 0);
      break;

    case LED_ON:
      analogWrite(LED_PIN, peak);
      break;

    case LED_BLINK: {
      bool on = (millis() % LED_BLINK_PERIOD_MS) < (LED_BLINK_PERIOD_MS / 2);
      analogWrite(LED_PIN, on ? peak : 0);
      break;
    }

    case LED_FADE: {
      float t = (float)(millis() % LED_FADE_PERIOD_MS) / LED_FADE_PERIOD_MS;
      float level = (t < 0.5) ? (t * 2.0) : (2.0 - t * 2.0);
      analogWrite(LED_PIN, (uint8_t)(level * peak));
      break;
    }
    case LED_STROBE: {
      uint16_t t = millis() % 1000;
      bool on = (t < 30) || (t > 100 && t < 130);   // two 30ms flashes, then dark until next cycle
      analogWrite(LED_PIN, on ? peak : 0);
      break;
  }
    case LED_BREATHE: {
      float t = (millis() % LED_FADE_PERIOD_MS) / (float)LED_FADE_PERIOD_MS;
      float level = (sin(t * 2 * PI - PI/2) + 1.0) / 2.0;   // smooth 0→1→0, sine-shaped
      analogWrite(LED_PIN, (uint8_t)(level * peak));
      break;
  }
    case LED_HEARTBEAT: {
      uint16_t t = millis() % 1200;
      uint8_t level = 0;
      if(t < 150)      level = (uint8_t)((sin((t/150.0)*PI)) * peak);
      else if(t >= 250 && t < 400) level = (uint8_t)((sin(((t-250)/150.0)*PI)) * peak);
      analogWrite(LED_PIN, level);
      break;
  }

  }
}
void forwardConfigToPM2()
{
  float pP_roll  = ((int16_t)((rxPacket[2]  << 8) | rxPacket[3]))  / 1000.0;
  float pI_roll  = ((int16_t)((rxPacket[4]  << 8) | rxPacket[5]))  / 1000.0;
  float pD_roll  = ((int16_t)((rxPacket[6]  << 8) | rxPacket[7]))  / 1000.0;
  float pP_pitch = ((int16_t)((rxPacket[8]  << 8) | rxPacket[9]))  / 1000.0;
  float pI_pitch = ((int16_t)((rxPacket[10] << 8) | rxPacket[11])) / 1000.0;
  float pD_pitch = ((int16_t)((rxPacket[12] << 8) | rxPacket[13])) / 1000.0;
  float pP_yaw   = ((int16_t)((rxPacket[14] << 8) | rxPacket[15])) / 1000.0;
  float pI_yaw   = ((int16_t)((rxPacket[16] << 8) | rxPacket[17])) / 1000.0;
  float pD_yaw   = ((int16_t)((rxPacket[18] << 8) | rxPacket[19])) / 1000.0;
  float trimPitch= ((int16_t)((rxPacket[20] << 8) | rxPacket[21])) / 100.0;
  float trimRoll = ((int16_t)((rxPacket[22] << 8) | rxPacket[23])) / 100.0;

  uint8_t frame[47];
  frame[0] = 0xFC;
  memcpy(&frame[1],  &pP_roll,   4);  memcpy(&frame[5],  &pI_roll,   4);  memcpy(&frame[9],  &pD_roll,  4);
  memcpy(&frame[13], &pP_pitch,  4);  memcpy(&frame[17], &pI_pitch,  4);  memcpy(&frame[21], &pD_pitch, 4);
  memcpy(&frame[25], &pP_yaw,    4);  memcpy(&frame[29], &pI_yaw,    4);  memcpy(&frame[33], &pD_yaw,   4);
  memcpy(&frame[37], &trimPitch, 4);  memcpy(&frame[41], &trimRoll,  4);
  frame[45] = calcChecksum(frame, 1, 44);
  frame[46] = 0xFD;

  pm2Link.write(frame, 47);
  Serial.println(F("Config update forwarded to PM2"));
}
void handleLedButtons()
{
  if(g_buttons & PCF_BTN_LED_MODE) {
    g_ledMode = (g_ledMode + 1) % 7;
    
  }
  if(g_buttons & PCF_BTN_LED_BRIGHT) {
    g_ledBrightnessIdx = (g_ledBrightnessIdx + 1) % 7;
    
  }
}

// ═══════════════════════════════════════════
//  TOF SENSORS — Pololu library
// ═══════════════════════════════════════════
#define XSHUT_FRONT 4
#define XSHUT_BACK  A1   // ← moved from pin 5 to free it for LED PWM
#define XSHUT_LEFT  6
#define XSHUT_RIGHT 7
#define XSHUT_DOWN  8

#define ADDR_FRONT 0x30
#define ADDR_BACK  0x31
#define ADDR_LEFT  0x32
#define ADDR_RIGHT 0x33
#define ADDR_DOWN  0x34

VL53L0X tofFront;
VL53L0X tofBack;
VL53L0X tofLeft;
VL53L0X tofRight;
VL53L0X tofDown;

uint16_t g_tofFrontMm = 9999, g_tofBackMm = 9999, g_tofLeftMm = 9999, g_tofRightMm = 9999, g_tofDownMm = 9999;
uint8_t  g_obstacleFlags = 0;
uint8_t  g_avoidFlags    = 0;

#define OBSTACLE_THRESHOLD_MM 150
#define AVOID_FRONT_MM 90
#define AVOID_SIDE_MM  80
#define TOF_MAX_VALID_MM 1200
#define TOF_FAIL_LIMIT 5

uint8_t g_tofFailStreak[5] = {0,0,0,0,0};

uint32_t lastToFPrintMs = 0;

uint8_t calcChecksum(uint8_t* data, uint8_t from, uint8_t to)
{
  uint8_t cs = 0;
  for(uint8_t i = from; i <= to; i++) cs ^= data[i];
  return cs;
}

static inline uint8_t safeBitPos(uint8_t slot) { return (slot == 0) ? 0 : slot + 1; }

void fecEncode(uint8_t* pkt)
{
  pkt[28] = pkt[29] = pkt[30] = pkt[31] = 0;
  for(uint8_t i = 0; i < DATA_BYTES; i++) {
    uint8_t trueBit = (pkt[i] >> 1) & 0x01;
    uint8_t fecByte = i / 7;
    uint8_t fecBit  = safeBitPos(i % 7);
    pkt[28 + fecByte] |= (trueBit << fecBit);
    pkt[i] |= 0x02;
  }
}

void fecDecode(uint8_t* pkt)
{
  for(uint8_t i = 0; i < DATA_BYTES; i++) {
    uint8_t fecByte = i / 7;
    uint8_t fecBit  = safeBitPos(i % 7);
    uint8_t trueBit = (pkt[28 + fecByte] >> fecBit) & 0x01;
    pkt[i] &= ~0x02;
    pkt[i] |= (trueBit << 1);
  }
}

uint8_t battVoltToPercent(float v)
{
  if(v >= 4.20) return 100;
  if(v <= 3.30) return 0;
  return (uint8_t)((v - 3.30) / (4.20 - 3.30) * 100.0);
}

void updateDroneBattery()
{
  static uint32_t lastBattMs = 0;
  if(millis() - lastBattMs < 1000) return;
  lastBattMs = millis();

  uint32_t sum = 0;
  for(uint8_t i = 0; i < BATT_SAMPLES; i++) {
    sum += analogRead(BATT_ADC_PIN);
    delayMicroseconds(100);
  }
  uint16_t raw = sum / BATT_SAMPLES;
  float voltage = (raw / 1023.0) * 5.0;
  droneBatPct = battVoltToPercent(voltage);
}

bool initOneToF(VL53L0X &sensor, uint8_t xshutPin, uint8_t newAddr, const __FlashStringHelper* label)
{
  digitalWrite(xshutPin, HIGH);
  delay(10);

  if(!sensor.init()) {
    Serial.print(F("ToF FAILED: ")); Serial.println(label);
    return false;
  }

  sensor.setAddress(newAddr);
  sensor.setTimeout(50);
  sensor.setMeasurementTimingBudget(20000);
  sensor.startContinuous();

  Serial.print(F("ToF OK: ")); Serial.print(label);
  Serial.print(F(" @ 0x")); Serial.println(newAddr, HEX);
  return true;
}

void setupToF()
{
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_BACK,  OUTPUT);
  pinMode(XSHUT_LEFT,  OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);
  pinMode(XSHUT_DOWN,  OUTPUT);

  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_BACK,  LOW);
  digitalWrite(XSHUT_LEFT,  LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  digitalWrite(XSHUT_DOWN,  LOW);
  delay(10);

  initOneToF(tofFront, XSHUT_FRONT, ADDR_FRONT, F("FRONT"));
  initOneToF(tofBack,  XSHUT_BACK,  ADDR_BACK,  F("BACK"));
  initOneToF(tofLeft,  XSHUT_LEFT,  ADDR_LEFT,  F("LEFT"));
  initOneToF(tofRight, XSHUT_RIGHT, ADDR_RIGHT, F("RIGHT"));
  initOneToF(tofDown,  XSHUT_DOWN,  ADDR_DOWN,  F("DOWN"));
}

uint16_t readToF(VL53L0X &sensor, uint8_t idx)
{
  uint16_t mm = sensor.readRangeContinuousMillimeters();
  bool bad = sensor.timeoutOccurred() || (mm > TOF_MAX_VALID_MM);

  if(bad) { g_tofFailStreak[idx]++; return 9999; }

  g_tofFailStreak[idx] = 0;
  return mm;
}

void checkToFRecovery(VL53L0X &sensor, uint8_t xshutPin, uint8_t addr, const __FlashStringHelper* label, uint8_t idx)
{
  if(g_tofFailStreak[idx] < TOF_FAIL_LIMIT) return;

  Serial.print(F("ToF auto-recovering: ")); Serial.println(label);

  digitalWrite(xshutPin, LOW);
  delay(5);

  sensor = VL53L0X();
  bool ok = initOneToF(sensor, xshutPin, addr, label);

  if(ok) {
    Serial.println(F("  -> recovery OK"));
    g_tofFailStreak[idx] = 0;
  } else {
    Serial.println(F("  -> recovery FAILED, will retry"));
  }
}

void updateObstacleFlags()
{
  g_obstacleFlags = 0;
  if(g_tofFrontMm < OBSTACLE_THRESHOLD_MM) g_obstacleFlags |= 0b00000001;
  if(g_tofBackMm  < OBSTACLE_THRESHOLD_MM) g_obstacleFlags |= 0b00000010;
  if(g_tofLeftMm  < OBSTACLE_THRESHOLD_MM) g_obstacleFlags |= 0b00000100;
  if(g_tofRightMm < OBSTACLE_THRESHOLD_MM) g_obstacleFlags |= 0b00001000;
}

void updateAvoidFlags()
{
  g_avoidFlags = 0;
  if(g_tofFrontMm < AVOID_FRONT_MM) g_avoidFlags |= 0b0001;
  if(g_tofBackMm  < AVOID_SIDE_MM)  g_avoidFlags |= 0b0010;
  if(g_tofLeftMm  < AVOID_SIDE_MM)  g_avoidFlags |= 0b0100;
  if(g_tofRightMm < AVOID_SIDE_MM)  g_avoidFlags |= 0b1000;
}

void updateToF()
{
  static uint32_t lastToFMs = 0;
  static uint8_t which = 0;
  if(millis() - lastToFMs < 15) return;
  lastToFMs = millis();

  switch(which) {
    case 0: g_tofFrontMm = readToF(tofFront, 0); checkToFRecovery(tofFront, XSHUT_FRONT, ADDR_FRONT, F("FRONT"), 0); break;
    case 1: g_tofBackMm  = readToF(tofBack, 1);  checkToFRecovery(tofBack,  XSHUT_BACK,  ADDR_BACK,  F("BACK"), 1);  break;
    case 2: g_tofLeftMm  = readToF(tofLeft, 2);  checkToFRecovery(tofLeft,  XSHUT_LEFT,  ADDR_LEFT,  F("LEFT"), 2);  break;
    case 3: g_tofRightMm = readToF(tofRight, 3); checkToFRecovery(tofRight, XSHUT_RIGHT, ADDR_RIGHT, F("RIGHT"), 3); updateObstacleFlags(); updateAvoidFlags(); break;
    case 4: g_tofDownMm  = readToF(tofDown, 4);  checkToFRecovery(tofDown,  XSHUT_DOWN,  ADDR_DOWN,  F("DOWN"), 4);  break;
  }
  which = (which + 1) % 5;
}

uint16_t correctAltitudeForTilt(uint16_t measuredMm, int16_t rollTenths, int16_t pitchTenths)
{
  if(measuredMm >= 9999) return measuredMm;

  float rollRad  = (rollTenths  / 10.0) * PI / 180.0;
  float pitchRad = (pitchTenths / 10.0) * PI / 180.0;
  float tiltFactor = cos(rollRad) * cos(pitchRad);

  if(tiltFactor < 0.5) return 9999;

  return (uint16_t)(measuredMm * tiltFactor);
}

void printToFDistances()
{
  if(millis() - lastToFPrintMs < 300) return;
  lastToFPrintMs = millis();

  Serial.print(F("F:")); Serial.print(g_tofFrontMm);
  Serial.print(F(" B:")); Serial.print(g_tofBackMm);
  Serial.print(F(" L:")); Serial.print(g_tofLeftMm);
  Serial.print(F(" R:")); Serial.print(g_tofRightMm);
  Serial.print(F(" D:")); Serial.print(g_tofDownMm);
  Serial.print(F(" | avoid:0b"));
  Serial.println(g_avoidFlags, BIN);
}

bool parseRxPacket()
{
  fecDecode(rxPacket);
  if(rxPacket[0] != 0xFF || rxPacket[27] != 0xFE) return false;

  uint8_t cs = calcChecksum(rxPacket, 1, 25);
  if(cs != rxPacket[26]) return false;

  if(rxPacket[1] == PKT_REMOTE_TO_DRONE) {
    g_throttle   = (int16_t)((rxPacket[3] << 8) | rxPacket[4]);
    g_roll       = (int16_t)((rxPacket[5] << 8) | rxPacket[6]);
    g_pitch      = (int16_t)((rxPacket[7] << 8) | rxPacket[8]);
    g_yaw        = (int16_t)((rxPacket[9] << 8) | rxPacket[10]);
    g_buttons    = rxPacket[11];
    g_flightMode = rxPacket[13];
    lastGoodPacketMs = millis();
    return true;
  }

  if(rxPacket[1] == PKT_CONFIG_UPDATE) {
    forwardConfigToPM2();
    lastGoodPacketMs = millis();
    return true;
  }

  return false;
}

void sendReplyToRemote()
{
  memset(txPacket, 0, PACKET_SIZE);
  txPacket[0] = 0xFF;
  txPacket[1] = PKT_DRONE_TO_REMOTE;
  txPacket[2] = replyCounter++;

  txPacket[3] = droneBatPct;

  uint16_t correctedAltMm = correctAltitudeForTilt(g_tofDownMm, g_droneRoll, g_dronePitch);
  txPacket[4] = (correctedAltMm >> 8) & 0xFF;
  txPacket[5] = correctedAltMm & 0xFF;

  txPacket[6]  = (g_tofFrontMm >> 8) & 0xFF;  txPacket[7]  = g_tofFrontMm & 0xFF;
  txPacket[8]  = (g_tofBackMm  >> 8) & 0xFF;  txPacket[9]  = g_tofBackMm  & 0xFF;
  txPacket[10] = (g_tofLeftMm  >> 8) & 0xFF;  txPacket[11] = g_tofLeftMm  & 0xFF;
  txPacket[12] = (g_tofRightMm >> 8) & 0xFF;  txPacket[13] = g_tofRightMm & 0xFF;

  txPacket[14] = (g_droneRoll >> 8) & 0xFF;   txPacket[15] = g_droneRoll & 0xFF;
  txPacket[16] = (g_dronePitch >> 8) & 0xFF;  txPacket[17] = g_dronePitch & 0xFF;

  txPacket[18] = (g_flightMode == 1) ? FLAG_ARMED : 0;
  txPacket[19] = lastRssiPct;
  txPacket[20] = g_obstacleFlags;
  txPacket[21] = g_flightMode;

  txPacket[26] = calcChecksum(txPacket, 1, 25);
  txPacket[27] = 0xFE;
  fecEncode(txPacket);

  radio.stopListening();
  radio.write(txPacket, PACKET_SIZE);

  uint8_t arc = radio.getARC();
  lastRssiPct = 100 - (arc * 100 / 5);

  radio.startListening();
}

void sendToPM2()
{
  memset(uartFrame, 0, UART_FRAME_SIZE);
  uartFrame[0] = 0xFF;
  uartFrame[1] = (g_throttle >> 8) & 0xFF;
  uartFrame[2] = g_throttle & 0xFF;
  uartFrame[3] = (g_roll >> 8) & 0xFF;
  uartFrame[4] = g_roll & 0xFF;
  uartFrame[5] = (g_pitch >> 8) & 0xFF;
  uartFrame[6] = g_pitch & 0xFF;
  uartFrame[7] = (g_yaw >> 8) & 0xFF;
  uartFrame[8] = g_yaw & 0xFF;
  uartFrame[9] = ((g_flightMode == 1) ? 0x01 : 0x00) |
               ((g_buttons & PCF_BTN_CALIBRATE) ? 0x02 : 0x00);
  uartFrame[10] = g_avoidFlags;
  uartFrame[11] = calcChecksum(uartFrame, 1, 10);
  uartFrame[12] = 0xFE;

  pm2Link.write(uartFrame, UART_FRAME_SIZE);
}

void readTelemetryFromPM2()
{
  while(pm2Link.available())
  {
    uint8_t b = pm2Link.read();
    if(telemIdx == 0 && b != 0xFA) continue;
    telemBuf[telemIdx++] = b;

    if(telemIdx >= TELEM_FRAME_SIZE) {
      telemIdx = 0;
      if(telemBuf[6] != 0xFB) continue;
      if(calcChecksum(telemBuf, 1, 4) != telemBuf[5]) continue;

      g_droneRoll  = (int16_t)((telemBuf[1] << 8) | telemBuf[2]);
      g_dronePitch = (int16_t)((telemBuf[3] << 8) | telemBuf[4]);
    }
  }
}

void reinitRadio()
{
  Serial.println(F("Radio stuck — reinitializing..."));
  radio.begin();
  radio.setChannel(108);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setPayloadSize(PACKET_SIZE);
  radio.setAutoAck(true);
  radio.setRetries(3, 5);
  radio.openWritingPipe(SEND_PIPE);
  radio.openReadingPipe(1, RECV_PIPE);
  radio.startListening();
  lastGoodPacketMs = millis();
}

void setup()
{
  wdt_disable();

  Serial.begin(115200);
  Serial.println(F("PM1 - NRF Receiver Starting..."));

  pinMode(LED_PIN, OUTPUT);
  analogWrite(LED_PIN, 0);

  pm2Link.begin(9600);

  setupToF();

  if(!radio.begin()) {
    Serial.println(F("NRF init FAILED!"));
    while(1);
  }

  radio.setChannel(108);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setPayloadSize(PACKET_SIZE);
  radio.setAutoAck(true);
  radio.setRetries(3, 5);

  radio.openWritingPipe(SEND_PIPE);
  radio.openReadingPipe(1, RECV_PIPE);
  radio.startListening();

  Serial.print(F("NRF chip connected: "));
  Serial.println(radio.isChipConnected() ? F("YES") : F("NO"));

  wdt_enable(WDTO_2S);
}

void loop()
{
  wdt_reset();

  updateDroneBattery();
  updateToF();
  printToFDistances();
  updateLED();
  readTelemetryFromPM2();

  if(radio.available()) {
    radio.read(rxPacket, PACKET_SIZE);
    if(parseRxPacket()) {
      handleLedButtons();
      sendReplyToRemote();
      sendToPM2();
    }
  }

  if(millis() - lastGoodPacketMs > SIGNAL_LOST_MS) {
    g_throttle   = 0;
    g_flightMode = 0;
    sendToPM2();
    delay(20);
  }

  if(millis() - lastGoodPacketMs > RADIO_STUCK_MS) {
    reinitRadio();
  }
}