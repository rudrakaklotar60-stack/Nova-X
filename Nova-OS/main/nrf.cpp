#include "nrf.h"
#include "graphics.h"
#include "os.h"
#include <RF24.h>

RF24    radio(NRF_CE, NRF_CSN);
uint8_t txPacket[PACKET_SIZE];
uint8_t rxPacket[PACKET_SIZE];
static  uint8_t pktCounter = 0;

static bool panicMode = false;

void nrfSetPanic(bool active)
{
  panicMode = active;
  if(active) novaX.remote.throttle = 0;
}
#define RSSI_WINDOW_SIZE 20   // ~400ms window at your 50Hz nrfLoop rate
static bool packetHistory[RSSI_WINDOW_SIZE] = {false};
static uint8_t historyIdx = 0;

static void updateLinkQuality(bool gotGoodPacket)
{
  packetHistory[historyIdx] = gotGoodPacket;
  historyIdx = (historyIdx + 1) % RSSI_WINDOW_SIZE;

  uint8_t hits = 0;
  for(uint8_t i = 0; i < RSSI_WINDOW_SIZE; i++)
    if(packetHistory[i]) hits++;

  novaX.drone.rssi = (hits * 100) / RSSI_WINDOW_SIZE;
}
bool nrfIsPanic() { return panicMode; }

static void busToNRF()
{
  digitalWrite(COUNTER_MR, LOW);
  hSPI.endTransaction();
  delayMicroseconds(50);
}

static void busToDisplay()
{
  hSPI.beginTransaction(SPISettings(DISPLAY_SPEED, MSBFIRST, SPI_MODE0));
  syncCounter();
}

uint8_t calcChecksum(uint8_t* data, uint8_t from, uint8_t to)
{
  uint8_t cs = 0;
  for(uint8_t i = from; i <= to; i++) cs ^= data[i];
  return cs;
}

// ═══════════════════════════════════════════
//  FEC — bit-1 is stuck HIGH at the NRF chip
//  level on this hardware. safeBitPos() skips
//  bit-1 in the mask bytes too, since they're
//  transmitted over the same broken link.
// ═══════════════════════════════════════════
static inline uint8_t safeBitPos(uint8_t slot)
{
  return (slot == 0) ? 0 : slot + 1;
}

void fecEncode(uint8_t* pkt)
{
  pkt[28] = pkt[29] = pkt[30] = pkt[31] = 0;
  for(uint8_t i = 0; i < DATA_BYTES; i++)
  {
    uint8_t trueBit = (pkt[i] >> 1) & 0x01;
    uint8_t fecByte = i / 7;
    uint8_t fecBit  = safeBitPos(i % 7);
    pkt[28 + fecByte] |= (trueBit << fecBit);
    pkt[i] |= 0x02;
  }
}

void fecDecode(uint8_t* pkt)
{
  for(uint8_t i = 0; i < DATA_BYTES; i++)
  {
    uint8_t fecByte = i / 7;
    uint8_t fecBit  = safeBitPos(i % 7);
    uint8_t trueBit = (pkt[28 + fecByte] >> fecBit) & 0x01;
    pkt[i] &= ~0x02;
    pkt[i] |= (trueBit << 1);
  }
}

bool nrfInit()
{
  busToNRF();

  if(!radio.begin(&hSPI)) {
    Serial.println("NRF init failed!");
    busToDisplay();
    return false;
  }

  radio.setChannel(NRF_CHANNEL);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setPayloadSize(PACKET_SIZE);
  radio.setAutoAck(true);
  radio.setRetries(3, 5);

  radio.openWritingPipe(NRF_SEND_PIPE);
  radio.openReadingPipe(1, NRF_RECV_PIPE);
  radio.startListening();

  bool connected = radio.isChipConnected();
  Serial.print("NRF chip connected: ");
  Serial.println(connected ? "YES" : "NO — check wiring/bus!");

  busToDisplay();
  Serial.println("NRF init OK!");
  return connected;
}

static void buildTxPacket()
{
  memset(txPacket, 0, PACKET_SIZE);

  txPacket[0]  = 0xFF;
  txPacket[1]  = PKT_REMOTE_TO_DRONE;
  txPacket[2]  = pktCounter++;

  int16_t thr = panicMode ? 0 : novaX.remote.throttle;
  txPacket[3]  = (thr >> 8) & 0xFF;
  txPacket[4]  = thr & 0xFF;

  int16_t roll = novaX.remote.roll;
  txPacket[5]  = (roll >> 8) & 0xFF;
  txPacket[6]  = roll & 0xFF;

  int16_t pitch = -novaX.remote.pitch;
  txPacket[7]  = (pitch >> 8) & 0xFF;
  txPacket[8]  = pitch & 0xFF;

  int16_t yaw = novaX.remote.yaw;
  txPacket[9]  = (yaw >> 8) & 0xFF;
  txPacket[10] = yaw & 0xFF;

  txPacket[11] = novaX.remote.buttons;
  txPacket[12] = 0;
  txPacket[13] = (uint8_t)novaX.remote.commandedFlightMode;  // ← intent, not confirmed state
  txPacket[14] = 0;

  txPacket[26] = calcChecksum(txPacket, 1, 25);
  txPacket[27] = 0xFE;

  fecEncode(txPacket);

  novaX.remote.buttons = 0;   // ← consume it, one-shot per press — don't resend stale bit
}

static bool parseRxPacket()
{
  static uint32_t lastDump = 0;
  bool doDump = (millis() - lastDump > 500);
  if(doDump) lastDump = millis();

  fecDecode(rxPacket);

  if(rxPacket[0]  != 0xFF) { if(doDump) Serial.println("RX: bad start"); return false; }
  if(rxPacket[27] != 0xFE) { if(doDump) Serial.println("RX: bad end");   return false; }
  if(rxPacket[1]  != PKT_DRONE_TO_REMOTE) {
    if(doDump) { Serial.print("RX: wrong id 0x"); Serial.println(rxPacket[1], HEX); }
    return false;
  }

  uint8_t cs = calcChecksum(rxPacket, 1, 25);
  if(cs != rxPacket[26]) {
    if(doDump) {
      Serial.print("RX: bad cs calc=0x"); Serial.print(cs, HEX);
      Serial.print(" got=0x"); Serial.println(rxPacket[26], HEX);
    }
    return false;
  }

  novaX.drone.droneBatPct   = rxPacket[3];
  novaX.drone.altitudeMm    = (uint16_t)(rxPacket[4]  << 8) | rxPacket[5];
  novaX.drone.tofFront      = (uint16_t)(rxPacket[6]  << 8) | rxPacket[7];
  novaX.drone.tofBack       = (uint16_t)(rxPacket[8]  << 8) | rxPacket[9];
  novaX.drone.tofLeft       = (uint16_t)(rxPacket[10] << 8) | rxPacket[11];
  novaX.drone.tofRight      = (uint16_t)(rxPacket[12] << 8) | rxPacket[13];
  novaX.drone.tofTop        = (uint16_t)(rxPacket[22] << 8) | rxPacket[23];   
  novaX.drone.roll          = (int16_t)((rxPacket[14] << 8) | rxPacket[15]);
  novaX.drone.pitch         = (int16_t)((rxPacket[16] << 8) | rxPacket[17]);

  uint8_t flags             = rxPacket[18];
  novaX.drone.isArmed       = (flags & FLAG_ARMED) != 0;
  novaX.drone.obstacleFlags = rxPacket[20];
  novaX.drone.flightMode    = (FlightMode)rxPacket[21];
  novaX.drone.lastPacketMs  = millis();
  novaX.drone.connState     = CONN_CONNECTED;

  if(panicMode && novaX.remote.throttle > 50)
    panicMode = false;

  return true;
}

void nrfSend()
{
  buildTxPacket();
  busToNRF();
  radio.stopListening();
  radio.write(txPacket, PACKET_SIZE);
  radio.startListening();
  busToDisplay();
}

bool nrfReceive()
{
  busToNRF();
  bool available = radio.available();
  if(available) {
    radio.read(rxPacket, PACKET_SIZE);
    busToDisplay();
    return parseRxPacket();
  }
  busToDisplay();
  return false;
}

void nrfCheckSignal()
{
  if(novaX.drone.connState != CONN_CONNECTED) return;
  uint32_t now = millis();
  if(now - novaX.drone.lastPacketMs > SIGNAL_LOST_MS) {
    novaX.drone.connState    = CONN_LOST;
    novaX.drone.signalLostMs = now;
    Serial.println("Signal LOST!");
  }
}

void nrfLoop()
{
  if(sdBusy) return;

  ScreenID s = osState.current;
  bool onFly = (s == SCR_DISCONNECTED   ||
                s == SCR_CONNECTED_INFO ||
                s == SCR_HUD_POSITION_HOLD ||
                s == SCR_HUD_ACTIVE_FLIGHT);
  if(!onFly) return;

  static uint32_t lastNrfMs = 0;
  uint32_t now = millis();
  if(now - lastNrfMs < 20) return;
  lastNrfMs = now;

  buildTxPacket();

  busToNRF();
  radio.stopListening();
  radio.write(txPacket, PACKET_SIZE);
  radio.startListening();

  bool available = radio.available();
  if(available)
    radio.read(rxPacket, PACKET_SIZE);

  busToDisplay();
  bool gotGoodPacket = available && parseRxPacket();
  updateLinkQuality(gotGoodPacket);
  if(available)
    parseRxPacket();

  nrfCheckSignal();
}