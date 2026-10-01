#pragma once
#include <Arduino.h>
#include "drone.h"
#include "os.h"

// ═══════════════════════════════════════════
//  NRF24L01 CONFIG
// ═══════════════════════════════════════════
#define NRF_CE         16
#define NRF_CSN        17
#define NRF_CHANNEL    108
#define PACKET_SIZE    32
#define DATA_BYTES     28
#define SIGNAL_LOST_MS 500

// Separate TX/RX pipes — avoids pipe 0/1 conflict
static const uint8_t NRF_SEND_PIPE[6] = "NovaX";  // remote→drone
static const uint8_t NRF_RECV_PIPE[6] = "evaXN";  // drone→remote

#define PKT_REMOTE_TO_DRONE  0x01
#define PKT_DRONE_TO_REMOTE  0x02
#define FLAG_ARMED           0x01

extern uint8_t txPacket[PACKET_SIZE];
extern uint8_t rxPacket[PACKET_SIZE];
extern volatile bool sdBusy;

uint8_t calcChecksum(uint8_t* data, uint8_t from, uint8_t to);
void    fecEncode(uint8_t* pkt);
void    fecDecode(uint8_t* pkt);
bool    nrfInit();
void    nrfSend();
bool    nrfReceive();
void    nrfCheckSignal();
void    nrfLoop();
void    nrfSetPanic(bool active);
bool    nrfIsPanic();