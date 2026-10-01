#pragma once
#include <Arduino.h>
#include "os.h"

void logsInit();
void logsLoop();
void logsHandleAction(uint8_t actionID);

// Flight recording — called from main.ino
void logsStartFlight();
void logsWriteSample();
void logsEndFlight();
