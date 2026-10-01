#pragma once
#include <Arduino.h>
#include "os.h"

void settingsInit();
void settingsLoop();
void settingsHandleAction(uint8_t actionID);
void settingsSaveCurrent();

