#include "about.h"
#include "drone.h"
#include "os.h"
#include "zones.h"
#include "graphics.h"
#include <SD_MMC.h>

// ═══════════════════════════════════════════
//  CONSTANTS
// ═══════════════════════════════════════════
#define ABOUT_REFRESH_MS  500
#define NOVA_VERSION      "2.0.0"
#define NOVA_BUILD        "June 2026"
#define NOVA_AUTHOR       "Rudra"

// ═══════════════════════════════════════════
//  INTERNAL STATE
// ═══════════════════════════════════════════
static uint32_t lastDrawMs = 0;

// ═══════════════════════════════════════════
//  HELPER: CALCULATE DIRECTORY SIZE
// ═══════════════════════════════════════════
static uint32_t getDirSize(const char* path) 
{
  uint32_t sz = 0;
  File dir = SD_MMC.open(path);
  if(dir) {
    File f = dir.openNextFile();
    while(f) { 
      sz += f.size(); 
      f = dir.openNextFile(); 
    }
  }
  return sz;
}

// ═══════════════════════════════════════════
//  CHECK IF ON ABOUT SCREEN
// ═══════════════════════════════════════════
static bool onAboutScreen()
{
  ScreenID s = osState.current;
  return (s == SCR_ABOUT      ||
          s == SCR_CREDITS    ||
          s == SCR_HW_INFO    ||
          s == SCR_FLASH_STATS||
          s == SCR_LICENSES);
}

// ═══════════════════════════════════════════
//  ABOUT SCREEN
//  Nova-X version, build date, author
// ═══════════════════════════════════════════
static void drawAbout()
{
  DrawZone* verDZ   = zonesGetDraw("version");
  DrawZone* buildDZ = zonesGetDraw("build_date");
  DrawZone* authDZ  = zonesGetDraw("author");
  DrawZone* descDZ  = zonesGetDraw("description");

  if(verDZ)
    printText(verDZ->x, verDZ->y, "Nova OS v" + String(NOVA_VERSION), CYAN, 1);

  if(buildDZ)
    printText(buildDZ->x, buildDZ->y, "Build: " + String(NOVA_BUILD), WHITE, 1);

  if(authDZ)
    printText(authDZ->x, authDZ->y, "By: " + String(NOVA_AUTHOR), YELLOW, 1);

  if(descDZ)
    printText(descDZ->x, descDZ->y, "Custom Drone Remote Controller", WHITE, 1);
}

// ═══════════════════════════════════════════
//  CREDITS SCREEN
// ═══════════════════════════════════════════
static void drawCredits()
{
  DrawZone* c1 = zonesGetDraw("credit_1");
  DrawZone* c2 = zonesGetDraw("credit_2");
  DrawZone* c3 = zonesGetDraw("credit_3");
  DrawZone* c4 = zonesGetDraw("credit_4");

  if(c1) printText(c1->x, c1->y, "Hardware Design : Rudra",  CYAN,   1);
  if(c2) printText(c2->x, c2->y, "Firmware        : Rudra",  CYAN,   1);
  if(c3) printText(c3->x, c3->y, "UI Design       : Rudra",  CYAN,   1);
  if(c4) printText(c4->x, c4->y, "AI Assist       : Claude", YELLOW, 1);
}

// ═══════════════════════════════════════════
//  HARDWARE INFO SCREEN
//  Show ESP32 chip info + pinmap summary
// ═══════════════════════════════════════════
static void drawHWInfo()
{
  DrawZone* chipDZ  = zonesGetDraw("chip_info");
  DrawZone* freqDZ  = zonesGetDraw("cpu_freq");
  DrawZone* flashDZ = zonesGetDraw("flash_size");
  DrawZone* ramDZ   = zonesGetDraw("ram_free");
  DrawZone* sdDZ    = zonesGetDraw("sd_size");
  DrawZone* dispDZ  = zonesGetDraw("disp_info");

  if(chipDZ)
    printText(chipDZ->x, chipDZ->y, "ESP32 " + String(ESP.getChipModel()), WHITE, 1);

  if(freqDZ)
    printText(freqDZ->x, freqDZ->y, String(ESP.getCpuFreqMHz()) + " MHz", WHITE, 1);

  if(flashDZ)
    printText(flashDZ->x, flashDZ->y, String(ESP.getFlashChipSize() / 1024 / 1024) + " MB Flash", WHITE, 1);

  if(ramDZ)
    printText(ramDZ->x, ramDZ->y, String(ESP.getFreeHeap() / 1024) + " KB Free RAM", GREEN, 1);

  if(sdDZ) {
    uint64_t sdBytes = SD_MMC.totalBytes();
    uint64_t sdUsed  = SD_MMC.usedBytes();
    String sdStr = String((uint32_t)(sdUsed  / 1024 / 1024)) + "/" +
                   String((uint32_t)(sdBytes / 1024 / 1024)) + " MB";
    printText(sdDZ->x, sdDZ->y, sdStr, WHITE, 1);
  }

  if(dispDZ)
    printText(dispDZ->x, dispDZ->y, "ILI9341 320x240 SPI", WHITE, 1);
}

// ═══════════════════════════════════════════
//  FLASH STATS SCREEN
//  SD card usage breakdown
// ═══════════════════════════════════════════
static void drawFlashStats()
{
  DrawZone* totalDZ   = zonesGetDraw("sd_total");
  DrawZone* screensDZ = zonesGetDraw("sd_screens");
  DrawZone* configDZ  = zonesGetDraw("sd_config");
  DrawZone* logsDZ    = zonesGetDraw("sd_logs");
  DrawZone* animDZ    = zonesGetDraw("sd_anim");
  DrawZone* freeDZ    = zonesGetDraw("sd_free");
  DrawZone* barDZ     = zonesGetDraw("sd_usage_bar");

  uint64_t total = SD_MMC.totalBytes();
  uint64_t used  = SD_MMC.usedBytes();
  uint64_t free  = total - used;

  if(totalDZ)
    printText(totalDZ->x, totalDZ->y, "Total: " + String((uint32_t)(total/1024/1024)) + " MB", WHITE, 1);

  if(freeDZ)
    printText(freeDZ->x, freeDZ->y, "Free: " + String((uint32_t)(free/1024/1024)) + " MB", GREEN, 1);

  // Usage bar
  if(barDZ) {
    uint8_t pct = (uint8_t)(used * 100 / total);
    zonesSetValue("sd_usage_bar", pct);
    uint16_t col = pct < 70 ? GREEN : pct < 90 ? YELLOW : RED;
    zonesSetColor("sd_usage_bar", col);
  }

  // Individual folder sizes (Using new helper function)
  if(screensDZ)
    printText(screensDZ->x, screensDZ->y, "Screens: " + String(getDirSize("/screens")/1024) + " KB", WHITE, 1);

  if(configDZ)
    printText(configDZ->x, configDZ->y, "Config: " + String(getDirSize("/config")/1024) + " KB", WHITE, 1);

  if(logsDZ)
    printText(logsDZ->x, logsDZ->y, "Logs: " + String(getDirSize("/logs")/1024) + " KB", WHITE, 1);

  // Animation file
  if(animDZ) {
    File f = SD_MMC.open("/anim.dlt");
    uint32_t sz = f ? f.size() : 0;
    if(f) f.close();
    printText(animDZ->x, animDZ->y, "Animation: " + String(sz/1024) + " KB", WHITE, 1);
  }
}

// ═══════════════════════════════════════════
//  LICENSES SCREEN
//  Show open source credits
// ═══════════════════════════════════════════
static void drawLicenses()
{
  DrawZone* l1 = zonesGetDraw("lic_1");
  DrawZone* l2 = zonesGetDraw("lic_2");
  DrawZone* l3 = zonesGetDraw("lic_3");
  DrawZone* l4 = zonesGetDraw("lic_4");
  DrawZone* l5 = zonesGetDraw("lic_5");

  if(l1) printText(l1->x, l1->y, "RF24 Library — TMRh20", WHITE, 1);
  if(l2) printText(l2->x, l2->y, "ArduinoJson — Benoit B.", WHITE, 1);
  if(l3) printText(l3->x, l3->y, "SD_MMC — Espressif", WHITE, 1);
  if(l4) printText(l4->x, l4->y, "Arduino ESP32 Core", WHITE, 1);
  if(l5) printText(l5->x, l5->y, "All MIT / Apache 2.0", YELLOW, 1);
}

// ═══════════════════════════════════════════
//  ABOUT INIT
// ═══════════════════════════════════════════
void aboutInit()
{
  lastDrawMs = 0;
}

// ═══════════════════════════════════════════
//  ABOUT LOOP
// ═══════════════════════════════════════════
void aboutLoop()
{
  if(!onAboutScreen()) return;

  uint32_t now = millis();
  if(now - lastDrawMs < ABOUT_REFRESH_MS) return;
  lastDrawMs = now;

  zonesRenderDraws();

  switch(osState.current) {
    case SCR_ABOUT:       drawAbout();      break;
    case SCR_CREDITS:     drawCredits();    break;
    case SCR_HW_INFO:     drawHWInfo();     break;
    case SCR_FLASH_STATS: drawFlashStats(); break;
    case SCR_LICENSES:    drawLicenses();   break;
    default: break;
  }
}
