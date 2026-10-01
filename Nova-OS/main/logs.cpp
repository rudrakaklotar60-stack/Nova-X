#include "logs.h"
#include "drone.h"
#include "os.h"
#include "zones.h"
#include "graphics.h"
#include "storage.h"
#include <SD_MMC.h>
#include <ArduinoJson.h>

// ═══════════════════════════════════════════
//  CONSTANTS
// ═══════════════════════════════════════════
#define LOGS_REFRESH_MS  200
#define MAX_LOG_FILES    50
#define LOG_FILENAME_LEN 32

// ═══════════════════════════════════════════
//  INTERNAL STATE
// ═══════════════════════════════════════════
static uint32_t lastDrawMs    = 0;
static uint8_t  logCount      = 0;
static uint8_t  selectedLog   = 0;
static uint8_t  activeTab     = 0;   // 0=overview 1=alt 2=bat 3=att 4=events
static char     logFiles[MAX_LOG_FILES][LOG_FILENAME_LEN];

// Current log data (loaded from SD)
static StaticJsonDocument<4096> currentLog;
static bool logLoaded = false;

// ═══════════════════════════════════════════
//  CHECK IF ON LOGS SCREEN
// ═══════════════════════════════════════════
static bool onLogsScreen()
{
  ScreenID s = osState.current;
  return (s == SCR_LOGS_EMPTY         ||
          s == SCR_LOGS_LIST          ||
          s == SCR_LOG_OVERVIEW       ||
          s == SCR_LOG_ALT_GRAPH      ||
          s == SCR_LOG_BAT_GRAPH      ||
          s == SCR_LOG_ATT_GRAPH      ||
          s == SCR_LOG_EVENTS         ||
          s == SCR_LOG_DELETE_CONFIRM ||
          s == SCR_LOG_EXPORT         ||
          s == SCR_STATS_DASHBOARD);
}

// ═══════════════════════════════════════════
//  SCAN LOG FILES FROM SD
// ═══════════════════════════════════════════
static void scanLogFiles()
{
  logCount = 0;
  File dir = SD_MMC.open("/logs");
  if(!dir) return;

  File f = dir.openNextFile();
  while(f && logCount < MAX_LOG_FILES)
  {
    if(!f.isDirectory()) {
      String name = f.name();
      if(name.endsWith(".json")) {
        name.toCharArray(logFiles[logCount], LOG_FILENAME_LEN);
        logCount++;
      }
    }
    f = dir.openNextFile();
  }
  dir.close();
}

// ═══════════════════════════════════════════
//  LOAD SELECTED LOG FILE
// ═══════════════════════════════════════════
static bool loadLog(uint8_t idx)
{
  if(idx >= logCount) return false;

  String path = "/logs/" + String(logFiles[idx]);
  File f = SD_MMC.open(path, FILE_READ);
  if(!f) return false;

  currentLog.clear();
  DeserializationError err = deserializeJson(currentLog, f);
  f.close();

  if(err) {
    Serial.print("Log parse err: "); 
    Serial.println(err.c_str());
    return false;
  }

  logLoaded = true;
  return true;
}

// ═══════════════════════════════════════════
//  DRAW LOG LIST
// ═══════════════════════════════════════════
static void drawLogList()
{
  if(logCount == 0) {
    osGoto(SCR_LOGS_EMPTY);
    return;
  }

  DrawZone* listDZ = zonesGetDraw("log_list_area");
  if(!listDZ) return;

  uint16_t y = listDZ->y;
  uint16_t lineH = 20;

  // Show max 8 logs at a time
  uint8_t start = (selectedLog / 8) * 8;
  uint8_t end   = min((int)(start + 8), (int)logCount);

  for(uint8_t i = start; i < end; i++)
  {
    uint16_t col = (i == selectedLog) ? CYAN : WHITE;
    
    // Draw highlight box for selected
    if(i == selectedLog)
      fillRect(listDZ->x, y, listDZ->w, lineH - 2, 0x0821);

    printText(listDZ->x + 4, y + 4, String(logFiles[i]), col, 1);
    y += lineH;
  }
}

// ═══════════════════════════════════════════
//  DRAW LOG OVERVIEW
// ═══════════════════════════════════════════
static void drawLogOverview()
{
  if(!logLoaded) return;

  DrawZone* dateDZ   = zonesGetDraw("log_date");
  DrawZone* durDZ    = zonesGetDraw("log_duration");
  DrawZone* maxAltDZ = zonesGetDraw("log_max_alt");
  DrawZone* minBatDZ = zonesGetDraw("log_min_bat");

  if(dateDZ)   printText(dateDZ->x, dateDZ->y, currentLog["date"] | "Unknown", WHITE, 1);
  if(durDZ)    printText(durDZ->x, durDZ->y, String(currentLog["duration"] | 0) + "s", WHITE, 1);
  if(maxAltDZ) printText(maxAltDZ->x, maxAltDZ->y, String(currentLog["maxAlt"] | 0) + "mm", WHITE, 1);
  if(minBatDZ) printText(minBatDZ->x, minBatDZ->y, String(currentLog["minBat"] | 0) + "%", WHITE, 1);
}

// ═══════════════════════════════════════════
//  DRAW SIMPLE LINE GRAPH
//  data = array of values, count = how many
//  box position from draw zone id
// ═══════════════════════════════════════════
static void drawGraph(const char* dzID,
                      JsonArray data,
                      uint16_t minVal,
                      uint16_t maxVal,
                      uint16_t color)
{
  DrawZone* box = zonesGetDraw(dzID);
  if(!box) return;

  // Clear graph area & draw border
  fillRect(box->x, box->y, box->w, box->h, BLACK);
  drawRect(box->x, box->y, box->w, box->h, WHITE);

  uint16_t count = data.size();
  if(count < 2) return;

  // Plot points
  uint16_t prevX = 0, prevY = 0;
  for(uint16_t i = 0; i < count; i++)
  {
    uint16_t val = data[i] | 0;
    uint16_t px  = box->x + (i * box->w / (count - 1));
    uint16_t py  = box->y + box->h - ((val - minVal) * box->h / (maxVal - minVal));

    // Clamp
    if(py < box->y)          py = box->y;
    if(py > box->y + box->h) py = box->y + box->h;

    if(i > 0)
      drawLine(prevX, prevY, px, py, color);

    prevX = px;
    prevY = py;
  }
}

// ═══════════════════════════════════════════
//  DRAW GRAPHS
// ═══════════════════════════════════════════
static void drawAltGraph()
{
  if(!logLoaded) return;
  drawGraph("alt_graph", currentLog["altData"].as<JsonArray>(), 0, 2000, CYAN);
}

static void drawBatGraph()
{
  if(!logLoaded) return;
  drawGraph("bat_graph", currentLog["batData"].as<JsonArray>(), 0, 100, GREEN);
}

static void drawAttGraph()
{
  if(!logLoaded) return;
  drawGraph("att_graph_r", currentLog["rollData"].as<JsonArray>(),  -900, 900, RED);
  drawGraph("att_graph_p", currentLog["pitchData"].as<JsonArray>(), -900, 900, YELLOW);
}

// ═══════════════════════════════════════════
//  DRAW EVENTS TIMELINE
// ═══════════════════════════════════════════
static void drawEvents()
{
  if(!logLoaded) return;

  DrawZone* listDZ = zonesGetDraw("events_area");
  if(!listDZ) return;

  JsonArray events = currentLog["events"].as<JsonArray>();
  uint16_t y = listDZ->y;

  for(JsonObject ev : events) {
    if(y > listDZ->y + listDZ->h) break;
    String line = String(ev["time"] | 0) + "s  " + (ev["msg"] | "");
    printText(listDZ->x, y, line, WHITE, 1);
    y += 18;
  }
}

// ═══════════════════════════════════════════
//  DELETE SELECTED LOG
// ═══════════════════════════════════════════
static void deleteSelectedLog()
{
  return;  // SD write-protected — deletion disabled for now
  if(selectedLog >= logCount) return;
  
  String path = "/logs/" + String(logFiles[selectedLog]);
  SD_MMC.remove(path.c_str());
  logLoaded = false;
  scanLogFiles();
  
  if(selectedLog >= logCount && selectedLog > 0)
    selectedLog--;
    
  osGoto(logCount == 0 ? SCR_LOGS_EMPTY : SCR_LOGS_LIST);
}

// ═══════════════════════════════════════════
//  LOG WRITING — called during flight
//  Creates/appends to flight log on SD
// ═══════════════════════════════════════════
static File activeLogFile;
static bool logWriting   = false;
static uint32_t logStartMs = 0;
static uint16_t logSampleCount = 0;

void logsStartFlight()
{
  return; // SD write-protected — recording disabled for now

  // Generate filename from timestamp
  String filename = "/logs/flight_" + String(millis()) + ".json";

  activeLogFile = SD_MMC.open(filename.c_str(), FILE_WRITE);
  if(!activeLogFile) {
    Serial.println("Log file open failed!");
    return;
  }

  // Write JSON header
  activeLogFile.print("{\"date\":\"flight\",");
  activeLogFile.print("\"altData\":[");
  logWriting     = true;
  logStartMs     = millis();
  logSampleCount = 0;
  Serial.println("Log started");
}

void logsWriteSample()
{
  return;  // SD write-protected — recording disabled for now
  
  if(!logWriting) return;

  // Write sample every 500ms
  static uint32_t lastSampleMs = 0;
  uint32_t now = millis();
  if(now - lastSampleMs < 500) return;
  lastSampleMs = now;

  if(logSampleCount > 0)
    activeLogFile.print(",");
    
  activeLogFile.print(novaX.drone.altitudeMm);
  logSampleCount++;
}

void logsEndFlight()
{
  return;  // SD write-protected — recording disabled for now
  
  if(!logWriting) return;

  uint32_t dur = (millis() - logStartMs) / 1000;

  // Close arrays + write summary
  activeLogFile.print("],");
  activeLogFile.print("\"duration\":");
  activeLogFile.print(dur);
  activeLogFile.print(",\"maxAlt\":0");
  activeLogFile.print(",\"minBat\":");
  activeLogFile.print(novaX.drone.droneBatPct);
  activeLogFile.print("}");
  activeLogFile.close();

  logWriting = false;
  Serial.println("Log saved");
}

// ═══════════════════════════════════════════
//  LOGS INIT
// ═══════════════════════════════════════════
void logsInit()
{
  logCount     = 0;
  selectedLog  = 0;
  activeTab    = 0;
  logLoaded    = false;
  logWriting   = false;
  
  scanLogFiles();

  if(logCount == 0) osGoto(SCR_LOGS_EMPTY);
  else              osGoto(SCR_LOGS_LIST);
}

// ═══════════════════════════════════════════
//  LOGS LOOP
// ═══════════════════════════════════════════
void logsLoop()
{
  if(!onLogsScreen()) return;

  uint32_t now = millis();
  if(now - lastDrawMs < LOGS_REFRESH_MS) return;
  lastDrawMs = now;

  zonesRenderDraws();

  switch(osState.current) {
    case SCR_LOGS_LIST:          drawLogList(); break;
    case SCR_LOG_OVERVIEW:       drawLogOverview(); break;
    case SCR_LOG_ALT_GRAPH:      drawAltGraph(); break;
    case SCR_LOG_BAT_GRAPH:      drawBatGraph(); break;
    case SCR_LOG_ATT_GRAPH:      drawAttGraph(); break;
    case SCR_LOG_EVENTS:         drawEvents(); break;
    default:                     break;
  }
}

// ═══════════════════════════════════════════
//  ACTION HANDLER
//  Called from input when zone actionID > 0
// ═══════════════════════════════════════════
void logsHandleAction(uint8_t actionID)
{
  switch(osState.current)
  {
    case SCR_LOGS_LIST:
      if(actionID == 1) { // open log
        loadLog(selectedLog);
        osGoto(SCR_LOG_OVERVIEW);
      }
      break;

    case SCR_LOG_OVERVIEW:
    case SCR_LOG_ALT_GRAPH:
    case SCR_LOG_BAT_GRAPH:
    case SCR_LOG_ATT_GRAPH:
    case SCR_LOG_EVENTS:
      // Tab switching via actionID
      switch(actionID) {
        case 1: osGoto(SCR_LOG_OVERVIEW);       break;
        case 2: osGoto(SCR_LOG_ALT_GRAPH);      break;
        case 3: osGoto(SCR_LOG_BAT_GRAPH);      break;
        case 4: osGoto(SCR_LOG_ATT_GRAPH);      break;
        case 5: osGoto(SCR_LOG_EVENTS);         break;
        case 6: osGoto(SCR_LOG_DELETE_CONFIRM); break;
      }
      break;

    case SCR_LOG_DELETE_CONFIRM:
      if(actionID == 1) deleteSelectedLog(); // confirm
      if(actionID == 2) osBack();            // cancel
      break;

    default:
      break;
  }
}
