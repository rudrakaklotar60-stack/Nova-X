#pragma once

#include <Arduino.h>
#include <SPI.h>

struct GlyphInfo { uint8_t width; uint16_t dataOffset; uint8_t bytesPerRow; };

#include "font_small.h"
#include "font_large.h"


// ── Screen ────────────────────────────────────────────
#define SCREEN_W  320
#define SCREEN_H  240

// ── SPI Speed ─────────────────────────────────────────
#define DISPLAY_SPEED  40000000   // 40MHz (80MHz unstable)

// ── Co-processor Pin ──────────────────────────────────
#define COUNTER_MR  25   // MR/CLR of 74HC161 + 74HC74

// ── SR#2 Control Bit Positions ────────────────────────
#define SR_RS   0   // Register Select (Data/Command)
#define SR_CS   1   // Display Chip Select
#define SR_RST  2   // Display Reset
#define SR_RD   3   // Read (tied HIGH, unused)
#define SR_W25  4   // W25Q256 CS
#define SR_SD   5   // SD CS (legacy, unused with SD_MMC)

// ── Colors RGB565 ─────────────────────────────────────
#define BLACK    0x0000
#define WHITE    0xFFFF
#define RED      0xF800
#define GREEN    0x07E0
#define BLUE     0x001F
#define YELLOW   0xFFE0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define ORANGE   0xFD20
#define GRAY     0x8410

// ── Dirty Rect System ─────────────────────────────────
#define MAX_DIRTY 16

struct DirtyRect { int x, y, w, h; };
extern DirtyRect dirtyRects[MAX_DIRTY];
extern int       dirtyCount;

// ── Global SPI + Control Byte ─────────────────────────
extern SPIClass hSPI;
extern uint8_t  control595;

// ── Function Declarations ─────────────────────────────

// Bus
void initBus();
void initSD();
void syncCounter();
void writeBus(uint8_t data);
void writeBusFast(uint16_t packet);

// Display commands
void writeCommand(uint8_t cmd);
void writeData(uint8_t data);
void setAddressWindow(uint16_t x0, uint16_t y0,
                      uint16_t x1, uint16_t y1);
void initDisplay();

// Chip selects
void w25Enable();
void w25Disable();

// Dirty rect
void markDirty(int x, int y, int w, int h);
void eraseDirty(uint16_t color);
void clearDirty();

// Drawing
void drawPixel(uint16_t x, uint16_t y, uint16_t color);
void fillRect(uint16_t x, uint16_t y,
              uint16_t w, uint16_t h, uint16_t color);
void fillScreen(uint16_t color);
void drawRect(uint16_t x, uint16_t y,
              uint16_t w, uint16_t h, uint16_t color);
void drawLine(int x0, int y0, int x1, int y1, uint16_t color);
void drawCircle(int x0, int y0, int r, uint16_t color);
void fillCircle(int x0, int y0, int r, uint16_t color);
void fastFillCircle(int x0, int y0, int r,
                    uint16_t color, uint16_t bg);

// Text
void drawChar(int x, int y, char c,
              uint16_t color, int size);
void printText(int x, int y, String text,
               uint16_t color, int size);

enum FontID { FONT_SMALL, FONT_LARGE };

void drawCharF(int x, int y, char c, uint16_t color, FontID font);
int  printTextF(int x, int y, String text, uint16_t color, FontID font);
int  textWidthF(String text, FontID font);

// SD Image + Animation
bool loadImageFromSD(const char* filename, int x, int y);
bool drawImageSD(const char* filename, int x, int y);
void playAnimation(const char* filename);
