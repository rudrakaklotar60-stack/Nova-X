#include "graphics.h"
#include <SPI.h>
#include <math.h>
#include <SD_MMC.h>
#include <FS.h>

#define SR_DATA  23
#define SR_CLOCK 18

// ========================
// ANIMATION (4-bit SDMMC)
// ========================
#define ROWS_PER_CHUNK 16
#define BATCH_SIZE     10240

static uint8_t rowBuf[320 * 2 * ROWS_PER_CHUNK];    // 10,240 bytes
static uint8_t batchBuf[BATCH_SIZE];                // 10,240 bytes
static uint8_t txBuf[320 * 2 * ROWS_PER_CHUNK * 2]; // 20,480 bytes

SPIClass hSPI(VSPI);

uint8_t control595 = 0b00111100;

// ========================
// DIRTY RECT SYSTEM
// ========================
DirtyRect dirtyRects[MAX_DIRTY];
int       dirtyCount = 0;

void markDirty(int x, int y, int w, int h)
{
  if(x < 0)            { w += x; x = 0; }
  if(y < 0)            { h += y; y = 0; }
  if(x + w > SCREEN_W)  w = SCREEN_W - x;
  if(y + h > SCREEN_H)  h = SCREEN_H - y;
  if(w <= 0 || h <= 0)  return;
  if(dirtyCount >= MAX_DIRTY) return;
  dirtyRects[dirtyCount++] = { x, y, w, h };
}

void eraseDirty(uint16_t color)
{
  for(int i = 0; i < dirtyCount; i++)
    fillRect(dirtyRects[i].x, dirtyRects[i].y,
             dirtyRects[i].w, dirtyRects[i].h, color);
}

void clearDirty() { dirtyCount = 0; }

// ========================
// FONT
// ========================
static const uint8_t font5x7[][5] = {
  {0x00,0x00,0x00,0x00,0x00}, // 32 space
  {0x00,0x00,0x5F,0x00,0x00}, // 33 !
  {0x00,0x07,0x00,0x07,0x00}, // 34 "
  {0x14,0x7F,0x14,0x7F,0x14}, // 35 #
  {0x24,0x2A,0x7F,0x2A,0x12}, // 36 $
  {0x23,0x13,0x08,0x64,0x62}, // 37 %
  {0x36,0x49,0x55,0x22,0x50}, // 38 &
  {0x00,0x05,0x03,0x00,0x00}, // 39 '
  {0x00,0x1C,0x22,0x41,0x00}, // 40 (
  {0x00,0x41,0x22,0x1C,0x00}, // 41 )
  {0x14,0x08,0x3E,0x08,0x14}, // 42 *
  {0x08,0x08,0x3E,0x08,0x08}, // 43 +
  {0x00,0x50,0x30,0x00,0x00}, // 44 ,
  {0x08,0x08,0x08,0x08,0x08}, // 45 -
  {0x00,0x60,0x60,0x00,0x00}, // 46 .
  {0x20,0x10,0x08,0x04,0x02}, // 47 /
  {0x3E,0x51,0x49,0x45,0x3E}, // 48 0
  {0x00,0x42,0x7F,0x40,0x00}, // 49 1
  {0x42,0x61,0x51,0x49,0x46}, // 50 2
  {0x21,0x41,0x45,0x4B,0x31}, // 51 3
  {0x18,0x14,0x12,0x7F,0x10}, // 52 4
  {0x27,0x45,0x45,0x45,0x39}, // 53 5
  {0x3C,0x4A,0x49,0x49,0x30}, // 54 6
  {0x01,0x71,0x09,0x05,0x03}, // 55 7
  {0x36,0x49,0x49,0x49,0x36}, // 56 8
  {0x06,0x49,0x49,0x29,0x1E}, // 57 9
  {0x00,0x36,0x36,0x00,0x00}, // 58 :
  {0x00,0x56,0x36,0x00,0x00}, // 59 ;
  {0x08,0x14,0x22,0x41,0x00}, // 60 <
  {0x14,0x14,0x14,0x14,0x14}, // 61 =
  {0x00,0x41,0x22,0x14,0x08}, // 62 >
  {0x02,0x01,0x51,0x09,0x06}, // 63 ?
  {0x32,0x49,0x79,0x41,0x3E}, // 64 @
  {0x7E,0x11,0x11,0x11,0x7E}, // 65 A
  {0x7F,0x49,0x49,0x49,0x36}, // 66 B
  {0x3E,0x41,0x41,0x41,0x22}, // 67 C
  {0x7F,0x41,0x41,0x22,0x1C}, // 68 D
  {0x7F,0x49,0x49,0x49,0x41}, // 69 E
  {0x7F,0x09,0x09,0x09,0x01}, // 70 F
  {0x3E,0x41,0x49,0x49,0x7A}, // 71 G
  {0x7F,0x08,0x08,0x08,0x7F}, // 72 H
  {0x00,0x41,0x7F,0x41,0x00}, // 73 I
  {0x20,0x40,0x41,0x3F,0x01}, // 74 J
  {0x7F,0x08,0x14,0x22,0x41}, // 75 K
  {0x7F,0x40,0x40,0x40,0x40}, // 76 L
  {0x7F,0x02,0x0C,0x02,0x7F}, // 77 M
  {0x7F,0x04,0x08,0x10,0x7F}, // 78 N
  {0x3E,0x41,0x41,0x41,0x3E}, // 79 O
  {0x7F,0x09,0x09,0x09,0x06}, // 80 P
  {0x3E,0x41,0x51,0x21,0x5E}, // 81 Q
  {0x7F,0x09,0x19,0x29,0x46}, // 82 R
  {0x46,0x49,0x49,0x49,0x31}, // 83 S
  {0x01,0x01,0x7F,0x01,0x01}, // 84 T
  {0x3F,0x40,0x40,0x40,0x3F}, // 85 U
  {0x1F,0x20,0x40,0x20,0x1F}, // 86 V
  {0x3F,0x40,0x38,0x40,0x3F}, // 87 W
  {0x63,0x14,0x08,0x14,0x63}, // 88 X
  {0x07,0x08,0x70,0x08,0x07}, // 89 Y
  {0x61,0x51,0x49,0x45,0x43}, // 90 Z
  {0x00,0x7F,0x41,0x41,0x00}, // 91 [
  {0x02,0x04,0x08,0x10,0x20}, // 92 '\'
  {0x00,0x41,0x41,0x7F,0x00}, // 93 ]
  {0x04,0x02,0x01,0x02,0x04}, // 94 ^
  {0x40,0x40,0x40,0x40,0x40}, // 95 _
  {0x00,0x01,0x02,0x04,0x00}, // 96 `
  {0x20,0x54,0x54,0x54,0x78}, // 97 a
  {0x7F,0x48,0x44,0x44,0x38}, // 98 b
  {0x38,0x44,0x44,0x44,0x20}, // 99 c
  {0x38,0x44,0x44,0x48,0x7F}, // 100 d
  {0x38,0x54,0x54,0x54,0x18}, // 101 e
  {0x08,0x7E,0x09,0x01,0x02}, // 102 f
  {0x0C,0x52,0x52,0x52,0x3E}, // 103 g
  {0x7F,0x08,0x04,0x04,0x78}, // 104 h
  {0x00,0x44,0x7D,0x40,0x00}, // 105 i
  {0x20,0x40,0x44,0x3D,0x00}, // 106 j
  {0x7F,0x10,0x28,0x44,0x00}, // 107 k
  {0x00,0x41,0x7F,0x40,0x00}, // 108 l
  {0x7C,0x04,0x18,0x04,0x78}, // 109 m
  {0x7C,0x08,0x04,0x04,0x78}, // 110 n
  {0x38,0x44,0x44,0x44,0x38}, // 111 o
  {0x7C,0x14,0x14,0x14,0x08}, // 112 p
  {0x08,0x14,0x14,0x18,0x7C}, // 113 q
  {0x7C,0x08,0x04,0x04,0x08}, // 114 r
  {0x48,0x54,0x54,0x54,0x20}, // 115 s
  {0x04,0x3F,0x44,0x40,0x20}, // 116 t
  {0x3C,0x40,0x40,0x20,0x7C}, // 117 u
  {0x1C,0x20,0x40,0x20,0x1C}, // 118 v
  {0x3C,0x40,0x30,0x40,0x3C}, // 119 w
  {0x44,0x28,0x10,0x28,0x44}, // 120 x
  {0x0C,0x50,0x50,0x50,0x3C}, // 121 y
  {0x44,0x64,0x54,0x4C,0x44}, // 122 z
  {0x00,0x08,0x36,0x41,0x00}, // 123 {
  {0x00,0x00,0x7F,0x00,0x00}, // 124 |
  {0x00,0x41,0x36,0x08,0x00}, // 125 }
  {0x10,0x08,0x08,0x10,0x08}, // 126 ~
};

// ========================
// SD INIT (4-bit SDMMC)
// ========================
void initSD()
{
  // false = 4-bit mode, no pin remapping needed
  if(!SD_MMC.begin("/sdcard", true))
    Serial.println("SD_MMC init failed!");
  else
    Serial.println("SD_MMC init OK!");
}

// ========================
// COUNTER SYNC
// ========================
void syncCounter()
{
  digitalWrite(COUNTER_MR, LOW);
  delayMicroseconds(1);
  digitalWrite(COUNTER_MR, HIGH);
}

// ========================
// BUS INIT
// ========================
void initBus()
{
  pinMode(COUNTER_MR, OUTPUT);
  digitalWrite(COUNTER_MR, LOW);
  delayMicroseconds(1);

  hSPI.begin(SR_CLOCK, 19, SR_DATA, -1);
  hSPI.beginTransaction(
    SPISettings(DISPLAY_SPEED, MSBFIRST, SPI_MODE0)
  );
  syncCounter();
}

// ========================
// WRITE BUS
// ========================
void writeBus(uint8_t data)
{
  hSPI.write16(((uint16_t)control595 << 8) | data);
}

void writeBusFast(uint16_t packet)
{
  hSPI.write16(packet);
}

// ========================
// COMMAND / DATA
// ========================
void writeCommand(uint8_t cmd)
{
  control595 &= ~(1 << SR_RS);
  writeBus(cmd);
}

void writeData(uint8_t data)
{
  control595 |= (1 << SR_RS);
  writeBus(data);
}

// ========================
// CHIP SELECTS
// ========================
void w25Enable()  { control595 &= ~(1 << SR_W25); writeBus(0x00); }
void w25Disable() { control595 |=  (1 << SR_W25); writeBus(0x00); }

// ========================
// ADDRESS WINDOW
// ========================
void setAddressWindow(uint16_t x0, uint16_t y0,
                      uint16_t x1, uint16_t y1)
{
  writeCommand(0x2A);
  writeData(x0 >> 8); writeData(x0 & 0xFF);
  writeData(x1 >> 8); writeData(x1 & 0xFF);
  writeCommand(0x2B);
  writeData(y0 >> 8); writeData(y0 & 0xFF);
  writeData(y1 >> 8); writeData(y1 & 0xFF);
  writeCommand(0x2C);
}

// ========================
// ILI9341 INIT
// ========================
void initDisplay()
{
  control595 &= ~(1 << SR_RST); writeBus(0x00); delay(10);
  control595 |=  (1 << SR_RST); writeBus(0x00); delay(120);

  writeCommand(0x01); delay(150);
  writeCommand(0xEF);
  writeData(0x03); writeData(0x80); writeData(0x02);
  writeCommand(0xCF);
  writeData(0x00); writeData(0xC1); writeData(0x30);
  writeCommand(0xED);
  writeData(0x64); writeData(0x03);
  writeData(0x12); writeData(0x81);
  writeCommand(0xE8);
  writeData(0x85); writeData(0x00); writeData(0x78);
  writeCommand(0xCB);
  writeData(0x39); writeData(0x2C); writeData(0x00);
  writeData(0x34); writeData(0x02);
  writeCommand(0xF7); writeData(0x20);
  writeCommand(0xEA); writeData(0x00); writeData(0x00);
  writeCommand(0xC0); writeData(0x23);
  writeCommand(0xC1); writeData(0x10);
  writeCommand(0xC5); writeData(0x3E); writeData(0x28);
  writeCommand(0xC7); writeData(0x86);
  writeCommand(0x36); writeData(0x28);
  writeCommand(0x3A); writeData(0x55);
  writeCommand(0xB1); writeData(0x00); writeData(0x18);
  writeCommand(0xB6);
  writeData(0x08); writeData(0x82); writeData(0x27);
  writeCommand(0xF2); writeData(0x00);
  writeCommand(0x26); writeData(0x01);
  writeCommand(0xE0);
  writeData(0x0F); writeData(0x31); writeData(0x2B);
  writeData(0x0C); writeData(0x0E); writeData(0x08);
  writeData(0x4E); writeData(0xF1); writeData(0x37);
  writeData(0x07); writeData(0x10); writeData(0x03);
  writeData(0x0E); writeData(0x09); writeData(0x00);
  writeCommand(0xE1);
  writeData(0x00); writeData(0x0E); writeData(0x14);
  writeData(0x03); writeData(0x11); writeData(0x07);
  writeData(0x31); writeData(0xC1); writeData(0x48);
  writeData(0x08); writeData(0x0F); writeData(0x0C);
  writeData(0x31); writeData(0x36); writeData(0x0F);
  writeCommand(0x11); delay(120);
  writeCommand(0x29); delay(25);
}

// ========================
// DIRECT DRAW
// ========================
void drawPixel(uint16_t x, uint16_t y, uint16_t color)
{
  if(x >= SCREEN_W || y >= SCREEN_H) return;
  setAddressWindow(x, y, x, y);
  syncCounter();
  control595 |= (1 << SR_RS);
  writeBus(color >> 8);
  writeBus(color & 0xFF);
}

void fillRect(uint16_t x, uint16_t y,
              uint16_t w, uint16_t h, uint16_t color)
{
  if(x >= SCREEN_W || y >= SCREEN_H) return;
  if(x + w > SCREEN_W) w = SCREEN_W - x;
  if(y + h > SCREEN_H) h = SCREEN_H - y;

  setAddressWindow(x, y, x+w-1, y+h-1);
  syncCounter();
  control595 |= (1 << SR_RS);

  uint16_t pHi = ((uint16_t)control595 << 8) | (color >> 8);
  uint16_t pLo = ((uint16_t)control595 << 8) | (color & 0xFF);

  uint32_t count = (uint32_t)w * h;
  for(uint32_t i = 0; i < count; i++)
  {
    writeBusFast(pHi);
    writeBusFast(pLo);
  }
}

void fillScreen(uint16_t color)
{
  fillRect(0, 0, SCREEN_W, SCREEN_H, color);
}

void drawRect(uint16_t x, uint16_t y,
              uint16_t w, uint16_t h, uint16_t color)
{
  drawLine(x,     y,     x+w-1, y,     color);
  drawLine(x,     y+h-1, x+w-1, y+h-1, color);
  drawLine(x,     y,     x,     y+h-1, color);
  drawLine(x+w-1, y,     x+w-1, y+h-1, color);
}

void drawLine(int x0, int y0, int x1, int y1, uint16_t color)
{
  int dx =  abs(x1-x0), sx = x0<x1 ? 1 : -1;
  int dy = -abs(y1-y0), sy = y0<y1 ? 1 : -1;
  int err = dx+dy;
  while(true)
  {
    drawPixel(x0, y0, color);
    if(x0==x1 && y0==y1) break;
    int e2 = 2*err;
    if(e2 >= dy){ err += dy; x0 += sx; }
    if(e2 <= dx){ err += dx; y0 += sy; }
  }
}

void drawCircle(int x0, int y0, int r, uint16_t color)
{
  int x = r, y = 0, err = 0;
  while(x >= y)
  {
    drawPixel(x0+x, y0+y, color); drawPixel(x0+y, y0+x, color);
    drawPixel(x0-y, y0+x, color); drawPixel(x0-x, y0+y, color);
    drawPixel(x0-x, y0-y, color); drawPixel(x0-y, y0-x, color);
    drawPixel(x0+y, y0-x, color); drawPixel(x0+x, y0-y, color);
    y++;
    if(err <= 0) err += 2*y+1;
    else { x--; err += 2*(y-x)+1; }
  }
}

void fillCircle(int x0, int y0, int r, uint16_t color)
{
  for(int dy = -r; dy <= r; dy++)
  {
    int dx = (int)sqrt((float)(r*r - dy*dy));
    fillRect(x0-dx, y0+dy, dx*2, 1, color);
  }
}

void fastFillCircle(int x0, int y0, int r,
                    uint16_t color, uint16_t bg)
{
  int left = x0-r, top = y0-r;
  if(left < 0) left = 0;
  if(top  < 0) top  = 0;
  int right  = min(x0+r, SCREEN_W-1);
  int bottom = min(y0+r, SCREEN_H-1);

  setAddressWindow(left, top, right, bottom);
  syncCounter();
  control595 |= (1 << SR_RS);

  uint16_t pHi = ((uint16_t)control595 << 8) | (color >> 8);
  uint16_t pLo = ((uint16_t)control595 << 8) | (color & 0xFF);
  uint16_t bHi = ((uint16_t)control595 << 8) | (bg >> 8);
  uint16_t bLo = ((uint16_t)control595 << 8) | (bg & 0xFF);

  for(int dy = top; dy <= bottom; dy++)
    for(int dx = left; dx <= right; dx++)
    {
      int ddx = dx-x0, ddy = dy-y0;
      if(ddx*ddx + ddy*ddy <= r*r)
        { writeBusFast(pHi); writeBusFast(pLo); }
      else
        { writeBusFast(bHi); writeBusFast(bLo); }
    }
}

void drawChar(int x, int y, char c,
              uint16_t color, int size)
{
  if(c < 32 || c > 126) return;
  const uint8_t* glyph = font5x7[c-32];
  for(int col = 0; col < 5; col++)
  {
    uint8_t colData = glyph[col];
    for(int row = 0; row < 7; row++)
      if(colData & (1 << row))
      {
        if(size == 1) drawPixel(x+col, y+row, color);
        else fillRect(x+col*size, y+row*size, size, size, color);
      }
  }
}

void printText(int x, int y, String text,
               uint16_t color, int size)
{
  int curX = x;
  for(int i = 0; i < (int)text.length(); i++)
  {
    drawChar(curX, y, text[i], color, size);
    curX += (5+1)*size;
  }
}

// ========================
// SD IMAGE (4-bit SDMMC)
// ========================
volatile bool sdBusy = false;

bool drawImageSD(const char* filename, int x, int y)
{
  sdBusy = true;

  File f = SD_MMC.open(filename, FILE_READ);
  if(!f) { 
    Serial.print("SD open failed: "); 
    Serial.println(filename); 
    sdBusy = false;
    return false; 
  }

  uint16_t w = (f.read() << 8) | f.read();
  uint16_t h = (f.read() << 8) | f.read();

  setAddressWindow(x, y, x + w - 1, y + h - 1);
  syncCounter();
  
  control595 |= (1 << SR_RS);
  uint8_t ctrl = control595;

  const uint32_t READ_CHUNK = 2048;
  static uint8_t readBuf[READ_CHUNK];
  static uint8_t txBuf[READ_CHUNK * 2];

  uint32_t totalBytes = (uint32_t)w * h * 2;
  uint32_t bytesDone = 0;

  while(bytesDone < totalBytes)
  {
    uint32_t chunk = min((uint32_t)READ_CHUNK, totalBytes - bytesDone);
    size_t r = f.read(readBuf, chunk);
    if(r == 0) break; 

    for(uint32_t i = 0; i < r; i++)
    {
      txBuf[i * 2]     = ctrl;
      txBuf[i * 2 + 1] = readBuf[i];
    }

    hSPI.writeBytes(txBuf, r * 2);
    bytesDone += r;
    yield(); 
  }

  f.close();
  sdBusy = false;
  return true;
}

bool loadImageFromSD(const char* filename, int x, int y)
{
  #define IMG_HALF 38400
  sdBusy = true;

  File f = SD_MMC.open(filename, FILE_READ);
  if(!f)
  {
    Serial.println("SD open failed!");
    sdBusy = false;
    return false;
  }

  uint16_t w = (f.read() << 8) | f.read();
  uint16_t h = (f.read() << 8) | f.read();
  Serial.print("Image: "); Serial.print(w);
  Serial.print("x");       Serial.println(h);

  uint16_t* buf0 = (uint16_t*)malloc(IMG_HALF * sizeof(uint16_t));
  uint16_t* buf1 = (uint16_t*)malloc(IMG_HALF * sizeof(uint16_t));

  if(!buf0 || !buf1)
  {
    Serial.println("Malloc failed!");
    if(buf0) free(buf0);
    if(buf1) free(buf1);
    f.close();
    sdBusy = false;
    return false;
  }

  uint8_t* raw0   = (uint8_t*)buf0;
  uint32_t toRead = IMG_HALF * 2, done = 0;
  while(done < toRead)
  {
    uint32_t chunk = min((uint32_t)512, toRead - done);
    f.read(raw0 + done, chunk);
    done += chunk;
  }

  uint8_t* raw1 = (uint8_t*)buf1;
  toRead = IMG_HALF * 2; done = 0;
  while(done < toRead)
  {
    uint32_t chunk = min((uint32_t)512, toRead - done);
    f.read(raw1 + done, chunk);
    done += chunk;
  }

  f.close();

  setAddressWindow(x, y, x+w-1, y+(h/2)-1);
  syncCounter();
  control595 |= (1 << SR_RS);
  uint16_t ctrl = (uint16_t)control595 << 8;
  for(uint32_t i = 0; i < IMG_HALF; i++)
  {
    uint16_t c = buf0[i];
    hSPI.write16(ctrl | (c & 0xFF));
    hSPI.write16(ctrl | (c >> 8));
  }

  setAddressWindow(x, y+(h/2), x+w-1, y+h-1);
  syncCounter();
  control595 |= (1 << SR_RS);
  ctrl = (uint16_t)control595 << 8;
  for(uint32_t i = 0; i < IMG_HALF; i++)
  {
    uint16_t c = buf1[i];
    hSPI.write16(ctrl | (c & 0xFF));
    hSPI.write16(ctrl | (c >> 8));
  }

  free(buf0);
  free(buf1);
  Serial.println("Done!");
  sdBusy = false;
  return true;
}

void playAnimation(const char* filename)
{
  sdBusy = true;

  File f = SD_MMC.open(filename, FILE_READ);
  if(!f) { sdBusy = false; return; }

  uint16_t w          = (f.read() << 8) | f.read();
  uint16_t h          = (f.read() << 8) | f.read();
  uint16_t frameCount = (f.read() << 8) | f.read();

  for(uint16_t frame = 0; frame < frameCount; frame++)
  {
    uint16_t marker = (f.read() << 8) | f.read();

    if(marker == 0xFFFF)
    {
      setAddressWindow(0, 0, w-1, h-1);
      syncCounter();
      control595 |= (1 << SR_RS);
      uint8_t ctrl = control595;

      for(int row = 0; row < h; row += ROWS_PER_CHUNK)
      {
        int      rows  = min(ROWS_PER_CHUNK, (int)h - row);
        uint32_t bytes = (uint32_t)rows * w * 2;

        uint32_t done = 0;
        while(done < bytes)
        {
          size_t r = f.read(rowBuf + done, min(bytes - done, (uint32_t)512));
          if(r == 0) break; 
          done += r;
        }

        for(uint32_t i = 0; i < bytes; i++)
        {
          txBuf[i * 2]     = ctrl;
          txBuf[i * 2 + 1] = rowBuf[i];
        }

        hSPI.writeBytes(txBuf, bytes * 2);
        yield();
      }
    }
    else
    {
      uint16_t spanCount  = marker;
      uint32_t bufPos     = 0;
      uint16_t batchSpans = 0;
      uint16_t spansDone  = 0;
      uint8_t  header[5];

      while(spansDone < spanCount)
      {
        if(f.read(header, 5) != 5) break; 
        
        uint8_t  xh  = header[0], xl = header[1];
        uint8_t  yh  = header[2], yl = header[3];
        uint8_t  len = header[4];
        uint32_t spanBytes = 5 + (uint32_t)len * 2;

        if(bufPos + spanBytes > BATCH_SIZE && batchSpans > 0)
        {
          uint32_t readPos = 0;
          for(uint16_t s = 0; s < batchSpans; s++)
          {
            uint16_t bsx = ((uint16_t)batchBuf[readPos] << 8) | batchBuf[readPos + 1];
            uint16_t bsy = ((uint16_t)batchBuf[readPos + 2] << 8) | batchBuf[readPos + 3];
            uint8_t blen = batchBuf[readPos + 4];
            readPos += 5;

            setAddressWindow(bsx, bsy, bsx + blen - 1, bsy);
            syncCounter();
            control595 |= (1 << SR_RS);
            uint8_t ctrl = control595;

            uint32_t pixBytes = (uint32_t)blen * 2;
            for(uint32_t i = 0; i < pixBytes; i++) {
              txBuf[i * 2]     = ctrl;
              txBuf[i * 2 + 1] = batchBuf[readPos + i];
            }
            hSPI.writeBytes(txBuf, pixBytes * 2);
            readPos += pixBytes;
          }
          bufPos     = 0;
          batchSpans = 0;
          yield();
        }

        batchBuf[bufPos]     = xh;
        batchBuf[bufPos + 1] = xl;
        batchBuf[bufPos + 2] = yh;
        batchBuf[bufPos + 3] = yl;
        batchBuf[bufPos + 4] = len;
        bufPos += 5;

        uint32_t pixBytes = (uint32_t)len * 2;
        uint32_t done = 0;
        while(done < pixBytes)
        {
          size_t r = f.read(batchBuf + bufPos + done, min(pixBytes - done, (uint32_t)512));
          if(r == 0) break;
          done += r;
        }
        bufPos += pixBytes;
        batchSpans++;
        spansDone++;
      }

      if(batchSpans > 0)
      {
        uint32_t readPos = 0;
        for(uint16_t s = 0; s < batchSpans; s++)
        {
          uint16_t bsx = ((uint16_t)batchBuf[readPos] << 8) | batchBuf[readPos + 1];
          uint16_t bsy = ((uint16_t)batchBuf[readPos + 2] << 8) | batchBuf[readPos + 3];
          uint8_t blen = batchBuf[readPos + 4];
          readPos += 5;

          setAddressWindow(bsx, bsy, bsx + blen - 1, bsy);
          syncCounter();
          control595 |= (1 << SR_RS);
          uint8_t ctrl = control595;

          uint32_t pixBytes = (uint32_t)blen * 2;
          for(uint32_t i = 0; i < pixBytes; i++) {
            txBuf[i * 2]     = ctrl;
            txBuf[i * 2 + 1] = batchBuf[readPos + i];
          }
          hSPI.writeBytes(txBuf, pixBytes * 2);
          readPos += pixBytes;
        }
      }
    }
        
    setAddressWindow(0, 0, 319, 239);
  }
  f.close();
  sdBusy = false;
}


// ========================
// PROPORTIONAL BITMAP FONTS (Oxanium)
// ========================
static uint8_t getGlyphByte(const uint8_t* data, uint16_t offset, uint8_t bytesPerRow, uint8_t row, uint8_t byteIdx)
{
  return pgm_read_byte(&data[offset + row * bytesPerRow + byteIdx]);
}

void drawCharF(int x, int y, char c, uint16_t color, FontID font)
{
  if(c < 32 || c > 126) return;
  uint8_t idx = c - 32;

  const GlyphInfo* glyphs;
  const uint8_t* data;
  uint8_t height;

  if(font == FONT_SMALL) {
    glyphs = fontSmall_glyphs;
    data   = fontSmall_data;
    height = FONTSMALL_HEIGHT;
  } else {
    glyphs = fontLarge_glyphs;
    data   = fontLarge_data;
    height = FONTLARGE_HEIGHT;
  }

  GlyphInfo g;
  memcpy_P(&g, &glyphs[idx], sizeof(GlyphInfo));

  for(uint8_t row = 0; row < height; row++)
  {
    for(uint8_t col = 0; col < g.width; col++)
    {
      uint8_t byteIdx = col / 8;
      uint8_t bitIdx  = 7 - (col % 8);
      uint8_t byteVal = getGlyphByte(data, g.dataOffset, g.bytesPerRow, row, byteIdx);
      if(byteVal & (1 << bitIdx))
        drawPixel(x + col, y + row, color);
    }
  }
}

int printTextF(int x, int y, String text, uint16_t color, FontID font)
{
  const GlyphInfo* glyphs = (font == FONT_SMALL) ? fontSmall_glyphs : fontLarge_glyphs;
  int curX = x;

  for(int i = 0; i < (int)text.length(); i++)
  {
    char c = text[i];
    if(c < 32 || c > 126) continue;
    uint8_t idx = c - 32;

    GlyphInfo g;
    memcpy_P(&g, &glyphs[idx], sizeof(GlyphInfo));

    drawCharF(curX, y, c, color, font);
    curX += g.width + 1;  // +1px spacing between characters
  }
  return curX - x;  // total width drawn, useful for centering
}

int textWidthF(String text, FontID font)
{
  const GlyphInfo* glyphs = (font == FONT_SMALL) ? fontSmall_glyphs : fontLarge_glyphs;
  int total = 0;

  for(int i = 0; i < (int)text.length(); i++)
  {
    char c = text[i];
    if(c < 32 || c > 126) continue;
    GlyphInfo g;
    memcpy_P(&g, &glyphs[c - 32], sizeof(GlyphInfo));
    total += g.width + 1;
  }
  return total;
}