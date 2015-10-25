#ifndef ILI9325i2c_16_h
#define ILI9325i2c_16_h

#if defined(__AVR__)
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#endif

#define XSIZE 239
#define YSIZE 319

#define LEFT 0
#define RIGHT 9999
#define CENTER 9998

#define PORTRAIT 0
#define LANDSCAPE 1

#define fontbyte(x) pgm_read_byte(&cfont.font[x])
#define swap(type, i, j) {type t = i; i = j; j = t;}


// define ILL9325
#define IODIRA 0x00
#define IODIRB 0x01
#define PRTA 0x12
#define PRTB 0x13

struct currentFont
{
  uint8_t* font;
  uint8_t x_size;
  uint8_t y_size;
  uint8_t offset;
  uint8_t numchars;
};
enum Display {
  
  SS2_4 = 0,
  SS2_8 = 1
  
  
};


class ILI9325i2c_16 {

  public:
    ILI9325i2c_16();
    ILI9325i2c_16(byte addrs, byte rs, byte wr, byte rst, byte cs);
    ILI9325i2c_16(byte addrs);
    void initTFT();
    void print(char *st, int x, int y, int deg = 0);
    void print(String st, int x, int y, int deg = 0);
    void setColor(byte r, byte g, byte b);
    void setBackColor(byte r, byte g, byte b);
    void setFont(uint8_t* font);
    void clrScr();
    void drawRect(int x1, int y1, int x2, int y2);
    void drawLine(int x1, int y1, int x2, int y2);
    void drawPixel(int x, int y);
    void drawCircle(int x, int y, int radius);
    void fillCircle(int x, int y, int radius);
    void fillRoundRect(int x1, int y1, int x2, int y2);
    void fillRect(int x1, int y1, int x2, int y2);
    void drawRoundRect(int x1, int y1, int x2, int y2);

  private:
    Display displayModel;
    byte fcolorr, fcolorg, fcolorb;
    byte bcolorr, bcolorg, bcolorb;
    byte regAValue = 0x00;
    byte address;
    byte orient;
    byte RS = 0;
    byte WR;
    byte RST;
    byte CS;
    void setPixel(byte r, byte g, byte b);
    void drawHLine(int x, int y, int l);
    void drawVLine(int x, int y, int l);
    void printChar(byte c, int x, int y);
    void setXY(word x1, word y1, word x2, word y2);
    void clrXY();
    void rotateChar(byte c, int x, int y, int pos, int deg);
    void writeComData(byte com, word data);
    void writeData(byte vh, byte vl);
    void writeCom(byte com);
    void writeBus(byte vh, byte vl);
    void sendData(byte dataa, byte datab);
    void sendData(byte data);
    void setBit(byte portMask);
    void clearBit(byte portMask);
    void pulseBitLow(byte pin);
    void setTFTProperties();
    currentFont  cfont;
};

#endif
