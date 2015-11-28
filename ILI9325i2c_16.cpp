/*
 *  Author: PhillyNJ
 *
 * The ILI9325i2c_16 class is a modified version of the UTFT Library
 * http://www.rinkydinkelectronics.com/library.php?id=51
 *
 * The ILI9325i2c_16 class drives the ILI9325 TFT driver with a
 * MCP23017 i2c 16 input/output port expander.
 *
 * Much of the code was ported over from the UTFT library and was tested
 * with the SainSmart 2.8" & 2.4" TFT LCD
 *
 * Note: Driving a TFT with i2c is extremely slow. Painfully slow. I ported
 * this code as a learning exercise.
 *
 *      Pin Out
 * **************************
 *  MCP23017  ILI9325 TFT 2.4
 *  GPA0      RS     
 *  GPA1      WR
 *  GPA2      RST
 *  GPA3      CS
 *  
 *  GPB0      DB8     
 *  GPB1      DB9
 *  GPB2      DB10
 *  GPB3      DB11
 *  GPB4      DB12
 *  GPB5      DB13
 *  GPB6      DB14
 *  GPB7      DB15
 *  
 * MCP23017   ILI9325 TFT 2.8
 * ------------------------
 *  GPA0      DB0
 *  GPA1      DB1
 *  GPA2      DB2
 *  GPA3      DB3
 *  GPA4      DB4
 *  GPA5      DB5
 *  GPA6      DB6
 *  GPA7      DB7
 *
 *  GPB0      DB8
 *  GPB1      DB9
 *  GPB2      DB10
 *  GPB3      DB11
 *  GPB4      DB12
 *  GPB5      DB13
 *  GPB6      DB14
 *  GPB7      DB15
 *
 *  MCP23017  TFT (all)   Uno
 *  RESET                 5V
 *  A1                    GND
 *  A2                    GND
 *  A3                    GND
 *  VDD       VCC,LEDA    5v
 *            RD          3.3v
 *  VSS       GND         GND
 *  SCL                   A5
 *  SDA                   A4
 */

#include "ILI9325i2c_16.h"
#include <Wire.h>

ILI9325i2c_16::ILI9325i2c_16() {

}

ILI9325i2c_16::ILI9325i2c_16(byte addrs, byte rs, byte wr, byte rst, byte cs) {

  RS = rs;
  WR = wr;
  RST = rst;
  CS = cs;
  address = addrs;
  displayModel = SS2_8;

}
ILI9325i2c_16::ILI9325i2c_16(byte addrs) {

  address = addrs;
  displayModel = SS2_4;

}
void ILI9325i2c_16::initTFT() {

  if (displayModel == SS2_8) {

    pinMode(RS, OUTPUT);
    pinMode(WR, OUTPUT);
    pinMode(RST, OUTPUT);
    pinMode(CS, OUTPUT);
  } else {

    RS = 0x01;
    WR = 0x02;
    RST = 0x04;
    CS = 0x08;
  }


  Wire.begin();

  Wire.beginTransmission(address);
  Wire.write(IODIRA); // Select IODIRA Register
  Wire.write(0x00); // set directions to output
  Wire.endTransmission();

  Wire.beginTransmission(address);
  Wire.write(IODIRB); // Select IODIRB Register
  Wire.write(0x00); // set directions to output
  Wire.endTransmission();

  // clear prt b
  Wire.beginTransmission(address);
  Wire.write(PRTB); // Select IODIRB Register
  Wire.write(0x00); // set directions to output
  Wire.endTransmission();
  //orient = LANDSCAPE;

  setBit(RST);

  delay(5);
  clearBit(RST);

  delay(5);
  setBit(RST);

  clearBit(CS);
  setTFTProperties();

}

void ILI9325i2c_16::writeComData(byte com, word data) {

  writeCom(com);
  writeData(data >> 8, data );

}

void ILI9325i2c_16::writeData(byte vh, byte vl) {

  setBit(RS);
  writeBus(vh, vl);

}

void ILI9325i2c_16::writeCom(byte com) {

  clearBit(RS);
  writeBus(0x00, com);

}

void ILI9325i2c_16::writeBus(byte vh, byte vl) {
  
  if (displayModel == SS2_8) {
  
    sendData(vh, vl);
    pulseBitLow(WR);
  
  } else {

    sendData(vh);
    pulseBitLow(WR);
    sendData(vl);
    pulseBitLow(WR);
  }


}

void ILI9325i2c_16::sendData(byte dataa, byte datab) {

  // send data
  Wire.beginTransmission(address);
  Wire.write(PRTA); // select PORTA
  Wire.write(datab); // send value data
  Wire.endTransmission();

  Wire.beginTransmission(address);
  Wire.write(PRTB); // select PORTB
  Wire.write(dataa); // send value data
  Wire.endTransmission();

}


void ILI9325i2c_16::sendData(byte data) {

  // send data
  Wire.beginTransmission(address);
  Wire.write(PRTB); // select PORTB
  Wire.write(data); // send value data
  Wire.endTransmission();

}


void ILI9325i2c_16::setBit(byte pin) {
  if (displayModel == SS2_8) {
    digitalWrite(pin, HIGH);
  } else {
    Wire.beginTransmission(address);
    Wire.write(PRTA); // select PORTA
    regAValue =  (regAValue | pin);
    Wire.write(regAValue); // send value 1
    Wire.endTransmission();

  }
}

void ILI9325i2c_16::clearBit(byte pin) {

  if (displayModel == SS2_8) {
    digitalWrite(pin, LOW);
  } else {
    Wire.beginTransmission(address);
    Wire.write(PRTA); // select PORTA
    regAValue &=  ~(0 ^ pin);//~portMask;
    Wire.write(regAValue); // send value 1
    Wire.endTransmission();


  }

}


void ILI9325i2c_16::pulseBitLow(byte pin) {

  clearBit(pin);
 // delay(5);
  setBit(pin);
}

void ILI9325i2c_16::print(String st, int x, int y, int deg)
{
  char buf[st.length() + 1];

  st.toCharArray(buf, st.length() + 1);
  print(buf, x, y, deg);
}

void ILI9325i2c_16::print(char *st, int x, int y, int deg)
{
  int stl, i;

  stl = strlen(st);

  if (orient == PORTRAIT)
  {
    if (x == RIGHT)
      x = (XSIZE + 1) - (stl * cfont.x_size);
    if (x == CENTER)
      x = ((XSIZE + 1) - (stl * cfont.x_size)) / 2;
  }
  else
  {
    if (x == RIGHT)
      x = (YSIZE + 1) - (stl * cfont.x_size);
    if (x == CENTER)
      x = ((YSIZE + 1) - (stl * cfont.x_size)) / 2;
  }

  for (i = 0; i < stl; i++)
    if (deg == 0)
      printChar(*st++, x + (i * (cfont.x_size)), y);
    else
      rotateChar(*st++, x, y, i, deg);
}



void ILI9325i2c_16::printChar(byte c, int x, int y)
{
  byte i, ch;
  word j;
  word temp;

  clearBit(CS);

  if (orient == PORTRAIT)
  {
    setXY(x, y, x + cfont.x_size - 1, y + cfont.y_size - 1);

    temp = ((c - cfont.offset) * ((cfont.x_size / 8) * cfont.y_size)) + 4;

    for (j = 0; j < ((cfont.x_size / 8)*cfont.y_size); j++)
    {
      ch = pgm_read_byte(&cfont.font[temp]);
      for (i = 0; i < 8; i++)
      {
        if ((ch & (1 << (7 - i))) != 0)
        {
          setPixel(fcolorr, fcolorg, fcolorb);
        }
        else
        {
          setPixel(bcolorr, bcolorg, bcolorb);
        }
      }
      temp++;
    }
  }
  else
  {
    temp = ((c - cfont.offset) * ((cfont.x_size / 8) * cfont.y_size)) + 4;

    for (j = 0; j < ((cfont.x_size / 8)*cfont.y_size); j += (cfont.x_size / 8))
    {
      setXY(x, y + (j / (cfont.x_size / 8)), x + cfont.x_size - 1, y + (j / (cfont.x_size / 8)));
      for (int zz = (cfont.x_size / 8) - 1; zz >= 0; zz--)
      {
        ch = pgm_read_byte(&cfont.font[temp + zz]);
        for (i = 0; i < 8; i++)
        {
          if ((ch & (1 << i)) != 0)
          {
            setPixel(fcolorr, fcolorg, fcolorb);
          }
          else
          {
            setPixel(bcolorr, bcolorg, bcolorb);
          }
        }
      }
      temp += (cfont.x_size / 8);
    }
  }
  setBit(CS);
  clrXY();
}


void ILI9325i2c_16::rotateChar(byte c, int x, int y, int pos, int deg)
{
  byte i, j, ch;
  word temp;
  int newx, newy;
  double radian;
  radian = deg * 0.0175;

  clearBit(CS);

  temp = ((c - cfont.offset) * ((cfont.x_size / 8) * cfont.y_size)) + 4;
  for (j = 0; j < cfont.y_size; j++)
  {
    for (int zz = 0; zz < (cfont.x_size / 8); zz++)
    {
      ch = pgm_read_byte(&cfont.font[temp + zz]);
      for (i = 0; i < 8; i++)
      {
        newx = x + (((i + (zz * 8) + (pos * cfont.x_size)) * cos(radian)) - ((j) * sin(radian)));
        newy = y + (((j) * cos(radian)) + ((i + (zz * 8) + (pos * cfont.x_size)) * sin(radian)));

        setXY(newx, newy, newx + 1, newy + 1);

        if ((ch & (1 << (7 - i))) != 0)
        {
          setPixel(fcolorr, fcolorg, fcolorb);
        }
        else
        {
          setPixel(bcolorr, bcolorg, bcolorb);
        }
      }
    }
    temp += (cfont.x_size / 8);
  }

  setBit(CS);
  clrXY();
}


void ILI9325i2c_16::setPixel(byte r, byte g, byte b)
{
  writeData(((r & 248) | g >> 5), ((g & 28) << 3 | b >> 3));
}
void ILI9325i2c_16::setColor(byte r, byte g, byte b)
{
  fcolorr = r;
  fcolorg = g;
  fcolorb = b;
}

void ILI9325i2c_16::setBackColor(byte r, byte g, byte b)
{
  bcolorr = r;
  bcolorg = g;
  bcolorb = b;
}

void ILI9325i2c_16::setFont(uint8_t* font)
{
  cfont.font = font;
  cfont.x_size = fontbyte(0);
  cfont.y_size = fontbyte(1);
  cfont.offset = fontbyte(2);
}

void ILI9325i2c_16::clrScr()
{

  long i;

  clearBit(CS);

  clrXY();

  setBit(RS);
  long pixels = 76800;
  for (i = 0; i < pixels; i++)
  {

    writeBus(0, 0);

  }

  setBit(CS);

}

void ILI9325i2c_16::clrXY()
{
  if (orient == PORTRAIT)
    setXY(0, 0, XSIZE, YSIZE);
  else
    setXY(0, 0, YSIZE, XSIZE);
}

void ILI9325i2c_16::setXY(word x1, word y1, word x2, word y2)
{
  int tmp;

  if (orient == LANDSCAPE)
  {
    swap(word, x1, y1);
    swap(word, x2, y2)
    y1 = YSIZE - y1;
    y2 = YSIZE - y2;
    swap(word, y1, y2)
  }

  writeComData(0x20, x1);
  writeComData(0x21, y1);
  writeComData(0x50, x1);
  writeComData(0x52, y1);
  writeComData(0x51, x2);
  writeComData(0x53, y2);
  writeCom(0x22);



}

void ILI9325i2c_16::drawRect(int x1, int y1, int x2, int y2)
{
  int tmp;

  if (x1 > x2)
  {
    swap(int, x1, x2);
  }
  if (y1 > y2)
  {
    swap(int, y1, y2);
  }

  drawHLine(x1, y1, x2 - x1);
  drawHLine(x1, y2, x2 - x1);
  drawVLine(x1, y1, y2 - y1);
  drawVLine(x2, y1, y2 - y1);
}

void ILI9325i2c_16::drawPixel(int x, int y)
{
  clearBit(CS);
  setXY(x, y, x, y);
  setPixel(fcolorr, fcolorg, fcolorb);
  setBit(CS);
  clrXY();
}

void ILI9325i2c_16::drawCircle(int x, int y, int radius)
{
  int f = 1 - radius;
  int ddF_x = 1;
  int ddF_y = -2 * radius;
  int x1 = 0;
  int y1 = radius;
  char ch, cl;

  ch = ((fcolorr & 248) | fcolorg >> 5);
  cl = ((fcolorg & 28) << 3 | fcolorb >> 3);

  clearBit(CS);
  setXY(x, y + radius, x, y + radius);
  writeData(ch, cl);
  setXY(x, y - radius, x, y - radius);
  writeData(ch, cl);
  setXY(x + radius, y, x + radius, y);
  writeData(ch, cl);
  setXY(x - radius, y, x - radius, y);
  writeData(ch, cl);

  while (x1 < y1)
  {
    if (f >= 0)
    {
      y1--;
      ddF_y += 2;
      f += ddF_y;
    }
    x1++;
    ddF_x += 2;
    f += ddF_x;
    setXY(x + x1, y + y1, x + x1, y + y1);
    writeData(ch, cl);
    setXY(x - x1, y + y1, x - x1, y + y1);
    writeData(ch, cl);
    setXY(x + x1, y - y1, x + x1, y - y1);
    writeData(ch, cl);
    setXY(x - x1, y - y1, x - x1, y - y1);
    writeData(ch, cl);
    setXY(x + y1, y + x1, x + y1, y + x1);
    writeData(ch, cl);
    setXY(x - y1, y + x1, x - y1, y + x1);
    writeData(ch, cl);
    setXY(x + y1, y - x1, x + y1, y - x1);
    writeData(ch, cl);
    setXY(x - y1, y - x1, x - y1, y - x1);
    writeData(ch, cl);
  }
  setBit(CS);
  clrXY();
}

void ILI9325i2c_16::drawRoundRect(int x1, int y1, int x2, int y2)
{
  int tmp;

  if (x1 > x2)
  {
    swap(int, x1, x2);
  }
  if (y1 > y2)
  {
    swap(int, y1, y2);
  }
  if ((x2 - x1) > 4 && (y2 - y1) > 4)
  {
    drawPixel(x1 + 1, y1 + 1);
    drawPixel(x2 - 1, y1 + 1);
    drawPixel(x1 + 1, y2 - 1);
    drawPixel(x2 - 1, y2 - 1);
    drawHLine(x1 + 2, y1, x2 - x1 - 4);
    drawHLine(x1 + 2, y2, x2 - x1 - 4);
    drawVLine(x1, y1 + 2, y2 - y1 - 4);
    drawVLine(x2, y1 + 2, y2 - y1 - 4);
  }
}

void ILI9325i2c_16::fillRect(int x1, int y1, int x2, int y2)
{
  int tmp;

  if (x1 > x2)
  {
    swap(int, x1, x2);
  }
  if (y1 > y2)
  {
    swap(int, y1, y2);
  }

  if (orient == PORTRAIT)
  {
    for (int i = 0; i < ((y2 - y1) / 2) + 1; i++)
    {
      drawHLine(x1, y1 + i, x2 - x1);
      drawHLine(x1, y2 - i, x2 - x1);
    }
  }
  else
  {
    for (int i = 0; i < ((x2 - x1) / 2) + 1; i++)
    {
      drawVLine(x1 + i, y1, y2 - y1);
      drawVLine(x2 - i, y1, y2 - y1);
    }
  }
}

void ILI9325i2c_16::fillRoundRect(int x1, int y1, int x2, int y2)
{
  int tmp;

  if (x1 > x2)
  {
    swap(int, x1, x2);
  }
  if (y1 > y2)
  {
    swap(int, y1, y2);
  }

  if ((x2 - x1) > 4 && (y2 - y1) > 4)
  {
    for (int i = 0; i < ((y2 - y1) / 2) + 1; i++)
    {
      switch (i)
      {
        case 0:
          drawHLine(x1 + 2, y1 + i, x2 - x1 - 4);
          drawHLine(x1 + 2, y2 - i, x2 - x1 - 4);
          break;
        case 1:
          drawHLine(x1 + 1, y1 + i, x2 - x1 - 2);
          drawHLine(x1 + 1, y2 - i, x2 - x1 - 2);
          break;
        default:
          drawHLine(x1, y1 + i, x2 - x1);
          drawHLine(x1, y2 - i, x2 - x1);
      }
    }
  }
}

void ILI9325i2c_16::fillCircle(int x, int y, int radius)
{
  clearBit(CS);
  for (int y1 = -radius; y1 <= radius; y1++)
    for (int x1 = -radius; x1 <= radius; x1++)
      if (x1 * x1 + y1 * y1 <= radius * radius)
      {
        setXY(x + x1, y + y1, x + x1, y + y1);
        writeData(((fcolorr & 248) | fcolorg >> 5), ((fcolorg & 28) << 3 | fcolorb >> 3));
      }
  setBit(CS);
  clrXY();
}


void ILI9325i2c_16::drawLine(int x1, int y1, int x2, int y2)
{
  int tmp;
  double delta, tx, ty;
  double m, b, dx, dy;
  char ch, cl;

  ch = ((fcolorr & 248) | fcolorg >> 5);
  cl = ((fcolorg & 28) << 3 | fcolorb >> 3);

  if (((x2 - x1) < 0))
  {
    swap(int, x1, x2);
    swap(int, y1, y2);
  }
  if (((y2 - y1) < 0))
  {
    swap(int, x1, x2);
    swap(int, y1, y2);
  }

  if (y1 == y2)
  {
    if (x1 > x2)
    {
      swap(int, x1, x2);
    }
    drawHLine(x1, y1, x2 - x1);
  }
  else if (x1 == x2)
  {
    if (y1 > y2)
    {
      swap(int, y1, y2);
    }
    drawVLine(x1, y1, y2 - y1);
  }
  else if (abs(x2 - x1) > abs(y2 - y1))
  {
    clearBit(CS);
    delta = (double(y2 - y1) / double(x2 - x1));
    ty = double(y1);
    if (x1 > x2)
    {
      for (int i = x1; i >= x2; i--)
      {
        setXY(i, int(ty + 0.5), i, int(ty + 0.5));
        writeData(ch, cl);
        ty = ty - delta;
      }
    }
    else
    {
      for (int i = x1; i <= x2; i++)
      {
        setXY(i, int(ty + 0.5), i, int(ty + 0.5));
        writeData(ch, cl);
        ty = ty + delta;
      }
    }
    setBit(CS);
  }
  else
  {
    clearBit(CS);
    delta = (float(x2 - x1) / float(y2 - y1));
    tx = float(x1);
    if (y1 > y2)
    {
      for (int i = y2 + 1; i > y1; i--)
      {
        setXY(int(tx + 0.5), i, int(tx + 0.5), i);
        writeData(ch, cl);
        tx = tx + delta;
      }
    }
    else
    {
      for (int i = y1; i < y2 + 1; i++)
      {
        setXY(int(tx + 0.5), i, int(tx + 0.5), i);
        writeData(ch, cl);
        tx = tx + delta;
      }
    }
    setBit(CS);
  }

  clrXY();
}

void ILI9325i2c_16::drawHLine(int x, int y, int l)
{
  char ch, cl;

  ch = ((fcolorr & 248) | fcolorg >> 5);
  cl = ((fcolorg & 28) << 3 | fcolorb >> 3);

  clearBit(CS);
  setXY(x, y, x + l, y);
  for (int i = 0; i < l + 1; i++)
  {
    writeData(ch, cl);
  }
  setBit(CS);
  clrXY();
}


void ILI9325i2c_16::drawVLine(int x, int y, int l)
{
  char ch, cl;

  ch = ((fcolorr & 248) | fcolorg >> 5);
  cl = ((fcolorg & 28) << 3 | fcolorb >> 3);

  clearBit(CS);
  setXY(x, y, x, y + l);
  for (int i = 0; i < l; i++)
  {
    writeData(ch, cl);
  }
  setBit(CS);
  clrXY();
}


void ILI9325i2c_16::setTFTProperties() {

  writeComData(0xE5, 0x78F0);
  writeComData(0x01, 0x0100);
  writeComData(0x02, 0x0200); // set 1 line inversion
  writeComData(0x03, 0x1030); // set GRAM write direction and BGR=1.
  writeComData(0x04, 0x0000); // Resize register
  writeComData(0x08, 0x0207); // set the back porch and front porch
  writeComData(0x09, 0x0000); // set non-display area refresh cycle ISC[3:0]
  writeComData(0x0A, 0x0000); // FMARK function
  writeComData(0x0C, 0x0000); // RGB interface setting
  writeComData(0x0D, 0x0000); // Frame marker Position
  writeComData(0x0F, 0x0000); // RGB interface polarity
  //*************Power On sequence ****************//
  writeComData(0x10, 0x0000); // SAP, BT[3:0], AP, DSTB, SLP, STB
  writeComData(0x11, 0x0007); // DC1[2:0], DC0[2:0], VC[2:0]
  writeComData(0x12, 0x0000); // VREG1OUT voltage
  writeComData(0x13, 0x0000); // VDV[4:0] for VCOM amplitude
  writeComData(0x07, 0x0001);
  delay(200); // Dis-charge capacitor power voltage
  writeComData(0x10, 0x1690); // SAP, BT[3:0], AP, DSTB, SLP, STB
  writeComData(0x11, 0x0227); // Set DC1[2:0], DC0[2:0], VC[2:0]
  delay(50); // Delay 50ms
  writeComData(0x12, 0x000D); // 0012
  delay(50); // Delay 50ms
  writeComData(0x13, 0x1200); // VDV[4:0] for VCOM amplitude
  writeComData(0x29, 0x000A); // 04  VCM[5:0] for VCOMH
  writeComData(0x2B, 0x000D); // Set Frame Rate
  delay(50); // Delay 50ms
  writeComData(0x20, 0x0000); // GRAM horizontal Address
  writeComData(0x21, 0x0000); // GRAM Vertical Address
  // ----------- Adjust the Gamma Curve ----------//
  writeComData(0x30, 0x0000);
  writeComData(0x31, 0x0404);
  writeComData(0x32, 0x0003);
  writeComData(0x35, 0x0405);
  writeComData(0x36, 0x0808);
  writeComData(0x37, 0x0407);
  writeComData(0x38, 0x0303);
  writeComData(0x39, 0x0707);
  writeComData(0x3C, 0x0504);
  writeComData(0x3D, 0x0808);
  //------------------ Set GRAM area ---------------//
  writeComData(0x50, 0x0000); // Horizontal GRAM Start Address
  writeComData(0x51, 0x00EF); // Horizontal GRAM End Address
  writeComData(0x52, 0x0000); // Vertical GRAM Start Address
  writeComData(0x53, 0x013F); // Vertical GRAM Start Address
  writeComData(0x60, 0xA700); // Gate Scan Line
  writeComData(0x61, 0x0001); // NDL,VLE, REV
  writeComData(0x6A, 0x0000); // set scrolling line
  //-------------- Partial Display Control ---------//
  writeComData(0x80, 0x0000);
  writeComData(0x81, 0x0000);
  writeComData(0x82, 0x0000);
  writeComData(0x83, 0x0000);
  writeComData(0x84, 0x0000);
  writeComData(0x85, 0x0000);
  //-------------- Panel Control -------------------//
  writeComData(0x90, 0x0010);
  writeComData(0x92, 0x0000);
  writeComData(0x07, 0x0133); // 262K color and display ON

  setBit(CS);

  setColor(255, 255, 255);
  setBackColor(0, 0, 0);
  cfont.font = 0;

}
