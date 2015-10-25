/* Sain Smart 2.8 TFT w/ILI9325 driver
 *  Connected to a MCP23017 i2c 16 input/output port expander.
 *
 * ***   Pin Out  ***
 * MCP23017   ILI9325 TFT
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
 *  RD > 3.3v
 *  VCC > 5v
 *  GND > GND
 *  LED_A > 5v
 *
 *  UNO       ILI9325 TFT
 * ------------------------
 *  RS        RS
 *  WR        WR
 *  RST       RST
 *  CS        CS
 *
 */


#include <Wire.h>
#include "ILI9325i2c_16.h"
#define ADDRESS 0x21  // i2c address
//#define ADDRESS 0x20
extern uint8_t SmallFont[];

// use these pins if using 2.8 TFT
#define RSPIN 2
#define WRPIN 3
#define RSTPIN 4
#define CSPIN 5

ILI9325i2c_16  tft(ADDRESS, RSPIN, WRPIN, RSTPIN, CSPIN); // constructor for the Sain Smart 2.8 TFT - Runs in 16 bit mode
//ILI9325i2c_16  tft(ADDRESS); // // constructor for the Sain Smart 2.4 TFT - Runs in 8 but mode

void setup() {

  tft.initTFT();
  tft.setBackColor(0, 0, 0);
  tft.clrScr();
  tft.setFont(SmallFont);
  tft.print("HELLO WORLD!", 0, 0);
}

void loop() {
  // put your main code here, to run repeatedly:

}
