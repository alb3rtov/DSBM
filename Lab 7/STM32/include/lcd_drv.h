
#define __RGB_LCD_H__

#include <inttypes.h>

// Device I2C Arress
#define RGB (0x62<<1)
#define LCD (0x3E<<1)

// color define 
#define WHITE           0
#define RED             1
#define GREEN           2
#define BLUE            3

#define REG_RED         0x04        // pwm2
#define REG_GREEN       0x03        // pwm1
#define REG_BLUE        0x02        // pwm0

#define REG_MODE1       0x00
#define REG_MODE2       0x01
#define REG_OUTPUT      0x08

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00


uint8_t charsize = LCD_5x8DOTS;
void begin(uint8_t cols, uint8_t rows, uint8_t charsize );

void clear();
void home();

void noDisplay();
void display();
void noBlink();
void blink();
void noCursor();
void cursor();
void scrollDisplayLeft();
void scrollDisplayRight();
void leftToRight();
void rightToLeft();
void autoscroll();
void noAutoscroll();

void createChar(uint8_t, uint8_t[]);
void setCursor(uint8_t, uint8_t); 
  
size_t write(char  value[]);
void command(uint8_t);
void setReg(unsigned char addr, unsigned char dta);

// color control
void setRGB(unsigned char r, unsigned char g, unsigned char b);               // set rgb
void setPWM(unsigned char color, unsigned char pwm){setReg(color, pwm);}      // set pwm
  
void setColor(unsigned char color);
void setColorAll(){setRGB(0, 0, 0);}
void setColorWhite(){setRGB(0, 0, 0);}
void setColorGreen(){setRGB(0,255,0);}
void setColorBlue(){setRGB(0,0,255);}
void setColorRed(){setRGB(255,0,0);}
void setColorYellow(){setRGB(255, 233, 0);}

// blink the LED backlight
void blinkLED(void);
void noBlinkLED(void);

void send(uint8_t, uint8_t);
  
uint8_t _displayfunction;
uint8_t _displaycontrol;
uint8_t _displaymode;
uint8_t _initialized;
uint8_t _numlines,_currline;