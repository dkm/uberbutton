#include "lcd1602d.h"

#include "periph/gpio.h"
#include "xtimer.h"

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

#define LOW 0
#define HIGH 1

void lcd1602d_init_lcd(struct lcd_ctx *lcd){
  gpio_init(lcd->rs_pin, GPIO_DIR_OUT, GPIO_NOPULL);
  gpio_init(lcd->enable_pin, GPIO_DIR_OUT, GPIO_NOPULL);
  lcd->currline = 0;
  
  xtimer_usleep(50000);
  gpio_clear(lcd->rs_pin);
  gpio_clear(lcd->enable_pin);

  
  lcd1602d_write4bits(lcd, 0x03);
  xtimer_usleep(4500);
  lcd1602d_write4bits(lcd, 0x03);
  xtimer_usleep(4500);
  lcd1602d_write4bits(lcd, 0x03);
  xtimer_usleep(150);

  lcd1602d_write4bits(lcd, 0x02);
  lcd1602d_command(lcd, LCD_FUNCTIONSET | lcd->displayfunctions);

  lcd->displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
  lcd1602d_display(lcd);

  // clear it off
  lcd1602d_clear(lcd);

  // Initialize to default text direction (for romance languages)
  lcd->displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
  // set the entry mode
  lcd1602d_command(lcd, LCD_ENTRYMODESET | lcd->displaymode);
}

void lcd1602d_clear(struct lcd_ctx* lcd)
{
  lcd1602d_command(lcd, LCD_CLEARDISPLAY);  // clear display, set cursor position to zero
  xtimer_usleep(2000);  // this command takes a long time!
}

void lcd1602d_home(struct lcd_ctx* lcd)
{
  lcd1602d_command(lcd, LCD_RETURNHOME);  // set cursor position to zero
  xtimer_usleep(2000);  // this command takes a long time!
}

void lcd1602d_setCursor(struct lcd_ctx* lcd, uint8_t col, uint8_t row)
{
  int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
  if ( row >= lcd->numlines ) {
    row = lcd->numlines-1;    // we count rows starting w/0
  }
  
  lcd1602d_command(lcd, LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

// Turn the display on/off (quickly)
void lcd1602d_noDisplay(struct lcd_ctx* lcd) {
  lcd->displaycontrol &= ~LCD_DISPLAYON;
  lcd1602d_command(lcd, LCD_DISPLAYCONTROL | lcd->displaycontrol);
}
void lcd1602d_display(struct lcd_ctx* lcd) {
  lcd->displaycontrol |= LCD_DISPLAYON;
  lcd1602d_command(lcd, LCD_DISPLAYCONTROL | lcd->displaycontrol);
}

// Turns the underline cursor on/off
void lcd1602d_noCursor(struct lcd_ctx* lcd) {
  lcd->displaycontrol &= ~LCD_CURSORON;
  lcd1602d_command(lcd, LCD_DISPLAYCONTROL | lcd->displaycontrol);
}
void lcd1602d_cursor(struct lcd_ctx* lcd) {
  lcd->displaycontrol |= LCD_CURSORON;
  lcd1602d_command(lcd, LCD_DISPLAYCONTROL | lcd->displaycontrol);
}

// Turn on and off the blinking cursor
void lcd1602d_noBlink(struct lcd_ctx* lcd) {
  lcd->displaycontrol &= ~LCD_BLINKON;
  lcd1602d_command(lcd, LCD_DISPLAYCONTROL | lcd->displaycontrol);
}
void lcd1602d_blink(struct lcd_ctx* lcd) {
  lcd->displaycontrol |= LCD_BLINKON;
  lcd1602d_command(lcd, LCD_DISPLAYCONTROL | lcd->displaycontrol);
}

// These commands scroll the display without changing the RAM
void lcd1602d_scrollDisplayLeft(struct lcd_ctx* lcd) {
  lcd1602d_command(lcd, LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
void lcd1602d_scrollDisplayRight(struct lcd_ctx* lcd) {
  lcd1602d_command(lcd, LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

// This is for text that flows Left to Right
void lcd1602d_leftToRight(struct lcd_ctx* lcd) {
  lcd->displaymode |= LCD_ENTRYLEFT;
  lcd1602d_command(lcd, LCD_ENTRYMODESET | lcd->displaymode);
}

// This is for text that flows Right to Left
void lcd1602d_rightToLeft(struct lcd_ctx* lcd) {
  lcd->displaymode &= ~LCD_ENTRYLEFT;
  lcd1602d_command(lcd, LCD_ENTRYMODESET | lcd->displaymode);
}

// This will 'right justify' text from the cursor
void lcd1602d_autoscroll(struct lcd_ctx* lcd) {
  lcd->displaymode |= LCD_ENTRYSHIFTINCREMENT;
  lcd1602d_command(lcd, LCD_ENTRYMODESET | lcd->displaymode);
}

// This will 'left justify' text from the cursor
void lcd1602d_noAutoscroll(struct lcd_ctx* lcd) {
  lcd->displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
  lcd1602d_command(lcd, LCD_ENTRYMODESET | lcd->displaymode);
}

// Allows us to fill the first 8 CGRAM locations
// with custom characters
void lcd1602d_createChar(struct lcd_ctx* lcd, uint8_t location, uint8_t charmap[]) {
  location &= 0x7; // we only have 8 locations 0-7
  lcd1602d_command(lcd, LCD_SETCGRAMADDR | (location << 3));
  for (int i=0; i<8; i++) {
    lcd1602d_write(lcd, charmap[i]);
  }
}


void lcd1602d_command(struct lcd_ctx* lcd, uint8_t value) {
  lcd1602d_send(lcd, value, LOW);
}

void lcd1602d_write(struct lcd_ctx* lcd, uint8_t value) {
  lcd1602d_send(lcd, value, HIGH);
  //  return 1; // assume sucess
}

void lcd1602d_printstr(struct lcd_ctx* lcd, int col, int row, const char* value){
  lcd1602d_setCursor(lcd, col, row);
  while(*value){
    lcd1602d_write(lcd, *value);
    value++;
  }
}

void lcd1602d_send(struct lcd_ctx* lcd, uint8_t value, uint8_t mode) {
  if (mode)
    gpio_set(lcd->rs_pin);
  else
    gpio_clear(lcd->rs_pin);

  lcd1602d_write4bits(lcd, value>>4);
  lcd1602d_write4bits(lcd, value);
}


void lcd1602d_write4bits(struct lcd_ctx* lcd, uint8_t value) {
  for (int i = 0; i < 4; i++) {
    gpio_init(lcd->data_pins[i], GPIO_DIR_OUT, GPIO_NOPULL);

    if ((value >> i) & 0x01)
      gpio_set(lcd->data_pins[i]);
    else
      gpio_clear(lcd->data_pins[i]);
  }

  lcd1602d_pulseEnable(lcd);
}


void lcd1602d_pulseEnable(struct lcd_ctx* lcd) {
  gpio_clear(lcd->enable_pin);
  xtimer_usleep(1);    
  gpio_set(lcd->enable_pin);
  xtimer_usleep(1);    // enable pulse must be >450ns
  gpio_clear(lcd->enable_pin);
  xtimer_usleep(100);   // commands need > 37us to settle
}
