#pragma once

#include <inttypes.h>
#include "periph/gpio.h"

struct lcd_ctx {
  gpio_t rs_pin;
  gpio_t enable_pin;
  gpio_t data_pins[4];

  uint8_t displayfunctions;
  uint8_t displaymode;
  uint8_t displaycontrol;
  uint8_t numlines;

  int currline;
};
void lcd1602d_setCursor(struct lcd_ctx* lcd, uint8_t col, uint8_t row);
void lcd1602d_display(struct lcd_ctx* lcd);
void lcd1602d_noDisplay(struct lcd_ctx* lcd);
void lcd1602d_noCursor(struct lcd_ctx* lcd);
void lcd1602d_cursor(struct lcd_ctx* lcd);
void lcd1602d_noBlink(struct lcd_ctx* lcd);
void lcd1602d_blink(struct lcd_ctx* lcd);
void lcd1602d_scrollDisplayLeft(struct lcd_ctx* lcd);
void lcd1602d_scrollDisplayRight(struct lcd_ctx* lcd);
void lcd1602d_leftToRight(struct lcd_ctx* lcd);
void lcd1602d_rightToLeft(struct lcd_ctx* lcd);
void lcd1602d_autoscroll(struct lcd_ctx* lcd);
void lcd1602d_noAutoscroll(struct lcd_ctx* lcd);
void lcd1602d_createChar(struct lcd_ctx* lcd, uint8_t location, uint8_t charmap[]);
void lcd1602d_clear(struct lcd_ctx* lcd);
void lcd1602d_write4bits(struct lcd_ctx* lcd, uint8_t value);
void lcd1602d_init_lcd(struct lcd_ctx *lcd);
void lcd1602d_command(struct lcd_ctx* lcd, uint8_t value) ;
void lcd1602d_send(struct lcd_ctx* lcd, uint8_t value, uint8_t mode);
void lcd1602d_pulseEnable(struct lcd_ctx* lcd);
void lcd1602d_write(struct lcd_ctx* lcd, uint8_t value);
void lcd1602d_printstr(struct lcd_ctx* lcd, int col, int row, const char* value);

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

