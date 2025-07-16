/*
 * lcd_rgb.c
 *
 *  Created on: Jul 10, 2025
 *      Author: jaso
 */


#include <lcd.h>
#include "string.h"

#define LCD_ADDR        (0x27 << 1)  // Change if your backpack is at 0x3F
#define LCD_BACKLIGHT   0x08
#define ENABLE_BIT      0x04

static I2C_HandleTypeDef *lcd_i2c;

static void lcd_send_cmd(uint8_t cmd);
static void lcd_send_data(uint8_t data);
static void lcd_send(uint8_t data, uint8_t mode);
static void lcd_write_4bits(uint8_t nibble);

void lcd_init(I2C_HandleTypeDef *hi2c) {
    lcd_i2c = hi2c;

    HAL_Delay(50);
    lcd_write_4bits(0x30);
    HAL_Delay(5);
    lcd_write_4bits(0x30);
    HAL_Delay(1);
    lcd_write_4bits(0x30);
    HAL_Delay(1);
    lcd_write_4bits(0x20); // 4-bit mode

    lcd_send_cmd(0x28); // 2-line, 5x8 font
    lcd_send_cmd(0x0C); // Display ON, cursor OFF
    lcd_send_cmd(0x06); // Entry mode
    lcd_clear();
}

void lcd_clear(void) {
    lcd_send_cmd(0x01);
    HAL_Delay(2);
}

void lcd_set_cursor(uint8_t col, uint8_t row) {
    uint8_t offsets[] = {0x00, 0x40, 0x14, 0x54}; // for 20x4 LCD
    lcd_send_cmd(0x80 | (col + offsets[row]));
}

void lcd_write_string(const char *str) {
    while (*str) {
        lcd_send_data(*str++);
    }
}

// Internal functions

static void lcd_send_cmd(uint8_t cmd) {
    lcd_send(cmd, 0);
}

static void lcd_send_data(uint8_t data) {
    lcd_send(data, 1);
}

static void lcd_send(uint8_t data, uint8_t mode) {
    uint8_t high_nib = (data & 0xF0);
    uint8_t low_nib = ((data << 4) & 0xF0);
    lcd_write_4bits(high_nib | mode);
    lcd_write_4bits(low_nib | mode);
}

static void lcd_write_4bits(uint8_t nibble) {
    uint8_t data = nibble | LCD_BACKLIGHT;
    HAL_I2C_Master_Transmit(lcd_i2c, LCD_ADDR, &data, 1, HAL_MAX_DELAY);
    data |= ENABLE_BIT;
    HAL_I2C_Master_Transmit(lcd_i2c, LCD_ADDR, &data, 1, HAL_MAX_DELAY);
    data &= ~ENABLE_BIT;
    HAL_I2C_Master_Transmit(lcd_i2c, LCD_ADDR, &data, 1, HAL_MAX_DELAY);
    HAL_Delay(1);
}
