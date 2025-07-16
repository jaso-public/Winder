/*
 * lcd_rgb.h
 *
 *  Created on: Jul 10, 2025
 *      Author: jaso
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

#include "main.h"  // or your STM32 series
#include <stdint.h>

void lcd_init(I2C_HandleTypeDef *hi2c);
void lcd_clear(void);
void lcd_set_cursor(uint8_t col, uint8_t row);
void lcd_write_string(const char *str);


#endif /* INC_LCD_H_ */
