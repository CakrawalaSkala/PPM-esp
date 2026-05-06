#ifndef OLED_H
#define OLED_H

#include <stdint.h>

void oled_init();
void oled_clear();
void oled_set_cursor(uint8_t x, uint8_t y);
void oled_print(char *str);
void oled_draw_bitmap(const uint8_t *bitmap, uint8_t w, uint8_t h);

#endif