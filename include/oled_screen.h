#ifndef _OLED_SCREEN_H_
#define _OLED_SCREEN_H_

#include <stdint.h>
#include "driver/i2c.h"



enum RESOLUTION {RES_128x32, RES_128x64};

// DO NOT USE DIVISIONS !!!!!
#define OLED_SCREEN_128x32_BUFF_SIZE 128*4 // 128*32 bits
#define OLED_SCREEN_128x64_BUFF_SIZE 128*8 // 128*64 bits

typedef struct {
  i2c_port_t i2c_num;
  enum RESOLUTION resolution;

  uint8_t address;
  uint16_t buff_size;
} oled_screen_t;

extern void oled_screen__init(oled_screen_t* sr, uint8_t sda, uint8_t scl, i2c_port_t i2c_num, enum RESOLUTION res);
extern uint16_t oled_screen__get_buff_size(oled_screen_t* os);
extern void oled_screen__write(oled_screen_t* os, uint8_t* buff);

#endif
