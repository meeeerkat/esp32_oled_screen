
#include "oled_screen.h"

#include <stdio.h>
#include <stdlib.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#define ADDR_128x32 0x3C
#define ADDR_128x64 0x3D


static esp_err_t i2c_master_init(uint8_t sda, uint8_t scl, i2c_port_t i2c_num) {
  i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = sda,
    .sda_pullup_en = true,
    .scl_io_num = scl,
    .scl_pullup_en = true,
    .master.clk_speed = 1000000,
    .clk_flags = 0
  };
  i2c_param_config(i2c_num, &conf);
  return i2c_driver_install(i2c_num, conf.mode, 0, 0, 0);
}


const uint8_t INIT_COMMANDS[] = {

  0x00,

  // Turn off
  0xAE,

  // Fundamental settings
  0x81,
  0x7F,

  0xA4,

  0xA6,

  // Deactivate scrolling
  0x2E,

  // Addressing settings
  0x00,

  0x10,

  0x20,
  0,

  0x21,
  0,
  127,

  0x22,
  0,
  3,

  0xB0,

  // Hardware settings
  0x40,

  0xA0,

  0xA8,
  31,

  0xC0,

  0xD3,
  0,

  0xDA,
  0b00001000,

  // Time settings
  0xD5,
  0x80,

  0xD9,
  0x22,

  0xDB,
  0x20,

  // Charge pump settings
  0x8D,
  0x14,



  // Turn on
  0xAF,


  // Deactivate scrolling
  0x2E,
};
const size_t INIT_COMMANDS_SIZE = sizeof(INIT_COMMANDS);


static void i2c_master_write_slave(i2c_port_t i2c_num, uint8_t device_addr, uint8_t *data, size_t data_size) {
  int ret;
  i2c_cmd_handle_t cmd;
 
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  // Slave addr + write bit
  i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);

  // Initialisation control bytes (See page 64 of datasheet)
  i2c_master_write(cmd, INIT_COMMANDS, INIT_COMMANDS_SIZE, true);

  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 10000);
  printf("ret1= %x\n", ret);
  i2c_cmd_link_delete(cmd);


  vTaskDelay(pdMS_TO_TICKS(100));


  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  // Slave addr + write bit
  i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);

  // set data
  i2c_master_write_byte(cmd, 0b01000000, true);
  i2c_master_write(cmd, data, data_size, true);

  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 10000);
  printf("ret1= %x\n", ret);
  i2c_cmd_link_delete(cmd);


  vTaskDelay(pdMS_TO_TICKS(3000));


  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  // Slave addr + write bit
  i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);

  // clear
  i2c_master_write_byte(cmd, 0x00, true);
  i2c_master_write_byte(cmd, 0xAE, true);

  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 10000);
  printf("ret1= %x\n", ret);
  i2c_cmd_link_delete(cmd);


}

void oled_screen__init(oled_screen_t* os, uint8_t sda, uint8_t scl, i2c_port_t i2c_num, enum RESOLUTION res) {
  os->i2c_num = i2c_num;
  os->resolution = res;

  if (res == RES_128x32) {
    os->address = ADDR_128x32;
    os->buff_size = OLED_SCREEN_128x32_BUFF_SIZE;
  }
  else {
    os->address = ADDR_128x64;
    os->buff_size = OLED_SCREEN_128x64_BUFF_SIZE;
  }

  ESP_ERROR_CHECK(i2c_master_init(sda, scl, i2c_num));
}


uint16_t oled_screen__get_buff_size(oled_screen_t* os) {
  return os->buff_size;
}

void oled_screen__write(oled_screen_t* os, uint8_t* buff) {
  i2c_master_write_slave(os->i2c_num, os->address, buff, os->buff_size);
  printf("HERE\n");
}
