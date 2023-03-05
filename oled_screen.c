
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
    .master.clk_speed = 5000, // 100Hz because internal pullups resistors are too small, line is pulled up too slowly
    .clk_flags = 0
  };
  i2c_param_config(i2c_num, &conf);
  return i2c_driver_install(i2c_num, conf.mode, 0, 0, 0);
}


static void i2c_master_write_slave(i2c_port_t i2c_num, uint8_t device_addr, uint8_t *data, size_t size) {
  int ret;
  i2c_cmd_handle_t cmd;
 
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  // Slave addr + write bit
  i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
  // Control bytes
  //i2c_master_write_byte(cmd, 0x00, true);
  i2c_master_write_byte(cmd, 0x8D, true);
  i2c_master_write_byte(cmd, 0x14, true); 
  i2c_master_write_byte(cmd, 0xAF, true);
  i2c_master_write_byte(cmd, 0xA5, true);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 1000);
  printf("ret1= %x\n", ret);
  i2c_cmd_link_delete(cmd);

  vTaskDelay(pdMS_TO_TICKS(1000));

  return;
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  // Slave addr + write bit
  i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
  // Control byte
  i2c_master_write_byte(cmd, 0xA4, true);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 1000);
  printf("ret2= %x\n", ret);
  i2c_cmd_link_delete(cmd);

  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  // Slave addr + write bit
  i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
  // Control byte
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 1000);
  printf("ret3= %x\n", ret);
  i2c_cmd_link_delete(cmd);

  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  // Slave addr + write bit
  i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
  // Control byte
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 1000);
  printf("ret4= %x\n", ret);
  i2c_cmd_link_delete(cmd);

  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, 0b01000000, true);
  // Data bytes
  i2c_master_write(cmd, data, size, true);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 1000);
  printf("ret2= %x\n", ret);
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
