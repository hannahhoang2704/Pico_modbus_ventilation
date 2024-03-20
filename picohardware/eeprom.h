//
// Created by Hanh Hoang on 28.2.2024.
//

#ifndef PICO_MODBUS_EEPROM_H
#define PICO_MODBUS_EEPROM_H

#include <stdio.h>
#include "hardware/i2c.h"
#include "pico/stdlib.h"

#define I2C0_SDA_PIN 16
#define I2C0_SCL_PIN 17

#define DEVADDR 0x50
#define BAUDRATE 100000
#define I2C_MEMORY_SIZE 32768
#define WRITE_CYCLE_TIME_PER_BYTE 5
#define BITS_PER_BYTE 8

#define EEPROM_ADDR_LEN 2
#define MODE_ADDR (I2C_MEMORY_SIZE - 1)
#define PRESSURE_ADDR (I2C_MEMORY_SIZE - 2)
#define SPEED_ADDR (I2C_MEMORY_SIZE-3)
#define SSID_ADDR (0)
#define PASS_ADDR (64)
#define IP_ADDR (128)
#define WRITE_CYCLE_TIME 5

void init_eeprom();
void write_to_eeprom(uint16_t memory_address, const uint8_t *data, size_t length);
void read_from_eeprom(uint16_t memory_address, uint8_t *data_read, size_t length);
void get_network_eeprom(uint16_t memory_address, uint8_t *value);
void write_network_eeprom(uint16_t memory_address, const char *value);
#endif //PICO_MODBUS_EEPROM_H
