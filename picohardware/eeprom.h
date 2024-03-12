//
// Created by Hanh Hoang on 28.2.2024.
//

#ifndef PICO_MODBUS_EEPROM_H
#define PICO_MODBUS_EEPROM_H

#include <stdio.h>
#include <string>
#include <cstring>
#include <memory>
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
#define STRLEN_EEPROM 60
#define CRC_CHAR 2
#define MODE_ADDR (I2C_MEMORY_SIZE - 1)
#define PRESSURE_ADDR (I2C_MEMORY_SIZE - 2)
#define SPEED_ADDR (I2C_MEMORY_SIZE-3)
#define SSID_ADDR (I2C_MEMORY_SIZE-4)
#define PASSWORD_ADDR (I2C_MEMORY_SIZE-5)
#define IP_ADDR (I2C_MEMORY_SIZE-6)


void init_eeprom();
void write_to_eeprom(uint16_t memory_address, const uint8_t *data, size_t length);
void read_from_eeprom(uint16_t memory_address, uint8_t *data_read, size_t length);
uint8_t get_stored_value(uint16_t memory_address);
void write_value_to_eeprom(uint16_t memory_addres, const uint8_t value);
uint16_t crc16(const uint8_t *data_p, size_t length);
void write_string_to_eeprom( uint16_t eeprom_address, const char *str);
std::string read_string_from_eeprom(uint16_t eeprom_address);
//std::string read_string_from_eeprom(uint16_t memory_address, size_t length);
//void write_string_to_eeprom(uint16_t memory_address, const std::string& str);
#endif //PICO_MODBUS_EEPROM_H
