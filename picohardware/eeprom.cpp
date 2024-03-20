//
// Created by Hanh Hoang on 28.2.2024.
//
#include "eeprom.h"
#include "cstring"

void init_eeprom() {
    i2c_init(i2c0, BAUDRATE);
    gpio_set_function(I2C0_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL_PIN, GPIO_FUNC_I2C);
}

void write_to_eeprom(uint16_t memory_address, const uint8_t *data, size_t length) {
    uint8_t buf[EEPROM_ADDR_LEN + length];
    buf[0] = (uint8_t)(memory_address >> BITS_PER_BYTE);    //high byte of memory address
    buf[1] = (uint8_t)(memory_address);         //low byte of memory address
    for(size_t i = 0; i < length; ++i) {
        buf[i + EEPROM_ADDR_LEN] = data[i];
    }
    i2c_write_blocking(i2c0, DEVADDR, buf, length + EEPROM_ADDR_LEN, false);
    sleep_ms(WRITE_CYCLE_TIME);
}

// read an array of data from eeprom
void read_from_eeprom(uint16_t memory_address, uint8_t *data_read, size_t length) {
    uint8_t buf[EEPROM_ADDR_LEN];
    buf[0] = (uint8_t)(memory_address >> BITS_PER_BYTE); //high byte of memory address
    buf[1] = (uint8_t)memory_address;     //low byte of memory address
    i2c_write_blocking(i2c0, DEVADDR, buf, EEPROM_ADDR_LEN, true);
    i2c_read_blocking(i2c0, DEVADDR, data_read, length, false);
}

// Returns a value data stored in eeprom
void get_network_eeprom(uint16_t memory_address, uint8_t *value) {
    read_from_eeprom(memory_address, value, 64);
}

void write_network_eeprom(uint16_t memory_address, const char *value){
    uint8_t len = strlen(value);
    uint8_t eepromBuff[len+1];
    memcpy(eepromBuff, value, len);
    eepromBuff[len] = '\0';
    write_to_eeprom(memory_address, eepromBuff, len+1);
    printf("%s\n",eepromBuff);
}