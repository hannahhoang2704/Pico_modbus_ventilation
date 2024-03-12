//
// Created by Hanh Hoang on 28.2.2024.
//
#include "eeprom.h"

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
    sleep_ms(5);
}

// read an array of data from eeprom
void read_from_eeprom(uint16_t memory_address, uint8_t *data_read, size_t length) {
    uint8_t buf[EEPROM_ADDR_LEN + length];
    buf[0] = (uint8_t)(memory_address >> BITS_PER_BYTE); //high byte of memory address
    buf[1] = (uint8_t)(memory_address);     //low byte of memory address
    i2c_write_blocking(i2c0, DEVADDR, buf, EEPROM_ADDR_LEN, true);
    i2c_read_blocking(i2c0, DEVADDR, data_read, length, false);
}

// Returns a value data stored in eeprom
uint8_t get_stored_value(uint16_t memory_address) {
    uint8_t value;
    read_from_eeprom(memory_address, &value, 1);
    return value;
}

void write_value_to_eeprom(uint16_t memory_address, uint8_t value){
    uint8_t eepromBuff[1];
    eepromBuff[0] = value;
    write_to_eeprom(memory_address, eepromBuff, 1);
}

//void write_string_to_eeprom(uint16_t memory_address, const std::string& str) {
//    // Convert string to uint8_t buffer
//    const uint8_t* data = reinterpret_cast<const uint8_t*>(str.c_str());
//    size_t length = str.length();
//
//    // Call write_to_eeprom function
//    write_to_eeprom(memory_address, data, length);
//}
//
//std::string read_string_from_eeprom(uint16_t memory_address, size_t length) {
//    // Allocate memory for data to be read from EEPROM using std::shared_ptr
//    std::shared_ptr<uint8_t[]> data_read(new uint8_t[length]);
//
//    // Call function to read data from EEPROM
//    read_from_eeprom(memory_address, data_read.get(), length);
//
//    // Convert the received data to a string
//    std::string str(reinterpret_cast<char*>(data_read.get()), length);
//
//    return str;
//}

// cyclic redundancy check to detect error log in eeprom memory
uint16_t crc16(const uint8_t *data_p, size_t length) {
    uint8_t x;
    uint16_t crc = 0xFFFF;
    while (length--) {
        x = crc >> 8 ^ *data_p++;
        x ^= x >> 4;
        crc = (crc << 8) ^ ((uint16_t) (x << 5) ^ ((uint16_t) x));
    }
    return crc;
}

// Write a string to the specified EEPROM address, including CRC
void write_string_to_eeprom( uint16_t eeprom_address, const char *str) {
    size_t string_length = strlen(str) + 1; // Include NULL terminator
    if (string_length > STRLEN_EEPROM) {
        string_length = STRLEN_EEPROM;
    }
    uint8_t log_buf[string_length + CRC_CHAR];

    // Copy string to uint8_t array
    for (int a = 0; a < string_length; ++a) {
        log_buf[a] = (uint8_t) str[a];
    }

    // Add CRC to log buffer
    uint16_t crc = crc16(log_buf, string_length);
    log_buf[string_length] = (uint8_t)(crc >> BITS_PER_BYTE);
    log_buf[string_length + 1] = (uint8_t)crc;

    // Write to EEPROM
    write_to_eeprom(eeprom_address, log_buf, string_length + CRC_CHAR);
}

// Read the string from the specified EEPROM address, verifying CRC
std::string read_string_from_eeprom(uint16_t eeprom_address) {
    uint8_t read_buff[STRLEN_EEPROM + CRC_CHAR];
    read_from_eeprom(eeprom_address, read_buff, STRLEN_EEPROM + CRC_CHAR);

    // Verify CRC
    uint16_t crc = crc16(read_buff, STRLEN_EEPROM + CRC_CHAR - 2);
    if (crc != 0) {
        printf("CRC check failed. Data may be corrupted.\n");
        return "";
    }

    // Convert uint8_t buffer to string
    std::string str(reinterpret_cast<char*>(read_buff), STRLEN_EEPROM);
    return str;
}
