//
// Created by Hanh Hoang on 17.3.2024.
//

#ifndef PICO_MODBUS_VENTILATION_MAIN_H
#define PICO_MODBUS_VENTILATION_MAIN_H
#include <stdio.h>
#include <cstring>
#include <map>
#include <cmath>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/timer.h"
#include "uart/PicoUart.h"

#include "picohardware/button.h"
#include "picohardware/eeprom.h"
#include "IPStack.h"
#include "Countdown.h"
#include "MQTTClient.h"
#include "ModbusClient.h"
#include "ModbusRegister.h"
#include "ssd1306.h"
#include "screen_selection.h"
#include "speed_pressure_data.h"

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#if 0
#define UART_NR 0
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#else
#define UART_NR 1
#define UART_TX_PIN 4
#define UART_RX_PIN 5
#endif
#define MAX_FAN_SPEED 100
#define MAX_PRESSURE 120
#define SDP610_ADDR 0x40
#define BAUD_RATE 9600
#define STOP_BITS 1 // for simulator
//#define STOP_BITS 2 // for real system

#define SCALE_FACTOR 240    //SDP610-125pa model
#define ALTITUDE_CORR_FACTOR 0.92   //adjust this to get 125pa with max fan speed
#define CO2_REGISISTER_ADDR 256
#define HUMIDITY_REGISTER_ADDR 256
#define TEMP_REGISTER_ADDR 257
#define FAN_REGISER_ADDR 0
#define GMP252_ADDR 240
#define HMP60_ADDR 241
#define FAN_SERVER_ADDR 1

#define USE_MODBUS
#define USE_MQTT
#define USE_SSD1306
#define DEBOUNCE_TIME 1000
#define TIMEOUT 60000000    //60s
#define led_pin 22
#define OFFSET 1
#define NUMBER_OF_GPIO_PINS 3

static const char *pub_topic = "hannah/controller/status";
static const char *sub_topic = "hannah/controller/settings";
static volatile bool rotPressed = false;
static volatile bool swPressed = false;
static int temp = 0, humidity = 0, co2 = 0;
static volatile uint64_t startTimeOut = 0;
static volatile int rotCount = 0;
static volatile bool mqtt_mode;
static volatile int mqtt_value = 0;
//volatile uint8_t menu = 2;
static bool autoMode = false;
static volatile bool receivedNewMsg = false;
static int option = 0;
static volatile bool irqReady = true;
static volatile int valP = 0;
static volatile int valS = 0;

uint64_t gpioTimeStamp[NUMBER_OF_GPIO_PINS];

std::map<uint, uint8_t> gpioIndexMap = {
        {SW_0, 0},
        {ROT_A, 1},
        {ROT_SW, 2}
};

typedef enum {
    MAIN_MENU,
    SETPOINT_MENU,
    STATUS_MENU,
    ERROR_MENU,
    MQTT_MENU
}MenuState;

MenuState menu = STATUS_MENU;

void last_interrupt_time(uint gpio);
uint64_t time_since_last_interrupt(uint gpio);
void messageArrived(MQTT::MessageData &md);
bool timeout(const uint64_t start);
void getPressure(int *pressure);
void set_limit_pressure(int setPoint, int &pressure_lim_high, int &pressure_lim_low);
#endif //PICO_MODBUS_VENTILATION_MAIN_H
