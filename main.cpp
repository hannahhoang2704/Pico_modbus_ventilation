#include <stdio.h>
#include <cstring>
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
#define SCALE_FACTOR 240    //SDP610-125pa model
#define ALTITUDE_CORR_FACTOR 0.92   //adjust this to get 125pa with max fan speed

#define USE_MODBUS
#define USE_MQTT
#define USE_SSD1306
#define EVENT_DEBOUNCE_US 8000
#define TIMEOUT 60000000    //60s
#define led_pin 22
#define OFFSET 1

using namespace std;

static const char *pub_topic = "hannah/controller/status";
static const char *sub_topic = "hannah/controller/settings";
volatile bool rotPressed = false;
static int temp = 0, humidity = 0, co2 = 0;
volatile uint64_t startTimeOut = 0;
volatile int rotCount = 0;
volatile bool mqtt_mode;
volatile int mqtt_value;

void messageArrived(MQTT::MessageData &md) {
    MQTT::Message &message = md.message;
    char payload_str[message.payloadlen +1];
    memcpy(payload_str, message.payload, message.payloadlen);
    payload_str[message.payloadlen] = '\0';

    printf("Message arrived: qos %d, retained %d, dup %d, packetid %d\n",
           message.qos, message.retained, message.dup, message.id);
//    printf("Payload %s\n", (char *) message.payload);
    printf("Payload %s", (char *)payload_str);

    //extract mode and value from arrived message in format `{"auto": true, "pressure": 10}`
    //need to test out if it's working//
    char* auto_mode = strstr(payload_str, "\"auto\":");
    if (auto_mode != nullptr){
        mqtt_mode = (*(auto_mode + 8) == 't');
    }
    sscanf(payload_str, "%*[^0-9]%d", &mqtt_value);
    printf("Mqtt mode: %d, received value: %d\n", mqtt_mode, mqtt_value);
}

// rotary encoder interrupt handler
void rot_handler(uint gpio, uint32_t event_mask) {
    if(gpio == ROT_A){
        startTimeOut = time_us_64();
        if (gpio_get(ROT_B)){
            if (rotCount < 2) rotCount++;
        }
        else{
            if (rotCount > -2) rotCount--;
        }
    } else if (gpio == ROT_SW){
        startTimeOut = time_us_64();
        rotPressed = true;
    }
}

bool timeout(const uint64_t start){
    return (time_us_64() - start) > TIMEOUT;
}

void getPressure(int *pressure){
    int pressureData;
    uint8_t start_cmd[] = {0xF1};
    uint8_t pressure_data[2];
    i2c_write_blocking(i2c1, SDP610_ADDR, start_cmd, 1, false);
    sleep_ms(10);
    i2c_read_blocking(i2c1, SDP610_ADDR, pressure_data, 2, false);
    sleep_ms(100);
    pressureData = ((pressure_data[0] << 8) | pressure_data[1])/SCALE_FACTOR*ALTITUDE_CORR_FACTOR;
    *pressure = pressureData > 130 ? 0 : pressureData;
    printf("Pressure = %dpa\n",*pressure);
}

int main() {

    static uint8_t menu = 2;
    bool autoMode = false;
    int pressure = 0;
    float speed = 0;
    int speedInt = 0;
    int valP = pressure;
    int valS = speedInt;
    bool valM = autoMode;
    int setPoint = 0;   //used for set speed or pressure
    int setPointP_L = 0; //under limit pressure
    int setPointP_H = 0; //upper limit pressure
    uint16_t fanDelay = 200;  //delay time after setting fan speed
    uint8_t measureCount = 0;   //count number of measure that does not get desired pressure
    uint8_t eepromBuff[1];
    bool error = false;

    // Initialize hw
    stdio_init_all();
    // Initialize LED pin
    gpio_init(led_pin);
    gpio_set_dir(led_pin, GPIO_OUT);

    Button sw0(SW_0);
    Button sw1(SW_1);
    Button sw2(SW_2);
    init_eeprom();
    init_rotary_knob();

    gpio_set_irq_enabled_with_callback(ROT_A, GPIO_IRQ_EDGE_FALL, true, &rot_handler);
    gpio_set_irq_enabled_with_callback(ROT_SW, GPIO_IRQ_EDGE_FALL, true, &rot_handler);

    printf("\nBoot\n");

    //get data stored from EEPROM
    autoMode = get_stored_value(MODE_ADDR);
    if (autoMode){
        pressure = get_stored_value(PRESSURE_ADDR);
        if(pressure < 0 || pressure > MAX_PRESSURE) pressure = 0;
        speed = getSpeed(pressure);
        setPoint = pressure;
        setPointP_H = setPoint + OFFSET > MAX_PRESSURE ? MAX_PRESSURE:setPoint + OFFSET;
        setPointP_L = setPoint - OFFSET < 0 ? 0:setPoint - OFFSET;
    }
    else{
        speed = get_stored_value(SPEED_ADDR);
        if(speed < 0 || speed > MAX_FAN_SPEED) speed = 0;
        pressure = 0;
    }
#ifdef USE_SSD1306
    // I2C is "open drain",
    // pull ups to keep signal high when no data is being sent
    i2c_init(i2c1, 100 * 1000);
    gpio_set_function(14, GPIO_FUNC_I2C); // the display has external pull-ups
    gpio_set_function(15, GPIO_FUNC_I2C); // the display has external pull-ups
    auto display = std::make_shared<ssd1306>(i2c1);
    currentScreen screen(display);
#endif


#ifdef USE_MQTT
    //IPStack ipstack("SSID", "PASSWORD"); // example
    IPStack ipstack("Rhod's wifi 2.4G", "0413113368"); // example
    auto client = MQTT::Client<IPStack, Countdown>(ipstack);
    int rc = ipstack.connect("192.168.0.100", 1883);
    if (rc != 1) {
        printf("rc from TCP connect is %d\n", rc);
    }

    printf("MQTT connecting\n");
    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = 3;
    data.clientID.cstring = (char *) "PicoW-hannah";
    rc = client.connect(data);
    if (rc != 0) {
        printf("rc from MQTT connect is %d\n", rc);
        while (true) {
            tight_loop_contents();
        }
    }
    printf("MQTT connected\n");

    // We subscribe QoS2. Messages sent with lower QoS will be delivered using the QoS they were sent with
    rc = client.subscribe(sub_topic, MQTT::QOS2, messageArrived);
    if (rc != 0) {
        printf("rc from MQTT subscribe is %d\n", rc);
    }
    printf("MQTT subscribed\n");

    auto mqtt_send = make_timeout_time_ms(3000);
    int mqtt_qos = 0;
    int msg_count = 0;
#endif

#ifdef USE_MODBUS
    auto uart{std::make_shared<PicoUart>(UART_NR, UART_TX_PIN, UART_RX_PIN, BAUD_RATE)};
    auto rtu_client{std::make_shared<ModbusClient>(uart)};
    ModbusRegister c2o(rtu_client, 240, 256);
    ModbusRegister rh(rtu_client, 241, 256);
    ModbusRegister tem(rtu_client, 241, 257);
    ModbusRegister fanSpeed(rtu_client, 1, 0);

    sleep_ms(100);
    if (speed > 0){
        fanSpeed.write(500);
        sleep_ms(100);
        fanSpeed.write((int)speed*10);
    }
    sleep_ms(100);
    auto modbus_poll = make_timeout_time_ms(3000);
#endif

    while (true) {
        if(menu == 0){
            if (!timeout(startTimeOut)){
                if (rotCount == 2 || rotCount == -2) {
                    valM = !valM;
                    rotCount = 0;
                }
                screen.modeSelection(valM);
            } else {    //timeout, show info screen
                menu = 2;
            }

        }else if(menu == 1){
            if (!timeout(startTimeOut)){
                if (autoMode){
                    if (rotCount > 0) {
                        if (valP < MAX_PRESSURE) valP++;
                    }
                    else if (rotCount < 0){
                        if (valP > 0) valP--;
                    }
                    rotCount = 0;
                    screen.paramSet(autoMode,valP);
                }else{
                    if (rotCount > 0) {
                        if (valS < MAX_FAN_SPEED) valS++;
                    }
                    else if (rotCount < 0){
                        if (valS > 0) valS--;
                    }
                    fanSpeed.write(valS*10);
                    sleep_ms(fanDelay);
                    rotCount = 0;
                    screen.paramSet(autoMode, valS);
                }
            }else {    //timeout, show info screen
                menu = 2;
            }
        }
        else if(menu == 2){
            screen.info(autoMode,(float )speed, pressure, temp, humidity, co2);
        }
        else{
            screen.error();
        }
        //handle rot switch pressed to confirm setting and move to next screen
        if (rotPressed){
            sleep_ms(30);
            if (!gpio_get(ROT_SW)){
                if (menu < 2){
                    if (menu == 0){
                        autoMode = valM;
                        valS = (int)speed;
                        valP = pressure;
                        if (autoMode){
                            setPoint = pressure;
                            setPointP_H = setPoint + OFFSET > MAX_PRESSURE ? MAX_PRESSURE:setPoint + OFFSET;
                            setPointP_L = setPoint - OFFSET < 0 ? 0:setPoint - OFFSET;
                        } else{
                            setPoint = (int)speed;
                        }
                        //write autoMode to eeprom
                        eepromBuff[0] = autoMode;
                        write_to_eeprom(MODE_ADDR, eepromBuff, 1);
                    } else if (menu == 1){
                        if (autoMode){
                            setPoint = valP;
                            setPointP_H = setPoint + OFFSET > MAX_PRESSURE ? MAX_PRESSURE:setPoint + OFFSET;
                            setPointP_L = setPoint - OFFSET < 0 ? 0:setPoint - OFFSET;
                            speed = getSpeed(setPoint);
                            fanSpeed.write(500);
                            sleep_ms(100);
                            fanSpeed.write((int)(speed*10));
                            screen.info(autoMode,(float )speed, pressure, temp, humidity, co2);
                            sleep_ms(1000);
                            //write autoMode to eeprom
                            eepromBuff[0] = valP;
                            write_to_eeprom(PRESSURE_ADDR, eepromBuff, 1);
                        }
                        else {
                            speed = (float )valS;
                            setPoint = valS;
                            //write autoMode to eeprom
                            eepromBuff[0] = valS;
                            write_to_eeprom(SPEED_ADDR, eepromBuff, 1);
                        }
                    }
                    menu++;
                }
            }
            rotPressed = false;
        }
        //handle sw2 switch pressed to change to menu 0
        if(sw2.debounced_pressed()){
            if (menu == 2){
                rotCount = 0;
            }
            else if (menu == 3){
                measureCount = 0;
                error = false;
            }
            menu = 0;
            valM = autoMode;
            startTimeOut = time_us_64();
        }
        //handle sw1 switch pressed to change to menu 1
        if(sw1.debounced_pressed()){
            if (menu == 2){
                rotCount = 0;
            }
            else if (menu == 3){
                measureCount = 0;
                error = false;
            }
            menu = 1;
            valP = pressure;
            valS = (int)speed;
            startTimeOut = time_us_64();
        }
        //handle sw0 switch pressed to change to menu 2
        if(sw0.debounced_pressed()){
            if (menu == 3) {
                measureCount = 0;
                error = false;
            }
            menu = 2;
            startTimeOut = time_us_64();
        }
//start measurement

#ifdef USE_MODBUS
        if (time_reached(modbus_poll)) {
            gpio_put(led_pin, !gpio_get(led_pin)); // toggle  led
            modbus_poll = delayed_by_ms(modbus_poll, 3000);
            co2 = c2o.read();
            printf("CO2   = %d ppm\n", co2);
            humidity = rh.read()/10;
            printf("RH    = %d%%\n", humidity);
            temp = tem.read()/10;
            printf("T     = %d C\n", temp);
            printf("S     = %.1f\n", speed);
            getPressure(&pressure);
            if (autoMode){
                if (pressure < setPointP_L){
                    measureCount++;
                    if (measureCount >= 20){
                        menu = 3;   //show error screen
                        measureCount = 20;
                        error = true;
                    }
                }
                while (pressure < setPointP_L){
                    speedInt = (int)(speed*10);
                    if (speedInt >= MAX_FAN_SPEED*10){
                        break;
                    }
                    else{
                        speedInt++;
                        speed = (float )speedInt / 10;
                    }
                    fanSpeed.write(speedInt);
                    sleep_ms(fanDelay);
                    getPressure(&pressure);
                    if (menu == 2){
                        screen.info(autoMode,(float )speed, (int)setPoint, temp, humidity, co2);
                    }
                }
                while (pressure > setPointP_H){
                    speedInt = (int)(speed*10);
                    measureCount = 0;
                    error = false;
                    if (speedInt > 0) speedInt--;
                    fanSpeed.write(speedInt);
                    speed = (float)speedInt/10;
                    sleep_ms(fanDelay);
                    getPressure(&pressure);
                    if (menu == 2){
                        screen.info(autoMode,(float )speed, (int)setPoint, temp, humidity, co2);
                    }
                }
                pressure = (int)setPoint;   // in auto mode, pressure keeps unchanged
            }
        }

#endif
#ifdef USE_MQTT
        if (time_reached(mqtt_send)) {
            mqtt_send = delayed_by_ms(mqtt_send, 3000);
            if (!client.isConnected()) {
                printf("Not connected...\n");
                rc = client.connect(data);
                if (rc != 0) {
                    printf("rc from MQTT connect is %d\n", rc);
                }
            }
            //char buf[100]="Message sent";
            // Construct JSON message
            char buf[256];
            rc = 0;
            sprintf(buf, R"({"nr": %d, "speed": %.1f, "setpoint": %d, "pressure": %d, "auto": %s, "error": %s, "co2": %d, "rh": %d, "temp": %d})",
                    ++msg_count, speed, (int)setPoint, pressure, autoMode ? "true" : "false", "false", co2, humidity, temp);
            //sprintf(buf, R"({"nr":%d,"setpoint":%d,"pressure":%d,"auto":%s,"error":%s})",
            //        ++msg_count, (int)setPoint, pressure, autoMode ? "true" : "false", error ? "true" : "false");

            MQTT::Message message;
            message.retained = false;
            message.dup = false;
            message.payload = (void *)buf;

            switch (mqtt_qos) {
                case 0:
                    message.qos = MQTT::QOS0;
                    message.payloadlen = strlen(buf) + 1;
                    rc = client.publish(pub_topic, message);
                    printf("Publish rc=%d\n", rc);
                    ++mqtt_qos;
                    break;
                case 1:
                    message.qos = MQTT::QOS1;
                    message.payloadlen = strlen(buf) + 1;
                    rc = client.publish(pub_topic, message);
                    printf("Publish rc=%d\n", rc);
                    ++mqtt_qos;
                    break;
#if MQTTCLIENT_QOS2
                    case 2:
                        message.qos = MQTT::QOS2;
                        message.payloadlen = strlen(buf) + 1;
                        rc = client.publish(pub_topic, message);
                        printf("Publish rc=%d\n", rc);
                        ++mqtt_qos;
                        break;
#endif
                default:
                    mqtt_qos = 0;
                    break;
            }
        }

        cyw43_arch_poll(); // obsolete? - see below
        client.yield(100); // socket that client uses calls cyw43_arch_poll()
#endif
    }
}

