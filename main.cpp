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
#include "hardware/resets.h"

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

#define USE_MODBUS
#define USE_MQTT
#define USE_SSD1306
#define DEBOUNCE_TIME 20000 //20ms
#define MENU_TIMEOUT 60000000    //60s
#define SW_0_TIMEOUT 1000000  //1s
#define led_pin 22
#define OFFSET 1
#define SSID_POS_X 44
#define PASS_POS_X 44
#define IP_POS_X 2
#define MAX_POS_X 120

using namespace std;

static const char *pub_topic = "hannah/controller/status";
static const char *sub_topic = "hannah/controller/settings";
volatile bool sw0Pressed = false;
volatile bool sw2Pressed = false;
static int temp = 0, humidity = 0, co2 = 0;
volatile uint64_t startTimeOut = 0;
volatile int rotCount = 0;
volatile bool mqtt_mode;
volatile int mqtt_value = 0;
volatile uint8_t menu = 2;
bool autoMode = false;
volatile bool receivedNewMsg = false;
volatile int option = 0;
volatile bool irqReady = true;
volatile int valP = 0;
volatile int valS = 0;
//volatile bool enterEditing = false;
volatile int asciiChar = START_ASCII;
volatile int posX = 44;
volatile uint64_t markTime;

uint64_t gpioTimeStamp[3];

void last_interrupt_time(uint gpio){
    uint8_t index = 0;
    if (gpio == SW_2){
        index = 0;
    } else if (gpio == ROT_A){
        index = 1;
    } else if (gpio == SW_0){
        index = 2;
    }
    gpioTimeStamp[index] = time_us_64();
}

// Function to get the time since the last interrupt for a given GPIO pin
uint64_t time_since_last_interrupt(uint gpio) {
    uint64_t current_time = time_us_64();
    uint8_t index = 0;
    if (gpio == SW_2){
        index = 0;
    } else if (gpio == ROT_A){
        index = 1;
    } else if (gpio == SW_0){
        index = 2;
    }
    return current_time - gpioTimeStamp[index];
}

void messageArrived(MQTT::MessageData &md) {
    MQTT::Message &message = md.message;
    char payload_str[message.payloadlen +1];
    memcpy(payload_str, message.payload, message.payloadlen);
    payload_str[message.payloadlen] = '\0';

    //extract mode and value from arrived message in format `{"auto": true, "pressure": 10}`
    char* auto_mode = strstr(payload_str, "\"auto\":");
    if (auto_mode != nullptr){
        mqtt_mode = (*(auto_mode + 8) == 't');
    }
    sscanf(payload_str, "%*[^0-9]%d", &mqtt_value);
    autoMode = mqtt_mode;
    menu = 2;
    receivedNewMsg = true;
}

// rotary encoder interrupt handler
void rot_handler(uint gpio, uint32_t event_mask) {
    if ((time_us_64() - markTime) >= DEBOUNCE_TIME){
        markTime = time_us_64();
        if(gpio == ROT_A){
            if (irqReady){
                irqReady = false;
                startTimeOut = time_us_64();
                if (gpio_get(ROT_B)){
                    if (menu == 0){
                        option = (option + 1) % 4;
                    } else if (menu == 1){
                        if (autoMode){
                            if (valP < MAX_PRESSURE) valP++;
                        } else{
                            if (valS < MAX_FAN_SPEED) valS++;
                        }
                    }
                    else if (menu == 4){
                        option = (option + 1) % 5;
                    } else if (menu == 5){
                        if (++asciiChar > END_ASCII) asciiChar = START_ASCII;
                    }
                }
                else{
                    if (menu == 0){
                        option = (option + 3) % 4; // Handle underflow properly
                    }else if (menu == 1) {
                        if (autoMode) {
                            if (valP > 0) valP--;
                        } else {
                            if (valS > 0) valS--;
                        }
                    }else if (menu == 4){
                        option = (option + 4) % 5;
                    } else if (menu == 5){
                        if (--asciiChar < START_ASCII) asciiChar = END_ASCII;
                    }
                }
            }
        } else {
            busy_wait_ms(5);
            if (!gpio_get(gpio)){
                startTimeOut = time_us_64();
                if (gpio == SW_0) {
                    sw0Pressed = true;
                } else if (gpio == SW_2) {
                    sw2Pressed = true;
                }
            }
        }
    }
}


bool timeout(const uint64_t start, const uint64_t time = MENU_TIMEOUT){
    return (time_us_64() - start) > time;
}

void getPressure(int *pressure){
    int pressureData;
    uint8_t start_cmd[] = {0xF1};
    uint8_t pressure_data[2];
    i2c_write_blocking(i2c1, SDP610_ADDR, start_cmd, 1, false);
    //sleep_ms(10);
    i2c_read_blocking(i2c1, SDP610_ADDR, pressure_data, 2, false);
    //sleep_ms(100);
    pressureData = ((pressure_data[0] << 8) | pressure_data[1])/SCALE_FACTOR*ALTITUDE_CORR_FACTOR;
    *pressure = pressureData > 130 ? 0 : pressureData;
}

int main() {

    char ssid[50];
    char truncatedSSID[50];
    char pass[50];
    char truncatedPass[50];
    char ip[16];
    int pressure = 0;
    int speed = 0;

    int setPoint = 0;   //used for set speed or pressure
    int setPointP_L = 0; //under limit pressure
    int setPointP_H = 0; //upper limit pressure
    uint16_t fanDelay = 200;  //delay time after setting fan speed
    uint8_t measureCount = 0;   //count number of measure that does not get desired pressure
    bool error = false;
    bool enableMeasurement = true;
    int delta = 10;
    bool pressureDroped = false;
    bool connectedMQTT = false;
    uint8_t tempBuf[1];

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
    gpio_set_irq_enabled_with_callback(SW_0, GPIO_IRQ_EDGE_FALL, true, &rot_handler);
    gpio_set_irq_enabled_with_callback(SW_2, GPIO_IRQ_EDGE_FALL, true, &rot_handler);

    printf("\nBoot\n");

    //get data stored from EEPROM
    read_from_eeprom(MODE_ADDR, tempBuf, 1);
    autoMode = tempBuf[0];
    if (autoMode != 0 && autoMode != 1){
        autoMode = true;
    }
    if (autoMode){
        read_from_eeprom(PRESSURE_ADDR, tempBuf, 1);
        setPoint = tempBuf[0];
        if(setPoint < 0 || setPoint > MAX_PRESSURE) setPoint = 0;
        speed = getSpeed(setPoint);
        setPointP_H = setPoint + OFFSET > MAX_PRESSURE ? MAX_PRESSURE:setPoint + OFFSET;
        setPointP_L = setPoint - OFFSET < 0 ? 0:setPoint - OFFSET;
    }
    else{
        read_from_eeprom(SPEED_ADDR, tempBuf, 1);
        setPoint = tempBuf[0];
        if(setPoint < 0 || setPoint > MAX_FAN_SPEED) setPoint = 0;
        speed = setPoint;
        pressure = 0;
    }
    //get network data
    get_network_eeprom(IP_ADDR, reinterpret_cast<uint8_t *>(ip));
    get_network_eeprom(PASS_ADDR, reinterpret_cast<uint8_t *>(pass));
    get_network_eeprom(SSID_ADDR, reinterpret_cast<uint8_t *>(ssid));

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
    screen.networkConnecting();
    //IPStack ipstack("SSID", "PASSWORD"); // example
    IPStack ipstack(ssid, pass); // example
    auto client = MQTT::Client<IPStack, Countdown, 256>(ipstack);
    int rc = ipstack.connect(ip, 1883);
    if (rc != 1) {
        printf("rc from TCP connect is %d\n", rc);
    }
    screen.networkConnecting();
    printf("MQTT connecting\n");
    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = 3;
    data.clientID.cstring = (char *) "PicoW-hannah";
    rc = client.connect(data);
    if (rc != 0) {
        printf("rc from MQTT connect is %d\n", rc);
        printf("Failed to connect to MQTT broker\n");
        connectedMQTT = false;
    }else{
        connectedMQTT = true;
        printf("MQTT connected\n");
    }
    if (connectedMQTT){
        // We subscribe QoS2. Messages sent with lower QoS will be delivered using the QoS they were sent with
        rc = client.subscribe(sub_topic, MQTT::QOS2, messageArrived);
        if (rc != 0) {
            printf("rc from MQTT subscribe is %d\n", rc);
            printf("Failed to subscribe MQTT\n");
        }
        printf("MQTT subscribed\n");
    }

    //auto mqtt_send = make_timeout_time_ms(3000);
    int msg_count = 0;
#endif

#ifdef USE_MODBUS
    auto uart{std::make_shared<PicoUart>(UART_NR, UART_TX_PIN, UART_RX_PIN, BAUD_RATE, STOP_BITS)};
    auto rtu_client{std::make_shared<ModbusClient>(uart)};
    ModbusRegister c2o(rtu_client, 240, 256);
    ModbusRegister rh(rtu_client, 241, 256);
    ModbusRegister tem(rtu_client, 241, 257);
    ModbusRegister fanSpeed(rtu_client, 1, 0);

    sleep_ms(100);
    if (speed > 8 && speed < 50){
        fanSpeed.write(500);
        sleep_ms(100);
    }
    fanSpeed.write(speed*10);
    sleep_ms(100);
    auto modbus_poll = make_timeout_time_ms(3000);
#endif

    markTime = time_us_64();    //start marking time for irq handler

    while (true) {
        if (menu != 2 && menu != 3){
            enableMeasurement = false;
        }
        if(menu == 0){
            if (!timeout(startTimeOut)){
                screen.screenSelection(option);
                irqReady = true;
            } else {    //timeout, show info screen
                menu = 2;
            }
        } else if(menu == 1){
            if (!timeout(startTimeOut)){
                if (autoMode){
                    screen.setPressure(valP);
                }else{
                    screen.setSpeed(valS);
                }
                irqReady = true;
            }else {    //timeout, show info screen
                menu = 2;
            }
        } else if(menu == 2){
            enableMeasurement = true;
            screen.info(autoMode,speed, pressure, temp, humidity, co2);
            if (receivedNewMsg){
                error = false;
                tempBuf[0] = autoMode;
                write_to_eeprom(MODE_ADDR, tempBuf, 1);
                setPoint = mqtt_value;
                if (autoMode){
                    if (setPoint > MAX_PRESSURE) setPoint = MAX_PRESSURE;
                    setPointP_H = setPoint + OFFSET > MAX_PRESSURE ? MAX_PRESSURE:setPoint + OFFSET;
                    setPointP_L = setPoint - OFFSET < 0 ? 0:setPoint - OFFSET;
                    speed = getSpeed(setPoint);
                    tempBuf[0] = setPoint;
                    write_to_eeprom(PRESSURE_ADDR, tempBuf, 1);
                }
                else{
                    speed = setPoint;
                    tempBuf[0] = setPoint;
                    write_to_eeprom(SPEED_ADDR, tempBuf, 1);
                    if (speed > 8 && speed < 50){
                        fanSpeed.write(500);
                        sleep_ms(100);
                    }
                    fanSpeed.write(speed*10);
                    sleep_ms(100);
                }
                receivedNewMsg = false;
            }
        } else if (menu == 3){
            screen.error();
        } else if (menu == 4){
            if (!timeout(startTimeOut)){
                screen.mqtt(option, ssid, pass, ip);
                irqReady = true;
            } else {    //timeout, show info screen
                menu = 2;
            }
        } else if (menu == 5){
            screen.asciiCharSelection(posX, option, asciiChar);
            irqReady = true;
        } else if (menu == 6){
            screen.askRestart();
        }
        //handle rot switch pressed to confirm setting and move to next screen
        if (sw0Pressed){
            if (menu < 2){
                measureCount = 0;
                if (menu == 0){
                    switch (option) {
                        case 0:
                            menu = 1;
                            autoMode = false;
                            valS = speed;
                            //write autoMode to eeprom
                            tempBuf[0] = autoMode;
                            write_to_eeprom(MODE_ADDR, tempBuf, 1);
                            break;
                        case 1:
                            menu = 1;
                            autoMode = true;
                            valP = pressure;
                            //write autoMode to eeprom
                            tempBuf[0] = autoMode;
                            write_to_eeprom(MODE_ADDR, tempBuf, 1);
                            break;
                        case 2:
                            menu = 2;
                            break;
                        case 3:
                            menu = 4;
                            option = 0;
                            break;
                        default:
                            break;
                    }
                } else if (menu == 1){
                    if (autoMode){
                        setPoint = valP;
                        setPointP_H = setPoint + OFFSET > MAX_PRESSURE ? MAX_PRESSURE:setPoint + OFFSET;
                        setPointP_L = setPoint - OFFSET < 0 ? 0:setPoint - OFFSET;
                        speed = getSpeed(setPoint);
                        //write autoMode to eeprom
                        tempBuf[0] = valP;
                        write_to_eeprom(PRESSURE_ADDR, tempBuf, 1);
                    } else {
                        setPoint = valS;
                        speed = setPoint;
                        //write autoMode to eeprom
                        tempBuf[0] = valS;
                        write_to_eeprom(SPEED_ADDR, tempBuf, 1);
                    }
                    menu = 2;
                    if (speed > 8 && speed < 50){
                        fanSpeed.write(500);
                        sleep_ms(100);
                    }
                    fanSpeed.write(speed*10);
                    if (autoMode){
                        sleep_ms(2000);
                    } else{
                        sleep_ms(100);
                    }
                }
                sw0Pressed = false;
            } else if (menu == 4){
                if (option == 3){
                    //save data, ask user to reset board
                    write_network_eeprom(SSID_ADDR, ssid);
                    write_network_eeprom(PASS_ADDR, pass);
                    write_network_eeprom(IP_ADDR, ip);
                    sw0Pressed = false;
                    menu = 6;
                } else if (option == 4){
                    menu = 2;
                    sw0Pressed = false;
                } else {
                    if (!timeout(startTimeOut, SW_0_TIMEOUT)){
                        if (gpio_get(SW_0)){
                            sw0Pressed = false;
                        }
                    } else {
                        sw0Pressed = false;
                        // enter editing
                        switch (option) {
                            case 0:
                                strcpy(ssid, "");
                                strcpy(truncatedSSID, "");
                                posX = SSID_POS_X;
                                break;
                            case 1:
                                strcpy(pass, "");
                                strcpy(truncatedPass, "");
                                posX = PASS_POS_X;
                                break;
                            case 2:
                                strcpy(ip, "");
                                posX = IP_POS_X;
                                break;
                            default:
                                break;
                        }
                        asciiChar = START_ASCII - 1;
                        screen.mqtt(option, ssid, pass, ip);
                        menu = 5;
                        sleep_ms(100);
                    }
                }
            } else if (menu == 5){
                if (!timeout(startTimeOut, SW_0_TIMEOUT)){
                    if (gpio_get(SW_0)){
                        char c[2];
                        sprintf(c, "%c", asciiChar);
                        sw0Pressed = false;
                        //confirm letter
                        switch (option) {
                            case 0:
                                strcat(ssid, c);
                                strcat(truncatedSSID, c);
                                break;
                            case 1:
                                strcat(pass, c);
                                strcat(truncatedPass, c);
                                break;
                            case 2:
                                strcat(ip, c);
                                break;
                        }
                        posX += CHAR_WIDTH;
                        if (posX > MAX_POS_X){
                            posX -= CHAR_WIDTH;
                            //scroll back 1 char
                            if (option == 0){
                                for (int i = 0; i < strlen(truncatedSSID); i++){
                                    truncatedSSID[i] = truncatedSSID[i+1];
                                }
                                screen.mqtt(option, truncatedSSID, pass, ip);
                            } else if (option == 1){
                                for (int i = 0; i < strlen(truncatedPass); i++){
                                    truncatedPass[i] = truncatedPass[i+1];
                                }
                                screen.mqtt(option, ssid, truncatedPass, ip);
                            }
                        }
                        asciiChar = START_ASCII - 1;
                    }
                } else {
                    sw0Pressed = false;
                    // exit editing
                    menu = 4;
                }
            } else {
                sw0Pressed = false;
            }
        }
        //handle sw0 switch pressed to change to menu 2
        if(sw2Pressed){
            option = 0;
            measureCount = 0;
            error = false;
            menu = 0;
            sw2Pressed = false;
        }
//start measurement

        if (enableMeasurement){
#ifdef USE_MODBUS
            if (time_reached(modbus_poll)) {
                gpio_put(led_pin, !gpio_get(led_pin)); // toggle  led
                modbus_poll = delayed_by_ms(modbus_poll, 3000);
                co2 = c2o.read();
                //printf("CO2   = %d ppm\n", co2);
                humidity = rh.read()/10;
                //printf("RH    = %d%%\n", humidity);
                temp = tem.read()/10;
                //printf("T     = %d C\n", temp);
                //printf("S     = %.1f\n", speed);

                getPressure(&pressure);
                if (autoMode){
                    if (pressure < setPointP_L){
                        measureCount++;
                        if (measureCount >= 20){
                            menu = 3;   //show error screen
                            measureCount = 20;
                            error = true;
                        }
                        if ((setPointP_L - pressure) > 5){
                            speed += delta;
                            pressureDroped = true;
                        } else{
                            speed++;
                            pressureDroped = false;
                        }
                        if (speed > MAX_FAN_SPEED) speed = MAX_FAN_SPEED;
                        fanSpeed.write(speed*10);
                        sleep_ms(fanDelay);
                    } else if ((pressure > setPointP_H) && !pressureDroped){
                        measureCount = 0;
                        error = false;
                        if (speed > 8){
                            if ((pressure - setPointP_H) > 5){
                                speed = getSpeed(setPoint);
                            } else {
                                speed--;
                            }
                        }
                        fanSpeed.write(speed*10);
                        sleep_ms(fanDelay);
                        if (menu >= 2){
                            menu = 2;
                        }
                    } else if ((pressure > setPointP_H) && pressureDroped){
                        measureCount = 0;
                        error = false;
                        if ((pressure - setPointP_H) > 5){
                            speed -= delta-2;
                            if (speed < 0) speed = 0;
                        } else {
                            speed--;
                            pressureDroped = false;
                        }
                        if (speed < 0) speed = 0;
                        fanSpeed.write(speed*10);
                        sleep_ms(fanDelay);
                        if (menu >= 2){
                            menu = 2;
                        }
                    }
                }
#ifdef USE_MQTT
                if (connectedMQTT){
                    if (!client.isConnected()) {
                        //printf("Not connected...\n");
                        rc = client.connect(data);
                        if (rc != 0) {
                            //printf("rc from MQTT connect is %d\n", rc);
                            connectedMQTT = false;
                        }
                    }
                    // Construct JSON message
                    char buf[256];
                    sprintf(buf, R"({"nr": %d, "speed": %d, "setpoint": %d, "pressure": %d, "auto": %s, "error": %s, "co2": %d, "rh": %d, "temp": %d})",
                            ++msg_count, speed, setPoint, pressure, autoMode ? "true" : "false", error ? "true" : "false", co2, humidity, temp);
                    MQTT::Message message;
                    message.retained = false;
                    message.dup = false;
                    message.payload = (void *)buf;
                    message.qos = MQTT::QOS0;
                    message.payloadlen = strlen(buf);
                    rc = client.publish(pub_topic, message);
                    //printf("Publish rc=%d\n", rc);
                }
#endif
            }
#endif
#ifdef USE_MQTT
                if (connectedMQTT){
                    cyw43_arch_poll(); // obsolete? - see below
                    client.yield(100); // socket that client uses calls cyw43_arch_poll()
                }

#endif
        }
    }
}

