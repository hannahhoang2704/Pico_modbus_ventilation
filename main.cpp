#include <stdio.h>
#include <string.h>
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
#include "pico/util/queue.h"

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
#define BAUD_RATE 9600

//#define USE_MODBUS
//#define USE_MQTT
#define USE_SSD1306
#define EVENT_DEBOUNCE_US 8000
#define QUEUE_SIZE 1
#define TIMEOUT 60000000    //60s

void messageArrived(MQTT::MessageData &md) {
    MQTT::Message &message = md.message;

    printf("Message arrived: qos %d, retained %d, dup %d, packetid %d\n",
           message.qos, message.retained, message.dup, message.id);
    printf("Payload %s\n", (char *) message.payload);
}

static const char *topic = "test-topic";
static uint8_t menu = 0;
volatile bool autoMode = false;
volatile int pressure = 0;
volatile int speed = 0;
volatile bool rotPressed = false;
static int temp = 0, humidity = 0, co2 = 0;
volatile uint64_t startTimeOut = 0;

queue_t modeQueue, pressureQueue, speedQueue;

void rot_handler(uint gpio, uint32_t event_mask) {
    if(gpio == ROT_A){
        startTimeOut = time_us_64();
        if (menu == 0){
            autoMode = !autoMode;
            queue_try_add(&modeQueue, (const void *) &autoMode);
        } else if (menu == 1){
            if (autoMode){
                if(gpio_get(ROT_B)){
                    pressure = pressure == 120 ? 120 : pressure + 1;
                } else {
                    pressure = pressure == 0 ? 0 : pressure - 1;
                }
                queue_try_add(&pressureQueue, (const void *) &pressure);
            } else{
                if(gpio_get(ROT_B)){
                    speed = speed == 100 ? 100 : speed + 1 ;
                } else {
                    speed = speed == 0 ? 0 : speed - 1;
                }
                queue_try_add(&speedQueue, (const void *) &speed);
            }
        }
    } else if (gpio == ROT_SW){
        startTimeOut = time_us_64();
        rotPressed = true;
    }
}

bool timeout(const uint64_t start){
    return (time_us_64() - start) > TIMEOUT;
}

int main() {

    const uint led_pin = 22;
    // Initialize LED pin
    gpio_init(led_pin);
    gpio_set_dir(led_pin, GPIO_OUT);

    Button sw0(SW_0);
    Button sw1(SW_1);
    Button sw2(SW_2);
    //init_eeprom();
    init_rotary_knob();

    queue_init(&modeQueue, sizeof(bool), QUEUE_SIZE);
    queue_init(&pressureQueue, sizeof(int), QUEUE_SIZE);
    queue_init(&speedQueue, sizeof(int), QUEUE_SIZE);

    gpio_set_irq_enabled_with_callback(ROT_A, GPIO_IRQ_EDGE_FALL, true, &rot_handler);
    gpio_set_irq_enabled_with_callback(ROT_SW, GPIO_IRQ_EDGE_FALL, true, &rot_handler);

    // Initialize chosen serial port
    stdio_init_all();

    printf("\nBoot\n");
/*
    //get data stored from EEPROM
    mode = get_stored_value(MODE_ADDR);
    if(mode < 0 || mode > 1) mode = 0;
    speed = get_stored_value(SPEED_ADDR);
    if(speed < 0 || speed > MAX_FAN_SPEED) speed = 0;
    pressure = get_stored_value(PRESSURE_ADDR);
    if(pressure < 0 || pressure > MAX_PRESSURE) pressure = 0;
*/
#ifdef USE_SSD1306
    // I2C is "open drain",
    // pull ups to keep signal high when no data is being sent
    i2c_init(i2c1, 100 * 1000);
    gpio_set_function(14, GPIO_FUNC_I2C); // the display has external pull-ups
    gpio_set_function(15, GPIO_FUNC_I2C); // the display has external pull-ups
    auto display = std::make_shared<ssd1306>(i2c1);
    currentScreen screen(display);
    screen.modeSelection(autoMode);
//    sleep_ms(2000);
//    screen.modeSelection(1);
//    sleep_ms(2000);
//    for (int i=0;i<121;i++){
//        screen.paramSet(0,i);
//        sleep_ms(50);
//    }
//    sleep_ms(2000);
//    for (int i=0;i<101;i++){
//        screen.paramSet(1,i);
//        sleep_ms(50);
//    }
//    sleep_ms(2000);
//    screen.info(100, 100, 20, 50, 100);
//    sleep_ms(2000);
//    screen.error();

#endif


#ifdef USE_MQTT
    //IPStack ipstack("SSID", "PASSWORD"); // example
    IPStack ipstack("KME662", "SmartIot"); // example
    auto client = MQTT::Client<IPStack, Countdown>(ipstack);

    int rc = ipstack.connect("192.168.1.10", 1883);
    if (rc != 1) {
        printf("rc from TCP connect is %d\n", rc);
    }

    printf("MQTT connecting\n");
    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = 3;
    data.clientID.cstring = (char *) "PicoW-sample";
    rc = client.connect(data);
    if (rc != 0) {
        printf("rc from MQTT connect is %d\n", rc);
        while (true) {
            tight_loop_contents();
        }
    }
    printf("MQTT connected\n");

    // We subscribe QoS2. Messages sent with lower QoS will be delivered using the QoS they were sent with
    rc = client.subscribe(topic, MQTT::QOS2, messageArrived);
    if (rc != 0) {
        printf("rc from MQTT subscribe is %d\n", rc);
    }
    printf("MQTT subscribed\n");

    auto mqtt_send = make_timeout_time_ms(2000);
    int mqtt_qos = 0;
    int msg_count = 0;
#endif

#ifdef USE_MODBUS
    auto uart{std::make_shared<PicoUart>(UART_NR, UART_TX_PIN, UART_RX_PIN, BAUD_RATE)};
    auto rtu_client{std::make_shared<ModbusClient>(uart)};
    ModbusRegister rh(rtu_client, 241, 256);
    auto modbus_poll = make_timeout_time_ms(3000);
#endif
    bool lastMode = autoMode;
    int lastPressureVal = pressure;
    int lastSpeedval = speed;
    bool modeInQueue;
    int valInQueue;
    while (true) {
        modeInQueue = false;
        valInQueue = 0;
        if(menu == 0){
            if (!timeout(startTimeOut)){
                if (queue_try_remove(&modeQueue, &modeInQueue)) {
                    // update only if mode has changed
                    if (modeInQueue != lastMode) {
                        autoMode = modeInQueue;
                        lastMode = autoMode;
                    }
                }
                screen.modeSelection(autoMode);
            } else {    //timeout, show info screen
                menu = 2;
            }

        }else if(menu == 1){
            if (!timeout(startTimeOut)){
                if (autoMode){
                    if (queue_try_remove(&pressureQueue, &valInQueue)) {
                        if (valInQueue != lastPressureVal){
                            pressure = valInQueue;
                            lastPressureVal = valInQueue;
                        }
                    }
                    screen.paramSet(autoMode,pressure);
                }else{
                    if (queue_try_remove(&speedQueue, &valInQueue)) {
                        if (valInQueue != lastSpeedval){
                            speed = valInQueue;
                            lastSpeedval = valInQueue;
                        }
                    }
                    screen.paramSet(autoMode, speed);
                }
            }else {    //timeout, show info screen
                menu = 2;
            }
        }
        else if(menu == 2)
            screen.info(speed, pressure, temp, humidity, co2);
        if (rotPressed){
            sleep_ms(30);
            if (!gpio_get(ROT_SW)){
                if (menu < 2){
                    menu++;
                }
            }
            rotPressed = false;
        }
        if(sw2.debounced_pressed()){
            menu = 0;
            startTimeOut = time_us_64();
        }
        if(sw1.debounced_pressed()){
            menu = 1;
            startTimeOut = time_us_64();
        }
        if(sw0.debounced_pressed()){
            menu = 2;
            startTimeOut = time_us_64();
        }


#ifdef USE_MODBUS
        if (time_reached(modbus_poll)) {
            gpio_put(led_pin, !gpio_get(led_pin)); // toggle  led
            modbus_poll = delayed_by_ms(modbus_poll, 3000);
            printf("RH=%5.1f%%\n", rh.read() / 10.0);
        }
#endif
#ifdef USE_MQTT
        if (time_reached(mqtt_send)) {
            mqtt_send = delayed_by_ms(mqtt_send, 2000);
            if (!client.isConnected()) {
                printf("Not connected...\n");
                rc = client.connect(data);
                if (rc != 0) {
                    printf("rc from MQTT connect is %d\n", rc);
                }

            }
            char buf[100];
            int rc = 0;
            MQTT::Message message;
            message.retained = false;
            message.dup = false;
            message.payload = (void *) buf;
            switch (mqtt_qos) {
                case 0:
                    // Send and receive QoS 0 message
                    sprintf(buf, "Msg nr: %s QoS 0 message", ++msg_count);
                    printf("%s\n", buf);
                    message.qos = MQTT::QOS0;
                    message.payloadlen = strlen(buf) + 1;
                    rc = client.publish(topic, message);
                    printf("Publish rc=%d\n", rc);
                    ++mqtt_qos;
                    break;
                case 1:
                    // Send and receive QoS 1 message
                    sprintf(buf, "Msg nr: %s QoS 1 message", ++msg_count);
                    printf("%s\n", buf);
                    message.qos = MQTT::QOS1;
                    message.payloadlen = strlen(buf) + 1;
                    rc = client.publish(topic, message);
                    printf("Publish rc=%d\n", rc);
                    ++mqtt_qos;
                    break;
#if MQTTCLIENT_QOS2
                    case 2:
                        // Send and receive QoS 2 message
                        sprintf(buf, "Msg nr: %s QoS 2 message", ++msg_count);
                        printf("%s\n", buf);
                        message.qos = MQTT::QOS2;
                        message.payloadlen = strlen(buf) + 1;
                        rc = client.publish(topic, message);
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

