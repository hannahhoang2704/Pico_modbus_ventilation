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

#define USE_MODBUS
//#define USE_MQTT
#define USE_SSD1306
#define EVENT_DEBOUNCE_US 8000

#ifdef USE_SSD1306
static const uint8_t raspberry26x32[] =
        {0x0, 0x0, 0xe, 0x7e, 0xfe, 0xff, 0xff, 0xff,
         0xff, 0xff, 0xfe, 0xfe, 0xfc, 0xf8, 0xfc, 0xfe,
         0xfe, 0xff, 0xff,0xff, 0xff, 0xff, 0xfe, 0x7e,
         0x1e, 0x0, 0x0, 0x0, 0x80, 0xe0, 0xf8, 0xfd,
         0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff,
         0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd,
         0xf8, 0xe0, 0x80, 0x0, 0x0, 0x1e, 0x7f, 0xff,
         0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
         0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
         0xff, 0xff, 0xff, 0xff, 0x7f, 0x1e, 0x0, 0x0,
         0x0, 0x3, 0x7, 0xf, 0x1f, 0x1f, 0x3f, 0x3f,
         0x7f, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x7f, 0x3f,
         0x3f, 0x1f, 0x1f, 0xf, 0x7, 0x3, 0x0, 0x0 };
#endif

void messageArrived(MQTT::MessageData &md) {
    MQTT::Message &message = md.message;

    printf("Message arrived: qos %d, retained %d, dup %d, packetid %d\n",
           message.qos, message.retained, message.dup, message.id);
    printf("Payload %s\n", (char *) message.payload);
}

static const char *topic = "test-topic";
static uint8_t menu = 0;
static uint8_t pressure = 0, mode = 0, speed = 0;

static void rot_handler(uint gpio, uint32_t event_mask) {
    static uint64_t prev_event_time = 0;
    uint64_t curr_time = time_us_64();
    if (gpio == ROT_A && !gpio_get(ROT_B)) {
        if(menu == 0){          //mode selection screen
            if(mode < 1){
                mode = 1;
            }
        }else if (menu == 1 && mode == 0){   //pressure adjustment screen
            if(pressure < 100) pressure++;
        } else if (menu == 1 && mode == 1){
            if(speed < 100) speed++;
        }
    } else if (gpio == ROT_A && gpio_get(ROT_B)) {
        if(menu == 0){
            if(mode >= 1) mode = 0;
        }else if(menu == 1 && mode == 0){
            if(pressure > 0) pressure--;
        }else if(menu == 1 && mode == 1){
            if(speed > 0) speed--;
        }
    }else if (gpio == ROT_SW){
        if (curr_time - prev_event_time > EVENT_DEBOUNCE_US){
            prev_event_time = curr_time;
            if(menu < 2) menu++;
        }
    }
}
int main() {

    const uint led_pin = 22;
    // Initialize LED pin
    gpio_init(led_pin);
    gpio_set_dir(led_pin, GPIO_OUT);

    Button sw0(SW_0);
    Button sw1(SW_1);
    Button sw2(SW_2);
    init_eeprom();
    init_rotary_knob();

    gpio_set_irq_enabled_with_callback(ROT_A, GPIO_IRQ_EDGE_RISE, true, &rot_handler);
    gpio_set_irq_enabled_with_callback(ROT_SW, GPIO_IRQ_EDGE_RISE, true, &rot_handler);

    // Initialize chosen serial port
    stdio_init_all();

    printf("\nBoot\n");

    //get data stored from EEPROM
    mode = get_stored_value(MODE_ADDR);
    if(mode < 0 || mode > 1) mode = 0;
    speed = get_stored_value(SPEED_ADDR);
    if(speed < 0 || speed > MAX_FAN_SPEED) speed = 0;
    pressure = get_stored_value(PRESSURE_ADDR);
    if(pressure < 0 || pressure > MAX_PRESSURE) pressure = 0;

#ifdef USE_SSD1306
    // I2C is "open drain",
    // pull ups to keep signal high when no data is being sent
    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(14, GPIO_FUNC_I2C); // the display has external pull-ups
    gpio_set_function(15, GPIO_FUNC_I2C); // the display has external pull-ups
    auto display = std::make_shared<ssd1306>(i2c1);
//    ssd1306 display(i2c1);
//    display.fill(0);
//    display.text("Hello", 0, 0);
//    mono_vlsb rb(raspberry26x32, 26, 32);
//    display.blit(rb, 20, 20);
//    display.rect(15, 15, 35, 45, 1);
//    display.line(60, 5, 120, 60, 1);
//    display.line(60, 60, 120, 5, 1);
//    display.show();
    currentScreen screen(display);
//    screen.modeSelection(0);
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

#if 0
    for(int i = 0; i < 128; ++i) {
        sleep_ms(50);
        display.scroll(1, 0);
        display.show();
    }
    display.text("Done", 20, 20);
    display.show();
#endif

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
    while (true) {
        if(menu==0){
            screen.modeSelection(mode);
        }else if(menu==1 && mode == 0){
            screen.paramSet(mode, pressure);
        }else if(menu==1 && mode == 1){
            screen.paramSet(mode, speed);
        }else if(menu==2){
            screen.info(speed, pressure, 25, 30, 300);
        }
        if(sw2.debounced_pressed()){
            menu = 0;
        }
        if(sw1.debounced_pressed()){
            menu = 1;
        }
        if(sw0.debounced_pressed()){
            menu = 2;
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

