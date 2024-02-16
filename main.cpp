#include <stdio.h>
#include <string.h>
#include <cmath>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/timer.h"
#include "nanomodbus.h"
#include "PicoUart.h"

#include "IPStack.h"
#include "Countdown.h"
#include "MQTTClient.h"
#include "ModbusClient.h"
#include "ModbusRegister.h"

#define STRLEN 80


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

#define BAUD_RATE 9600
#define DBG_PIN1 16
#define DBG_PIN2 16

int32_t uart_transport_read(uint8_t *buf, uint16_t count, int32_t byte_timeout_ms,
                            void *arg); /*!< Bytes read transport function pointer */
int32_t uart_transport_write(const uint8_t *buf, uint16_t count, int32_t byte_timeout_ms,
                             void *arg);/*!< Bytes write transport function pointer */

void messageArrived(MQTT::MessageData &md) {
    static int arrivedcount = 0; //yuck, get rid of this
    MQTT::Message &message = md.message;

    printf("Message %d arrived: qos %d, retained %d, dup %d, packetid %d\n",
           ++arrivedcount, message.qos, message.retained, message.dup, message.id);
    printf("Payload %s\n", (char *) message.payload);
}

static const char *topic = "test-topic";

int main() {

    const uint led_pin = 22;
    const uint button = 9;

    // Initialize LED pin
    gpio_init(led_pin);
    gpio_set_dir(led_pin, GPIO_OUT);

    // Initialize debug pins
    gpio_init(DBG_PIN1);
    gpio_set_dir(DBG_PIN1, GPIO_OUT);
    gpio_put(DBG_PIN1, false);

    gpio_init(DBG_PIN2);
    gpio_set_dir(DBG_PIN2, GPIO_OUT);
    gpio_put(DBG_PIN2, false);

    gpio_init(button);
    gpio_set_dir(button, GPIO_IN);
    gpio_pull_up(button);

    // Initialize chosen serial port
    stdio_init_all();

    printf("\nBoot\n");

    //PicoUart uart(UART_NR, UART_TX_PIN, UART_RX_PIN, BAUD_RATE);
    //sleep_ms(100);
    //printf("Flushed: %d\n", uart.flush());
    auto uart = std::make_shared<PicoUart>(UART_NR, UART_TX_PIN, UART_RX_PIN, BAUD_RATE);
    auto rtu_client = std::make_shared<ModbusClient>(uart);
    ModbusRegister rh(rtu_client, 241, 256);

    //IPStack ipstack("SSID", "PASSWORD"); // example
    IPStack ipstack("KME662", "SmartIot"); // example
    //IPStack ipstack("DSP_INTRA", "deadbeef42");
    auto client = MQTT::Client<IPStack, Countdown>(ipstack);

    int rc = ipstack.connect("192.168.1.163", 1883);
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
    }
    printf("MQTT connected\n");

    rc = client.subscribe(topic, MQTT::QOS2, messageArrived);
    if (rc != 0) {
        printf("rc from MQTT subscribe is %d\n", rc);
    }
    printf("MQTT subscribed\n");

    auto modbus_poll = make_timeout_time_ms(3000);
    auto mqtt_send = make_timeout_time_ms(2000);
    int mqtt_qos = 0;

    while (true) {
        if (time_reached(modbus_poll)) {
            modbus_poll = delayed_by_ms(modbus_poll, 3000);
            printf("RH=%5.1f%%\n", rh.read() / 10.0);
        }
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
                    sprintf(buf, "Hello World! QoS 0 message");
                    printf("%s\n", buf);
                    message.qos = MQTT::QOS0;
                    message.payloadlen = strlen(buf) + 1;
                    rc = client.publish(topic, message);
                    ++mqtt_qos;
                    break;
                case 1:
                    // Send and receive QoS 1 message
                    sprintf(buf, "Hello World!  QoS 1 message");
                    printf("%s\n", buf);
                    message.qos = MQTT::QOS1;
                    message.payloadlen = strlen(buf) + 1;
                    rc = client.publish(topic, message);
                    ++mqtt_qos;
                    //mqtt_qos = 0; // seems that QoS2 message breaks connection - socket issue?
                    break;
                case 2:
#ifdef MQTTCLIENT_QOS2
#if MQTTCLIENT_QOS2
                    // Send and receive QoS 2 message
                    sprintf(buf, "Hello World!  QoS 2 message");
                    printf("%s\n", buf);
                    message.qos = MQTT::QOS2;
                    message.payloadlen = strlen(buf) + 1;
                    rc = client.publish(topic, message);
#endif
#endif
                    mqtt_qos = 0;
                    break;
                default:
                    mqtt_qos = 0;
                    break;
            }
            printf("Publish rc=%d\n", rc);
        }

        cyw43_arch_poll(); // obsolete? - see below
        client.yield(100); // socket that client uses calls cyw43_arch_poll()
    }

    // Loop forever
    while (true) {

        // Blink LED
        gpio_put(led_pin, true);
        sleep_ms(1000);
        gpio_put(led_pin, false);
        sleep_ms(1000);
    }

}

int32_t uart_transport_write(const uint8_t *buf, uint16_t count, int32_t byte_timeout_ms, void *arg) {
    // we can ignore write timeout with UART
    (void) byte_timeout_ms;
    return static_cast<PicoUart *>(arg)->write(buf, count);
}

int32_t uart_transport_read(uint8_t *buf, uint16_t count, int32_t byte_timeout_ms, void *arg) {
    auto uart = static_cast<PicoUart *>(arg);
    bool notimeout = byte_timeout_ms < 0;
    uint64_t timeout = byte_timeout_ms < 0 ? 0 : byte_timeout_ms * 1000ULL;
    uint flv = uart->get_fifo_level();
    if (flv) {
        // The timeout must be atleast fifo level x bits/ch x bit time.
        // Minimum fifo level is 4.
        // There is also fifo inactivity timeout which is fixed at 32 bit time
        // Worst case delay is fifo level - 1 + inactivity timeout
        // To play safe we set it to fifo level + inactivity timeout
        uint64_t fifo_to = ((flv * 10 + 32) * 1000000ULL) / uart->get_baud();
        // if delay caused by fifo is longer than requested byte timeout
        // then use the fifo timeout
        if (timeout < fifo_to) timeout = fifo_to;
    }
    // debug printout
    //printf("flv=%u, bto=%d, cnt=%d, to=%u\n",flv,byte_timeout_ms,count, (uint) timeout);

    absolute_time_t to = make_timeout_time_us(timeout);
    int32_t rcnt = 0;
    while (rcnt < count && (notimeout || !time_reached(to))) {
        //gpio_put(DBG_PIN1, true);
        int32_t cnt = uart->read(buf + rcnt, count - rcnt);
        rcnt += cnt;
        if (cnt > 0) {
            //gpio_put(DBG_PIN1, false);
            // if we received new data update the timeout
            to = make_timeout_time_us(timeout);
        }
    }
    //gpio_put(DBG_PIN1, false);

    return rcnt;
}
