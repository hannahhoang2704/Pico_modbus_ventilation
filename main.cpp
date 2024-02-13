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

int32_t uart_transport_read(uint8_t* buf, uint16_t count, int32_t byte_timeout_ms,
                void* arg); /*!< Bytes read transport function pointer */
int32_t uart_transport_write(const uint8_t* buf, uint16_t count, int32_t byte_timeout_ms,
                 void* arg);/*!< Bytes write transport function pointer */

void messageArrived(MQTT::MessageData& md)
{
    static int arrivedcount = 0; //yuck, get rid of this
    MQTT::Message &message = md.message;

    printf("Message %d arrived: qos %d, retained %d, dup %d, packetid %d\n",
            ++arrivedcount, message.qos, message.retained, message.dup, message.id);
    printf("Payload %s\n", (char*)message.payload);
}


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

    PicoUart uart(UART_NR, UART_TX_PIN, UART_RX_PIN, BAUD_RATE);
    sleep_ms(100);
    printf("Flushed: %d\n", uart.flush());
    // my_transport_read() and my_transport_write() are implemented by the user
    nmbs_platform_conf platform_conf;
    platform_conf.transport = NMBS_TRANSPORT_RTU;
    platform_conf.read = uart_transport_read;
    platform_conf.write = uart_transport_write;
    platform_conf.arg =  (void *)&uart;    // Passing our uart handle to the read/write functions

    // Create the modbus client
    nmbs_t nmbs;
    // client create clears the structure so we need to set parameters after calling create
    nmbs_error err = nmbs_client_create(&nmbs, &platform_conf);
    if (err != NMBS_ERROR_NONE) {
        fprintf(stderr, "Error creating modbus client\n");
        return 1;
    }
    nmbs_set_destination_rtu_address(&nmbs, 241);
    // Set only the response timeout.
    nmbs_set_read_timeout(&nmbs, 1000);
    // set byte timeout. Standard says 1.5 x byte time between chars and 3.5 x byte time to end frame
    // so we choose 3 x byte time --> 3 ms @ 9600bps
    nmbs_set_byte_timeout(&nmbs, 3);
#if 0
    // Write 2 holding registers at address 26
    uint16_t w_regs[2] = {123, 124};
    err = nmbs_write_multiple_registers(&nmbs, 26, 2, w_regs);
    if (err != NMBS_ERROR_NONE) {
        fprintf(stderr, "Error writing register at address 26 - %s", nmbs_strerror(err));
        return 1;
    }
#endif

    IPStack ipstack;
    MQTT::Client<IPStack, Countdown> client = MQTT::Client<IPStack, Countdown>(ipstack);

    int rc = ipstack.connect("192.168.1.10", 1883);
    if (rc != 1)
    {
        printf("rc from TCP connect is %d\n", rc);
    }

    printf("MQTT connecting\n");
    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = 3;
    data.clientID.cstring = (char*)"arduino-sample";
    rc = client.connect(data);
    if (rc != 0)
    {
        printf("rc from MQTT connect is %d\n", rc);
    }
    printf("MQTT connected\n");

    rc = client.subscribe("test-topic", MQTT::QOS2, messageArrived);
    if (rc != 0)
    {
        printf("rc from MQTT subscribe is %d\n", rc);
    }
    printf("MQTT subscribed\n");


    while(true) {
        // Read 1 holding registers from address 256
        uint16_t r_regs[2];
        err = nmbs_read_holding_registers(&nmbs, 256, 1, r_regs);
        if (err != NMBS_ERROR_NONE) {
            fprintf(stderr, "Error reading 1 holding registers at address 256 - %s\n", nmbs_strerror(err));
            //return 1;
            sleep_ms(100);
        }
        else {
            printf("RH=%5.1f%%\n", r_regs[0] / 10.0);
            sleep_ms(3000);
        }
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
    if(flv) {
        // The timeout must be atleast fifo level x bits/ch x bit time.
        // Minimum fifo level is 4.
        // There is also fifo inactivity timeout which is fixed at 32 bit time
        // Worst case delay is fifo level - 1 + inactivity timeout
        // To play safe we set it to fifo level + inactivity timeout
        uint64_t fifo_to = ((flv * 10 + 32) * 1000000ULL) / uart->get_baud();
        // if delay caused by fifo is longer than requested byte timeout
        // then use the fifo timeout
        if(timeout < fifo_to) timeout = fifo_to;
    }
    // debug printout
    //printf("flv=%u, bto=%d, cnt=%d, to=%u\n",flv,byte_timeout_ms,count, (uint) timeout);

    absolute_time_t to = make_timeout_time_us(timeout );
    int32_t rcnt = 0;
    while(rcnt < count && (notimeout || !time_reached(to)) ) {
        //gpio_put(DBG_PIN1, true);
        int32_t cnt = uart->read(buf + rcnt, count - rcnt);
        rcnt += cnt;
        if(cnt > 0){
            //gpio_put(DBG_PIN1, false);
            // if we received new data update the timeout
            to = make_timeout_time_us(timeout);
        }
    }
    //gpio_put(DBG_PIN1, false);

    return rcnt;
}
