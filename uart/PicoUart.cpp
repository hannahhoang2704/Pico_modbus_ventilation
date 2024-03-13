//
// Created by Keijo Länsikunnas on 4.2.2024.
//

#include <hardware/gpio.h>
#include <cstring>
#include "PicoUart.h"


static PicoUart *pu0;
static PicoUart *pu1;


void pico_uart0_handler(void) {
    if(pu0) {
        pu0->uart_irq_rx();
        pu0->uart_irq_tx();
    }
    else irq_set_enabled(UART0_IRQ, false);
}

void pico_uart1_handler(void) {
    if(pu1) {
        pu1->uart_irq_rx();
        pu1->uart_irq_tx();
    }
    else irq_set_enabled(UART1_IRQ, false);
}


PicoUart::PicoUart(int uart_nr, int tx_pin, int rx_pin, int speed, int stop, int tx_size, int rx_size) :tx(tx_size), rx(rx_size), speed{speed} {
    irqn = uart_nr==0 ? UART0_IRQ : UART1_IRQ;
    uart = uart_nr==0 ? uart0 : uart1;
    if(uart_nr == 0) {
        pu0 = this;
    }
    else {
        pu1 = this;
    }

    // ensure that we don't get any interrupts from the uart during configuration
    irq_set_enabled(irqn, false);

    // Set up our UART with the required speed.
    uart_init(uart, speed);
    uart_set_format(uart, 8, stop, UART_PARITY_NONE);

    // Set the TX and RX pins by using the function select on the GPIO
    // See datasheet for more information on function select
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);

    irq_set_exclusive_handler(irqn, uart_nr == 0 ? pico_uart0_handler : pico_uart1_handler);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(uart, true, false);
    // enable UART0 interrupts on NVIC
    irq_set_enabled(irqn, true);
}

int PicoUart::read(uint8_t *buffer, int size) {
    int count = 0;
    while(count < size && !rx.empty()) {
        *buffer++ = rx.get();
        ++count;
    }
    return count;
}

int PicoUart::write(const uint8_t *buffer, int size) {
    int count = 0;
    // write data to ring buffer
    while(count < size && !tx.full()) {
        tx.put(*buffer++);
        ++count;
    }
    // disable interrupts on NVIC while managing transmit interrupts
    irq_set_enabled(irqn, false);
    // if transmit interrupt is not enabled we need to enable it and give fifo an initial filling
    if(!(uart_get_hw(uart)->imsc & (1 << UART_UARTIMSC_TXIM_LSB))) {
        // enable transmit interrupt
        uart_set_irq_enables(uart, true, true);
        // fifo requires initial filling
        uart_irq_tx();
    }
    // enable interrupts on NVIC
    irq_set_enabled(irqn, true);

    return count;
}

int PicoUart::send(const char *str) {
    write(reinterpret_cast<const uint8_t *>(str), strlen(str));
    return 0;
}

int PicoUart::send(const std::string &str) {
    write(reinterpret_cast<const uint8_t *>(str.c_str()), str.length());
    return 0;
}

int PicoUart::flush() {
    int count = 0;
    while(!rx.empty()) {
        (void) rx.get();
        ++count;
    }
    return count;
}

void PicoUart::uart_irq_rx() {
    while(uart_is_readable(uart)) {
        uint8_t c = uart_getc(uart);
        // ignoring return value for now
        rx.put(c);
    }

}

void PicoUart::uart_irq_tx() {
    while(!tx.empty() && uart_is_writable(uart)) {
        uart_get_hw(uart)->dr = tx.get();
    }

    if (tx.empty()) {
        // disable tx interrupt if transmit buffer is empty
        uart_set_irq_enables(uart, true, false);
    }

}

int PicoUart::get_fifo_level() {
    const uint8_t flv[]={4, 8,16, 24, 28, 0, 0, 0, 0 };
    // figure out fifo level to calculate timeout
    uint32_t lcr_h = uart_get_hw(uart)->lcr_h;
    uint32_t fcr = (uart_get_hw(uart)->ifls >> 3) & 0x7;
    // if fifo is enabled we need to take into account delay caused by the fifo
    if(!(lcr_h | UART_UARTLCR_H_FEN_BITS)) {
        fcr = 8; // last is dummy entry that is outside of normal fcr range. it is used to ensure we return zero
    }
    return flv[fcr];
}

int PicoUart::get_baud() {
    return speed;
}
