//
// Created by Keijo Länsikunnas on 4.2.2024.
//

#ifndef UART_IRQ_PICOUART_H
#define UART_IRQ_PICOUART_H

#include <hardware/uart.h>
#include <hardware/irq.h>
#include <string>
#include "RingBuffer.h"

class PicoUart {
    friend void pico_uart0_handler(void);
    friend void pico_uart1_handler(void);
public:
    PicoUart(int uart_nr, int tx_pin, int rx_pin, int speed, int tx_size = 256, int rx_size = 256);
    PicoUart(const PicoUart &) = delete; // prevent copying because each instance is associated with a HW peripheral
    int read(uint8_t *buffer, int size);
    int write(const uint8_t *buffer, int size);
    int send(const char *str);
    int send(const std::string &str);
    int flush();
    int get_fifo_level();
    int get_baud();
private:
    void uart_irq_rx();
    void uart_irq_tx();
    RingBuffer tx;
    RingBuffer rx;
    uart_inst_t *uart;
    int irqn;
    int speed;
};


#endif //UART_IRQ_PICOUART_H
