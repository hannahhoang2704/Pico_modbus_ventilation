//
// Created by Keijo Länsikunnas on 11.2.2024.
//

#ifndef UART_IRQ_COUNTDOWN_H
#define UART_IRQ_COUNTDOWN_H
#include "pico/time.h"


class Countdown
{
public:
    Countdown();
    explicit Countdown(int ms);
    bool expired();
    void countdown_ms(int ms);
    void countdown(int seconds);
    int left_ms();
private:
    absolute_time_t target_time;
};


#endif //UART_IRQ_COUNTDOWN_H
