//
// Created by Hanh Hoang on 28.2.2024.
//

#ifndef PICO_MODBUS_BUTTON_H
#define PICO_MODBUS_BUTTON_H

#include <stdio.h>
#include "pico/stdlib.h"

#define SW_0 9
#define SW_1 8
#define SW_2 7
//define for rotary button
#define ROT_A 10
#define ROT_B 11
#define ROT_SW 12

class Button{
public:
    explicit Button(uint btn_nr):pin_nr(btn_nr){
        init_pin();
    };
    void init_pin();
    bool debounced_pressed();
    uint get_pin(){return pin_nr;}
private:
    uint pin_nr;
//    bool pressed= false;
};

void init_rotary_knob();

#endif //PICO_MODBUS_BUTTON_H
