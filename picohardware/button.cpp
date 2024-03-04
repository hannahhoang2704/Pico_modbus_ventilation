//
// Created by Hanh Hoang on 28.2.2024.
//
#include "button.h"

void Button::init_pin() {
    gpio_init(pin_nr);
    gpio_set_dir(pin_nr, GPIO_IN);
    gpio_pull_up(pin_nr);
}

bool Button::debounced_pressed() {
    if (!gpio_get(pin_nr) && !pressed) {
        return (pressed = true);
    } else if (gpio_get(pin_nr) && pressed) {
        pressed = false;
    }
    return false;
}
//bool Button::debounced_pressed() {
//    if(!gpio_get(pin_nr)){
//        sleep_ms(30);
//        if(!gpio_get(pin_nr)){
//            return true;
//        }
//    }
//    return false;
//}

void init_rotary_knob(){
    gpio_init(ROT_A);
    gpio_set_dir(ROT_A, GPIO_IN);

    gpio_init(ROT_B);
    gpio_set_dir(ROT_B, GPIO_IN);

    gpio_init(ROT_SW);
    gpio_set_dir(ROT_SW, GPIO_IN);
    gpio_pull_up(ROT_SW);
}
