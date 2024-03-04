//
// Created by ADMIN on 2/28/2024.
//

#ifndef PICO_MODBUS_SCREEN_SELECTION_H
#define PICO_MODBUS_SCREEN_SELECTION_H
//#include <memory>
#include "ssd1306.h"

class currentScreen{
public:
    explicit currentScreen(const std::shared_ptr<ssd1306> &lcd): lcd(lcd), color1(0), color2(0){};
    void modeSelection (int modeNum=0);
    void paramSet(int mode, int val);
    void info(int speed, int pressure, int temp, int humid, int co2);
    void error();
private:
    std::shared_ptr<ssd1306> lcd;
    int color1;
    int color2;
};

#endif //PICO_MODBUS_SCREEN_SELECTION_H
