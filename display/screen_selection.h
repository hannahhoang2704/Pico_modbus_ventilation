//
// Created by ADMIN on 2/28/2024.
//

#ifndef PICO_MODBUS_SCREEN_SELECTION_H
#define PICO_MODBUS_SCREEN_SELECTION_H
#include <cstring>
#include "ssd1306.h"

#define START_ASCII 32
#define END_ASCII 126
#define CHAR_WIDTH 8;

using namespace std;

class currentScreen{
public:
    explicit currentScreen(const std::shared_ptr<ssd1306> &lcd)
    : lcd(lcd), color0(1),color1(1), color2(1), color3(1){};
    void networkConnecting();
    void screenSelection (int option);
    void setSpeed(int val);
    void setPressure(int val);
    void info(bool autoMode, int speed, int pressure, int temp, int humid, int co2);
    void error();
    void mqtt(int option, const char* ssid, const char* pw, const char* ip);
    void asciiCharSelection(int posX, int option, int asciiChar);
    void askRestart();
private:
    std::shared_ptr<ssd1306> lcd;
    int color0, color1, color2, color3;
    string text;
    int percent{};
};

#endif