//
// Created by ADMIN on 2/28/2024.
//

#ifndef PICO_MODBUS_SCREEN_SELECTION_H
#define PICO_MODBUS_SCREEN_SELECTION_H
#include <cstring>
#include "ssd1306.h"

using namespace std;
class currentScreen{
public:
    explicit currentScreen(const std::shared_ptr<ssd1306> &lcd): lcd(lcd), color0(1),color1(1), color2(1), color3(1){};
    void networkConnecting();
    void screenSelection (int option);
    void setSpeed(int val);
    void setPressure(int val);
    void info(bool autoMode, int speed, int pressure, int temp, int humid, int co2);
    void error();
    void mqtt();
private:
    std::shared_ptr<ssd1306> lcd;
    int color0, color1, color2, color3;
    string text;
    char str[5]{};
    int percent{};
    float sVal{};
};

#endif //PICO_MODBUS_SCREEN_SELECTION_H

/*
 //
// Created by ADMIN on 2/28/2024.
//

#ifndef PICO_MODBUS_SCREEN_SELECTION_H
#define PICO_MODBUS_SCREEN_SELECTION_H
#include <cstring>
#include "ssd1306.h"

using namespace std;
class currentScreen{
public:
    explicit currentScreen(const std::shared_ptr<ssd1306> &lcd): lcd(lcd), color1(0), color2(0){};
    void modeSelection (bool modeAuto = false);
    void paramSet(bool modeAuto, int val);
    void info(bool autoMode, float speed, int pressure, int temp, int humid, int co2);
    void error();
private:
    std::shared_ptr<ssd1306> lcd;
    int color1;
    int color2;
    string text;
    char str[5]{};
    int percent{};
    float sVal{};
};

#endif //PICO_MODBUS_SCREEN_SELECTION_H
*/
