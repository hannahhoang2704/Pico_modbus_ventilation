//
// Created by ADMIN on 2/28/2024.
//
#include "screen_selection.h"

void currentScreen::modeSelection(const bool modeAuto) {
    lcd->fill(0);
    lcd->text("SELECT MODE", 20, 1);
    if (modeAuto){   //auto mode
        color1 = 0;
        color2 = 1;
        lcd->fill_rect(0, 15, 128, 20, 1);
    }
    else{   //manual mode
        color1 = 1;
        color2 = 0;
        lcd->fill_rect(0, 40, 128, 20, 1);
    }
    lcd->text("AUTO", 10, 20, color1);
    lcd->text("AUTO", 10, 21, color1);
    lcd->text("MANUAL", 10, 45, color2);
    lcd->text("MANUAL", 10, 46, color2);
    lcd->show();
}

void currentScreen::paramSet(const int mode, const int val) {
    int percent;
    lcd->fill(0);
    lcd->rect(0, 35, 128, 20, 1);
    if (!mode){ //auto mode
        percent = val * 128 / 120;
        lcd->text("SET PRESSURE", (128-12*8)/2, 1);
        lcd->text("pa", 50, 20);
    }
    else{   //manual mode
        percent = val * 128 / 100;
        lcd->text("SET FAN SPEED", (128-13*8)/2, 1);
        lcd->text("%", 50, 20);
    }
    lcd->fill_rect(0, 35, percent, 20, 1);
    lcd->text(std::to_string(val), 20, 20);
    lcd->show();
}

void currentScreen::info(const int speed, const int pressure, const int temp, const int humid, const int co2) {
    lcd->fill(0);
    lcd->text("Fan speed:", 24, 2);
    lcd->text(std::to_string(speed),105,2);
    lcd->text("Air pressure:", 0, 2+13);
    lcd->text(std::to_string(pressure),105,2+13);
    lcd->text("Temperature:", 8, 2+13+13);
    lcd->text(std::to_string(temp),105,2+13+13);
    lcd->text("Humidity:", 32, 2+13+13+13);
    lcd->text(std::to_string(humid),105,2+13+13+13);
    lcd->text("CO2:", 72, 2+13+13+13+13);
    lcd->text(std::to_string(co2),105,2+13+13+13+13);
    lcd->show();
}

void currentScreen::error() {
    lcd->fill(0);
    lcd->text("ERROR:", (128-6*8)/2, 2);
    lcd->text("Target pressure", 0, 2+15);
    lcd->text("can't be reached", 0, 2+15+13);
    lcd->text("within 1 minute", 0, 2+15+13+13);
    lcd->show();
}
