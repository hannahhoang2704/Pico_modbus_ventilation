#include "screen_selection.h"
#include "icons.h"

using namespace std;
void currentScreen::screenSelection(const int option) {
    lcd->fill(0);
    switch (option) {
        case 0:
            color0 = 0;
            color1 = 1;
            color2 = 1;
            color3 = 1;
            lcd->fill_rect(0, 4, 128, 14, 1);
            break;
        case 1:
            color0 = 1;
            color1 = 0;
            color2 = 1;
            color3 = 1;
            lcd->fill_rect(0, 18, 128, 14, 1);
            break;
        case 2:
            color0 = 1;
            color1 = 1;
            color2 = 0;
            color3 = 1;
            lcd->fill_rect(0, 32, 128, 14, 1);
            break;
        case 3:
            color0 = 1;
            color1 = 1;
            color2 = 1;
            color3 = 0;
            lcd->fill_rect(0, 46, 128, 14, 1);
            break;
        default:
            break;
    }
    lcd->text("Set Fan speed", 5, 7, color0);
    lcd->text("Set Pressure", 5, 21, color1);
    lcd->text("Show Status", 5, 35, color2);
    lcd->text("MQTT Broker", 5, 49, color3);
    lcd->show();
}

void currentScreen::setSpeed(const int val) {
    //manual mode
    lcd->fill(0);
    lcd->rect(0, 35, 128, 20, 1);
    percent = val * 128 / 100;
    mono_vlsb icon(fan_icon, 20, 20);
    lcd->blit(icon, 108, 0);
    lcd->text("SET FAN SPEED", 0, 1);
    text = to_string(val) + " %";
    lcd->fill_rect(0, 35, percent, 20, 1);
    lcd->text(text, 20, 20);
    lcd->show();
}

void currentScreen::setPressure(const int val) {
    //auto mode
    lcd->fill(0);
    lcd->rect(0, 35, 128, 20, 1);
    percent = val * 128 / 120;
    mono_vlsb icon(pressure_icon, 20, 20);
    lcd->blit(icon, 108, 0);
    lcd->text("SET PRESSURE", 0, 1);
    text = to_string(val) + " pa";
    lcd->fill_rect(0, 35, percent, 20, 1);
    lcd->text(text, 20, 20);
    lcd->show();
}

void currentScreen::info(const bool autoMode, const float speed, const int pressure, const int temp, const int humid, const int co2) {
    sprintf(str, "%.1f", speed);
    lcd->fill(0);
    mono_vlsb icon(info_icon, 20, 20);
    lcd->blit(icon, 0, 0);
    text = "CO2:" + to_string(co2) + " ppm";
    lcd->text(text, 40, 2);
    text = "RH:" + to_string(humid) + " %";
    lcd->text(text, 48, 2+13);
    text = "T:" + to_string(temp) + " C";
    lcd->text(text, 56, 2+13+13);
    text = "S:" + string(str) + " %";
    lcd->text(text, 56,2+13+13+13);
    text = "AP:" + to_string(pressure) + " pa";
    lcd->text(text, 48, 2+13+13+13+13);
    lcd->fill_rect(0, 52, 36, 12, 1);
    if (autoMode){
        lcd->text("AUTO", 2, 54, 0);
    } else{
        lcd->text("MAN", 6, 54, 0);
    }
    lcd->show();
}

void currentScreen::error() {
    lcd->fill(0);
    mono_vlsb icon(error_icon, 20, 20);
    lcd->blit(icon, 0, 0);
    lcd->text("ERROR:", (128-6*8)/2, 2);
    lcd->text("Target pressure", 0, 2+22);
    lcd->text("can't be reached", 0, 2+22+13);
    lcd->text("within 1 minute", 0, 2+22+13+13);
    lcd->show();
}
/*
//
// Created by ADMIN on 2/28/2024.
//
#include "screen_selection.h"
#include "icons.h"

using namespace std;
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

void currentScreen::paramSet(const bool modeAuto, const int val) {
    lcd->fill(0);
    lcd->rect(0, 35, 128, 20, 1);
    if (modeAuto){ //auto mode
        percent = val * 128 / 120;
        mono_vlsb icon(pressure_icon, 20, 20);
        lcd->blit(icon, 108, 0);
        lcd->text("SET PRESSURE", 0, 1);
        text = to_string(val) + " pa";
    }
    else{   //manual mode
        percent = val * 128 / 100;
        mono_vlsb icon(fan_icon, 20, 20);
        lcd->blit(icon, 108, 0);
        lcd->text("SET FAN SPEED", 0, 1);
        text = to_string(val) + " %";
    }
    lcd->fill_rect(0, 35, percent, 20, 1);
    lcd->text(text, 20, 20);
    lcd->show();
}

void currentScreen::info(const bool autoMode, const float speed, const int pressure, const int temp, const int humid, const int co2) {
    sprintf(str, "%.1f", speed);
    lcd->fill(0);
    mono_vlsb icon(info_icon, 20, 20);
    lcd->blit(icon, 0, 0);
    text = "CO2:" + to_string(co2) + " ppm";
    lcd->text(text, 40, 2);
    text = "RH:" + to_string(humid) + " %";
    lcd->text(text, 48, 2+13);
    text = "T:" + to_string(temp) + " C";
    lcd->text(text, 56, 2+13+13);
    text = "S:" + string(str) + " %";
    lcd->text(text, 56,2+13+13+13);
    text = "AP:" + to_string(pressure) + " pa";
    lcd->text(text, 48, 2+13+13+13+13);
    lcd->fill_rect(0, 52, 36, 12, 1);
    if (autoMode){
        lcd->text("AUTO", 2, 54, 0);
    } else{
        lcd->text("MAN", 6, 54, 0);
    }
    lcd->show();
}

void currentScreen::error() {
    lcd->fill(0);
    mono_vlsb icon(error_icon, 20, 20);
    lcd->blit(icon, 0, 0);
    lcd->text("ERROR:", (128-6*8)/2, 2);
    lcd->text("Target pressure", 0, 2+22);
    lcd->text("can't be reached", 0, 2+22+13);
    lcd->text("within 1 minute", 0, 2+22+13+13);
    lcd->show();
}
*/
