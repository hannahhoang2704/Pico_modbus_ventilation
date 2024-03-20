#include "screen_selection.h"
#include "icons.h"
#include "cstring"

using namespace std;

void currentScreen::networkConnecting() {
    for (int i = 0; i < 2; i++){
        lcd->fill(0);
        mono_vlsb icon(mqtt_icon, 50, 50);
        lcd->blit(icon, 39, 0);
        lcd->text("Connecting...",7,53);
        lcd->show();
        sleep_ms(500);
        lcd->fill(0);
        lcd->show();
        sleep_ms(500);
        lcd->fill(0);
        lcd->blit(icon, 39, 0);
        lcd->text("Connecting...",7,53);
        lcd->show();
    }
}

void currentScreen::screenSelection(const int option) {
    color0 = 1;
    color1 = 1;
    color2 = 1;
    color3 = 1;
    lcd->fill(0);
    switch (option) {
        case 0:
            color0 = 0;
            lcd->fill_rect(0, 4, 128, 14, 1);
            break;
        case 1:
            color1 = 0;
            lcd->fill_rect(0, 18, 128, 14, 1);
            break;
        case 2:
            color2 = 0;
            lcd->fill_rect(0, 32, 128, 14, 1);
            break;
        case 3:
            color3 = 0;
            lcd->fill_rect(0, 46, 128, 14, 1);
            break;
        default:
            break;
    }
    lcd->text("Set Fan speed", 5, 7, color0);
    lcd->text("Set Pressure", 5, 21, color1);
    lcd->text("Show Status", 5, 35, color2);
    lcd->text("WiFi & MQTT", 5, 49, color3);
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

void currentScreen::info(const bool autoMode, const int speed, const int pressure, const int temp, const int humid, const int co2) {
    lcd->fill(0);
    mono_vlsb icon(info_icon, 20, 20);
    lcd->blit(icon, 0, 0);
    text = "CO2:" + to_string(co2) + " ppm";
    lcd->text(text, 40, 2);
    text = "RH:" + to_string(humid) + " %";
    lcd->text(text, 48, 2+13);
    text = "T:" + to_string(temp) + " C";
    lcd->text(text, 56, 2+13+13);
    text = "S:" + to_string(speed) + " %";
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

void currentScreen::mqtt(const int option, const char* ssid, const char* pw, const char* ip) {
    color0 = 1;
    color1 = 1;
    lcd->fill(0);
    switch (option) {
        case 0:
            lcd->rect(0,0,128,12,1);
            break;
        case 1:
            lcd->rect(0,11,128,12,1);
            break;
        case 2:
            lcd->rect(0,22,128,22,1);
            break;
        case 3:
            lcd->fill_rect(0,51,36,12,1);
            color0 = 0;
            break;
        case 4:
            lcd->fill_rect(75,51,52,12,1);
            color1 = 0;
            break;
        default:
            break;
    }
    lcd->text("SSID:", 2,2);
    lcd->text(ssid, 44, 2);
    lcd->text("PASS:",2,13);
    lcd->text(pw, 44, 13);
    lcd->text("MQTT IP:",2,24);
    lcd->text(ip, 2, 35);
    lcd->rect(0,51,36,12,1);
    lcd->text("save",2,53, color0);
    lcd->rect(75,51,52,12,1);
    lcd->text("cancel",77,53, color1);
    lcd->show();
}

void currentScreen::asciiCharSelection(const int posX, const int option, const int asciiChar){
    int posY = 64;
    char c[2];
    sprintf(c, "%c", asciiChar);
    switch (option) {
        case 0:
            posY = 2;
            break;
        case 1:
            posY = 13;
            break;
        case 2:
            posY = 35;
            break;
        default:
            break;
    }
    if (asciiChar < START_ASCII){
        lcd->fill_rect(posX,posY,8,8,1);
    } else {
        lcd->fill_rect(posX,posY,8,8,0);
        lcd->text(c, posX, posY);
        //lcd->fill_rect(posX+8,posY,8,8,1);
    }
    lcd->show();
}

void currentScreen::askRestart(){
    lcd->fill(0);
    lcd->text("*Reset device*", 0, 30);
    lcd->show();
}