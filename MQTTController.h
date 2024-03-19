//
// Created by Hanh Hoang on 18.3.2024.
//

#ifndef PICO_MODBUS_VENTILATION_MQTTCONTROLLER_H
#define PICO_MODBUS_VENTILATION_MQTTCONTROLLER_H
#include <string>
#include "IPStack.h"
#include "MQTTClient.h"
#include "Countdown.h"

class MQTTController {
private:
    IPStack ipstack;
    MQTT::Client<IPStack, Countdown, 256> client;
    MQTTPacket_connectData connectData;
    const char* client_id;
    const char* broker_address;
    int port;
    bool is_connected;

public:
    MQTTController(const char* ssid, const char* password, const char* clientId, const char* brokerAddress, int brokerPort) : ipstack(ssid, password), client(ipstack), client_id(clientId), broker_address(brokerAddress), port(brokerPort), is_connected(false) {
        connectData = MQTTPacket_connectData_initializer;
        connectData.MQTTVersion = 3;
        connectData.clientID.cstring = (char *)client_id;
    };
    bool connect();
    bool subscribe(const char* sub_topic, void (*messageHandler)(MQTT::MessageData&));
    bool publish(const char* pub_topic, const char* message);
    void yield(unsigned long timeout =100);
    bool get_MQTT_connected() const;
    bool isConnected();
    void reconnect();

};
#endif //PICO_MODBUS_VENTILATION_MQTTCONTROLLER_H
