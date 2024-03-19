//
// Created by Hanh Hoang on 18.3.2024.
//
#include "MQTTController.h"

bool MQTTController::connect() {
    int rc = ipstack.connect(broker_address, port);
    if (rc != 1) {
        printf("rc from TCP connect is %d\n", rc);
    }
//    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
//    data.MQTTVersion = 3;
//    data.clientID.cstring = (char *)client_id;
    rc = client.connect(connectData);
    if(rc==0) {
        is_connected= true;
        printf("MQTT connected\n");
    }else{
        is_connected = false;
        printf("MQTT connected failed, %d\n", rc);
    }
    return is_connected;
}

bool MQTTController::subscribe(const char* sub_topic, void (*messageHandler)(MQTT::MessageData&)) {
    if(!is_connected) {
        return false;
    }
    int rc = client.subscribe(sub_topic, MQTT::QOS0, messageHandler);
    return (rc == 0);
}

bool MQTTController::publish(const char* pub_topic, const char* message) {
    if(!is_connected) {
        return false;
    }
    MQTT::Message pubMessage;
    pubMessage.qos = MQTT::QOS0;
    pubMessage.retained = false;
    pubMessage.dup = false;
    pubMessage.payload = (void*)message;
    pubMessage.payloadlen = strlen(message);
    int rc = client.publish(pub_topic, pubMessage);
    return (rc == 0);
}

void MQTTController::yield(unsigned long timeout) {
    cyw43_arch_poll();
    client.yield(timeout);
}

bool MQTTController::get_MQTT_connected() const{
    return is_connected;
}

bool MQTTController::isConnected() {
    return client.isConnected();
}

void MQTTController::reconnect(){
    client.connect(connectData);
}