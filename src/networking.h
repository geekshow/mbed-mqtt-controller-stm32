#pragma once

#include "MQTTSocket.h"
#include "MQTTClient.h"

typedef MQTT::Client<MQTTSocket,Countdown> MClient;
typedef void (*recv_ctl_cb)(const char*, const char*);

int networking_init(MQTTSocket &sock, MClient &client, const char *broker,const char* sensors[][2], const char* actuators[][2], recv_ctl_cb cb);
void publish_value(MClient &client, const char *topic, const char *buf);
