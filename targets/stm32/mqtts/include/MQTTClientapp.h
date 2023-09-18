#ifndef MQTT_CLIENT_APP_H_
#define MQTT_CLIENT_APP_H_

#include <string.h>

#include "MQTTClient.h"
#include "MQTTInterface.h"
#include "cmsis_os.h"
#include "config.h"

#define MQTT_PORT 1883
#define MQTT_BUFSIZE 1024
#define ERR_CODE -1

Network net;
MQTTClient mqttClient;

uint8_t sndBuffer[MQTT_BUFSIZE];
uint8_t rcvBuffer[MQTT_BUFSIZE];
uint8_t msgBuffer[MQTT_BUFSIZE];

void mqttClientSubTask(void const *argument);
void mqttClientPubTask(void const *argument);
int mqttConnectBroker(void);
void mqttMessageArrived(MessageData *msg);

#endif
