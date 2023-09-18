#ifndef CONFIG_H
#define CONFIG_H

#define ERR_CODE -1
#define CLIENTID "STM32F4"
#define MQTT_WILL_FLAG 0
#define MQTT_VERSION 3
#define MQTT_CLEAN_SESSION 1
#define TOPIC_BUFFER_SIZE 128

const char *mfThingId = " ";
const char *mfThingPass = " ";
const char *mfChannelId = " ";
char mfTopic[TOPIC_BUFFER_SIZE];

const char *server = " ";

#endif
