#ifndef CONFIG_H
#define CONFIG_H

#define MQTT_CLIENTID ""
#define SNTP_SERVER "0.pool.ntp.org
#define BROKER ""
#define BROKER_PORT "8883"
#define MQTT_BUFFER_SIZE 256u
#define APP_BUFFER_SIZE 4096u
#define MAX_RETRIES 10u
#define BACKOFF_EXP_BASE_MS 1000u
#define BACKOFF_EXP_MAX_MS 60000u
#define BACKOFF_CONST_MS 5000u
#define KEEP_ALIVE 60
#define TOPIC_BUFFER_SIZE 128

const char *mfThingId = " ";
const char *mfThingKey = " ";
const char *mfChannelId = " ";
char mfTopic[TOPIC_BUFFER_SIZE];

const char *brokername = ""

#endif
