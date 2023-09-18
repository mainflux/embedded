#include "main.h"
#include "MQTTClientapp.h"

#define MESSAGE_DELAY 1000
#define OS_DELAY 100
#define KEEP_ALIVE_INT 60

void createMainfluxChannel(void)
{
    const char *_preId = "channels/";
    const char *_postId = "/messages";
    strcpy(mfTopic, _preId);
    strcat(mfTopic, mfChannelId);
    strcat(mfTopic, _postId);
}

void mqttClientSubTask(void const *argument)
{
    while (1)
    {
        if (!mqttClient.isconnected)
        {
            MQTTDisconnect(&mqttClient);
            mqttConnectBroker();
            osDelay(MESSAGE_DELAY);
        }
        else
        {
            MQTTYield(&mqttClient, MESSAGE_DELAY);
            osDelay(OS_DELAY);
        }
    }
}

void mqttClientPubTask(void const *argument)
{
    const char *str = "{'message':'hello'}";
    MQTTMessage message;

    while (1)
    {
        if (mqttClient.isconnected)
        {
            message.payload = (void *)str;
            message.payloadlen = strlen(str);

            MQTTPublish(&mqttClient, mfTopic, &message);
        }
        osDelay(MESSAGE_DELAY);
    }
}

int mqttConnectBroker()
{
    int ret;

    net_clear();
    ret = net_init(&net);
    if (ret != MQTT_SUCCESS)
    {
        printf("net_init failed.\n");
        return ERR_CODE;
    }

    ret = net_connect(&net, server, MQTT_PORT);
    if (ret != MQTT_SUCCESS)
    {
        printf("net_connect failed.\n");
        return ERR_CODE;
    }

    MQTTClientInit(&mqttClient, &net, MESSAGE_DELAY, sndBuffer, sizeof(sndBuffer), rcvBuffer, sizeof(rcvBuffer));
    createMainfluxChannel();

    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.willFlag = MQTT_WILL_FLAG;
    data.MQTTVersion = MQTT_VERSION;
    data.clientID.cstring = CLIENTID;
    data.username.cstring = mfThingId;
    data.password.cstring = mfThingPass;
    data.keepAliveInterval = KEEP_ALIVE_INT;
    data.cleansession = MQTT_CLEAN_SESSION;

    ret = MQTTConnect(&mqttClient, &data);
    if (ret != MQTT_SUCCESS)
    {
        printf("MQTTConnect failed.\n");
        return ret;
    }

    ret = MQTTSubscribe(&mqttClient, mfChannelId, QOS0, mqttMessageArrived);
    if (ret != MQTT_SUCCESS)
    {
        printf("MQTT Subscribe failed.\n");
        return ret;
    }

    return MQTT_SUCCESS;
}

void mqttMessageArrived(MessageData *msg)
{
    MQTTMessage *message = msg->message;
    memset(msgBuffer, 0, sizeof(msgBuffer));
    memcpy(msgBuffer, message->payload, message->payloadlen);

    printf("MQTT MSG[%d]:%s\n", (int)message->payloadlen, msgBuffer);
}