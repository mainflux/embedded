#ifndef CONFIG_H
#define CONFIG_H

#include <string.h>

struct BrokerDetails
{
    const char *mfThingId = "";
    const char *mfThingPass = "";
    const char *mfChannelId = "";
    char mfTopic[150];
    BrokerDetails()
    {
        const char *_preId = "channels/";
        const char *_postId = "/messages";
        strcat(mfTopic, _preId);
        strcat(mfTopic, mfChannelId);
        strcat(mfTopic, _postId);
    }
} brConfig;


// IPAddress server(192, 168, 000, 000);

const char *server = "";

#endif