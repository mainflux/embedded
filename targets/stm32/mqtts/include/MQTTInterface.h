#ifndef _MQTTInterface_H
#define _MQTTInterface_H

typedef struct Timer Timer;

struct Timer
{
	unsigned long systick_period;
	unsigned long end_time;
};

typedef struct Network Network;

struct Network
{
	int (*mqttread)(Network *, unsigned char *, int, int);
	int (*mqttwrite)(Network *, unsigned char *, int, int);
	void (*disconnect)(Network *);
};

void initTimer(Timer *);
char timerIsExpired(Timer *);
void timerCountdownMS(Timer *, unsigned int);
void timerCountdown(Timer *, unsigned int);
int timerLeftMS(Timer *);

int netInit(Network *);
int netConnect(Network *, char *, int);
int netRead(Network *, unsigned char *, int, int);
int netWrite(Network *, unsigned char *, int, int);
void netDisconnect(Network *);
void netClear(void);

#endif
