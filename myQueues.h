/*
 * sensor_queue.h
 *
 *  Created on: Sep 7, 2020
 *      Author: Jhonie Geffa
 */

#ifndef MYQUEUES_H_
#define MYQUEUES_H_

#define SEND_TIME_MESSAGE 0x000000000000
#define SEND_SENSOR_MESSAGE 0x000100000000

#define QUEUE_SIZE 3

//MQTT Queue
//#define MQTT_MESSAGE_SIZE 310
#define MQTT_TOPIC_SIZE 40
#define MQTT_PAYLOAD_SIZE (310 - MQTT_TOPIC_SIZE - 4)
#define MQTT_MESSAGE_SIZE 4


#define MQTT_NUM_TOPICS 40  //40

// Publish Queue
#define PUBLISH_MESSAGE_SIZE 64
#define PUBLISH_TOPIC_SIZE 1
#define PUBLISH_PAYLOAD_SIZE (PUBLISH_MESSAGE_SIZE - PUBLISH_TOPIC_SIZE)

// Chain Queue
#define CHAIN_BOARD_SIZE 1
#define CHAIN_VAL_SIZE 4
#define CHAIN_SEQ_SIZE 4
#define CHAIN_MESSAGE_SIZE (CHAIN_VAL_SIZE + CHAIN_SEQ_SIZE + CHAIN_BOARD_SIZE)

// SERVO ANGLES
#define SERVO_ANGLE_1_SIZE 4    //pan
//#define SERVO_ANGLE_2_SIZE 4    //tilt
#define SERVO_TIME_MESSAGE_SIZE SERVO_ANGLE_1_SIZE

// SERVO DONE MOVING
#define SERVO_DONE  4   // 0 or 1
#define SERVO_DONE_MESSAGE_SIZE (SERVO_DONE)

// IR
#define IR_SIZE 4
#define IR_SEQ_SIZE 4
#define IR_MESSAGE_SIZE (IR_SIZE + IR_SEQ_SIZE)

// sensor data
#define IR_SIZE 4
#define ANGLE 4
#define CENTER_SIZE 4
#define WIDTH_SIZE 4
#define OBJ_SIZE 4
#define IR_SEQ_SIZE 4
#define SENSOR_DATA_MESSAGE_SIZE (IR_SIZE + ANGLE + CENTER_SIZE + WIDTH_SIZE + OBJ_SIZE + IR_SEQ_SIZE)



// IRv2
//#define IRV2_MSGNUM_SIZE 4
//#define IRV2_SENNUM_SIZE 4
//#define IRV2_AVGSEN_SIZE 4
//#define IRV2_SEQ_SIZE 4
//#define IRV2_MESSAGE_SIZE (IRV2_MSGNUM_SIZE + IRV2_SENNUM_SIZE + IRV2_AVGSEN_SIZE + IRV2_SEQ_SIZE)

// Stats
//#define STAT_BOARD_SIZE 1
//#define STAT_PUBATT_SIZE 4
//#define STAT_PUBSUC_SIZE 4
//#define STAT_PUBREC_SIZE 4
//#define STAT_PUBNOTREC_SIZE 4
//#define STAT_SEQ_SIZE 4
//#define STAT_MESSAGE_SIZE (STAT_BOARD_SIZE + STAT_PUBATT_SIZE + STAT_PUBSUC_SIZE + STAT_PUBREC_SIZE + STAT_PUBNOTREC_SIZE + STAT_SEQ_SIZE)

// LB
//#define LB_MESSAGE_SIZE 4

// topic numbers
//#define CHAIN1_TOPIC_NUM 1
//#define CHAIN2_TOPIC_NUM 2
//#define CHAIN3_TOPIC_NUM 3
//#define CHAIN4_TOPIC_NUM 4

//#define IRV2_1_TOPIC_NUM 6
//#define IRV2_2_TOPIC_NUM 7
//#define IRV2_3_TOPIC_NUM 8
//#define STAT_TOPIC_NUM 9
//#define LB1_TOPIC_NUM 10
//#define LB2_TOPIC_NUM 11
//#define LB3_TOPIC_NUM 12
//#define LB4_TOPIC_NUM 13


#define IR_TOPIC_NUM    14
#define PIXY_TOPIC_NUM  15
#define TIMER_TOPIC_NUM 16
#define PWM_TOPIC_NUM   17
#define SENSOR_DATA_TOPIC   18
#define COORD_DATA_TOPIC    19


#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdio.h>
#include "debug.h"
#include <string.h>
#include <math.h>

QueueHandle_t myQueue;

struct servoMoveQData{
    int position0;   // angle for servo 1
};

struct IRQData
{
    float val;
    unsigned int seq;
};

struct MQTTMsgQueue
{
    int   event;
    char  topic[MQTT_TOPIC_SIZE];
    char  payload[MQTT_PAYLOAD_SIZE];
};


struct SensorData{
    int irval;
    unsigned int angle;
    unsigned int xCenter;
    unsigned int xWidth;
    unsigned int objType;
    unsigned int numReadings;

};


struct CoordData{
    unsigned int objType;
    signed int xCoord;
    unsigned int yCoord;
    unsigned int objConf;
    unsigned int coordConf;
};


// below here unused
struct ChainQData
{
    uint8_t board;
    int val;
    unsigned int seq;
};
struct queueData{
    unsigned int timeVal;
    int mmDist;
    int type;
};

struct IRv2QData
{
    unsigned int msgNum;
    unsigned int senNum;
    float avgSen;
    unsigned int seq;
};
struct StatQData
{
    uint8_t board;
    unsigned int pubAttempts;
    unsigned int pubSuc;
    unsigned int pubRec;
    unsigned int pubNotRec;
    unsigned int seq;
};


int duty2Angle(int d);
int angle2Duty(int a);



// helper functions
void copyIntToMessage(unsigned int* copyMe, unsigned int indexStart, char* message);
void readIntFromMessage(unsigned int* copyMe, unsigned int indexStart, char* message);

// IR functions?
//int sendTimeMsgToQ1(unsigned int timeVal);
//int sendSensorMsgToQ1(int mmDist);
//int readMsgFromQueue(struct queueData *myQD);
//int initializeQueue();

//SERVO and SENSOR TIMING
QueueHandle_t initializeServoTimingQueue();
QueueHandle_t initializeMovingDoneQueue();
QueueHandle_t initializeSensorDataQueue();

int sendToServoTimingQueue(signed int pos0);
int sendToServoTimingQueueFromISR(signed int pos0);    // check on values

int readFromServoTimingQueue(struct servoMoveQData *myQD);// check on values

int readFromServoMovingDoneQueue(unsigned int *done, signed int *duty);
int sendToServoMovingDoneQueueFromISR(unsigned int done, signed int duty);
int sendToServoMovingDoneQueue(unsigned int done, signed int duty);

int sendServoMoveToPublishQueue(uint8_t topic);
int sendServoMoveToPublishQueueFromISR(uint8_t topic);

void readTimerValFromPublishQueue(char* message, unsigned int *seq);
int sendTimerValToPublishQueueFromISR(uint8_t topic, unsigned int sequenceVal);

int sendPWMValToPublishQueue(uint8_t topic, unsigned int duty0, unsigned int duty1);
void readPWMValFromPublishQueue(char* message, unsigned int *d0, unsigned int *d1);

int sendPixyToPublishQueue(uint8_t topic, struct SensorData data);
void readPixyFromPublishQueue(char *message, struct SensorData *data);
void readSensorDataFromPublishQueue(char* message, struct SensorData *data);
int sendSensorDataToPublishQueue(uint8_t topic, struct SensorData data);

int sendCoordDataToPublishQueue(uint8_t topic, struct CoordData cData);
void readCoordDataToPublishQueue(uint8_t topic, struct CoordData *cData);



int readFromSensorDataQueue(struct SensorData *data);
int sendToSensorDataQueueFromISR(struct SensorData data);
int sendToSensorDataQueue(struct SensorData data);




// MQTT Q
QueueHandle_t initializeMQTTQueue();
int sendToMQTTQueue(int event);
int sendToMQTTQueueFromISR(int event);
int readMsgFromMQTTQueue(int *myQD);

// Publish Q
QueueHandle_t initializePublishQueue();
int sendToPublishQueueFromISR(char* payload);
int sendToPublishQueue(char* payload);
int readMsgFromPublishQueue(char* message);

// IR Q
QueueHandle_t initializeIRQueue();
int sendToIRQueueFromISR(float irVal, unsigned int sequenceVal);
int sendToIRQueue(float irVal, unsigned int sequenceVal);
int readMsgFromIRQueue(struct IRQData *myQD);
void readIRFromPublishQueue(char* message, struct IRQData *myQD);
int sendIRToPublishQueueFromISR(uint8_t topic, float irVal, unsigned int sequenceVal);
int sendIRToPublishQueue(uint8_t topic, float irVal, unsigned int sequenceVal);

//TOF queue




#endif /* MYQUEUES_H_ */
