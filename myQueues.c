/*
 * sensor_queue.c
 *
 *  Created on: Sep 7, 2020
 *      Author: Work
 */


#include "myQueues.h"


int duty2Angle(int d){
    return (d-500)*180/2000;
}

int angle2Duty(int a){
    return 500+2000*(a)/180.0;
}

void copyIntToMessage(unsigned int* copyMe, unsigned int indexStart, char* message)
{
    message[indexStart]     = (0xFF000000 & *copyMe) >> 8*3;
    message[indexStart + 1] = (0x00FF0000 & *copyMe) >> 8*2;
    message[indexStart + 2] = (0x0000FF00 & *copyMe) >> 8*1;
    message[indexStart + 3] = 0x000000FF & *copyMe;
}

void readIntFromMessage(unsigned int* copyMe, unsigned int indexStart, char* message)
{
    *copyMe = 0;
    *copyMe |= message[indexStart] << 8*3;
    *copyMe |= message[indexStart + 1] << 8*2;
    *copyMe |= message[indexStart + 2] << 8*1;
    *copyMe |= message[indexStart + 3];
}

// Init Queues

int initializeQueue(){
    static int init = 0;
    if (init == 0){
        myQueue = xQueueCreate(3, 8);
        /*Check to see if queue was created*/
        if(myQueue == NULL){
            return 0;
        }
        init = 1;
    }

    return 1;

}
///////////////////////////////////////////// Servo Qs ///////////////////////////////////////////

QueueHandle_t initializeMovingDoneQueue(){
    static int init = 0;
    static QueueHandle_t myQueue;
    if (init == 0){
        // change me when copying
        myQueue = xQueueCreate(QUEUE_SIZE, SERVO_DONE_MESSAGE_SIZE + 4);
        init = 1;
    }

    return myQueue;
}

QueueHandle_t initializeServoTimingQueue(){
    static int init = 0;
    static QueueHandle_t myQueue;
    if (init == 0){
        // change me when copying
        myQueue = xQueueCreate(QUEUE_SIZE, SERVO_TIME_MESSAGE_SIZE);
        init = 1;
    }

    return myQueue;
}

QueueHandle_t initializeSensorDataQueue(){

    static int init = 0;
    static QueueHandle_t myQueue;
    if (init == 0){
        // change me when copying
        myQueue = xQueueCreate(QUEUE_SIZE, SENSOR_DATA_MESSAGE_SIZE);
        init = 1;
    }

    return myQueue;
}

int readFromServoTimingQueue(struct servoMoveQData *myQD){
    dbgOutputLoc(DBG_READ_FROM_TIMING);
    int success = 0;
    // change me when copying
    QueueHandle_t myQueue = initializeServoTimingQueue();
    if (myQueue == NULL){
        dbgOutputLoc(DBG_SERVO_TIMING_FAIL_INIT);
        dbgFailRoutine();
    }

    char message[SERVO_TIME_MESSAGE_SIZE];

    // change me when copying
    if(xQueueReceive(myQueue, (void*) message, portMAX_DELAY) == pdPASS){
        readIntFromMessage((unsigned int*) &myQD->position0, 0, message);
        ///////// changed above from float ///////////////
        success = 1;
        return success;
    }
    dbgFailRoutine();
    return success;
}


int sendToServoTimingQueueFromISR(signed int pos0){

    dbgOutputLoc(DBG_WRITE_TO_TIMING_ISR);

    int success = 0;
    // Change me when copying
    QueueHandle_t myQueue = initializeServoTimingQueue();
    if (myQueue == NULL){
       dbgOutputLoc(DBG_SERVO_TIMING_FAIL_INIT);
       dbgFailRoutine();
    }

    // change these when copying
    char message[SERVO_TIME_MESSAGE_SIZE];
    copyIntToMessage((unsigned int*) &pos0, 0, message);

    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    /*Send timeVal and wait for 10 ticks for space to be available if necessary.
    * Else return failure.*/
    if(xQueueSendToBackFromISR(myQueue, (void*) message, &xHigherPriorityTaskWoken) == pdPASS ){
       success = 1;
       return success;
    }

    dbgFailRoutine();
    return success;
}

int sendToServoTimingQueue(signed int pos0){

    dbgOutputLoc(DBG_WRITE_TO_TIMING_ISR);

    int success = 0;
    // Change me when copying
    QueueHandle_t myQueue = initializeServoTimingQueue();
    if (myQueue == NULL){
       dbgOutputLoc(DBG_SERVO_TIMING_FAIL_INIT);
       dbgFailRoutine();
    }

    // change these when copying
    char message[SERVO_TIME_MESSAGE_SIZE];
    copyIntToMessage((unsigned int*) &pos0, 0, message);
    //copyIntToMessage((unsigned int*) &time, 8, message);

    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    /*Send timeVal and wait for 10 ticks for space to be available if necessary.
    * Else return failure.*/
    if(xQueueSendToBack(myQueue, (void*) message, &xHigherPriorityTaskWoken) == pdPASS ){
       success = 1;
       return success;
    }

    dbgFailRoutine();
    return success;
}

int readFromServoMovingDoneQueue(unsigned int *done, signed int *duty){
    dbgOutputLoc(DBG_READ_FROM_DONE);
    int success = 0;
    // change me when copying
    QueueHandle_t myQueue = initializeMovingDoneQueue();
    if (myQueue == NULL){
        dbgOutputLoc(DBG_SERVO_DONE_FAIL_INIT);
        dbgFailRoutine();
    }

    char message[SERVO_DONE_MESSAGE_SIZE+4];

    //if(xQueuePeek(myQueue, (void*) message, portMAX_DELAY) == pdPASS){
    if(xQueueReceive(myQueue, (void*) message, portMAX_DELAY) == pdPASS){
        readIntFromMessage((unsigned int*) done, 0, message);
        readIntFromMessage((unsigned int*) duty, 4, message);

        success = 1;
        return success;
    }
    dbgFailRoutine();
    return success;
}

int sendToServoMovingDoneQueueFromISR(unsigned int done, signed int duty){

    dbgOutputLoc(DBG_WRITE_TO_MOVING_DONE);

    int success = 0;
    // Change me when copying
    QueueHandle_t myQueue = initializeMovingDoneQueue();
    if (myQueue == NULL){
        dbgOutputLoc(DBG_SERVO_DONE_FAIL_INIT);
        dbgFailRoutine();
    }

    // change these when copying
    char message[SERVO_DONE_MESSAGE_SIZE+4];
    copyIntToMessage((unsigned int*) &done, 0, message);
    copyIntToMessage((unsigned int*) &duty, 4, message);


    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    /*Send timeVal and wait for 10 ticks for space to be available if necessary.
    * Else return failure.*/
    if(xQueueSendToBackFromISR(myQueue, (void*) message, &xHigherPriorityTaskWoken) == pdPASS ){
        success = 1;
        //return success;
    }

    //dbgFailRoutine();
    return success;
}


int sendToServoMovingDoneQueue(unsigned int done, signed int duty){

    dbgOutputLoc(DBG_WRITE_TO_MOVING_DONE);

    int success = 0;
    // Change me when copying
    QueueHandle_t myQueue = initializeMovingDoneQueue();
    if (myQueue == NULL){
        dbgOutputLoc(DBG_SERVO_DONE_FAIL_INIT);
        dbgFailRoutine();
    }

    // change these when copying
    char message[SERVO_DONE_MESSAGE_SIZE+4];
    copyIntToMessage((unsigned int*) &done, 0, message);
    copyIntToMessage((unsigned int*) &duty, 4, message);


    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    /*Send timeVal and wait for 10 ticks for space to be available if necessary.
    * Else return failure.*/
    if(xQueueSendToBack(myQueue, (void*) message, &xHigherPriorityTaskWoken) == pdPASS ){
        success = 1;
        return success;
    }

    dbgFailRoutine();
    return success;
}

int readFromSensorDataQueue(struct SensorData *data){
    dbgOutputLoc(DBG_READ_FROM_SENSOR_DATA);
    int success = 0;
    // change me when copying
    QueueHandle_t myQueue = initializeSensorDataQueue();
    if (myQueue == NULL){
        dbgOutputLoc(DBG_SERVO_SENSOR_DATA_FAIL_INIT);
        dbgFailRoutine();
    }

    char message[SENSOR_DATA_MESSAGE_SIZE];

    // change me when copying
    if(xQueueReceive(myQueue, (void*) message, portMAX_DELAY) == pdPASS){
        readIntFromMessage((signed int*) &data->irval, 0, message);
        readIntFromMessage((unsigned int*) &data->angle, 4, message);
        readIntFromMessage((unsigned int*) &data->xWidth, 8, message);
        readIntFromMessage((unsigned int*) &data->xCenter, 12, message);
        readIntFromMessage((unsigned int*) &data->objType, 16, message);
        readIntFromMessage((unsigned int*) &data->numReadings, 20, message);
        success = 1;
        return success;
    }
    dbgFailRoutine();
    return success;

}

int sendToSensorDataQueueFromISR(struct SensorData data){
    dbgOutputLoc(DBG_WRITE_TO_SENSOR_DATA_FROM_ISR);

    int success = 0;
    // Change me when copying
    QueueHandle_t myQueue = initializeSensorDataQueue();
    if (myQueue == NULL){
        dbgOutputLoc(DBG_SENSORDATA_FAIL_INIT_FROM_ISR);
        dbgFailRoutine();
    }

    // change these when copying
    char message[SENSOR_DATA_MESSAGE_SIZE];
    copyIntToMessage((unsigned int*) &data.irval, 0, message);
//    unsigned int a = 180*(data.angle-500)/2000;//500+2000*(data.angle)/180.0;
    copyIntToMessage((unsigned int*) &data.angle, 4, message);
    copyIntToMessage((unsigned int*) &data.xCenter, 8, message);
    copyIntToMessage((unsigned int*) &data.xWidth, 12, message);
    copyIntToMessage((unsigned int*) &data.objType, 16, message);
    copyIntToMessage((unsigned int*) &data.numReadings, 20, message);


    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    /*Send timeVal and wait for 10 ticks for space to be available if necessary.
    * Else return failure.*/
    if(xQueueSendToBackFromISR(myQueue, (void*) message, &xHigherPriorityTaskWoken) == pdPASS ){
        success = 1;
        return success;
    }

    dbgFailRoutine();
    return success;

}

int sendToSensorDataQueue(struct SensorData data){
    dbgOutputLoc(DBG_WRITE_TO_SENSOR_DATA);

    int success = 0;
    // Change me when copying
    QueueHandle_t myQueue = initializeSensorDataQueue();
    if (myQueue == NULL){
        dbgOutputLoc(DBG_SENSORDATA_FAIL_INIT);
        dbgFailRoutine();
    }

    // change these when copying
    char message[SENSOR_DATA_MESSAGE_SIZE];
    copyIntToMessage((unsigned int*) &data.irval, 0, message);
//    unsigned int a = 180*(data.angle-500)/2000;//500+2000*(data.angle)/180.0;
    copyIntToMessage((unsigned int*) &data.angle, 4, message);
    copyIntToMessage((unsigned int*) &data.xCenter, 8, message);
    copyIntToMessage((unsigned int*) &data.xWidth, 12, message);
    copyIntToMessage((unsigned int*) &data.objType, 16, message);
    copyIntToMessage((unsigned int*) &data.numReadings, 20, message);

    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    /*Send timeVal and wait for 10 ticks for space to be available if necessary.
    * Else return failure.*/
    if(xQueueSendToBack(myQueue, (void*) message, &xHigherPriorityTaskWoken) == pdPASS ){
        success = 1;
        return success;
    }

    dbgFailRoutine();
    return success;

}


int sendPixyToPublishQueue(uint8_t topic, struct SensorData data){

    char message[PUBLISH_MESSAGE_SIZE];
    message[0] = topic;
    copyIntToMessage((unsigned int*) &data.xWidth, PUBLISH_TOPIC_SIZE, message);
    copyIntToMessage((unsigned int*) &data.xCenter, PUBLISH_TOPIC_SIZE+4, message);
    copyIntToMessage((unsigned int*) &data.objType, PUBLISH_TOPIC_SIZE+8, message);

    return sendToPublishQueue(message);
}

void readPixyFromPublishQueue(char *message, struct SensorData *data){
    readIntFromMessage((unsigned int*) &data->xCenter, PUBLISH_TOPIC_SIZE, message);
    readIntFromMessage((unsigned int*) &data->xWidth, PUBLISH_TOPIC_SIZE+4, message);
    readIntFromMessage((unsigned int*) &data->objType, PUBLISH_TOPIC_SIZE+8, message);
}

int sendSensorDataToPublishQueue(uint8_t topic, struct SensorData data){

    char message[PUBLISH_MESSAGE_SIZE];
    message[0] = topic;
//    unsigned int a = 180*(data.angle-500)/2000;//500+2000*(data.angle)/180.0;
    copyIntToMessage((unsigned int*) &data.irval, PUBLISH_TOPIC_SIZE, message);
    copyIntToMessage((unsigned int*) &data.angle, PUBLISH_TOPIC_SIZE+4, message);
    copyIntToMessage((unsigned int*) &data.xCenter, PUBLISH_TOPIC_SIZE+8, message);
    copyIntToMessage((unsigned int*) &data.xWidth, PUBLISH_TOPIC_SIZE+12, message);
    copyIntToMessage((unsigned int*) &data.objType, PUBLISH_TOPIC_SIZE+16, message);


    return sendToPublishQueue(message);
}
void readSensorDataFromPublishQueue(char* message, struct SensorData *data){

    readIntFromMessage((unsigned int*) &data->irval, PUBLISH_TOPIC_SIZE, message);
    readIntFromMessage((unsigned int*) &data->angle, PUBLISH_TOPIC_SIZE+4, message);
    readIntFromMessage((unsigned int*) &data->xWidth, PUBLISH_TOPIC_SIZE+8, message);
    readIntFromMessage((unsigned int*) &data->xCenter, PUBLISH_TOPIC_SIZE+12, message);
    readIntFromMessage((unsigned int*) &data->objType, PUBLISH_TOPIC_SIZE+16, message);

}

int sendCoordDataToPublishQueue(uint8_t topic, struct CoordData cData){
    char message[PUBLISH_MESSAGE_SIZE];
    message[0] = topic;
    copyIntToMessage((unsigned int*) &cData.xCoord, PUBLISH_TOPIC_SIZE, message);
    copyIntToMessage((unsigned int*) &cData.yCoord, PUBLISH_TOPIC_SIZE+4, message);
    copyIntToMessage((unsigned int*) &cData.objType, PUBLISH_TOPIC_SIZE+8, message);
    copyIntToMessage((unsigned int*) &cData.objConf, PUBLISH_TOPIC_SIZE+12, message);
    copyIntToMessage((unsigned int*) &cData.coordConf, PUBLISH_TOPIC_SIZE+16, message);

    return sendToPublishQueue(message);
}
void readCoordDataFromPublishQueue(char* message, struct CoordData *cData){
    readIntFromMessage((unsigned int*) &cData->xCoord, PUBLISH_TOPIC_SIZE, message);
    readIntFromMessage((unsigned int*) &cData->yCoord, PUBLISH_TOPIC_SIZE+4, message);
    readIntFromMessage((unsigned int*) &cData->objType, PUBLISH_TOPIC_SIZE+8, message);
    readIntFromMessage((unsigned int*) &cData->objConf, PUBLISH_TOPIC_SIZE+12, message);
    readIntFromMessage((unsigned int*) &cData->coordConf, PUBLISH_TOPIC_SIZE+16, message);
}


int sendPWMValToPublishQueue(uint8_t topic, unsigned int duty0, unsigned int duty1){

    char message[PUBLISH_MESSAGE_SIZE];
    message[0] = topic;
    copyIntToMessage((unsigned int*) &duty0, PUBLISH_TOPIC_SIZE, message);
    copyIntToMessage((unsigned int*) &duty1, PUBLISH_TOPIC_SIZE+4, message);
    return sendToPublishQueue(message);
}
void readPWMValFromPublishQueue(char* message, unsigned int *d0, unsigned int *d1)
{
    readIntFromMessage((unsigned int*) d0, PUBLISH_TOPIC_SIZE, message);
    readIntFromMessage((unsigned int*) d1, PUBLISH_TOPIC_SIZE+4, message);
}

//good
int sendTimerValToPublishQueueFromISR(uint8_t topic, unsigned int sequenceVal){

    char message[PUBLISH_MESSAGE_SIZE];
    message[0] = topic;
    copyIntToMessage((unsigned int*) &sequenceVal, PUBLISH_TOPIC_SIZE, message);
    return sendToPublishQueueFromISR(message);
}
//good
void readTimerValFromPublishQueue(char* message, unsigned int *seq)
{
    readIntFromMessage((unsigned int*) seq, PUBLISH_TOPIC_SIZE, message);
}


//////////////////////////////////////////// Publish Queue //////////////////////////////////
QueueHandle_t initializePublishQueue(){
    static int init = 0;
    static QueueHandle_t myQueue;
    if (init == 0){
        myQueue = xQueueCreate(20, PUBLISH_MESSAGE_SIZE);
        init = 1;
    }

    return myQueue;
}

int sendToPublishQueueFromISR(char* payload)
{
    int success = 0;
    QueueHandle_t myQueue = initializePublishQueue();
    if (myQueue == NULL)
    {
        dbgOutputLoc(DBG_MQTTQUEUE_FAIL_INIT);
        dbgFailRoutine();
    }

    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    /*Send timeVal and wait for 10 ticks for space to be available if necessary.
     * Else return failure.*/
    if(xQueueSendToBackFromISR(myQueue, (void*) payload, &xHigherPriorityTaskWoken) == pdPASS ){
        success = 1;
        return success;
    }

    dbgFailRoutine();
    return success;
}

int sendToPublishQueue(char* payload)
{
    int success = 0;
    QueueHandle_t myQueue = initializePublishQueue();
    if (myQueue == NULL)
    {
        dbgOutputLoc(DBG_MQTTQUEUE_FAIL_INIT);
        dbgFailRoutine();
    }

    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    /*Send timeVal and wait for 10 ticks for space to be available if necessary.
     * Else return failure.*/
    if(xQueueSendToBack(myQueue, (void*) payload, &xHigherPriorityTaskWoken) == pdPASS ){
        success = 1;
        return success;
    }

    dbgFailRoutine();
    return success;
}

int readMsgFromPublishQueue(char* message){
    int success = 0;
        QueueHandle_t myQueue = initializePublishQueue();
        if (myQueue == NULL)
        {
            dbgOutputLoc(DBG_MQTTQUEUE_FAIL_INIT);
            dbgFailRoutine();
        }


    if(xQueueReceive(myQueue, (void*) message, portMAX_DELAY) == pdPASS){
        success = 1;
        return success;
    }
    dbgFailRoutine();
    return success;
}


///////////////////////////////////////////// IR Q ///////////////////////////////////////////
QueueHandle_t initializeIRQueue(){
    static int init = 0;
    static QueueHandle_t myQueue;
    if (init == 0){
        // change me when copying
        myQueue = xQueueCreate(QUEUE_SIZE, IR_MESSAGE_SIZE);
        init = 1;
    }

    return myQueue;
}

int sendToIRQueueFromISR(float irVal, unsigned int sequenceVal)
{
    int success = 0;
    // Change me when copying
    QueueHandle_t myQueue = initializeIRQueue();
    if (myQueue == NULL)
    {
        dbgOutputLoc(DBG_MQTTQUEUE_FAIL_INIT);
        dbgFailRoutine();
    }

    // change these when copying
    char message[IR_MESSAGE_SIZE];
    copyIntToMessage((unsigned int*) &irVal, 0, message);
    copyIntToMessage((unsigned int*) &sequenceVal, IR_SIZE, message);

    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    /*Send timeVal and wait for 10 ticks for space to be available if necessary.
     * Else return failure.*/
    if(xQueueSendToBackFromISR(myQueue, (void*) message, &xHigherPriorityTaskWoken) == pdPASS ){
        success = 1;
        return success;
    }

    dbgFailRoutine();
    return success;
}

int sendToIRQueue(float irVal, unsigned int sequenceVal)
{
    int success = 0;
    // Change me when copying
    QueueHandle_t myQueue = initializeIRQueue();
    if (myQueue == NULL)
    {
        dbgOutputLoc(DBG_MQTTQUEUE_FAIL_INIT);
        dbgFailRoutine();
    }

    // change these when copying
    char message[IR_MESSAGE_SIZE];
    copyIntToMessage((unsigned int*) &irVal, 0, message);
    copyIntToMessage((unsigned int*) &sequenceVal, IR_SIZE, message);

    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    /*Send timeVal and wait for 10 ticks for space to be available if necessary.
     * Else return failure.*/
    if(xQueueSendToBack(myQueue, (void*) message, &xHigherPriorityTaskWoken) == pdPASS ){
        success = 1;
        return success;
    }

    dbgFailRoutine();
    return success;
}

int readMsgFromIRQueue(struct IRQData *myQD){
    int success = 0;
    // change me when copying
    QueueHandle_t myQueue = initializeIRQueue();
    if (myQueue == NULL)
    {
        dbgOutputLoc(DBG_MQTTQUEUE_FAIL_INIT);
        dbgFailRoutine();
    }

    char message[IR_MESSAGE_SIZE];

    // change me when copying
    if(xQueueReceive(myQueue, (void*) message, portMAX_DELAY) == pdPASS){
        readIntFromMessage((unsigned int*) &myQD->val, 0, message);
        readIntFromMessage((unsigned int*) &myQD->seq, IR_SIZE, message);
        success = 1;
        return success;
    }
    dbgFailRoutine();
    return success;
}

void readIRFromPublishQueue(char* message, struct IRQData *myQD)
{
    readIntFromMessage((unsigned int*) &myQD->val, PUBLISH_TOPIC_SIZE, message);
    readIntFromMessage((unsigned int*) &myQD->seq, PUBLISH_TOPIC_SIZE + IR_SIZE, message);
}

int sendIRToPublishQueueFromISR(uint8_t topic, float irVal, unsigned int sequenceVal)
{
    char message[PUBLISH_MESSAGE_SIZE];
    message[0] = topic;
    copyIntToMessage((unsigned int*) &irVal, PUBLISH_TOPIC_SIZE, message);
    copyIntToMessage((unsigned int*) &sequenceVal, PUBLISH_TOPIC_SIZE + IR_SIZE, message);
    return sendToPublishQueueFromISR(message);
}

int sendIRToPublishQueue(uint8_t topic, float irVal, unsigned int sequenceVal)
{
    char message[PUBLISH_MESSAGE_SIZE];
    message[0] = topic;
    copyIntToMessage((unsigned int*) &irVal, PUBLISH_TOPIC_SIZE, message);
    copyIntToMessage((unsigned int*) &sequenceVal, PUBLISH_TOPIC_SIZE + IR_SIZE, message);
    return sendToPublishQueue(message);
}



///////////////////////////////////////////// MQTT Q /////////////////////////////////////////////
QueueHandle_t initializeMQTTQueue(){
    static int init = 0;
    static QueueHandle_t MQTTQueue;
    if (init == 0){
        MQTTQueue = xQueueCreate(3, MQTT_MESSAGE_SIZE);
        init = 1;
    }

    return MQTTQueue;
}

int sendToMQTTQueueFromISR(int event)
{
    int success = 0;
    QueueHandle_t MQTTQueue = initializeMQTTQueue();
    if (MQTTQueue == NULL)
    {
        dbgOutputLoc(DBG_MQTTQUEUE_FAIL_INIT);
        dbgFailRoutine();
    }

    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    /*Send timeVal and wait for 10 ticks for space to be available if necessary.
     * Else return failure.*/
    if(xQueueSendToBackFromISR(MQTTQueue, (void*) &event, &xHigherPriorityTaskWoken) == pdPASS ){
        success = 1;
        return success;
    }

    dbgOutputLoc(DBG_MQTTQUEUE_FAIL);
    dbgFailRoutine();
    return success;
}

int sendToMQTTQueue(int event)
{
    int success = 0;
    QueueHandle_t MQTTQueue = initializeMQTTQueue();
    if (MQTTQueue == NULL)
    {
        dbgOutputLoc(DBG_MQTTQUEUE_FAIL_INIT);
        dbgFailRoutine();
    }

    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    /*Send timeVal and wait for 10 ticks for space to be available if necessary.
     * Else return failure.*/
    if(xQueueSendToBack(MQTTQueue, (void*) &event, &xHigherPriorityTaskWoken) == pdPASS ){
        success = 1;
        return success;
    }

    dbgOutputLoc(DBG_MQTTQUEUE_FAIL);
    dbgFailRoutine();
    return success;
}

int readMsgFromMQTTQueue(int *myQD){
    int success = 0;
        QueueHandle_t MQTTQueue = initializeMQTTQueue();
        if (MQTTQueue == NULL)
        {
            dbgOutputLoc(DBG_MQTTQUEUE_FAIL_INIT);
            dbgFailRoutine();
        }

    if(xQueueReceive(MQTTQueue, (void*) myQD, portMAX_DELAY) == pdPASS){
        success = 1;
        return success;
    }
    dbgFailRoutine();
    return success;
}


