/*
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*****************************************************************************

   Application Name     -   MQTT Client
   Application Overview -   The device is running a MQTT client which is
                           connected to the online broker. Three LEDs on the
                           device can be controlled from a web client by
                           publishing msg on appropriate topics. Similarly,
                           message can be published on pre-configured topics
                           by pressing the switch buttons on the device.

   Application Details  - Refer to 'MQTT Client' README.html

*****************************************************************************/
#include <mqtt_if.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <mqueue.h>

#include <ti/drivers/net/wifi/simplelink.h>
#include <ti/drivers/net/wifi/slnetifwifi.h>

#include <ti/drivers/SPI.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Timer.h>
#include <ti/drivers/ADC.h>

#include <ti/net/mqtt/mqttclient.h>

#include "network_if.h"
#include "uart_term.h"
#include "mqtt_if.h"
#include "debug_if.h"
#include "debug.h"
#include "jsonParser.h"
#include "myQueues.h"

#include "ti_drivers_config.h"

extern int32_t ti_net_SlNet_initConfig();

#define APPLICATION_NAME         "MQTT client"
#define APPLICATION_VERSION      "2.0.0"

#define SL_TASKSTACKSIZE            2048
#define SPAWN_TASK_PRIORITY         9

// un-comment this if you want to connect to an MQTT broker securely
//#define MQTT_SECURE_CLIENT

#define MQTT_MODULE_TASK_PRIORITY   2
#define MQTT_MODULE_TASK_STACK_SIZE 2048

#define MQTT_WILL_TOPIC             "stefan_will_topic"
#define MQTT_WILL_MSG               "stefan_will_msg"
#define MQTT_WILL_QOS               MQTT_QOS_0
#define MQTT_WILL_RETAIN            false

#define MQTT_CLIENT_PASSWORD        "R8Fk2nxcljf7"
#define MQTT_CLIENT_USERNAME        "uabxvuwu"
#define MQTT_CLIENT_KEEPALIVE       0
#define MQTT_CLIENT_CLEAN_CONNECT   true
#define MQTT_CLIENT_MQTT_V3_1       true
#define MQTT_CLIENT_BLOCKING_SEND   true

#ifndef MQTT_SECURE_CLIENT
#define MQTT_CONNECTION_FLAGS           MQTTCLIENT_NETCONN_URL
#define MQTT_CONNECTION_ADDRESS         "postman.cloudmqtt.com"
#define MQTT_CONNECTION_PORT_NUMBER     15062
#else
#define MQTT_CONNECTION_FLAGS           MQTTCLIENT_NETCONN_IP4 | MQTTCLIENT_NETCONN_SEC
#define MQTT_CONNECTION_ADDRESS         "192.168.178.67"
#define MQTT_CONNECTION_PORT_NUMBER     8883
#endif

// Thread BS
extern void *servoMover(void *arg0);
extern void *sensorReader(void *arg0);
extern void *coordComputer(void *arg0);


/* Stack size in bytes */
#define THREADSTACKSIZE   2048

/* Client ID                                                                 */
/* If ClientId isn't set, the MAC address of the device will be copied into  */
/* the ClientID parameter.                                                   */
char ClientId[13] = {'\0'};

enum{
    APP_MQTT_PUBLISH,
    APP_MQTT_CON_TOGGLE,
    APP_MQTT_DEINIT,
    APP_BTN_HANDLER
};

struct msgQueue
{
    int   event;
    char* payload;
};

MQTT_IF_InitParams_t mqttInitParams =
{
     MQTT_MODULE_TASK_STACK_SIZE,   // stack size for mqtt module - default is 2048
     MQTT_MODULE_TASK_PRIORITY      // thread priority for MQTT   - default is 2
};

MQTTClient_Will mqttWillParams =
{
     MQTT_WILL_TOPIC,    // will topic
     MQTT_WILL_MSG,      // will message
     MQTT_WILL_QOS,      // will QoS
     MQTT_WILL_RETAIN    // retain flag
};

MQTT_IF_ClientParams_t mqttClientParams =
{
     ClientId,                  // client ID
     MQTT_CLIENT_USERNAME,      // user name
     MQTT_CLIENT_PASSWORD,      // password
     MQTT_CLIENT_KEEPALIVE,     // keep-alive time
     MQTT_CLIENT_CLEAN_CONNECT, // clean connect flag
     MQTT_CLIENT_MQTT_V3_1,     // true = 3.1, false = 3.1.1
     MQTT_CLIENT_BLOCKING_SEND, // blocking send flag
     &mqttWillParams            // will parameters
};

#ifndef MQTT_SECURE_CLIENT
MQTTClient_ConnParams mqttConnParams =
{
     MQTT_CONNECTION_FLAGS,         // connection flags
     MQTT_CONNECTION_ADDRESS,       // server address
     MQTT_CONNECTION_PORT_NUMBER,   // port number of MQTT server
     0,                             // method for secure socket
     0,                             // cipher for secure socket
     0,                             // number of files for secure connection
     NULL                           // secure files
};
#else
/*
 * In order to connect to an MQTT broker securely, the MQTTCLIENT_NETCONN_SEC flag,
 * method for secure socket, cipher, secure files, number of secure files must be set
 * and the certificates must be programmed to the file system.
 *
 * The first parameter is a bit mask which configures the server address type and security mode.
 * Server address type: IPv4, IPv6 and URL must be declared with the corresponding flag.
 * All flags can be found in mqttclient.h.
 *
 * The flag MQTTCLIENT_NETCONN_SEC enables the security (TLS) which includes domain name
 * verification and certificate catalog verification. Those verifications can be skipped by
 * adding to the bit mask: MQTTCLIENT_NETCONN_SKIP_DOMAIN_NAME_VERIFICATION and
 * MQTTCLIENT_NETCONN_SKIP_CERTIFICATE_CATALOG_VERIFICATION.
 *
 * Note: The domain name verification requires URL Server address type otherwise, this
 * verification will be disabled.
 *
 * Secure clients require time configuration in order to verify the server certificate validity (date)
 */

/* Day of month (DD format) range 1-31                                       */
#define DAY                      1
/* Month (MM format) in the range of 1-12                                    */
#define MONTH                    5
/* Year (YYYY format)                                                        */
#define YEAR                     2020
/* Hours in the range of 0-23                                                */
#define HOUR                     4
/* Minutes in the range of 0-59                                              */
#define MINUTES                  00
/* Seconds in the range of 0-59                                              */
#define SEC                      00

char *MQTTClient_secureFiles[1] = {"ca-cert.pem"};

MQTTClient_ConnParams mqttConnParams =
{
    MQTT_CONNECTION_FLAGS,                  // connection flags
    MQTT_CONNECTION_ADDRESS,                // server address
    MQTT_CONNECTION_PORT_NUMBER,            // port number of MQTT server
    SLNETSOCK_SEC_METHOD_SSLv3_TLSV1_2,     // method for secure socket
    SLNETSOCK_SEC_CIPHER_FULL_LIST,         // cipher for secure socket
    1,                                      // number of files for secure connection
    MQTTClient_secureFiles                  // secure files
};

void setTime(){

    SlDateTime_t dateTime = {0};
    dateTime.tm_day = (uint32_t)DAY;
    dateTime.tm_mon = (uint32_t)MONTH;
    dateTime.tm_year = (uint32_t)YEAR;
    dateTime.tm_hour = (uint32_t)HOUR;
    dateTime.tm_min = (uint32_t)MINUTES;
    dateTime.tm_sec = (uint32_t)SEC;
    sl_DeviceSet(SL_DEVICE_GENERAL, SL_DEVICE_GENERAL_DATE_TIME,
                 sizeof(SlDateTime_t), (uint8_t *)(&dateTime));
}
#endif

//*****************************************************************************
//!
//! Set the ClientId with its own mac address
//! This routine converts the mac address which is given
//! by an integer type variable in hexadecimal base to ASCII
//! representation, and copies it into the ClientId parameter.
//!
//! \param  macAddress  -   Points to string Hex.
//!
//! \return void.
//!
//*****************************************************************************
int32_t SetClientIdNamefromMacAddress()
{
    int32_t ret = 0;
    uint8_t Client_Mac_Name[2];
    uint8_t Index;
    uint16_t macAddressLen = SL_MAC_ADDR_LEN;
    uint8_t macAddress[SL_MAC_ADDR_LEN];
    //char ClientId[13] = {'\0'};


    /*Get the device Mac address */
    ret = sl_NetCfgGet(SL_NETCFG_MAC_ADDRESS_GET, 0, &macAddressLen,
                       &macAddress[0]);


    /*When ClientID isn't set, use the mac address as ClientID               */
    if(ClientId[0] == '\0')
    {
        /*6 bytes is the length of the mac address                           */
        for(Index = 0; Index < SL_MAC_ADDR_LEN; Index++)
        {
            /*Each mac address byte contains two hexadecimal characters      */
            /*Copy the 4 MSB - the most significant character                */
            Client_Mac_Name[0] = (macAddress[Index] >> 4) & 0xf;
            /*Copy the 4 LSB - the least significant character               */
            Client_Mac_Name[1] = macAddress[Index] & 0xf;

            if(Client_Mac_Name[0] > 9)
            {
                /*Converts and copies from number that is greater than 9 in  */
                /*hexadecimal representation (a to f) into ascii character   */
                ClientId[2 * Index] = Client_Mac_Name[0] + 'a' - 10;
            }
            else
            {
                /*Converts and copies from number 0 - 9 in hexadecimal       */
                /*representation into ascii character                        */
                ClientId[2 * Index] = Client_Mac_Name[0] + '0';
            }
            if(Client_Mac_Name[1] > 9)
            {
                /*Converts and copies from number that is greater than 9 in  */
                /*hexadecimal representation (a to f) into ascii character   */
                ClientId[2 * Index + 1] = Client_Mac_Name[1] + 'a' - 10;
            }
            else
            {
                /*Converts and copies from number 0 - 9 in hexadecimal       */
                /*representation into ascii character                        */
                ClientId[2 * Index + 1] = Client_Mac_Name[1] + '0';
            }
        }
    }
    return(ret);
}

void MQTT_EventCallback(int32_t event){

    dbgOutputLoc(DBG_ENTER_MQTT_EVENTCALLBACK);
    struct msgQueue queueElement;

    switch(event){

        case MQTT_EVENT_CONNACK:
        {
 //         deinit = 0;
 //           connected = 1;
//            //LOG_INFO("MQTT_EVENT_CONNACK\r\n");
//            GPIO_clearInt(CONFIG_GPIO_BUTTON_1);
//            GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
            break;
        }

        case MQTT_EVENT_SUBACK:
        {
//            LOG_INFO("MQTT_EVENT_SUBACK\r\n");
            break;
        }

        case MQTT_EVENT_PUBACK:
        {
//            LOG_INFO("MQTT_EVENT_PUBACK\r\n");
            break;
        }

        case MQTT_EVENT_UNSUBACK:
        {
//            LOG_INFO("MQTT_EVENT_UNSUBACK\r\n");
            break;
        }

        case MQTT_EVENT_CLIENT_DISCONNECT:
        {
 //           connected = 0;
            //LOG_INFO("MQTT_EVENT_CLIENT_DISCONNECT\r\n");
//            if(deinit == 0){
//                GPIO_clearInt(CONFIG_GPIO_BUTTON_1);
//                GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
//            }
            break;
        }

        case MQTT_EVENT_SERVER_DISCONNECT:
        {
//            connected = 0;

            //LOG_INFO("MQTT_EVENT_SERVER_DISCONNECT\r\n");

            queueElement.event = APP_MQTT_CON_TOGGLE;
//            int res = mq_send(appQueue, (const char*)&queueElement, sizeof(struct msgQueue), 0);
//            if(res < 0){
//                //LOG_ERROR("msg queue send error %d", res);
//            }
            break;
        }

        case MQTT_EVENT_DESTROY:
        {
            //LOG_INFO("MQTT_EVENT_DESTROY\r\n");
            break;
        }
    }
    dbgOutputLoc(DBG_EXIT_MQTT_EVENTCALLBACK);
}

/*
 * Subscribe topic callbacks
 * Topic and payload data is deleted after topic callbacks return.
 * User must copy the topic or payload data if it needs to be saved.
 */
void servoCB(char* payload){

    struct pwm servos;
    parsePWM(payload, &servos);

    if (sendToServoTimingQueueFromISR(servos.position0) == 0)
        dbgFailRoutine();

}

void computeTestCB(char *payload){

    //struct SensorData sd;
    struct JSONSensorData jsd;
    parseSensorData(payload, &jsd);

    struct SensorData sd = {jsd.irval, jsd.angle, jsd.xCenter, jsd.xWidth, jsd.objType, 0};

    if (sendToSensorDataQueueFromISR(sd) == 0)
        dbgFailRoutine();
}


void dataCB(char* payload)
{
    unsigned int done;
    parseDone(payload, &done);
    if(sendToServoMovingDoneQueueFromISR(done, -1) == 0)
    {
        dbgFailRoutine();
    }
}

int WifiInit(){

    dbgOutputLoc(DBG_ENTER_WIFIINIT);
    int32_t ret;
    SlWlanSecParams_t security_params;
    pthread_t spawn_thread = (pthread_t) NULL;
    pthread_attr_t pattrs_spawn;
    struct sched_param pri_param;

    pthread_attr_init(&pattrs_spawn);
    pri_param.sched_priority = SPAWN_TASK_PRIORITY;
    ret = pthread_attr_setschedparam(&pattrs_spawn, &pri_param);
    ret |= pthread_attr_setstacksize(&pattrs_spawn, SL_TASKSTACKSIZE);
    ret |= pthread_attr_setdetachstate(&pattrs_spawn, PTHREAD_CREATE_DETACHED);
    ret = pthread_create(&spawn_thread, &pattrs_spawn, sl_Task, NULL);
    if(ret != 0){
        //LOG_ERROR("could not create simplelink task\n\r");
        while(1);
    }

    Network_IF_ResetMCUStateMachine();

    Network_IF_DeInitDriver();

    ret = Network_IF_InitDriver(ROLE_STA);
    if(ret < 0){
        //LOG_ERROR("Failed to start SimpleLink Device\n\r");
        while(1);
    }

    SetClientIdNamefromMacAddress();

    security_params.Key = (signed char*)SECURITY_KEY;
    security_params.KeyLen = strlen(SECURITY_KEY);
    security_params.Type = SECURITY_TYPE;

    ret = Network_IF_ConnectAP(SSID_NAME, security_params);
    if(ret < 0){
        //LOG_ERROR("Connection to an AP failed\n\r");
    }
    else{

        SlWlanSecParams_t securityParams;

        securityParams.Type = SECURITY_TYPE;
        securityParams.Key = (signed char*)SECURITY_KEY;
        securityParams.KeyLen = strlen((const char *)securityParams.Key);

        ret = sl_WlanProfileAdd((signed char*)SSID_NAME, strlen(SSID_NAME), 0, &securityParams, NULL, 7, 0);
        if(ret < 0){
            //LOG_ERROR("failed to add profile %s\r\n", SSID_NAME);
        }
        else{
            //LOG_INFO("profile added %s\r\n", SSID_NAME);
        }
    }

    dbgOutputLoc(DBG_EXIT_WIFIINIT);
    return ret;
}

void mainThread(void * args){

    int32_t ret;
    struct msgQueue queueElement;
    MQTTClient_Handle mqttClientHandle;

    GPIO_init();
    SPI_init();
    Timer_init();
    I2C_init();
    //ADC_init();
    UART_init();

    initializeMQTTQueue();
    initializeIRQueue();
    initializeSensorDataQueue();
    initializePublishQueue();

    dbgOutputLoc(DBG_ENTER_MAINTHREAD);


    ret = ti_net_SlNet_initConfig();
    if(0 != ret)
    {
        //LOG_ERROR("Failed to initialize SlNetSock\n\r");
    }

    ret = WifiInit();
    if(ret < 0){
        while(1);
    }

    ret = MQTT_IF_Init(mqttInitParams);
    if(ret < 0){
        while(1);
    }

#ifdef MQTT_SECURE_CLIENT
    setTime();
#endif

    // mqtt callbacks for testing
    ret |= MQTT_IF_Subscribe(mqttClientHandle, "DataTesting", MQTT_QOS_0, dataCB);
    ret |= MQTT_IF_Subscribe(mqttClientHandle, "ServoControl", MQTT_QOS_0, servoCB);
    ret |= MQTT_IF_Subscribe(mqttClientHandle, "SensorDataSpoof", MQTT_QOS_0, computeTestCB);


    if(ret < 0){
        while(1);
    }
    else{
        //LOG_INFO("Subscribed to all topics successfully\r\n");
    }

    mqttClientHandle = MQTT_IF_Connect(mqttClientParams, mqttConnParams, MQTT_EventCallback);
    if(mqttClientHandle < 0){
        while(1);
    }


    // wait for CONNACK
    while((int)mqttClientHandle == 0);

    pthread_t servoMove, sensorRead, coordCompute; //irThread, chainT, statT;
    //    SlWlanSecParams_t security_params;
    pthread_attr_t pAttrs;
    struct sched_param priParam;
    int retc;

    /* Set priority and stack size attributes */
    pthread_attr_init(&pAttrs);
    priParam.sched_priority = 1;

    retc = pthread_attr_setdetachstate(&pAttrs, PTHREAD_CREATE_DETACHED);
    if(retc != 0){
        /* pthread_attr_setdetachstate() failed */
        while(1);
    }

    pthread_attr_setschedparam(&pAttrs, &priParam);

    retc |= pthread_attr_setstacksize(&pAttrs, THREADSTACKSIZE);

    retc = pthread_create(&servoMove, &pAttrs, servoMover, NULL);
    if (retc != 0) {
             /*pthread_create() failed */
        while (1) {}
    }

    retc = pthread_create(&sensorRead, &pAttrs, sensorReader, NULL);
    if(retc != 0){
        //pthread_create() failed
        while(1);
    }

    retc = pthread_create(&coordCompute, &pAttrs, coordComputer, NULL);
    if (retc != 0) {
             /*pthread_create() failed */
        while (1) {}
    }

    //publish queue init
    //initializePublishQueue(); // done above
    char message[MQTT_PAYLOAD_SIZE];

    while(1){
        //if ((int)mqttClientHandle > 0){
            if (readMsgFromPublishQueue(message)){
                //if branch checks topic num

                if (message[0] == TIMER_TOPIC_NUM){
                    unsigned int seq;
                    readTimerValFromPublishQueue(message, &seq);
                    //serialize and publish
                    char format[MQTT_PAYLOAD_SIZE];
                    json_timer_val((uint32_t) seq, format);
                    //publish using format
                    if (strlen(format) < MQTT_PAYLOAD_SIZE){
                        MQTT_IF_Publish(mqttClientHandle,"SuiteTimer",format, strlen(format), MQTT_QOS_0);
                    }
                }
                if (message[0] == PWM_TOPIC_NUM){
                    unsigned int d0, d1;
                    readPWMValFromPublishQueue(message, &d0, &d1);
                    //serialize and publish
                    char format[MQTT_PAYLOAD_SIZE];
                    json_pwm_val((uint32_t) d0, d1, format);
                    //publish using format
                    if (strlen(format) < MQTT_PAYLOAD_SIZE){
                       MQTT_IF_Publish(mqttClientHandle,"ServoPWM",format, strlen(format), MQTT_QOS_0);
                    }
                }
                if (message[0] == IR_TOPIC_NUM){
                    struct IRQData irData;
                    readIRFromPublishQueue(message, &irData);
                    //serialize and publish
                    char format[MQTT_PAYLOAD_SIZE];
                    json_ir(irData.val, (uint32_t) irData.seq, format);
                    //publish using format
                    if (strlen(format) < MQTT_PAYLOAD_SIZE){
                        MQTT_IF_Publish(mqttClientHandle,"IR_Data",format,strlen(format), MQTT_QOS_0);
                    }
                }
                if (message[0] == PIXY_TOPIC_NUM){
                    struct SensorData sData;
                    readPixyFromPublishQueue(message, &sData);
                    //serialize and publish
                    char format[MQTT_PAYLOAD_SIZE];
                    json_pixy((float) sData.xCenter, (float) sData.xWidth, (unsigned int) sData.objType, format);
                    //publish using format
                    if (strlen(format) < MQTT_PAYLOAD_SIZE){
                        MQTT_IF_Publish(mqttClientHandle,"PIXY_Data",format,strlen(format), MQTT_QOS_0);
                    }
                }
                if (message[0] == SENSOR_DATA_TOPIC){
                    struct SensorData sData;
                    readSensorDataFromPublishQueue(message, &sData);
                    //serialize and publish
                    char format[MQTT_PAYLOAD_SIZE];
                    json_sensors(sData.irval, (uint32_t) sData.angle, (float) sData.xCenter, (float) sData.xWidth, (unsigned int) sData.objType, format);
                    //publish using format
                    if (strlen(format) < MQTT_PAYLOAD_SIZE){
                        MQTT_IF_Publish(mqttClientHandle,"Sensor_Data",format,strlen(format), MQTT_QOS_0);
                    }
                }
                if (message[0] == COORD_DATA_TOPIC){
                    struct CoordData cd;
                    readCoordDataFromPublishQueue(message, &cd);
                    //serialize and publish
                    char format[MQTT_PAYLOAD_SIZE];
                    json_coords(cd.xCoord, cd.yCoord, cd.coordConf, cd.objConf, cd.objType, format);
                    //publish using format
                    if (strlen(format) < MQTT_PAYLOAD_SIZE){
                        MQTT_IF_Publish(mqttClientHandle,"Coord_Data",format,strlen(format), MQTT_QOS_0);
                    }
                }
            }
        //}
        //end new publish
        if(queueElement.event == APP_MQTT_PUBLISH){

            //LOG_TRACE("APP_MQTT_PUBLISH\r\n");

        }
        else if(queueElement.event == APP_MQTT_CON_TOGGLE){

            //LOG_TRACE("APP_MQTT_CON_TOGGLE %d\r\n", connected);


                mqttClientHandle = MQTT_IF_Connect(mqttClientParams, mqttConnParams, MQTT_EventCallback);
                if((int)mqttClientHandle >= 0){
 //                   connected = 1;
                }
 //           }
        }
        else if(queueElement.event == APP_MQTT_DEINIT){
            break;
        }
        else if(queueElement.event == APP_BTN_HANDLER){

//            struct msgQueue queueElement;

            //ret = detectLongPress();
            ret = 0;
            if(ret == 0){

                //LOG_TRACE("APP_BTN_HANDLER SHORT PRESS\r\n");
                queueElement.event = APP_MQTT_CON_TOGGLE;
            }
            else{

                //LOG_TRACE("APP_BTN_HANDLER LONG PRESS\r\n");
                queueElement.event = APP_MQTT_DEINIT;
            }

        }
    }


}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
