/*
 * debug.h
 *
 *  Created on: Sep 5, 2020
 *      Author: Brinza
 */

#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Timer.h>
#include <ti/drivers/UART.h>
#include <ti/devices/cc32xx/inc/hw_types.h>
#include <ti/devices/cc32xx/driverlib/interrupt.h>

/* Board Header file */
#include "ti_drivers_config.h"

#ifndef DEBUG_H_
#define DEBUG_H_

#define DEBUG_GPIO_0 CONFIG_GPIO_1
#define DEBUG_GPIO_1 CONFIG_GPIO_2
#define DEBUG_GPIO_2 CONFIG_GPIO_3
#define DEBUG_GPIO_3 CONFIG_GPIO_4
#define DEBUG_GPIO_4 CONFIG_GPIO_5
#define DEBUG_GPIO_5 CONFIG_GPIO_6
#define DEBUG_GPIO_6 CONFIG_GPIO_7
#define DEBUG_GPIO_7 CONFIG_GPIO_0

#define UART_BAUD_RATE 115200

// mqtt_client_app.c
#define DBG_ENTER_MQTT_EVENTCALLBACK 0
#define DBG_EXIT_MQTT_EVENTCALLBACK 1
#define DBG_ENTER_WIFIINIT 2
#define DBG_EXIT_WIFIINIT 3
#define DBG_ENTER_MAINTHREAD 4

// mqtt_if.c
#define DBG_ENTER_MQTTCLIENTCALLBACK 5
#define DBG_EXIT_MQTTCLIENTCALLBACK 6
#define DBG_ENTER_MQTTHELPERTOPICMATCHING 7
#define DBG_EXIT_MQTTHELPERTOPICMATCHING 8
#define DBG_ENTER_MQTTAPPTHREAD 9
#define DBG_BEFORE_MQ_RECEIVE 10
#define DBG_AFTER_MQ_RECEIVE 11
#define DBG_ENTER_MQTTCONTEXTTHREAD 12
#define DBG_EXIT_MQTTCONTEXTTHREAD 13
#define DBG_ENTER_MQTT_INIT 14
#define DBG_EXIT_MQTT_INIT 15
#define DBG_ENTER_MQTTDEINIT 16
#define DBG_EXIT_MQTTDEINIT 17
#define DBG_ENTER_MQTT_IF_CONNECT 18
#define DBG_EXIT_MQTT_IF_CONNECT 19
#define DBG_ENTER_MQTT_IF_DISCONNECT 20
#define DBG_EXIT_MQTT_IF_DISCONNECT 21
#define DBG_ENTER_MQTT_IF_SUBSCRIBE 22
#define DBG_EXIT_MQTT_IF_SUBSCRIBE 23
#define DBG_ENTER_MQTT_IF_UNSUBSCRIBE 24
#define DBG_EXIT_MQTT_IF_UNSUBSCRIBE 25
#define DBG_ENTER_MQTT_IF_PUBLISH 26
#define DBG_EXIT_MQTT_IF_PUBLISH 27

// network_if.c
#define DBG_ENTER_SL_WLANEVENTHANDLER 28
#define DBG_EXIT_SL_WLANEVENTHANDLER 29
#define DBG_ENTER_SL_FATALERROREVENTHANDLER 30
#define DBG_EXIT_SL_FATALERROREVENTHANDLER 31
#define DBG_ENTER_SL_NETAPPEVENTHANDLER 32
#define DBG_EXIT_SL_NETAPPEVENTHANDLER 33
#define DBG_ENTER_SL_GENERALEVENTHANDLER 34
#define DBG_EXIT_SL_GENERALEVENTHANDLER 35
#define DBG_ENTER_SL_SOCKEVENTHANDLER 36
#define DBG_EXIT_SL_SOCKEVENTHANDLER 37
#define DBG_ENTER_NETWORK_IF_INITDRIVER 38
#define DBG_EXIT_NETWORK_IF_INITDRIVER 39
#define DBG_ENTER_NETWORK_IF_DEINITDRIVER 40
#define DBG_EXIT_NETWORK_IF_DEINITDRIVER 41
#define DBG_ENTER_NETWORK_IF_CONNECTAP 52 // 42 = dbg fail state, skip
#define DBG_EXIT_NETWORK_IF_CONNECTAP 43
#define DBG_ENTER_NETWORK_IF_DISCONNECTFROMAP 44 // have not put in debug for exiting
#define DBG_ENTER_NETWORK_IF_IPCONFIGGET 45
#define DBG_EXIT_NETWORK_IF_IPCONFIGGET 46
#define DBG_ENTER_NETWORK_IF_GETHOSTIP 47
#define DBG_EXIT_NETWORK_IF_GETHOSTIP 48

#define DBG_UART_MSG 49
#define DBG_UART_PUTCH 50
#define DBG_UART_REPORT 51
// skip 52


// fail codes
#define DBG_MQTTQUEUE_FAIL_INIT 127
#define DBG_MQTTQUEUE_FAIL 126
#define DBG_STACK_OVERFLOW 125
#define DBG_MALLOC_FAIL     124

/* ir_thread.c  -- [70,90] */
#define DBG_ENTER_SERVO_CONTROL     70
#define DBG_EXIT_SERVO_CONTROL      71
#define DBG_MAINTIMER_CALLBACK_ENTER 72
#define DBG_MAINTIMER_CALLBACK_EXIT 73
#define TIMER_0_INIT_FAIL           74
#define TIMER_0_OPEN_FAIL           75
#define ENTER_SENSOR_READ_TASK      76
#define DBG_SERVO_TIMING_FAIL_INIT  77
#define DBG_READ_FROM_TIMING        78
#define DBG_WRITE_TO_TIMING_ISR     79
#define DBG_SERVO_DONE_FAIL_INIT    80
#define DBG_PWM_0_FAIL              81
#define DBG_PWM_1_FAIL              82
#define DBG_WRITE_TO_MOVING_DONE    83
#define DBG_READ_FROM_DONE          84
#define DBG_BEFORE_UART_WRITE       85
#define DBG_AFTER_UART_WRITE        86
#define DBG_BEFORE_UART_READ        87
#define DBG_AFTER_UART_READ         88
#define DBG_ENTER_COORD_COMP        89
#define DBG_EXIT_COORD_COMP         90
#define DBG_WRITE_TO_SENSOR_DATA_FROM_ISR    91
#define DBG_SENSORDATA_FAIL_INIT_FROM_ISR    92
#define DBG_WRITE_TO_SENSOR_DATA    93
#define DBG_SENSORDATA_FAIL_INIT    94
#define DBG_READ_FROM_SENSOR_DATA   95
#define DBG_SERVO_SENSOR_DATA_FAIL_INIT 96
#define DBG_I2C_INIT_FAIL      97
#define PIXY_NOT_FOUND              98
#define MASTER_SPI_NOT_FOUND              99
//#define PIXY_NOT_FOUND              100
//#define PIXY_NOT_FOUND              101




void dbgUARTVal(unsigned char outVal);
void dbgOutputLoc(unsigned int outLoc);
void dbgFailRoutine(void);
void dbgWriteNumToGPIO(unsigned int num);
void printHelperNum(unsigned int num);
void printHelper(unsigned char str[]);



#endif /* DEBUG_H_ */
