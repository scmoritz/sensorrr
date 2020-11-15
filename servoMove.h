/*
 * servoMove.h
 *
 *  Created on: Sep 26, 2020
 *      Author: scm29
 */

#ifndef SERVOMOVE_H_
#define SERVOMOVE_H_


#include <stddef.h>
#include <math.h>

/* Board Header file */
#include "ti_drivers_config.h"

/* Driver Header files */
#include <ti/drivers/Capture.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/ADC.h>
#include <ti/drivers/Timer.h>
#include <ti/drivers/PWM.h>
#include <ti/drivers/dpl/SemaphoreP.h>

/* project specific*/
#include "myQueues.h"
#include "debug.h"
#include "mqtt_if.h"


/* RTOS header files */
#include "FreeRTOS.h"
#include "task.h"

//#define PUBLISHER        1                  // change this based on your board

#define ONE_SECOND       100
#define LONG_PERIOD      50 // us
#define SHORT_PERIOD     9
#define PERIOD           10000
#define TIME_CONSTANT    1000
#define PWM_PERIOD       20000  // 20ms
#define NOT_FINISHED     0
#define FINISHED         1
#define NO_ANGLE         -1
#define TIMER            1

enum servoState{
    WAITING,
    MOVING,
    DONE,
    NONE
};

/* project specific timer header */

/* method to convert reading to an integer */
//int convertIRtoMil(uint32_t rawValue);

void mainTimer(Timer_Handle myHandle, int_fast16_t status);




#endif /* SERVOMOVE_H_ */
