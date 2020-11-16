/*
 * sensorRead.h
 *
 *  Created on: Sep 26, 2020
 *      Author: scm29
 */

#ifndef SENSORREAD_H_
#define SENSORREAD_H_

#include <stddef.h>
#include <math.h>

/* Board Header file */
#include "ti_drivers_config.h"

/* Driver Header files */
#include <ti/drivers/Capture.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/ADC.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/Timer.h>
#include <ti/drivers/dpl/SemaphoreP.h>

/* project specific*/
#include "myQueues.h"
#include "debug.h"
#include "mqtt_if.h"


/* RTOS header files */
#include "FreeRTOS.h"
#include "task.h"

#define FIVE            5
#define FIFTEEN         15
#define TWENTY          20
#define LOW_END         10
#define HIGH_END        170
#define UP              1
#define DOWN            0
#define SCALE           256243
#define POWER           -1.49
#define FIVE    5
#define ELEVEN  11
#define FIFTEEN 15


// both are default addresses
#define PIXY_SLAVE_ADDRESS         0x54
#define TOF_SLAVE_ADDRESS          0x29

enum readState{
    READING,
    WAITING,
    NOT_READING,
    AIMING,
    NONE
};



int getNewAngle(int oldAngle);
int convertIRtoMil(uint32_t rawValue);

//i2c methods
/*
    I2C_Handle      pixy;
    I2C_Transaction pixyTransaction;
 */

// get blocks methods
void getBlocks(I2C_Handle pix, I2C_Transaction *transaction, struct SensorData *sd);

void useHeadlights(I2C_Handle pix, I2C_Transaction *transaction, bool lamps);

void getTOF(I2C_Handle flight, I2C_Transaction *transaction, struct SensorData *sd);

//static void i2cErrorHandler(I2C_Transaction *transaction);

#endif /* SENSORREAD_H_ */
