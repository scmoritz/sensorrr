/*
 * servoMove.c
 *
 *  Created on: Sep 26, 2020
 *      Author: scm29
 */

#include "servoMove.h"

void *servoMover(void *arg0)
{
    dbgOutputLoc(DBG_ENTER_SERVO_CONTROL);

    PWM_Params pwmParams;
    PWM_Handle pwm0;//, pwm1;

    PWM_init();

    uint32_t duty = 1500;     // start in the middle

    /* PWM Params init */
    PWM_Params_init(&pwmParams);
    pwmParams.dutyUnits = PWM_DUTY_US;
    pwmParams.dutyValue = duty;
    pwmParams.periodUnits = PWM_PERIOD_US;
    pwmParams.periodValue = PWM_PERIOD;

    /* Open PWM0 */
    pwm0 = PWM_open(CONFIG_PWM_0, &pwmParams);

    if (!pwm0) {
        dbgOutputLoc(DBG_PWM_0_FAIL);
        dbgFailRoutine();
    }

    PWM_start(pwm0);
    PWM_setDuty(pwm0, duty);

    //initialize queues
    initializePublishQueue();
    initializeMovingDoneQueue();
    initializeServoTimingQueue();

    /* initializing a single timer */
    Timer_Handle timer;
    Timer_Params params;

    Timer_Params_init(&params);
    params.period = PERIOD*3;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = mainTimer; // callback used for a publisher board
    timer = Timer_open(CONFIG_TIMER_0, &params);

    if (timer == NULL){
        dbgOutputLoc(TIMER_0_INIT_FAIL);
        dbgFailRoutine();   // stop everything
    }

    if (Timer_start(timer) == Timer_STATUS_ERROR){
        dbgOutputLoc(TIMER_0_OPEN_FAIL);
        dbgFailRoutine();   // stop everything
    }

    uint32_t oldDuty = duty;
    uint32_t newDuty = duty;

    while(1){

        struct servoMoveQData moveData;
        static enum servoState ss = WAITING;

        if (readFromServoTimingQueue(&moveData)){

            // if a valid position, 0 to 180deg, update the desired duty cycle
            if (180 >= moveData.position0 && moveData.position0 >= 0){
                newDuty = angle2Duty(moveData.position0); // servo goes 500-2500ms duty cycle
                ss = MOVING;
            }
        }

        switch (ss){

            case WAITING:

                break;
            case MOVING:
                // do some moving stuff
                if(oldDuty != newDuty){
                    if (oldDuty < newDuty){
                        if(oldDuty + 15 < newDuty){
                            oldDuty += 15;
                        }
                        else{
                            oldDuty = newDuty;
                            sendToServoMovingDoneQueue(FINISHED, oldDuty);
                            ss = WAITING;
                            break;
                        }
                    } else {
                        if(oldDuty - 15 > newDuty){
                            oldDuty -= 15;
                        }
                        else{
                            oldDuty = newDuty;
                            sendToServoMovingDoneQueue(FINISHED, oldDuty);
                            ss = WAITING;
                            break;
                        }
                    }

                    PWM_setDuty(pwm0, oldDuty);
                    //sendToServoMovingDoneQueue(NOT_FINISHED, oldDuty);
                } else{
                    sendToServoMovingDoneQueue(FINISHED, oldDuty);
                    ss = WAITING;

                }
                break;

        }
    }
}


void mainTimer(Timer_Handle myHandle, int_fast16_t status){

    dbgOutputLoc(DBG_MAINTIMER_CALLBACK_ENTER);
    static unsigned int seq = 1;

    // write to servo timing queue to trigger pwm to change if servo is in move mode
    sendToServoTimingQueueFromISR(NO_ANGLE);

    seq++;  // increment the sequence
    if (seq % 3 == 0){
        sendToServoMovingDoneQueueFromISR(NOT_FINISHED, -1);
        //sendTimerValToPublishQueueFromISR(TIMER_TOPIC_NUM, seq);
    }

    dbgOutputLoc(DBG_MAINTIMER_CALLBACK_EXIT);

}





