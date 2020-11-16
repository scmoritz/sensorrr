

#include "sensorRead.h"


void *sensorReader(void *arg0){

    dbgOutputLoc(ENTER_SENSOR_READ_TASK);

    //initializing queues
    initializePublishQueue();
    initializeMovingDoneQueue();    // we will use this as a timer
    initializeSensorDataQueue();    // used to comm between here and compute

    uint8_t         writeBuf[6];
    uint8_t         readBuf[22];
    I2C_Handle      i2c;
    I2C_Params      params;
    I2C_Transaction transaction;

    // i2c shit
    I2C_init();

    /* Create I2C for usage */
    I2C_Params_init(&params);
    params.bitRate = I2C_100kHz;    // from 400kHz
    i2c = I2C_open(CONFIG_I2C, &params);
    if (i2c == NULL) {
        dbgOutputLoc(DBG_I2C_INIT_FAIL);
        dbgFailRoutine();
    }

    /* Common I2C transaction setup */
    transaction.writeBuf   = writeBuf;
    transaction.writeCount = 6;
    transaction.readBuf    = readBuf;
    transaction.readCount  = 10;
    transaction.slaveAddress = PIXY_SLAVE_ADDRESS;

    if (!I2C_transfer(i2c, &transaction)) {
        dbgFailRoutine();
    }

    bool lights = false;
    useHeadlights(i2c, &transaction, lights);

    ADC_init();
    ADC_Handle   adc;
    ADC_Params   adcParams;

    ADC_Params_init(&adcParams);
    adc = ADC_open(CONFIG_ADC_0, &adcParams);

    if (adc == NULL){
        dbgFailRoutine();   // stop everything
    }

    uint16_t adcValue;
    uint32_t adcValue_uv;

    struct SensorData data = {0,90,0,0,0,0};

    // send initial angles for servo
    sendToServoTimingQueue(data.angle);

    int adcret = 0;
    unsigned int ServoDoneMoving = 0;
    int dutyHolder = -1;

    while(1){

        static enum readState rs = READING; // always reading

        //read from servo done queue
        if (readFromServoMovingDoneQueue(&ServoDoneMoving, &dutyHolder)){

            if (ServoDoneMoving == 1){

                if (dutyHolder >= 0){
                    data.angle = duty2Angle(dutyHolder);
                }

                rs = READING;
                ServoDoneMoving = 0;

            }
            else{
                if (dutyHolder >= 0 && dutyHolder <= 3000)
                    data.angle = duty2Angle(dutyHolder);
            }
        }

        switch (rs){
            case READING:
                adcret = ADC_convert(adc, &adcValue);
                if (adcret == ADC_STATUS_SUCCESS){
                    adcValue_uv = ADC_convertToMicroVolts(adc, adcValue);
                    data.irval = convertIRtoMil(adcValue_uv);
                }

                // read from i2c for pixy
                getBlocks(i2c, &transaction, &data);

                // publish data to mqtt
//                sendSensorDataToPublishQueue(SENSOR_DATA_TOPIC, data);

//                sendIRToPublishQueue(IR_TOPIC_NUM, data.irval, data.numReadings);
                sendPixyToPublishQueue(PIXY_TOPIC_NUM, data);

                // for coordinate compute thread
                sendToSensorDataQueue(data);

                sendToServoTimingQueue(getNewAngle(data.angle));
//                sendToServoTimingQueue(90);

                rs = NOT_READING;

                break;
            case NOT_READING:
                // do nothing yet
                break;

        }
    }

}


// get some mfn blocks
void getBlocks(I2C_Handle pix, I2C_Transaction *transaction, struct SensorData *sd){

    // get only a single block for now
    // final testing will handle multiple blocks
    uint8_t w[] = { 0xae, 0xc1, 0x20, 0x02, 0xff, 0x02 };
    uint8_t r[40];
    transaction->slaveAddress = PIXY_SLAVE_ADDRESS;
    transaction->writeBuf   = w;   // buffer to write
    transaction->writeCount = 6;
    transaction->readBuf    = r;
    transaction->readCount  = 40;

    I2C_transfer(pix, transaction);

    int mid1 = r[9] << 8 | r[8];
    int mid2 = r[23] << 8 | r[22];

    if (abs(157-mid1) < abs(157-mid2)){
        // below, center is width and width is center
        sd->objType = r[7] << 8 | r[6];
        sd->xWidth = mid1;
        sd->xCenter  =  r[13] << 8 | r[12];
    } else {
        // below, center is width and width is center
        sd->objType = r[21] << 8 | r[20];
        sd->xWidth = mid2;
        sd->xCenter  =  r[27] << 8 | r[26];
    }

    // check for dead center
    if (abs(157 - sd->xWidth) > 50 ){  // 100 pixel wide center
        sd->objType = 0;    // if invalid type, then the other vals are invalid also
        sd->xWidth = 0;
        sd->xCenter = 0;
    }
}

// turn the headlights on helps to regularize the lighting conditions
void useHeadlights(I2C_Handle pix, I2C_Transaction *transaction, bool lamps){

    uint8_t wOn[] = { 0xae, 0xc1, 0x16, 0x02, 0x01, 0x01 };
    uint8_t wOff[] = { 0xae, 0xc1, 0x16, 0x02, 0x00, 0x00 };

    if (lamps)
        transaction->writeBuf  = wOn;   // buffer to write
    else transaction->writeBuf = wOff;

    transaction->writeCount = 6;
    transaction->readCount  = 10;   // who cares about reading here..

    I2C_transfer(pix, transaction);
}

// some logic to get a new angle to move the sensor
// increments +-5deg between a set high and low threshold
int getNewAngle(int oldAngle){

    int ret = oldAngle;
    static bool up = true;

    if (up){
        if (oldAngle + 5 > HIGH_END){
            ret = HIGH_END;  // go back to start
            up = false;
        }
        else if (oldAngle + 5 <= HIGH_END)
            ret += 5;
    } else {
        ret = LOW_END;  // go back to start
        up = true;
//        if (oldAngle - 5 < LOW_END){
//            ret = LOW_END;  // go back to start
//            up = true;
//        }
//        else if (oldAngle - 5 >= LOW_END)
//            ret -= 5;
    }
    return ret;
}


// Used to convert ADC reading to an integer
int convertIRtoMil(uint32_t rawValue){

   /* from our testing and the data sheets, we found this sensor to be
    * highly variable below 10cm, and above 50 cm. These are our 'valid ranges.
    * any value outside this range will be ignored and we return -1 */
    int distance = -1;

    int x = rawValue/1000;

    if( rawValue != 0){
//        distance = 500000000*pow(rawValue, -1.111);
        distance = SCALE*pow(x, POWER);

        if (5 > distance || distance > 70)  // wtf was using mm here...
            return -1;

        if (distance > 65)
            distance = 0.75*distance;
        else if (distance > 45)
            distance = 0.8*distance;
        else if (distance > 25)
            distance = 0.9*distance;

    }

    return distance;
}

