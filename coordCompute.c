/*
 * coordCompute.c
 *
 *  Created on: Oct 4, 2020
 *      Author: scm29
 */

#include "coordCompute.h"

void *coordComputer(void *arg0){

    dbgOutputLoc(DBG_ENTER_COORD_COMP);

    initializePublishQueue();
    initializeSensorDataQueue();

    bool publishFlag = 0;
//    struct CoordData prior = {0,0,0,0,0};
//    int typeCount = 0;

    while(1){
        struct CoordData cd = {0,0,0,0,0};
        struct SensorData sData;

        if(readFromSensorDataQueue(&sData)){

            float ySOffset = 20*sin(sData.angle);
            float xSOffset = 20*cos(sData.angle);
            float irDist = 10*sData.irval +20;  // cm to mm

                if (sData.objType != 0){
                    cd.objType = sData.objType;
                    cd.objConf = MAX_CONFIDENCE*PIXY_OBJECT_CONFIDENCE;

                    signed int offset = 30-(sData.xCenter*60)/315;   // angle of pixy obj in pixy terms
                    float pixyDist = 10*(3.59+(1680.23/sData.xWidth));//(0.0413*pow(sData.xWidth,2)-4.8045*sData.xWidth+170.25)*10; // distance in pixy terms
                    int newAngle = sData.angle + offset;             // servo angle plus pixy obj angle


                    if (newAngle<0) newAngle = 0;
                    else if (newAngle >180) newAngle = 180;

                    int leftbb = sData.xCenter-sData.xWidth/2;
                    int rightbb = sData.xCenter+sData.xWidth/2;

                    // filter out all boxes on the edges of our view
                    if(leftbb > 30 && rightbb < 270 ){ // O.G. 30 to 270

                        if ( sData.xCenter < 110 || 205 < sData.xCenter ){
                            // if we arent in the center but somehow still have a valid box, use only pixy
                            cd.xCoord = cos(newAngle*PI/180)*pixyDist;
                            cd.yCoord = sin(newAngle*PI/180)*pixyDist;
                            cd.coordConf = MAX_CONFIDENCE*PIXY_ONLY_DISTANCE_CONFIDENCE;
                            publishFlag = true;
                        } else {
                            // use ir sensor if distances are similar
                            //if (((pixyDist-irDist) > 0 ? pixyDist-irDist : -(pixyDist-irDist)) < 80){
                            if(abs(pixyDist-irDist) < 80){
                                float newDist = (pixyDist+irDist)/2;
                                // weird ir error where close objects appear far away
//                                if (newDist > 1300)
//                                {
//                                    newDist = 10;
//                                }
                                cd.xCoord = cos(newAngle*PI/180)*newDist;
                                cd.yCoord = sin(newAngle*PI/180)*newDist;
                                cd.coordConf = MAX_CONFIDENCE*COMBINED_IR_PIXY_CONF;
                                publishFlag = true;
                            } else {                        // without distance sensor
                                cd.xCoord = cos(newAngle*PI/180)*pixyDist;
                                cd.yCoord = sin(newAngle*PI/180)*pixyDist;
                                cd.coordConf = MAX_CONFIDENCE*PIXY_ONLY_DISTANCE_CONFIDENCE;
                                publishFlag = true;
                            }
                        }
                    }
                } else {    // no objects seen but we are detecting an obstacle with IR

                    cd.objType = 0;
                    cd.objConf = MAX_CONFIDENCE*PIXYLESS_CONFIDENCE;

                    if ( 50 < irDist && irDist < 700 ){ // semi-confident within this range
                        cd.xCoord = cos(sData.angle*PI/180)*irDist;
                        cd.yCoord = sin(sData.angle*PI/180)*irDist;
                        cd.coordConf = MAX_CONFIDENCE*IR_ONLY_DISTANCE_CONFIDENCE;
                        publishFlag = true;
                    }
                }


            if (publishFlag){
                cd.yCoord += Y_OFFSET + ySOffset; // offset from rover center and from servo angle
                cd.xCoord += X_OFFSET + xSOffset;
                sendCoordDataToPublishQueue(COORD_DATA_TOPIC, cd);
                publishFlag = false;

            }
        }
    }

}



int distance(struct CoordData coords1, struct CoordData coords2){

    int x = coords2.xCoord - coords1.xCoord;    // order shouldn't matter
    int y = coords2.yCoord - coords1.yCoord;

    return sqrt((x*x+y*y));
}

int singleDistance(struct CoordData c){

    int x = c.xCoord;    // order shouldn't matter
    int y = c.yCoord;

    return sqrt((x*x+y*y));
}

bool equal(struct CoordData coords1, struct CoordData coords2){

    if (coords1.objType == coords2.objType)
        return (distance(coords1, coords2) < 80);

    return false;
}

void average(struct CoordData coords1, struct CoordData *coords2){

        // average the x and y locations
    coords2->xCoord = (coords1.xCoord+coords2->xCoord)/2;
    coords2->yCoord = (coords1.yCoord+coords2->yCoord)/2;
}

void copy(struct CoordData coords1, struct CoordData *coords2){

    coords2->coordConf = coords1.coordConf;
    coords2->objConf = coords1.objConf;
    coords2->objType = coords1.objType;
    coords2->xCoord = coords1.xCoord;
    coords2->yCoord = coords1.yCoord;

}


void destroy(struct CoordData *c){
    c->coordConf = 0;
    c->objConf = 0;
    c->objType = 0;
    c->xCoord = 0;
    c->yCoord = 0;
}

