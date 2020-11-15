/*
 * jsonParser.h
 *
 *  Created on: Sep 19, 2020
 *      Author: dstan
 */

#ifndef JSONPARSER_H_
#define JSONPARSER_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "stdint.h"

// JSONify
void json_ir(float irRawMM, uint32_t seqNum, char* format);
void json_timer_val(uint32_t s, char *format);
void json_pixy(uint32_t center, uint32_t width, uint32_t type, char* format);
void json_sensors(float ir, uint32_t angle, uint32_t center, uint32_t width, uint32_t type, char* format);
void json_coords(int x, uint32_t y, uint8_t cConf, uint8_t oConf, uint8_t type, char *format);


//json to c
struct IR {
    float irRawMM;
    uint32_t seqVal;
} IRQData;

struct pwm{
    unsigned int position0;
} servoMoveQData;


struct JSONSensorData{
    int irval;
    unsigned int angle;
    unsigned int xCenter;
    unsigned int xWidth;
    unsigned int objType;
} jsd;


void parsePWM(char *JSON_STRING, struct pwm *PWM);
void parseIR(char* JSON_STRING, struct IR *ir);
void parseSensorData(char *JSON_STRING, struct JSONSensorData *jsd);
void parseDone(char *JSON_STRING, unsigned int *done);


// unused below

struct IRVersion2 {
    uint32_t irval;
    uint32_t angle;
    uint32_t center;
    uint32_t width;
    uint32_t type;
} IRV2;
struct Chains {
    uint32_t irval;
    uint32_t chainVal;
    uint32_t seqVal;
} chain;
struct Stats {
    uint8_t board;
    uint8_t pubAttempts;
    uint8_t succPub;
    uint8_t pubRec;
    uint8_t pubNotRec;
    uint32_t seqVal;
} stat;

// jsonify
void json_junk(uint32_t seqNum, char* format);
void json_chain(uint8_t board, uint32_t chainVal, uint32_t seqNum, char* format);
void json_stats(uint8_t board, uint32_t pubAttempts, uint32_t succPub, uint32_t pubRec, uint32_t pubNotRec, uint32_t seqNum, char* format);


void parseChain(char* JSON_STRING, struct Chains *chain);
void parseStats(char* JSON_STRING, struct Stats *stat);
void parseIRV2(char* JSON_STRING, struct IRVersion2 *IRV2);
void parseJunk(char* JSON_STRING, uint32_t *seqVal);
#endif /* JSONPARSER_H_ */
