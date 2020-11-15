/*
 * jsonParser.c
 *
 *  Created on: Sep 19, 2020
 *      Author: dstan
 */
#include "jsonParser.h"
#include "jsmn.h"


//c to json
void json_junk(uint32_t seqNum, char* format){
    sprintf(format, "{\"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaA\":%zu}", seqNum);
}

void json_chain(uint8_t board, uint32_t chainVal, uint32_t seqNum, char* format){
    sprintf(format, "{\"board\":%zu,\"chainVal\":%zu,\"sequence\":%zu}", board, chainVal, seqNum);
}

void json_stats(uint8_t board, uint32_t pubAttempts, uint32_t succPub, uint32_t pubRec, uint32_t pubNotRec, uint32_t seqNum, char* format){
    sprintf(format, "{\"board\":%zu,\"pubAttempts\":%zu,\"succPub\":%zu,\"pubRec\":%zu,\"pubNotRec\":%zu,\"seqNum\":%zu}", board, pubAttempts, succPub, pubRec, pubNotRec, seqNum);
}

void json_ir(float irRawMM, uint32_t seqNum, char* format){
    sprintf(format, "{\"irRawMM\":%f,\"sequence\":%zu}", irRawMM, seqNum);
}


// these are used for publishing shit

void json_timer_val(uint32_t s, char *format){
    sprintf(format, "{\"timerCount\":%zu}", s);
}

void json_pwm_val(uint32_t d0, uint32_t d1, char* format){
    sprintf(format,"{\"pwmOldValue\":%zu, \"pwmNewValue\": %zu}", d0, d1);
}

void json_pixy(uint32_t center, uint32_t width, uint32_t type, char* format){
    sprintf(format,"{\"ObjCenter\":%zu, \"ObjWidth\": %zu, \"ObjType\": %zu}", center, width, type);
}

void json_sensors(float ir, uint32_t angle, uint32_t center, uint32_t width, uint32_t type, char* format){
    sprintf(format,"{\"IRdist\":%f, \"Angle\": %zu, \"ObjCenter\":%zu, \"ObjWidth\": %zu, \"ObjType\": %zu}", ir, angle, center, width, type);
}

void json_coords(int x, uint32_t y, uint8_t cConf, uint8_t oConf, uint8_t type, char *format){
    sprintf(format,"{\"X Coord\":%i, \"Y Coord\":%zu, \"Coord. Confidence\": %zu, \"Type\":%zu, \"Type Confidence\": %zu}", x, y, cConf, type, oConf);
}


///json to c
int jsoneq(const char *json, jsmntok_t *tok, const char *s) {
    if (tok->type == JSMN_STRING && (int) strlen(s) == tok->end - tok->start &&
            strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
        return 0;
    }
    return -1;
}




// parsing methods below are used by callbacks in the client_app

void parseDone(char *JSON_STRING, unsigned int *done){

    int i;
    int r;
    jsmn_parser p;
    jsmntok_t t[10]; //# of tokens we expect os actully only 1

    jsmn_init(&p);
    r = jsmn_parse(&p, JSON_STRING, strlen(JSON_STRING), t,
                 sizeof(t) / sizeof(t[0]));
    if (r < 0) {
        printf("Failed to parse JSON: %d\n", r);
    }

    /* Assume the top-level element is an object */
    if (r < 1 || t[0].type != JSMN_OBJECT) {
        printf("Object expected\n");
    }

    /* Loop over all keys of the root object */
    for (i = 1; i < r; i++) {
        if (jsoneq(JSON_STRING, &t[i], "done") == 0) {
          int size = t[i + 1].end - t[i + 1].start;
            char val[size + 1];
            sprintf(val,"%.*s",t[i + 1].end - t[i + 1].start, JSON_STRING + t[i + 1].start);
            unsigned int storeVal = atoi (val);
            done = storeVal;
          i++;
        }
    }

}



void parsePWM(char *JSON_STRING, struct pwm *PWM){
    int i;
    int r;
    jsmn_parser p;
    jsmntok_t t[10]; // # of tokens we expect is actually only 1

    jsmn_init(&p);
    r = jsmn_parse(&p, JSON_STRING, strlen(JSON_STRING), t,
                 sizeof(t) / sizeof(t[0]));
    if (r < 0) {
        printf("Failed to parse JSON: %d\n", r);
    }

    /* Assume the top-level element is an object */
    if (r < 1 || t[0].type != JSMN_OBJECT) {
        printf("Object expected\n");
    }

    /* Loop over all keys of the root object */
    for (i = 1; i < r; i++) {
        if (jsoneq(JSON_STRING, &t[i], "position") == 0) {
            int size = t[i + 1].end - t[i + 1].start;
            char val[size + 1];
            sprintf(val,"%.*s",t[i + 1].end - t[i + 1].start, JSON_STRING + t[i + 1].start);
            unsigned int storeVal = atoi (val);
            PWM->position0 = (uint32_t) storeVal;
            i++;
        }
    }
}


void parseSensorData(char *JSON_STRING, struct JSONSensorData *jsd){
    int i;
    int r;
    jsmn_parser p;
    jsmntok_t t[20]; // # of tokens we expect is actually only 6

    jsmn_init(&p);
    r = jsmn_parse(&p, JSON_STRING, strlen(JSON_STRING), t,
                 sizeof(t) / sizeof(t[0]));
    if (r < 0) {
        printf("Failed to parse JSON: %d\n", r);
    }

    /* Assume the top-level element is an object */
    if (r < 1 || t[0].type != JSMN_OBJECT) {
        printf("Object expected\n");
    }

//    unsigned int storeVal = 0;
    /* Loop over all keys of the root object */
    for (i = 1; i < r; i++) {
        if (jsoneq(JSON_STRING, &t[i], "irVal") == 0) {
            int size = t[i + 1].end - t[i + 1].start;
            char val[size + 1];
            sprintf(val,"%.*s", size, JSON_STRING + t[i + 1].start);
            int storeVal = atoi (val);
            jsd->irval = storeVal;
          i++;
        }else if (jsoneq(JSON_STRING, &t[i], "ObjCenter") == 0) {
            int size = t[i + 1].end - t[i + 1].start;
            char val[size + 1];
            sprintf(val,"%.*s", size, JSON_STRING + t[i + 1].start);
            unsigned int storeVal = atoi (val);
            jsd->xCenter = (uint16_t) storeVal;
            i++;
        } else if (jsoneq(JSON_STRING, &t[i], "ObjWidth") == 0) {
            int size = t[i + 1].end - t[i + 1].start;
            char val[size + 1];
            sprintf(val,"%.*s", size, JSON_STRING + t[i + 1].start);
            unsigned int storeVal = atoi (val);
            jsd->xWidth = (uint16_t) storeVal;
            i++;
        }else if (jsoneq(JSON_STRING, &t[i], "ObjType") == 0) {
            int size = t[i + 1].end - t[i + 1].start;
            char val[size + 1];
            sprintf(val,"%.*s", size, JSON_STRING + t[i + 1].start);
            unsigned int storeVal = atoi (val);
            jsd->objType = (uint16_t)storeVal;
            i++;
        }else if (jsoneq(JSON_STRING, &t[i], "Angle") == 0) {
            int size = t[i + 1].end - t[i + 1].start;
            char val[size + 1];
            sprintf(val,"%.*s", size, JSON_STRING + t[i + 1].start);
            unsigned int storeVal = atoi (val);
            jsd->angle = (uint16_t) storeVal;
            i++;
        }
    }


}
