/*
 * coordCompute.h
 *
 *  Created on: Oct 4, 2020
 *      Author: scm29
 */

#ifndef COORDCOMPUTE_H_
#define COORDCOMPUTE_H_

#include "debug.h"
#include "myQueues.h"
#include <math.h>


enum ComputeState {
    NONE,
    COMPUTING,
    PUBLISHING
};

enum itemType{
    GOAL,
    WAYPOINT,
    ROBOT,
    OBSTACLE
};


// must update these with further testing
#define MAX_CONFIDENCE          255
#define PIXYLESS_CONFIDENCE     0.75
#define PIXY_OBJECT_CONFIDENCE  0.75
#define IR_ONLY_DISTANCE_CONFIDENCE  0.75
#define PIXY_ONLY_DISTANCE_CONFIDENCE 0.75
#define COMBINED_IR_PIXY_CONF   0.75
#define PI                      3.14159

// these two offsets will ned adjusting when actually mounted to the rover
#define Y_OFFSET                220     // offset by 100 mm in front of rover
#define X_OFFSET                -25

int distance(struct CoordData coords1,struct CoordData coords2);
bool equal(struct CoordData coords1, struct CoordData coords2);
void average(struct CoordData coords1, struct CoordData *coords2);
void copy(struct CoordData coords1, struct CoordData *coords2);
void destroy(struct CoordData *c);
int singleDistance(struct CoordData c);


#endif /* COORDCOMPUTE_H_ */
