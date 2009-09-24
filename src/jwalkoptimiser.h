/**
 * @author Jason Kulk
 *
 */

#ifndef JWALKOPTIMISER_H
#define JWALKOPTIMISER_H

#include "Locomotion/Walk/jwalkincludes.h"

#define JWALKOPTIMISER_VERBOSITY            3
#define JWALKOPTIMISER_FREQUENCY            10

extern float networkVelocityX, networkVelocityY, networkVelocity, networkControl1, networkControl2, networkControl3;

extern pthread_t jWalkOptimiserThread;

void* runJWalkOptimiser(void* arg);

#endif

