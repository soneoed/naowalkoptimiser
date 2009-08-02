/**
 * @author Jason Kulk
 *
 */

#ifndef JWALKOPTIMISER_H
#define JWALKOPTIMISER_H

#include "Locmotion/Walk/jwalkincludes.h"

#define JWALKOPTIMISER_VERBOSITY            3
#define JWALKOPTIMISER_FREQUENCY            10

extern pthread_t jWalkOptimiserThread;

void* runJWalkOptimiser(void* arg);

#endif

