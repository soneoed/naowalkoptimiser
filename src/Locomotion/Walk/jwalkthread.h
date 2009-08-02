/**
 * @author Jason Kulk
 *
 * Version : $Id: jwalkthread.h,v 1.1 2009/05/06 02:42:29 jason Exp $
 */

#ifndef JWALKTHREAD_H
#define JWALKTHREAD_H

#include "jwalkincludes.h"
#define JWALKTHREAD_MONITOR_TIME        1
#define JWALKTHREAD_VERBOSITY           3

extern sem_t semaphoreNewSensorData;
extern pthread_t jWalkThread;

extern pthread_cond_t jWalkStoppedCondition;
extern pthread_mutex_t jWalkStoppedMutex;

void* runJWalk(void* arg);

#endif

