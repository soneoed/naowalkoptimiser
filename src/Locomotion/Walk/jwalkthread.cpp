/**
 * @author Jason Kulk
 *
 * Version : $Id
 */

#include "jwalkthread.h"
#include "jwalk.h"

void* runJWalk(void *arg)
{
    thelog << "JWALKTHREAD: Starting." << endl;
    
    JWalk* jwalk;
    jwalk = (JWalk*) arg;

    Sensors* sensors = jwalk->JWalkSensors;
    Actuators* actuators = jwalk->JWalkActuators;
    NUWalk* nuwalk = jwalk->nuWalk;
    
#if JWALKTHREAD_MONITOR_TIME
    struct timespec pretime, starttime, endtime;
    struct timespec relstarttime, relendtime;
    struct timespec prostarttime, proendtime;
    float runtime, waittime, relruntime, proruntime;       // the run time in ms
#endif

    int err;
    do 
    {
        #if JWALKTHREAD_MONITOR_TIME
            clock_gettime(CLOCK_REALTIME, &pretime);
        #endif

        err = sem_wait(&semaphoreNewSensorData);

        #if JWALKTHREAD_MONITOR_TIME
            clock_gettime(CLOCK_REALTIME, &starttime);
            clock_gettime(CLOCK_THREAD_CPUTIME_ID, &relstarttime);
            clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &prostarttime);
            waittime = (starttime.tv_nsec - pretime.tv_nsec)/1e6 + (starttime.tv_sec - pretime.tv_sec)*1e3;
            if (waittime > 25)
                thelog << "JWALKTHREAD: Waittime " << waittime << " ms."<< endl;
        #endif
        // -----------------------------------------------------------------------------------------------------------------------------------------------------------------
        sensors->onNewSensorData();
        
        // generate event based signals here
        if (jwalk->JWalkWaitingForFinish == true)
        {
            if (jwalk->walkIsActive() == false)
            {
                pthread_mutex_lock(&jWalkStoppedMutex);
                pthread_cond_signal(&jWalkStoppedCondition);
                pthread_mutex_unlock(&jWalkStoppedMutex);
            }
        }
        
        // Do critical tasks here
        if (balanceFallen == true)
        {
            //thelog << "JWALKTHREAD: I have fallen over, I should try and get up" << endl;
            jwalk->getUp();
        }
        else if (balanceFalling == true)
        {
            //thelog << "JWALKTHREAD: Help! I am falling over :(" << endl;
            jwalk->braceForImpact();
        }
        else if (touchOnGround == false && touchPreviousOnGround == true)
        {
            thelog << "JWALKTHREAD: The walk engine has decided the robot is no longer on the ground." << endl;
            jwalk->stop();
        }
        else
        {
            /* -------------------------------------------------------------------------------------------------------------------------------------------------------------
            The Actual 'Walk Engine' is down here */
            
            if (jwalk->JWalkMode == JWALK_TRANSITION)
            {
                if (jwalk->JWalkPreviousMode == JWALK_ALMODE)
                    actuators->setStiffnessNotHead(jwalk->alWalk->alWalkHardnesses);
                else if (jwalk->JWalkPreviousMode == JWALK_JMODE ||jwalk->JWalkPreviousMode == JWALK_GMODE)
                    nuwalk->doWalk(); 
            }
            else if (jwalk->JWalkMode == JWALK_ALMODE)
                actuators->setStiffnessNotHead(jwalk->alWalk->alWalkHardnesses);
            else if (jwalk->JWalkMode == JWALK_JMODE || jwalk->JWalkMode == JWALK_GMODE)
                nuwalk->doWalk();
        }

        // -----------------------------------------------------------------------------------------------------------------------------------------------------------------
        #if JWALKTHREAD_MONITOR_TIME
            clock_gettime(CLOCK_REALTIME, &endtime);
            clock_gettime(CLOCK_THREAD_CPUTIME_ID, &relendtime);
            clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &proendtime);
            runtime = (endtime.tv_nsec - starttime.tv_nsec)/1e6 + (endtime.tv_sec - starttime.tv_sec)*1e3;
            relruntime = (relendtime.tv_nsec - relstarttime.tv_nsec)/1e6 + (relendtime.tv_sec - relstarttime.tv_sec)*1e3;
            proruntime = (proendtime.tv_nsec - prostarttime.tv_nsec)/1e6 + (proendtime.tv_sec - prostarttime.tv_sec)*1e3;
            if (runtime > 8)
            {
                thelog << "JWALKTHREAD: Jason cycle time error: " << runtime << " ms. Time spent in this thread: " << relruntime << "ms, in this process: " << proruntime << endl;
            }
        #endif
    } 
    while (err != EINTR);
    pthread_exit(NULL);
}
