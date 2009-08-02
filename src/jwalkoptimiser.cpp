/**
 * @author Jason Kulk
 *
 * Version : $Id
 */

#include "jwalkoptimiser.h"
#include "Locomotion/Walk/jwalk.h"
#include "Locomotion/Walk/sensors.h"
#include "Locomotion/Walk/actuators.h"


void* runJWalkOptimiser(void *arg)
{
    thelog << "JWALKOPTIMISER: Starting. This behaviour is for testing only!" << endl;
    
    JWalk* jwalk;
    jwalk = (JWalk*) arg;

    Sensors* sensors = jwalk->JWalkSensors;
    Actuators* actuators = jwalk->JWalkActuators;
    
    struct timespec nextRunTime;                    // The absolute time for the main thread to be executed
    clock_gettime(CLOCK_REALTIME, &nextRunTime);    // Initialise the next run time to be now
    
    sleep(1);                         // the lazy man's guide to getting up
    jwalk->alWalk->goToAnglesWithVelocityNotHead(jwalk->JWalkCommonPositions, 0.5);
    actuators->setStiffnessNotHead(jwalk->JWalkCommonHardnesses);
    
    float headstiffnesses[2] = {0.25, 0.25};
    jwalk->setHeadStiffness(headstiffnesses);
    jwalk->setHeadYaw(0, 100);
    jwalk->setHeadPitch(0, 100);
    int timeheadmoved = dcmTime;
    float yawposition = 1.0;
    float pitchposition = 0.5;
    
    DodgeStateEnum DodgeState = DODGE_NONE;
    bool stopped = true;
    while (true)
    {
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &nextRunTime, NULL);
        //thelog << "JWALKOPTIMISER: Is running! I win, or you lose." << endl;
        
        if (stopped == true)
        {
            if (touchValues[T_CHEST_BUTTON])
            {
                thelog << "JWALKOPTIMISER: Chest button was pressed. Starting the robot." << endl;
                DodgeState = DODGE_NONE;
                stopped = false;
                balanceFallingEnabled = true;
                sleep(0.5);
            }
        }
        else
        {
            // Do a little bit of head motion
            if (timeheadmoved < dcmTime)
            {
                jwalk->setHeadYaw(yawposition, 1000);
                timeheadmoved = jwalk->setHeadPitch(pitchposition, 1000);
                yawposition = -yawposition;
                pitchposition = -pitchposition;
            }
            
            if (touchValues[T_CHEST_BUTTON])
            {
                thelog << "JWALKOPTIMISER: Chest button was pressed. Bring the robot to rest" << endl;
                jwalk->stop();
                DodgeState = DODGE_NONE;
                stopped = true;
                balanceFallingEnabled = false;
                sleep(0.5);
            }
            else if (touchOnGround == false)
            {
                thelog << "JWALKOPTIMISER: I have been picked up. Bring the robot to rest" << endl;
                jwalk->stop();
                DodgeState = DODGE_DECISION;
            }
            else
            {
                balanceFallingEnabled = true;
                if (touchValues[T_L_BUMP_L] || touchValues[T_L_BUMP_R] || touchValues[T_R_BUMP_L] || touchValues[T_R_BUMP_R])
                {
                    thelog << "JWALKOPTIMISER: I kicked something. Bring the robot to rest" << endl;
                    jwalk->stop();
                    DodgeState = DODGE_DECISION;
                }
                else
                {
                }
            }
        }
        
        // calculation of next run time
        nextRunTime.tv_nsec += 1e9/JWALKOPTIMISER_FREQUENCY;
        if (nextRunTime.tv_nsec > 1e9)              // we need to be careful with the nanosecond clock overflowing...
        {
            nextRunTime.tv_sec += 1;
            nextRunTime.tv_nsec -= 1e9;
        }
    }
    pthread_exit(NULL);
}
