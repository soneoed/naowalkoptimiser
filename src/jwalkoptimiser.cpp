/**
 * @author Jason Kulk
 *
 * Version : $Id
 */

#include "jwalkoptimiser.h"
#include "optimiser.h"
#include "Locomotion/Walk/jwalk.h"
#include "Locomotion/Walk/sensors.h"
#include "Locomotion/Walk/actuators.h"

#include "Network/Network.h"

float networkVelocityX, networkVelocityY, networkVelocity, networkControl1, networkControl2, networkControl3;


void* runJWalkOptimiser(void *arg)
{
    thelog << "JWALKOPTIMISER: Starting. This behaviour is for testing only!" << endl;
    
    JWalk* jwalk;
    jwalk = (JWalk*) arg;

    Sensors* sensors = jwalk->JWalkSensors;
    Actuators* actuators = jwalk->JWalkActuators;
    Optimiser* optimiser = new Optimiser();
    Network* network = new Network();
    networkVelocityX = 0;
    networkVelocityY = 0;
    networkVelocity = 0;
    networkControl1 = 0;    // distance
    networkControl2 = 0;    // bearing
    networkControl3 = 0;    // orientation
    
    jwalk->initWalk();
    
    static int lowbatterycount = 0;
    struct timespec nextRunTime;                    // The absolute time for the main thread to be executed
    clock_gettime(CLOCK_REALTIME, &nextRunTime);    // Initialise the next run time to be now
    
    bool stopped = true;
    while (true)
    {
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &nextRunTime, NULL);
        network->ProcessUpdates();
        
        if (batteryValues[E_CHARGE] < 0.1)
            lowbatterycount++;
        else
            lowbatterycount = 0;
        
        if (lowbatterycount > 30)
        {
            jwalk->disableFallingControl();
            jwalk->stop();
            system("aplay /home/root/SoundStates/battery.wav");
            usleep(2*1e6);
        }
        
        if (networkControl1 != -1000 || networkControl2 != -1000 || networkControl3 != -1000)
        {   // all network controls are set to -1000 when there is no connection;
            jwalk->enableFallingControl();
            if (networkControl1 < 0)
                jwalk->nuWalkOnBearing(networkControl2, 1, false);
            else if (networkControl2 > 1000)
                jwalk->nuWalkToPoint(networkControl1, networkControl2, false);
            else
                jwalk->nuWalkToPointWithOrientation(networkControl1, networkControl2, networkControl3, false);
        }
        else
        {
            jwalk->disableFallingControl();
            jwalk->stop();
        }
        
        optimiser->doOptimisation(jwalk->nuWalk->CurrentStep, networkVelocity);
        
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
