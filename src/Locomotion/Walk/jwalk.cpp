/**
 * @author Jason Kulk
 *
 * Version : $Id: jwalk.cpp,v 1.10 2009/07/01 18:26:56 jason Exp $
 */

#include "jwalkincludes.h"
#include "jwalk.h"

#if JWALK_STANDALONE
#else
    #include "../../Nao.h"    // We working with robocup code I share proxies to aldcm, almemory, and almotion 
#endif

#include "jwalkthread.h"
#include "boost/bind.hpp"

using namespace AL;
using namespace std;

#if DEBUG_TO_FILE
    ofstream thelog;
#endif

// Proxies to Aldebaran Rubbish
#if JWALK_ALMOTION
    ALPtr<ALMotionProxy> alMotion;
#endif
ALPtr<ALMemoryProxy> alStm;
ALMemoryFastAccess* alFastMem;
ALPtr<ALProxy> alDcm;

// Jason's Threads
sem_t semaphoreNewSensorData;
pthread_t jWalkThread;
#if JWALK_OPTIMISER
    #include "../../jwalkoptimiser.h"             // Include the walk optimiser behaviours
    pthread_t jWalkOptimiserThread;
#endif
pthread_cond_t jWalkStoppedCondition;
pthread_mutex_t jWalkStoppedMutex;

#if JWALK_STANDALONE        // When standing alone a pointer to the broker is passed
    JWalk::JWalk(ALPtr<ALBroker> pBroker, const std::string& pName): ALModule(pBroker, pName)
#else
    JWalk::JWalk()
#endif
{
    #if JWALK_STANDALONE
        setModuleDescription("Jason's walk and motion module.");
    #endif
    
#if DEBUG_TO_FILE
    thelog.open("/var/log/jason.log");
    thelog << "Jason's debug log!" << endl;
#endif

    // The order of initialisation here is very important DO NOT MESS WITH IT!
    initAldebaranProxies();
    initSelf();
    initThreads();
    
    // Bind functions --------------------------------------------------------------------------------------------
    
    // -----------------------------------------------------------------------------------------------------------
    
    // Syncronise to the DCM
#if JWALK_STANDALONE
    getParentBroker()->getProxy("DCM")->getModule()->onPostProcess().connect(boost::bind<void>(&JWalk::onDCMPostProcess, this));
#else
    NAO->DCM_Proxy->getModule()->onPostProcess().connect(boost::bind<void>(&JWalk::onDCMPostProcess, this));
#endif
    thelog << "JWALK: Syncronised with the DCM via onPostProcess" << endl;
}

/*! Initialises class members
 */
void JWalk::initSelf()
{
    JWalkPreviousMode = JWALK_SMODE;
    JWalkMode = JWALK_SMODE;             // On initialisation, I expect this to be the safest option because we are going to use a script to get into position anyway
    JWalkNextMode = JWALK_SMODE;         
    JWalkWaitingForFinish = false;
    
    #if JWALK_STANDALONE
        JWalkSensors = new Sensors(this->getParentBroker());
    #else
        JWalkSensors = new Sensors(NAO->Parent_Broker);
    #endif
    JWalkActuators = new Actuators();
    nuWalk = new NUWalk(this);
    alWalk = new ALWalk();
    
    // Initialise the common pose to be jwalk's default pose
    JWalkCommonPositions = nuWalk->DefaultPositions;
    JWalkCommonHardnesses = nuWalk->DefaultHardnesses;
    
    JWalkFunctionID = ID_UNDEFINED;
    JWalkParam1 = 0;
    JWalkParam2 = 0;
    JWalkParam3 = 0;
    
#if JWALK_STANDALONE
    getUpFront = new script("/home/root/StandUpFront.csv", this);
    getUpBack = new script("/home/root/StandUpBack.csv", this);
    getUpBackFall = new script("/home/root/StandUpFromBackFall.csv", this);
#endif
    return;
}

/*! Gets links to the Aldebaran modules using proxies
 - alMotion
 - alStm     (alMemory)
 - alDcm     (not a specialised proxy, so you need to use the call constructors)
 - alFastMem (alMemoryFastAccess, used to get regular access to large amounts of sensor data)
 */
void JWalk::initAldebaranProxies()
{
    // Get proxies to Aldebaran rubbish --------------------------------------------------------------------------
#if JWALK_STANDALONE
    ALError e;
    #if JWALK_ALMOTION
        try 
        {
            alMotion = this->getParentBroker()->getMotionProxy();
            thelog << "JWALK: Get MotionProxy success." << endl;
        }
        catch (ALError& e) 
        {
            thelog << "JWALK: Unable to get MotionProxy: " << e.toString() << endl;
            thelog << "JWALK: alMotion is " << alMotion << endl;
        }
    #endif
    
    try 
    {
        alStm = this->getParentBroker()->getMemoryProxy();
        thelog << "JWALK: Get MemoryProxy success." << endl;
    }
    catch (ALError& e) 
    {
        thelog << "JWALK: Unable to get MemoryProxy:" << e.toString() << endl;
    }
    
    try 
    {
        alDcm = this->getParentBroker()->getProxy("DCM");
        thelog << "JWALK: Get DCMProxy success." << endl;
    }
    catch (ALError& e) 
    {
        thelog << "JWALK: Unable to get a proxy on the DCM:" << e.toString() << endl;
    }
#else
    ALError e;
    #if JWALK_ALMOTION
        try 
        {
            alMotion = NAO->Parent_Broker->getMotionProxy();
            thelog << "JWALK: Get MotionProxy success." << endl;
        }
        catch (ALError& e) 
        {
            thelog << "JWALK: Unable to get MotionProxy: " << e.toString() << endl;
            thelog << "JWALK: alMotion is " << alMotion << endl;
        }
    #endif
    
    try 
    {
        alStm = NAO->Parent_Broker->getMemoryProxy();
        thelog << "JWALK: Get MemoryProxy success." << endl;
    }
    catch (ALError& e) 
    {
        thelog << "JWALK: Unable to get MemoryProxy:" << e.toString() << endl;
    }
    
    try 
    {
        alDcm = NAO->Parent_Broker->getProxy("DCM");
        thelog << "JWALK: Get DCMProxy success." << endl;
    }
    catch (ALError& e) 
    {
        thelog << "JWALK: Unable to get a proxy on the DCM:" << e.toString() << endl;
    }
#endif
    
    alFastMem = new ALMemoryFastAccess();              // The super secret almemoryfastaccess. Use this for very fast access to variables on a regular basis (ie sensor feedback)
    //------------------------------------------------------------------------------------------------------------
    return;
}

/*! Creates the threads
 - A thread is create for motion
 */
void JWalk::initThreads()
{
    // Create my real-time threads -------------------------------------------------------------------------------
    //    1. MotionThread (grabs sensor data, calculates response, sends data to DCM)
    sem_init(&semaphoreNewSensorData, 0, 0);
    pthread_mutex_init(&jWalkStoppedMutex, NULL);
    pthread_cond_init(&jWalkStoppedCondition, NULL);
#if JWALK_REALTIME > 0  
    thelog << "JWALK: Creating jwalkthread as realtime, with priority " << JWALK_REALTIME_PRIORITY << endl;
    
    int err = pthread_create(&jWalkThread, NULL, runJWalk, (void*) this);         // The last parameter is the arguement to the thread
    if (err > 0)
    {
        thelog << "JWALK: ********************* Failed to create jwalkthread *********************" << endl;
        thelog << "The error code was: " << err << endl;
    }
    
    sched_param param;
    param.sched_priority = JWALK_REALTIME_PRIORITY;
    pthread_setschedparam(jWalkThread, SCHED_FIFO, &param);
    
    // double check
    int actualpolicy;
    sched_param actualparam;
    pthread_getschedparam(jWalkThread, &actualpolicy, &actualparam);
    thelog << "JWALK: jwalkthread; Policy: " << actualpolicy << " Priority: " << actualparam.sched_priority << endl;
#else   
    thelog << "JWALK: Creating jwalkthread as non-realtime" << endl;
    int err = pthread_create(&jWalkThread, NULL, runJWalk, (void*) this);
    if (err > 0)
    {
        thelog << "JWALK: ********************* Failed to create jwalkthread *********************" << endl;
        thelog << "The error code was: " << err << endl;
    }
#endif
    // -----------------------------------------------------------------------------------------------------------
    
#if JWALK_OPTIMISER
    //   2. jWalkOptimiser (optimisation behaviours for the walk) 
    err = pthread_create(&jWalkOptimiserThread, NULL, runJWalkOptimiser, (void*) this);
    if (err > 0)
    {
        thelog << "JWALK: ********************* Failed to create jWalkOptimiserThread *********************" << endl;
        thelog << "The error code was: " << err << endl;
    }
    // -----------------------------------------------------------------------------------------------------------
#endif
    return;
}

/*! Sets a semaphore just after the DCM has finished updating alMemory with new sensor data
 */
void JWalk::onDCMPostProcess()
{
    sem_post(&semaphoreNewSensorData);           // trigger the running of threads waiting for new joint sensor data
    return;
}

/*!
 */
void JWalk::initWalk()
{    
#if JWALK_VERBOSITY > 1
    thelog << "JWALK: initWalk(): Moving alMotion into common pose." << endl;
#endif
    float stiffnesses[ALIAS_TARGETS_ALL_LENGTH];
    #if JWALK_ALMOTION
        // assume that the stiffnesses are off!
        for (unsigned char i=0; i<ALIAS_TARGETS_ALL_LENGTH; i++)
            stiffnesses[i] = 0.03;
        JWalkActuators->setStiffnessAll(stiffnesses);
        
        usleep(500);
        usleep(1000*(alWalk->goToAnglesWithVelocityNotHead(JWalkCommonPositions, 3.0) - dcmTime));          // move almotion into the common pose (I hope)
        usleep(1000*(alWalk->goToAnglesWithVelocityNotHead(JWalkCommonPositions, 3.0) - dcmTime));          // move almotion into the common pose (I hope)
        usleep(100);
        
        for (unsigned char i=0; i<ALIAS_TARGETS_ALL_LENGTH; i++)
            stiffnesses[i] = 0.0;
        JWalkActuators->setStiffnessAll(stiffnesses);
        usleep(100);
        JWalkActuators->clearDCM();
        usleep(100);
    #endif
    
    for (unsigned char i=0; i<ALIAS_TARGETS_ALL_LENGTH; i++)
        stiffnesses[i] = 0.8;
    usleep(1000*(JWalkActuators->setStiffnessAll(stiffnesses, 1000) - dcmTime));
    
    // don't forget the head
    float headstiffnesses[2] = {0.25, 0.25};  
    setHeadStiffness(headstiffnesses);
    
    balanceFallingEnabled = true;
    balanceFallen = true;
    
#if JWALK_VERBOSITY > 1
    thelog << "JWALK: initWalk(): Complete; the robot will now getup." << endl;
#endif
}

JWalk::~JWalk()
{
    pthread_cancel(jWalkThread);
    #if JWALK_OPTIMISER
        pthread_cancel(jWalkOptimiserThread);
    #endif
}

/*! Returns true if you are allowed to move the head, bad things happen if you move the head when this function returns false!
 */
bool JWalk::canUseHead()
{
    if (JWalkMode == JWALK_TRANSITION || balanceFallen == true || balanceFalling == true)
        return false;
    else
        return true;
}

/*! Returns true if jwalk is using the alDCM, returns false if jwalk is using alMotion
 */
bool JWalk::usingDCM()
{
    if (JWalkMode == JWALK_JMODE || JWalkMode == JWALK_GMODE || JWalkMode == JWALK_SMODE || JWalkMode == JWALK_TRANSITION)
        return true;
    else
        return false;
}

/*! Returns true if the JWalkMode will use the DCM, returns false if is will use alMotion
 */
bool JWalk::modeUsesDCM(JWalkModeEnum mode)
{
    if (mode == JWALK_JMODE || mode == JWALK_GMODE || mode == JWALK_SMODE || mode == JWALK_TRANSITION)
        return true;
    else
        return false;
}

/********************************************************************************************************************************************************************
 Head Movements
 ********************************************************************************************************************************************************************/
int JWalk::setHeadYaw(float angle, int time)
{
    if (usingDCM() == true)
        return JWalkActuators->goToAngleHeadYaw(angle, time);
    else
    {
        if (balanceFallen == true || balanceFalling == true)
            return 0;
        else
        {
            #if JWALK_ALMOTION
                alMotion->post.gotoAngle("HeadYaw", angle, time/1000.0, 0);
            #endif
            return dcmTime + time;
        }
    }
}

int JWalk::setHeadPitch(float angle, int time)
{
    if (usingDCM() == true)
        return JWalkActuators->goToAngleHeadPitch(angle, time);
    else
    {
        if (balanceFallen == true || balanceFalling == true)
            return 0;
        else
        {
            #if JWALK_ALMOTION
                alMotion->post.gotoAngle("HeadPitch", angle, time/1000.0, 0);
            #endif
            return dcmTime + time;
        }       
    }
    return dcmTime + time;
}

void JWalk::setHeadStiffness(float values[])
{
    JWalkActuators->setStiffnessHead(values);
    return;
}

/********************************************************************************************************************************************************************
 Universal Walk Commands
 ********************************************************************************************************************************************************************/

/*! Returns true if the robot is walking. Returns false otherwise.
 */
bool JWalk::walkIsActive()
{
#if JWALK_VERBOSITY > 4
    thelog << "JWALK: walkIsActive()" << endl;
#endif
    
    if (balanceFalling == true || balanceFallen == true)        // the walk can not be active if I have fallen over
        return false;
    else if (walkAmIWalking == true)                            // if sensors thinks I am moving then I am definitely walking
        return true;
    else if (alWalkIsActive())
        return true;
    else if (gIsActive())
        return true;
    else if (nuWalkIsActive())
        return true;
    else
        return false;                   // we are only not walking when all walk engines are inactive, and the sensors are sure
}

/*! Blocks until the robot has stopped walking
 
 The implementation of this function IS pretty; its event based so it won't use any CPU while waiting :).
 However, it will effectively put your thread to sleep for a some time
 */
void JWalk::waitUntilFinished()
{
    #if JWALK_VERBOSITY > 1
        thelog << "JWALK: waitUntilFinished()" << endl;
    #endif
    
    JWalkWaitingForFinish = true;
    
    thelog << "JWALK: waitUntilFinished() Waiting for finish." << endl;
    // block until finished
    pthread_mutex_lock(&jWalkStoppedMutex);
    pthread_cond_wait(&jWalkStoppedCondition, &jWalkStoppedMutex);
    pthread_mutex_unlock(&jWalkStoppedMutex);
    thelog << "JWALK: waitUntilFinished() Finished." << endl;
    
    JWalkWaitingForFinish = false;
}

/*! Brings the robot to rest as quickly as possible (regardless of which 'mode' of walk is active)
 
 It's a non-blocking function so you will need to keep track (using walkIsActive()) of when the robot is actually stopped
 */
void JWalk::stop()
{
    #if JWALK_VERBOSITY > 1
        thelog << "JWALK: stop()" << endl;
    #endif
    
    if (walkIsActive() == false || balanceFalling == true || balanceFallen == true)
        return;
    else
    {
        switch (JWalkMode) 
        {
            case JWALK_TRANSITION:
                return;         // you can't stop a transition
                break;
            case JWALK_JMODE:
                nuWalkStop();
                break;
            case JWALK_GMODE:
                gStop();
                break;
            case JWALK_ALMODE:
                alWalkStop();
                break;
            default:
                break;
        }
    }
}

void JWalk::emergencyStop()
{
#if JWALK_VERBOSITY > 1
    thelog << "JWALK: emergencyStop()" << endl;
#endif
    
    // there isn't time for a proper transition; so an emergency stop will leave almotion in an unknown position
    JWalkPreviousMode = JWalkMode;
    JWalkMode = JWALK_SMODE;
#if JWALK_ALMOTION
    alMotion->killAll();
#endif
    nuWalk->emergencyStop();
}

void JWalk::braceForImpact()
{
#if JWALK_VERBOSITY > 1
    thelog << "JWALK: braceForImpact()" << endl;
#endif
    emergencyStop();
    
    float stiffnesses[ALIAS_TARGETS_ALL_LENGTH];
    for (unsigned char i=0; i<ALIAS_TARGETS_ALL_LENGTH; i++)
        stiffnesses[i] = 0;
    
    // We always move the head into a safe place, I have decided to preserve the current hardnesses
    stiffnesses[J_HEAD_YAW] = jointHardnesses[J_HEAD_YAW];
    stiffnesses[J_HEAD_PITCH] = jointHardnesses[J_HEAD_PITCH];
    
    if (balanceFallingBackward == true)
    {
        #if JWALK_VERBOSITY > 1
            thelog << "JWALK: braceForImpact(). Bracing for backward impact." << endl;
        #endif
        // tilt the head back as far as possible
        JWalkActuators->goToAngle(J_HEAD_YAW, 0, 0);
        JWalkActuators->goToAngle(J_HEAD_PITCH, 0.707, 0);
        
        JWalkActuators->goToAngle(J_R_SHOULDER_PITCH, 2.1, 0);
        JWalkActuators->goToAngle(J_L_SHOULDER_PITCH, 2.1, 0);
        
        JWalkActuators->goToAngle(J_R_HIP_PITCH, -1.74, 0);
        JWalkActuators->goToAngle(J_L_HIP_PITCH, -1.74, 0);
        JWalkActuators->goToAngle(J_L_HIP_YAWPITCH, -1.0, 0);

        stiffnesses[J_L_SHOULDER_PITCH] = 0.5;
        stiffnesses[J_R_SHOULDER_PITCH] = 0.5;
        stiffnesses[J_L_HIP_PITCH] = 1.0;
        stiffnesses[J_R_HIP_PITCH] = 1.0;
        stiffnesses[J_L_HIP_YAWPITCH] = 1.0;
    }
    else if (balanceFallingForward == true)
    {
        #if JWALK_VERBOSITY > 1
            thelog << "JWALK: braceForImpact(). Bracing for forward impact." << endl;
        #endif
        // tilt the head back as far as possible
        JWalkActuators->goToAngle(J_HEAD_YAW, 0, 100);
        JWalkActuators->goToAngle(J_HEAD_PITCH, -0.707, 100);
        
        JWalkActuators->goToAngle(J_R_HIP_PITCH, 0.44, 100);
        JWalkActuators->goToAngle(J_L_HIP_PITCH, 0.44, 100);
        JWalkActuators->goToAngle(J_R_HIP_ROLL, -0.15, 0);
        JWalkActuators->goToAngle(J_L_HIP_ROLL, 0.15, 0);
        JWalkActuators->goToAngle(J_L_HIP_YAWPITCH, 0.0, 0);
        
        JWalkActuators->goToAngle(J_R_KNEE_PITCH, 2.27, 0);
        JWalkActuators->goToAngle(J_L_KNEE_PITCH, 2.27, 0);
        
        stiffnesses[J_L_HIP_PITCH] = 0.8;
        stiffnesses[J_R_HIP_PITCH] = 0.8;
        stiffnesses[J_L_HIP_ROLL] = 0.5;
        stiffnesses[J_R_HIP_ROLL] = 0.5;
        stiffnesses[J_L_KNEE_PITCH]= 0.5;
        stiffnesses[J_R_KNEE_PITCH]= 0.5;
        stiffnesses[J_L_HIP_YAWPITCH] = 0.5;
    }
    else if (balanceFallingLeft == true)
    {
        #if JWALK_VERBOSITY > 1
            thelog << "JWALK: braceForImpact(). Bracing for left impact." << endl;
        #endif
        
        JWalkActuators->goToAngle(J_HEAD_YAW, 0, 0);
        JWalkActuators->goToAngle(J_HEAD_PITCH, 0.707, 0);
        
        JWalkActuators->goToAngle(J_R_HIP_ROLL, -0.785, 0);
        JWalkActuators->goToAngle(J_L_HIP_ROLL, -0.44, 0);
        
        JWalkActuators->goToAngle(J_L_KNEE_PITCH, 2.27, 0);
        
        JWalkActuators->goToAngle(J_R_SHOULDER_ROLL, -1.64, 0);
        JWalkActuators->goToAngle(J_R_SHOULDER_PITCH, 1.28, 0);
        
        JWalkActuators->goToAngle(J_L_SHOULDER_ROLL, 0, 0);
        JWalkActuators->goToAngle(J_L_SHOULDER_PITCH, 1.37, 0);
        JWalkActuators->goToAngle(J_L_ELBOW_YAW, 0.59, 0);
        JWalkActuators->goToAngle(J_L_ELBOW_ROLL, -1.10, 0);
        
        stiffnesses[J_L_HIP_ROLL] = 0.8;
        stiffnesses[J_R_HIP_ROLL] = 0.8;
        stiffnesses[J_L_KNEE_PITCH] = 0.8;
        
        stiffnesses[J_R_SHOULDER_ROLL] = 0.8;
        stiffnesses[J_R_SHOULDER_PITCH] = 0.8;
        stiffnesses[J_L_SHOULDER_ROLL] = 0.8;
        stiffnesses[J_L_SHOULDER_PITCH] = 0.8;
        stiffnesses[J_L_ELBOW_YAW] = 0.8;
        stiffnesses[J_L_ELBOW_ROLL] = 0.8;
    }
    else if (balanceFallingRight == true)
    {
        #if JWALK_VERBOSITY > 1
            thelog << "JWALK: braceForImpact(). Bracing for right impact." << endl;
        #endif
        
        JWalkActuators->goToAngle(J_HEAD_YAW, 0, 0);
        JWalkActuators->goToAngle(J_HEAD_PITCH, 0.707, 0);
        
        JWalkActuators->goToAngle(J_R_HIP_ROLL, 0.44, 0);
        JWalkActuators->goToAngle(J_L_HIP_ROLL, 0.785, 0);
        
        JWalkActuators->goToAngle(J_R_KNEE_PITCH, 2.27, 0);
        
        JWalkActuators->goToAngle(J_L_SHOULDER_ROLL, 1.64, 0);
        JWalkActuators->goToAngle(J_L_SHOULDER_PITCH, 1.28, 0);
        
        JWalkActuators->goToAngle(J_R_SHOULDER_ROLL, 0, 0);
        JWalkActuators->goToAngle(J_R_SHOULDER_PITCH, 1.37, 0);
        JWalkActuators->goToAngle(J_R_ELBOW_YAW, 0.59, 0);
        JWalkActuators->goToAngle(J_R_ELBOW_ROLL, 1.10, 0);
        
        stiffnesses[J_L_HIP_ROLL] = 0.8;
        stiffnesses[J_R_HIP_ROLL] = 0.8;
        stiffnesses[J_R_KNEE_PITCH] = 0.8;
        
        stiffnesses[J_L_SHOULDER_ROLL] = 0.8;
        stiffnesses[J_L_SHOULDER_PITCH] = 0.8;
        stiffnesses[J_R_SHOULDER_ROLL] = 0.8;
        stiffnesses[J_R_SHOULDER_PITCH] = 0.8;
        stiffnesses[J_R_ELBOW_YAW] = 0.8;
        stiffnesses[J_R_ELBOW_ROLL] = 0.8;
    }
    else
    {
        thelog << "JWALK: braceForImpact() was called but the robot is not falling!" << endl;
    }
    
    JWalkActuators->setStiffnessAll(stiffnesses);
}

void JWalk::getUp()
{
    static unsigned char gettingupstate = 0;
    static int getupfinishtime = 0;
    
    if (gettingupstate == 0)
    {   // in the first state we issue the command to change to scripting mode
        checkAndRepair();
        if (JWalkMode != JWALK_TRANSITION)
        {
            #if JWALK_VERBOSITY > 1
                thelog << "JWALK: getUp() Starting at time: " << dcmTimeSinceStart << endl;
            #endif
            gettingupstate++;
            emergencyStop();
            if (JWalkMode != JWALK_SMODE)
            {
                changeModes(JWALK_SMODE);
                #if JWALK_VERBOSITY > 1
                    thelog << "JWALK: getUp() Issueing change of mode to script mode. Time: " << dcmTimeSinceStart << endl;
                #endif
            }
        }
    }
    else if (gettingupstate == 1)
    {   // in this mode wait until we are in scripting mode, then run the script
        if (JWalkMode == JWALK_SMODE)
        {
            if (balanceValues[B_ANGLE_Y] < -1.0)        // if on its back
            {
                #if JWALK_STANDALONE
                    getupfinishtime = getUpBack->play(false);
                #else
                    getupfinishtime = NAO->m_locomotion->getUpBack.play(false);
                #endif
            }
            else if (balanceValues[B_ANGLE_Y] > 1.0)    // if on its front
            {
                #if JWALK_STANDALONE
                    getupfinishtime = getUpFront->play(false);
                #else
                    getupfinishtime = NAO->m_locomotion->getUpFront.play(false);
                #endif
            }
            else if (inBackwardBracePose())
            {
                #if JWALK_STANDALONE
                    getupfinishtime = getUpBackFall->play(false);
                #else
                    getupfinishtime = NAO->m_locomotion->getUpBackFall.play(false);
                #endif
            }
            else if (fabs(balanceValues[B_ANGLE_X]) < 0.3 && fabs(balanceValues[B_ANGLE_Y]) < 0.3)
            {   // if torso is upright
                if (touchLeftFootOnGround == true || touchRightFootOnGround == true)
                {
                    float stiffnesses[ALIAS_TARGETS_ALL_LENGTH];
                    for (unsigned char i=0; i<ALIAS_TARGETS_ALL_LENGTH; i++)
                        stiffnesses[i] = 0.8;
                    JWalkActuators->setStiffnessAll(stiffnesses);
                    getupfinishtime = JWalkActuators->goToAnglesNotHead(JWalkCommonPositions, 2000);
                    setHeadYaw(0, 2000);
                    setHeadPitch(0, 2000);
                }
                else
                {
                    float stiffnesses[ALIAS_TARGETS_ALL_LENGTH];
                    for (unsigned char i=0; i<ALIAS_TARGETS_ALL_LENGTH; i++)
                        stiffnesses[i] = 0;
                    JWalkActuators->setStiffnessAll(stiffnesses);
                    gettingupstate--;
                }
            }
            else
            {
                float stiffnesses[ALIAS_TARGETS_ALL_LENGTH];
                for (unsigned char i=0; i<ALIAS_TARGETS_ALL_LENGTH; i++)
                    stiffnesses[i] = 0;
                JWalkActuators->setStiffnessAll(stiffnesses);
                gettingupstate--;
            }
            #if JWALK_VERBOSITY > 1
                thelog << "JWALK: getUp(): Motion started at: Time: " << dcmTimeSinceStart << " dcmTime: " << dcmTime << " " << getupfinishtime << endl;
            #endif
            gettingupstate++;
        }
    }
    else if (gettingupstate == 2)
    {   // in this mode wait until the script is completed
        if (dcmTime > getupfinishtime)
        {
            thelog << "JWALK: getUp(): Finished at: " << dcmTimeSinceStart << endl;
            float headstiffnesses[2] = {0.25, 0.25};
            setHeadStiffness(headstiffnesses);
            setHeadYaw(0, 100);
            setHeadPitch(0, 100);
            JWalkActuators->setStiffnessNotHead(JWalkCommonHardnesses);
            balanceFallen = false;
            gettingupstate = 0;
        }
    }
}

int JWalk::doScript(script scripttorun, bool scripturgent)
{
    static unsigned char scriptstate = 0;
    static int scriptfinishtime = 0;
    
    if (scriptstate == 0)
    {   // in the first state we issue the command to stop
        scriptfinishtime = dcmTime + 10000;     // At this point I don't know how long it will take to run the script. Guess 10 seconds
        if (JWalkMode != JWALK_TRANSITION)
        {
            #if JWALK_VERBOSITY > 1
                thelog << "JWALK: doScript() Issuing walkstop at time: " << dcmTimeSinceStart << " scriptfinishtime: " << scriptfinishtime << endl;
            #endif
            scriptstate++;
            if (scripturgent == true)
            {
                emergencyStop();
                if (JWalkMode != JWALK_SMODE)
                    changeModes(JWALK_SMODE);
                scriptstate++;
                #if JWALK_VERBOSITY > 1
                    thelog << "JWALK: doScript() Script is urgent issuing change of modes immediately. " << dcmTimeSinceStart << endl;
                #endif
            }
            else
                stop();
        }
    }
    else if (scriptstate == 1)
    {   // in this state we wait until the walk has finished, then issue the command to change to script mode
        if (!walkIsActive())
        {
            scriptfinishtime = dcmTime + 10000;     // At this point I don't know how long it will take to run the script. Guess 10 seconds
            if (JWalkMode != JWALK_SMODE)
                changeModes(JWALK_SMODE);
            scriptstate++;
            #if JWALK_VERBOSITY > 1
                thelog << "JWALK: doScript() Issueing change of mode to script mode. Time: " << dcmTimeSinceStart << endl;
            #endif
        }
    }
    else if (scriptstate == 2)
    {   // in this mode wait until we are in scripting mode, and then run the script
        if (JWalkMode == JWALK_SMODE)
        {
            balanceFallingEnabled = false;
            scriptfinishtime = scripttorun.play(false);
            thelog << "JWALK: doScript(): Playing the script at: " << dcmTimeSinceStart << " dcmTime: " << dcmTime << " scriptfinishtime: " << scriptfinishtime << endl;
            scriptstate++;
        }
    }
    else if (scriptstate == 3)
    {   // in this mode wait until the script is completed
        if (dcmTime > scriptfinishtime)
        {
            thelog << "JWALK: doScript(): Finished at: " << dcmTimeSinceStart << endl;
            float headstiffnesses[2] = {0.5, 0.5};
            setHeadStiffness(headstiffnesses);
            JWalkActuators->setStiffnessNotHead(JWalkCommonHardnesses);
            balanceFallingEnabled = true;
            scriptstate = 0;
        }
    }
    return scriptfinishtime;
}

void JWalk::enableFallingControl()
{
    balanceFallingEnabled = true;
    return;
}

void JWalk::disableFallingControl()
{
    balanceFallingEnabled = false;
    return;
}

void JWalk::checkAndRepair()
{
    if (checkHealth() == 3)
    {
        float stiffnesses[ALIAS_TARGETS_ALL_LENGTH];
        system("aplay /home/root/SoundStates/chestnotfound.wav");
        for (unsigned char i=0; i<ALIAS_TARGETS_ALL_LENGTH; i++)
            stiffnesses[i] = 0.0;
        usleep(100*(JWalkActuators->setStiffnessAll(stiffnesses, 100) - dcmTime));
        system("ssh root@localhost /etc/init.d/naoqi restart");
        std::exit(1);
    }
    else if (checkHealth() == 2)
    {
        float stiffnesses[ALIAS_TARGETS_ALL_LENGTH];
        system("aplay /home/root/SoundStates/broken.wav");
        for (unsigned char i=0; i<ALIAS_TARGETS_ALL_LENGTH; i++)
            stiffnesses[i] = 0.0;
        usleep(100*(JWalkActuators->setStiffnessAll(stiffnesses, 100) - dcmTime));
        JWalkActuators->resetMotorBoards();
        usleep(20*1e6);
        system("ssh root@localhost /etc/init.d/naoqi restart");
        std::exit(1);
    }
    else if (checkHealth() == 1)
        system("aplay /home/root/SoundStates/warning.wav");
    else
        system("aplay /home/root/SoundStates/ok.wav");
}

int JWalk::checkHealth()
{
    static int previousChestNack = 0;
    // From DCM Documentation: 97 to 111, 113 to 127, 209 to 211 are only warnings
    bool warning = false;
    thelog << "JWALK: checkHealth: Nack Chest:" << nackErrors[NK_CHEST] << " ";
    
    if (nackErrors[NK_CHEST] - previousChestNack > 50)
    {
        return 3;
    }
    for (unsigned char i=0; i<MB_NUM_BOARDS; i++)
    {
        thelog << boardErrors[i] << ", ";
        if (boardErrors[i] != 0)
        {
            if ((boardErrors[i] >= 97 && boardErrors[i] <= 111) || (boardErrors[i] >= 113 && boardErrors[i] <= 127) || (boardErrors[i] >= 209 && boardErrors[i] <= 211))
                warning = true;
            else
                return 2;
        }
    }
    if (warning == false)
        return 0;
    else
        return 1;
}

/********************************************************************************************************************************************************************
 'Mode' Control
 ********************************************************************************************************************************************************************/

void* runJWalkTransition(void* arg)
{
    #if JWALK_VERBOSITY > 1
        thelog << "JWALK: runJWalkTransition: Starting transition thread. Time: " << dcmTime << endl;
    #endif
    
    JWalk* jwalk = (JWalk*) arg;
    JWalkModeEnum PreviousMode = jwalk->JWalkPreviousMode;      
    JWalkModeEnum CurrentMode = jwalk->JWalkMode;
    JWalkModeEnum NextMode = jwalk->JWalkNextMode;
    
    // firstly stop the current walk (the walk is already stopped if it is no longer active)
    if (jwalk->walkIsActive() == true)
    {
        // stop the PREVIOUS mode
        switch (PreviousMode) 
        {
            case JWALK_JMODE:
                jwalk->nuWalkStop();
                break;
            case JWALK_GMODE:
                jwalk->gStop();
                break;
            case JWALK_ALMODE:
                jwalk->alWalkStop();
                break;
            default:
                break;
        }
        jwalk->waitUntilFinished();        // sleep until the walk has finished
    }
    
    if (NextMode != JWALK_SMODE && jwalk->inCommonPose() == false)
    {
        #if JWALK_VERBOSITY > 1
            thelog << "JWALK: not in CommonPose, so move there. Time: " << dcmTime << endl;
        #endif
        usleep(1000*(jwalk->getInCommonPose() - dcmTime));      // like all of my motion commands getInCommonPose returns the dcmTime the motion ends (not the number of milliseconds the motion lasts for!)
        #if JWALK_VERBOSITY > 1
            thelog << "JWALK: move to CommonPose complete. Time: " << dcmTime << endl;
        #endif
    }
    jwalk->JWalkMode = NextMode;
    jwalk->doPreviousWalkCall();
    
    #if JWALK_VERBOSITY > 1
        thelog << "JWALK: runTransitionThread completed at " << dcmTime << ". The walk mode is now " << jwalk->JWalkMode << endl;
    #endif
    pthread_exit(NULL);
}

/* Switch from JWalkMode to nextmode
 Preconditions: JWalkMode must be set to the current mode
 Input: nextmode, the new mode to switch to
 
 This is a non-blocking function
 
 Postconditions: The process is as follows:
        1. JWalkPreviousMode, JWalkMode and JWalkNextMode are updated to reflect that the engine is in transition
        2. JWalkMode changing from JWALK_transition to JWalkNextMode signals that the transition is complete
 */
void JWalk::changeModes(JWalkModeEnum nextmode)
{
    if (JWalkMode == JWALK_TRANSITION)          // bad things happen when you try to change modes before the previous one has completed!
        return;     
#if JWALK_VERBOSITY > 1
    thelog << "JWALK: changeModes from " << JWalkMode << " to " << nextmode << endl;
#endif
    if (usingDCM() && modeUsesDCM(nextmode))
    {
        JWalkPreviousMode = JWalkMode;
        JWalkMode = nextmode;
        #if JWALK_VERBOSITY > 1
            thelog << "JWALK: Turbo change modes; both use DCM mode is now: " << JWalkMode << endl;
        #endif
        doPreviousWalkCall();
        return;
    }
    else if (!usingDCM() && !modeUsesDCM(nextmode))
    {
        JWalkPreviousMode = JWalkMode;
        JWalkMode = nextmode;
        #if JWALK_VERBOSITY > 1
            thelog << "JWALK: Turbo change modes; both use alMotion mode is now: " << JWalkMode << endl;
        #endif
        doPreviousWalkCall();
        return;
    }
    else
    {
        JWalkPreviousMode = JWalkMode;
        JWalkMode = JWALK_TRANSITION;
        JWalkNextMode = nextmode;
        
        pthread_t transitionthread;
        int err = pthread_create(&transitionthread, NULL, runJWalkTransition, (void*) this);          // this thread will set JWalkMode to JWalkNextMode when it has concluded
        if (err > 0)
        {
            thelog << "JWALK: ********************* Failed to create transitionthread *********************" << endl;
            thelog << "The error code was: " << err << endl;
        }
    }
}

/* Moves the robot into the 'common pose'; a pose that is shared by all modes. Returns the dcmTime the motion will finish 
 Preconditions: JWalkPreviousMode must be set to the mode before the transition occured (it is this mode that will be used to get into the common pose)
 Postconditions: The robot will start moving to the initial pose
 */
int JWalk::getInCommonPose()
{
    #if JWALK_VERBOSITY > 1
        thelog << "JWALK: getInCommonPose()" << endl;
    #endif
    
    JWalkActuators->setStiffnessNotHead(JWalkCommonHardnesses);
    switch (JWalkPreviousMode) 
    {
        case JWALK_JMODE:
            return JWalkActuators->goToAnglesWithVelocityNotHead(JWalkCommonPositions, 1.5);
            break;
        case JWALK_GMODE:
            return alWalk->goToAnglesWithVelocityNotHead(JWalkCommonPositions, 1.5);
            break;
        case JWALK_ALMODE:
            return alWalk->goToAnglesWithVelocityNotHead(JWalkCommonPositions, 1.5);
            break;
        case JWALK_SMODE:
            return JWalkActuators->goToAnglesWithVelocityNotHead(JWalkCommonPositions, 1.5);
            break;
        default:
            break;
    }
}

/* Returns true if we are already in the common pose, false if a movement is required
 */
bool JWalk::inCommonPose()
{
    #if JWALK_VERBOSITY > 1
        thelog << "JWALK: inCommonPose()" << endl;
    #endif
    for (unsigned char i=JOINT_OFFSET_LLEG; i<ALIAS_TARGETS_NOT_HEAD_LENGTH; i++)
    {
        if (fabs(jointPositions[i+2] - JWalkCommonPositions[i]) > 0.06)
            return false;
    }
    return true;
}

/* Calls the function that triggered the change in modes
 */
void JWalk::doPreviousWalkCall()
{
    // I know this is a clumsy way to do this, but I didn't see this coming
    // I need to do the command that caused the transition; I save the ID of the calling function, and parameters
    
    if (JWalkMode == JWALK_SMODE)       // if we have changed into SMODE there was no walk call that triggered the transistion
        return;
    
    switch (JWalkFunctionID) 
    {
        case ID_NUWALKONBEARING:
            //jwalk->nuWalkOnBearing(jwalk->JWalkParam1, (unsigned char) jwalk->JWalkParam2);
            break;
        case ID_NUWALKTOPOINT:
            //jwalk->nuWalkToPoint(jwalk->JWalkParam1, jwalk->JWalkParam2);
            break;    
        case ID_NUWALKTOPOINTWITHORIENTATION:
            //jwalk->nuWalkToPointWithOrientation(jwalk->JWalkParam1, jwalk->JWalkParam2, jwalk->JWalkParam3);
            break;   
        case ID_NUWALKTOPOINTWITHMAINTAINORIENTATION:
            //jwalk->nuWalkToPointWithOrientation(jwalk->JWalkParam1, jwalk->JWalkParam2, jwalk->JWalkParam3);
            break;
        case ID_GFORWARD:
            gForward();
            break;   
        case ID_GBACKWARD:
            gBackward();
            break;   
        case ID_GARC:
            gArc(JWalkParam1);
            break;  
        case ID_GLEFT:
            gLeft();
            break;  
        case ID_GRIGHT:
            gRight();
            break;
        case ID_GTURNLEFT:
            gTurnLeft();
            break;
        case ID_GTURNRIGHT:
            gTurnRight();
            break;
        case ID_ALSTRAIGHT:
            alWalkStraight(JWalkParam1);
            break;
        case ID_ALARC:
            alWalkArc(JWalkParam1, JWalkParam2);
            break;
        case ID_ALSIDEWAYS:
            alWalkSideways(JWalkParam1);
            break;
        case ID_ALTURN:
            alWalkTurn(JWalkParam1);
            break;
        default:
            break;
    }
}

/* Returns true if we are in the backwards brace pose
 */
bool JWalk::inBackwardBracePose()
{
    if (fabs(jointPositions[J_R_SHOULDER_PITCH] - 2.1) > 0.4)
        return false;
    else if (fabs(jointPositions[J_R_SHOULDER_PITCH] - 2.1) > 0.4)
        return false;
    else if (fabs(jointPositions[J_R_HIP_PITCH] + 1.74) > 0.2)
        return false;
    else if (fabs(jointPositions[J_L_HIP_PITCH] + 1.74) > 0.2)
        return false;
    else if (fabs(jointPositions[J_L_HIP_YAWPITCH] + 1.0) > 0.2)
        return false;
    else if (fabs(balanceValues[B_ANGLE_Y]) > 1.0)
        return false;
    else
        return true;
}

/********************************************************************************************************************************************************************
 NUWalk Interface
 ********************************************************************************************************************************************************************/

void JWalk::nuWalkOnBearing(float bearing, unsigned char speed, bool dodge)
{
    if (touchOnGround == false || balanceFallen == true || balanceFalling == true)
        return;
    
    #if JWALK_VERBOSITY > 1
        thelog << "JWALK: nuWalkOnBearing()" << endl;
    #endif
    
    if (JWalkMode == JWALK_TRANSITION)
    {
        return;
    }
    else if (JWalkMode == JWALK_JMODE)
    {
        walkCyclesSinceCall = 0;
        nuWalk->onBearing(bearing, dodge);
        
        #if JWALK_VERBOSITY > 1
            thelog << "JWALK: nuWalkOnBearing(): " << bearing << endl;
        #endif
    }
    else
    {
        JWalkFunctionID = ID_NUWALKONBEARING;
        JWalkParam1 = bearing;
        JWalkParam2 = speed;
        changeModes(JWALK_JMODE);
        return;
    }
}

void JWalk::nuWalkToPoint(float distance, float bearing, bool dodge)
{
    if (touchOnGround == false || balanceFallen == true || balanceFalling == true)
        return;
    
    if (JWalkMode == JWALK_TRANSITION)
    {
        return;
    }
    else if (JWalkMode == JWALK_JMODE)
    {
        walkCyclesSinceCall = 0;
        nuWalk->toPoint(distance, bearing, dodge);
        
        #if JWALK_VERBOSITY > 1
            thelog << "JWALK: nuWalkToPoint(): " << distance << " " << bearing << endl;
        #endif
    }
    else
    {
        JWalkFunctionID = ID_NUWALKTOPOINT;
        JWalkParam1 = distance;
        JWalkParam2 = bearing;
        changeModes(JWALK_JMODE);
        return;
    }
}

void JWalk::nuWalkToPointWithOrientation(float distance, float bearing, float finalorientation, bool dodge)
{
    if (touchOnGround == false || balanceFallen == true || balanceFalling == true)
        return;
    
    if (JWalkMode == JWALK_TRANSITION)
    {
        return;
    }
    else if (JWalkMode == JWALK_JMODE)
    {
        walkCyclesSinceCall = 0;
        nuWalk->toPointWithOrientation(distance, bearing, finalorientation, dodge);
        
        #if JWALK_VERBOSITY > 1
            thelog << "JWALK: nuWalkToPointWithOrientation(): " << distance << " " << bearing << " " << finalorientation << endl;
        #endif
    }
    else
    {
        JWalkFunctionID = ID_NUWALKTOPOINTWITHORIENTATION;
        JWalkParam1 = distance;
        JWalkParam2 = bearing;
        JWalkParam3 = finalorientation;
        changeModes(JWALK_JMODE);
        return;
    }
}

void JWalk::nuWalkToPointWithMaintainOrientation(float distance, float bearing, float desiredorientation, bool dodge)
{
    if (touchOnGround == false || balanceFallen == true || balanceFalling == true)
        return;
    
    if (JWalkMode == JWALK_TRANSITION)
    {
        return;
    }
    else if (JWalkMode == JWALK_JMODE)
    {
        walkCyclesSinceCall = 0;
        nuWalk->toPointWithMaintainOrientation(distance, bearing, desiredorientation, dodge);
        
        #if JWALK_VERBOSITY > 1
                thelog << "JWALK: nuWalkToPointWithMaintainOrientation(): " << distance << " " << bearing << " " << desiredorientation << endl;
        #endif
    }
    else
    {
        JWalkFunctionID = ID_NUWALKTOPOINTWITHMAINTAINORIENTATION;
        JWalkParam1 = distance;
        JWalkParam2 = bearing;
        JWalkParam3 = desiredorientation;
        changeModes(JWALK_JMODE);
        return;
    }
}

bool JWalk::nuWalkIsActive()
{
    if (balanceFalling == true || balanceFallen == true)
        return false;
    return nuWalk->walkIsActive();
}

void JWalk::nuWalkStop()
{
#if JWALK_VERBOSITY > 1
    thelog << "JWALK: nuWalkStop()" << endl;
#endif
    nuWalk->stop();
}

/********************************************************************************************************************************************************************
 gWalk Interface
 
 This interface is sort of designed to implement scripted paths, where a component of the path is completed in a 'closed-loop' sense. 
 For example, a dodge maneuver would call gLeft() and then later when the obstacle has passed call stop. Or maybe, the search for the ball would just be gTurnLeft while 
 moving the head, when the ball is found just call stop() or nuWalkToPoint()
    
 This interface is implemented using alMotion, so it has all of the same problems as the alWalk interface
 ********************************************************************************************************************************************************************/

/*! Walk forward until another direction is called, or the walk is stopped (whether explicitly or implicitly)
 */
void JWalk::gForward()
{
    if (touchOnGround == false || balanceFallen == true || balanceFalling == true)
        return;
    
#if JWALK_VERBOSITY > 1
    thelog << "JWALK; gForward()." << endl;
#endif
    
    if (JWalkMode == JWALK_TRANSITION)
        return;
    else if (JWalkMode == JWALK_GMODE)
    {
        walkCyclesSinceCall = 0;
        nuWalk->goForward();
    }
    else
    {
        JWalkFunctionID = ID_GFORWARD;
        changeModes(JWALK_GMODE);
    }
}

/*! Walk backward until another direction is called, or the walk is stopped (whether explicitly or implicitly)
 */
void JWalk::gBackward()
{
    if (touchOnGround == false || balanceFallen == true || balanceFalling == true)
        return;
    
#if JWALK_VERBOSITY > 1
    thelog << "JWALK; gBackward()." << endl;
#endif
    
    if (JWalkMode == JWALK_TRANSITION)
        return;
    else if (JWalkMode == JWALK_GMODE)
    {
        walkCyclesSinceCall = 0;
        nuWalk->goBackward();
    }
    else
    {
        JWalkFunctionID = ID_GBACKWARD;
        changeModes(JWALK_GMODE);
    }
}

/*! Walk along an arc in the specified direction until the another direction is called, or the walk is stopped (whether explicitly or implicitly)
 */
void JWalk::gArc(float direction)
{
    if (touchOnGround == false || balanceFallen == true || balanceFalling == true)
        return;
    
#if JWALK_VERBOSITY > 1
    thelog << "JWALK; gArc(" << direction << ")." << endl;
#endif
    
    if (JWalkMode == JWALK_TRANSITION)
        return;
    else if (JWalkMode == JWALK_GMODE)
    {
        walkCyclesSinceCall = 0;
        //nuWalk->goArc(direction);
    }
    else
    {
        JWalkFunctionID = ID_GARC;
        changeModes(JWALK_GMODE);
    }
}

/*! Walk left (sideways) until the another direction is called, or the walk is stopped (whether explicitly or implicitly)
 */
void JWalk::gLeft()
{
    if (touchOnGround == false || balanceFallen == true || balanceFalling == true)
        return;
    
#if JWALK_VERBOSITY > 1
    thelog << "JWALK; gLeft()." << endl;
#endif
    
    if (JWalkMode == JWALK_TRANSITION)
        return;
    else if (JWalkMode == JWALK_GMODE)
    {
        walkCyclesSinceCall = 0;
        nuWalk->goLeft();
    }
    else
    {
        JWalkFunctionID = ID_GLEFT;
        changeModes(JWALK_GMODE);
    }
}

/*! Walk right (sideways) until the another direction is called, or the walk is stopped (whether explicitly or implicitly)
 */
void JWalk::gRight()
{
    if (touchOnGround == false || balanceFallen == true || balanceFalling == true)
        return;
    
#if JWALK_VERBOSITY > 1
    thelog << "JWALK; gRight()." << endl;
#endif
    
    if (JWalkMode == JWALK_TRANSITION)
        return;
    else if (JWalkMode == JWALK_GMODE)
    {
        walkCyclesSinceCall = 0;
        nuWalk->goRight();
    }
    else
    {
        JWalkFunctionID = ID_GRIGHT;
        changeModes(JWALK_GMODE);
    }
}

/*! Turn left (rotate anticlockwise) on the spot until another direction is called, or the walk is stopped (whether explicitly or implicitly)
 
 This turn is significantly slower (ie the turn velocity is smaller ;)) than other turns.
 */
void JWalk::gTurnLeft()
{
    if (touchOnGround == false || balanceFallen == true || balanceFalling == true)
        return;
    
#if JWALK_VERBOSITY > 1
    thelog << "JWALK; gTurnLeft()." << endl;
#endif
    
    if (JWalkMode == JWALK_TRANSITION)
        return;
    else if (JWalkMode == JWALK_GMODE)
    {
        walkCyclesSinceCall = 0;
        nuWalk->goTurnLeft();
    }
    else
    {
        JWalkFunctionID = ID_GTURNLEFT;
        changeModes(JWALK_GMODE);
    }
}

/*! Turn right (rotate clockwise) on the spot until another direction is called, or the walk is stopped (whether explicitly or implicitly)
 
 This turn is significantly slower (ie the turn velocity is smaller ;)) than other turns.
 */
void JWalk::gTurnRight()
{
    if (touchOnGround == false || balanceFallen == true || balanceFalling == true)
        return;
    
#if JWALK_VERBOSITY > 1
    thelog << "JWALK; gTurnRight()." << endl;
#endif
    
    if (JWalkMode == JWALK_TRANSITION)
        return;
    else if (JWalkMode == JWALK_GMODE)
    {
        walkCyclesSinceCall = 0;
        nuWalk->goTurnRight();
    }
    else
    {
        JWalkFunctionID = ID_GTURNRIGHT;
        changeModes(JWALK_GMODE);
    }
}

/*! Stops the gWalk
 */
void JWalk::gStop()
{
    if (balanceFallen == true || balanceFalling == true)
        return;
#if JWALK_VERBOSITY > 2
    thelog << "JWALK; gStop()." << endl;
#endif
    nuWalk->stop();
}

/*! Returns true if there is an active gWalk task. Returns false otherwise
 */
bool JWalk::gIsActive()
{
#if JWALK_VERBOSITY > 2
    thelog << "JWALK: gIsActive()" << endl;
#endif

    return nuWalk->walkIsActive();
}

/********************************************************************************************************************************************************************
 alWalk Interface
 
 A wrapper for almotion's walk. All units are now in cm, the walk parameters are configured for each walk primitive automatically, commanded distances and angles have
 been calibrated, the hardness is also changed (based on 'some' calculations) every dcm cycle.
 
 As this interface *uses* almotion this interface has several limitations:
     - all calls to walkStraight, walkArc, walkSideways and turn are ignored if the robot is already walking
     This is because I can not 'queue' changes of walk parameters.
     Instead, if a change of direction is required call stop, and then issue the new command
     Or, wait until the current walk task has completed before telling it to start walking in a different direction
     
     - walkIsActive and waitUntilFinished do not always return true/block at the very begining of a walk task --- there is a delay between you telling the robot to
     walk, and the walk being 'active'
     
     - avoid waitUntilFinished; it is not an event based function, it polls walkIsActive 20 times a second! Can't be fixed
 ********************************************************************************************************************************************************************/

/*! Walk straight (forwards or backwards) a distance in cm
 @param distance        the distance to walk in cm (forwards is positive)
 
 Note. Nothing happens if the robot is already walking 
 */
void JWalk::alWalkStraight(float distance)
{
    if (touchOnGround == false || balanceFallen == true || balanceFalling == true)
        return;
    
    if (JWalkMode == JWALK_TRANSITION)
        return;
    else if (JWalkMode == JWALK_ALMODE)
    {
        #if JWALK_VERBOSITY > 1
            thelog << "JWALK: alWalkStraight(" << distance << ")." << endl;
        #endif
        walkCyclesSinceCall = 0;
        alWalk->walkStraight(distance);
    }
    else
    {
        JWalkFunctionID = ID_ALSTRAIGHT;
        JWalkParam1 = distance;
        changeModes(JWALK_ALMODE);
    }
}

/*! Walk along a arc of radius (cm) for an angle (radians)
 @param angle           the angle of the arc (radians)
 @param radius          the radius of the circle the arc is on (cm)
 
 Note. Nothing happens if the robot is already walking
 */
void JWalk::alWalkArc(float angle, float radius)
{
    if (touchOnGround == false || balanceFallen == true || balanceFalling == true)
        return;
    
    if (JWalkMode == JWALK_TRANSITION)
        return;
    else if (JWalkMode == JWALK_ALMODE)
    {
        #if JWALK_VERBOSITY > 1
            thelog << "JWALK: alWalkArc(" << angle << ", " << radius << ")." << endl;
        #endif
        walkCyclesSinceCall = 0;
        alWalk->walkArc(angle, radius);
    }
    else
    {
        JWalkFunctionID = ID_ALARC;
        JWalkParam1 = angle;
        JWalkParam2 = radius;
        changeModes(JWALK_ALMODE);
    }
}

/*! Walk sideways 'distance' in centimetres
 @param distance        the distance to walk in cm (left is positive)
 
 Note. Nothing happens if the robot is already walking
 */
void JWalk::alWalkSideways(float distance)
{
    if (touchOnGround == false || balanceFallen == true || balanceFalling == true)
        return;
    
    if (JWalkMode == JWALK_TRANSITION)
        return;
    else if (JWalkMode == JWALK_ALMODE)
    {
        #if JWALK_VERBOSITY > 1
            thelog << "JWALK: alWalkSideways(" << distance << ")." << endl;
        #endif
        walkCyclesSinceCall = 0;
        alWalk->walkSideways(distance);
    }
    else
    {
        JWalkFunctionID = ID_ALSIDEWAYS;
        JWalkParam1 = distance;
        changeModes(JWALK_ALMODE);
    }
}

/*! Turn 'angle' radians on the spot
 @param angle       the angle to turn in radians (left/anticlockwise is postive)
 
 Note. Nothing happens if the robot is already walking
 */
void JWalk::alWalkTurn(float angle)
{
    if (touchOnGround == false || balanceFallen == true || balanceFalling == true)
        return;
    
    if (JWalkMode == JWALK_TRANSITION)
        return;
    else if (JWalkMode == JWALK_ALMODE)
    {
        #if JWALK_VERBOSITY > 1
            thelog << "JWALK: alWalkTurn(" << angle << ")." << endl;
        #endif
        walkCyclesSinceCall = 0;
        alWalk->turn(angle);
    }
    else
    {
        JWalkFunctionID = ID_ALTURN;
        JWalkParam1 = angle;
        changeModes(JWALK_ALMODE);
    }
}

/*! Returns true if there is an active alWalk task. Returns false otherwise
 
 Note. false does not mean the robot is stationary; it may be jwalking
 And there is a delay between the issuing of a walk task, and 'walk being active'
 */
bool JWalk::alWalkIsActive()
{
    #if JWALK_VERBOSITY > 2
        thelog << "JWALK: alWalkIsActive()" << endl;
    #endif
    
    return alWalk->walkIsActive();
}

/*! Blocks until along of the pending walk tasks have been completed.
 This could take some time if there are many steps in the queue
 
 The implementation of this function isn't pretty; it will poll walkIsActive 20 times a second until it is false :(
 */
void JWalk::alWalkWaitUntilFinished()
{
    #if JWALK_VERBOSITY > 2
        thelog << "JWALK: alWalkWaitUntilFinished()" << endl;
    #endif
    alWalk->waitUntilFinished();
}

/*! Safely stops the current walk as soon as possible (it will finish its current step, and then take a stopping step to bring the robot to rest)
 
 This is not a blocking function, ie. when it returns the robot will still be walking
 */
void JWalk::alWalkStop()
{
    if (balanceFallen == true || balanceFalling == true)
        return;
    #if JWALK_VERBOSITY > 2
        thelog << "JWALK: alWalkStop()" << endl;
    #endif
    alWalk->stop();
}


/*! Returns the module's version
 This method is inherited from ALModule, and must be implemented
 */
string JWalk::version()
{
    return "lolwut";
}

/*! Returns the true if the test succeeds, false if it fails
 This method is inherited from ALModule, and must be implemented
 */
bool JWalk::innerTest() 
{
  bool result = true;
  // put here code dedicaced to autotest this module.
  // return false if fail, success otherwise
  return result;
}


