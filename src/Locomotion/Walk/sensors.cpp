/** Sensor access functions and global variables
      - global store of arrays for feedback data
            * position, velocity, acceleration
            * current
            * stiffnesses and targets
            * temperatures
            * balance (accelerometers and gyros)
            * touch (all buttons + FSR)
            * battery (current, temp and voltage)
      - ALMemoryFastAccess is connected to the feedback data, and new data is copied to the global store
        
      - Feedback datalogging is also provided by this file
 * @author Jason Kulk
 * Version : $Id $
 
 * NUbots (c) 2008 All Rights Reserved
 */

#include "sensors.h"
#include "../../Kinematics/Kinematics.h"

// The current DCM time (ms) since object creation
int dcmTime = 0;
int dcmTimeSinceStart = 0;

// Proprioception Feedback Data. Velocities and Accelerations are measured by soft sensors
float jointPositions[J_NUM_JOINTS];
float jointVelocities[J_NUM_JOINTS];
float jointVelocitySum;
float jointAccelerations[J_NUM_JOINTS];
float jointCurrents[J_NUM_JOINTS];
float jointCurrentSum;
float jointTargets[J_NUM_JOINTS];
float jointHardnesses[J_NUM_JOINTS];
int boardErrors[MB_NUM_BOARDS];
int nackErrors[NK_NUM_BOARDS];
unsigned char walkCyclesSinceCall;   // I need this so that walkIsActive is true immediately after a call to walk!
bool walkAmIWalking;
bool walkPreviousAmIWalking;         // I need this one to detect when the robot stops

float odometryDeltaX;            // x odometry change in cm
float odometryDeltaY;            // y odometry change in cm
float odometryDeltaO;            // orientation change in cm

// Thermoception Feedback Data.
float jointTemperatures[J_NUM_JOINTS];

// Balance Feedback Data. Accelerometers and Gyros
float balanceValues[B_NUM_SENSORS];
bool balanceFallingEnabled;
bool balanceFalling;
bool balancePreviousFalling;
bool balanceFallingForward;
bool balanceFallingBackward;
bool balanceFallingLeft;
bool balanceFallingRight;
bool balanceFallen;
bool balancePreviousFallen;
int balanceFallenCount;

// Touch Feedback Data. Foot Pressure sensors, foot bump sensors, and chest buttons
float touchValues[T_NUM_SENSORS];
float touchLeftCoPX;
float touchLeftCoPY;
float touchRightCoPX;
float touchRightCoPY;
float touchPSValues[PS_NUM_SENSORS];
bool touchOnGround;
bool touchPreviousOnGround;
bool touchLeftFootOnGround;
bool touchRightFootOnGround;
SupportModeEnum leftSupportMode;
SupportModeEnum rightSupportMode;
string indexToSupportMode[SM_NUM_MODES] = {string("Stance"), string("Push"), string("Swing"), string("Impact")};

// Collision Feedback
bool collisionAny;
bool collisionLeftArm;
bool collisionLeftArmFront;
bool collisionLeftArmSide;
bool collisionLeftArmBack;
bool collisionRightArm;
bool collisionRightArmFront;
bool collisionRightArmSide;
bool collisionRightArmBack;
bool collisionLeftFoot;
bool collisionLeftFootFront;
bool collisionLeftFootBack;
bool collisionRightFoot;
bool collisionRightFootFront;
bool collisionRightFootBack;

// Battery Feedback Data. 
float batteryValues[E_NUM_SENSORS];

// Distance Feedback Data
float distanceValues[D_NUM_SENSORS];                // caution: these values are only to be used by me, use the sonic and infrared
float sonicValues[S_NUM_SENSORS];
float sonicObstacleDistance;

// Index to AL string device names to be used with the stm
string indexToName[] = {string("HeadYaw"), string("HeadPitch"), string("LShoulderRoll"), string("LShoulderPitch"), string("LElbowYaw"), string("LElbowRoll"), string("RShoulderRoll"), string("RShoulderPitch"), string("RElbowYaw"), string("RElbowRoll"), string("LHipYawPitch"), string("LHipRoll"), string("LHipPitch"), string("LKneePitch"), string("LAnklePitch"), string("LAnkleRoll"), string("RHipYawPitch"), string("RHipRoll"), string("RHipPitch"), string("RKneePitch"), string("RAnklePitch"), string("RAnkleRoll")};
string indexToPositionSensor[] = {DN_HEAD_YAW_POSITION, DN_HEAD_PITCH_POSITION, DN_L_SHOULDER_ROLL_POSITION, DN_L_SHOULDER_PITCH_POSITION, DN_L_ELBOW_YAW_POSITION, DN_L_ELBOW_ROLL_POSITION, DN_R_SHOULDER_ROLL_POSITION, DN_R_SHOULDER_PITCH_POSITION, DN_R_ELBOW_YAW_POSITION, DN_R_ELBOW_ROLL_POSITION, DN_L_HIP_YAWPITCH_POSITION, DN_L_HIP_ROLL_POSITION, DN_L_HIP_PITCH_POSITION, DN_L_KNEE_PITCH_POSITION, DN_L_ANKLE_PITCH_POSITION, DN_L_ANKLE_ROLL_POSITION, DN_R_HIP_YAWPITCH_POSITION, DN_R_HIP_ROLL_POSITION, DN_R_HIP_PITCH_POSITION, DN_R_KNEE_PITCH_POSITION, DN_R_ANKLE_PITCH_POSITION, DN_R_ANKLE_ROLL_POSITION};
string indexToCurrentSensor[] = {DN_HEAD_YAW_CURRENT, DN_HEAD_PITCH_CURRENT, DN_L_SHOULDER_ROLL_CURRENT, DN_L_SHOULDER_PITCH_CURRENT, DN_L_ELBOW_YAW_CURRENT, DN_L_ELBOW_ROLL_CURRENT, DN_R_SHOULDER_ROLL_CURRENT, DN_R_SHOULDER_PITCH_CURRENT, DN_R_ELBOW_YAW_CURRENT, DN_R_ELBOW_ROLL_CURRENT, DN_L_HIP_YAWPITCH_CURRENT, DN_L_HIP_ROLL_CURRENT, DN_L_HIP_PITCH_CURRENT, DN_L_KNEE_PITCH_CURRENT, DN_L_ANKLE_PITCH_CURRENT, DN_L_ANKLE_ROLL_CURRENT, DN_R_HIP_YAWPITCH_CURRENT, DN_R_HIP_ROLL_CURRENT, DN_R_HIP_PITCH_CURRENT, DN_R_KNEE_PITCH_CURRENT, DN_R_ANKLE_PITCH_CURRENT, DN_R_ANKLE_ROLL_CURRENT};
string indexToTargetSensor[] = {DN_HEAD_YAW_TARGET, DN_HEAD_PITCH_TARGET, DN_L_SHOULDER_ROLL_TARGET, DN_L_SHOULDER_PITCH_TARGET, DN_L_ELBOW_YAW_TARGET, DN_L_ELBOW_ROLL_TARGET, DN_R_SHOULDER_ROLL_TARGET, DN_R_SHOULDER_PITCH_TARGET, DN_R_ELBOW_YAW_TARGET, DN_R_ELBOW_ROLL_TARGET, DN_L_HIP_YAWPITCH_TARGET, DN_L_HIP_ROLL_TARGET, DN_L_HIP_PITCH_TARGET, DN_L_KNEE_PITCH_TARGET, DN_L_ANKLE_PITCH_TARGET, DN_L_ANKLE_ROLL_TARGET, DN_R_HIP_YAWPITCH_TARGET, DN_R_HIP_ROLL_TARGET, DN_R_HIP_PITCH_TARGET, DN_R_KNEE_PITCH_TARGET, DN_R_ANKLE_PITCH_TARGET, DN_R_ANKLE_ROLL_TARGET};
string indexToHardnessSensor[] = {DN_HEAD_YAW_HARDNESS, DN_HEAD_PITCH_HARDNESS, DN_L_SHOULDER_ROLL_HARDNESS, DN_L_SHOULDER_PITCH_HARDNESS, DN_L_ELBOW_YAW_HARDNESS, DN_L_ELBOW_ROLL_HARDNESS, DN_R_SHOULDER_ROLL_HARDNESS, DN_R_SHOULDER_PITCH_HARDNESS, DN_R_ELBOW_YAW_HARDNESS, DN_R_ELBOW_ROLL_HARDNESS, DN_L_HIP_YAWPITCH_HARDNESS, DN_L_HIP_ROLL_HARDNESS, DN_L_HIP_PITCH_HARDNESS, DN_L_KNEE_PITCH_HARDNESS, DN_L_ANKLE_PITCH_HARDNESS, DN_L_ANKLE_ROLL_HARDNESS, DN_R_HIP_YAWPITCH_HARDNESS, DN_R_HIP_ROLL_HARDNESS, DN_R_HIP_PITCH_HARDNESS, DN_R_KNEE_PITCH_HARDNESS, DN_R_ANKLE_PITCH_HARDNESS, DN_R_ANKLE_ROLL_HARDNESS};
string indexToTemperatureSensor[] = {DN_HEAD_YAW_TEMPERATURE, DN_HEAD_PITCH_TEMPERATURE, DN_L_SHOULDER_ROLL_TEMPERATURE, DN_L_SHOULDER_PITCH_TEMPERATURE, DN_L_ELBOW_YAW_TEMPERATURE, DN_L_ELBOW_ROLL_TEMPERATURE, DN_R_SHOULDER_ROLL_TEMPERATURE, DN_R_SHOULDER_PITCH_TEMPERATURE, DN_R_ELBOW_YAW_TEMPERATURE, DN_R_ELBOW_ROLL_TEMPERATURE, DN_L_HIP_YAWPITCH_TEMPERATURE, DN_L_HIP_ROLL_TEMPERATURE, DN_L_HIP_PITCH_TEMPERATURE, DN_L_KNEE_PITCH_TEMPERATURE, DN_L_ANKLE_PITCH_TEMPERATURE, DN_L_ANKLE_ROLL_TEMPERATURE, DN_R_HIP_YAWPITCH_TEMPERATURE, DN_R_HIP_ROLL_TEMPERATURE, DN_R_HIP_PITCH_TEMPERATURE, DN_R_KNEE_PITCH_TEMPERATURE, DN_R_ANKLE_PITCH_TEMPERATURE, DN_R_ANKLE_ROLL_TEMPERATURE};
string indexToBoardError[] = {DN_MB_CHEST, DN_MB_HEAD, DN_MB_R_SHOULDER, DN_MB_R_ARM, DN_MB_R_HAND, DN_MB_R_HIP, DN_MB_R_THIGH, DN_MB_R_SHIN, DN_MB_R_FOOT, DN_MB_L_SHOULDER, DN_MB_L_ARM, DN_MB_L_HAND, DN_MB_L_HIP, DN_MB_L_THIGH, DN_MB_L_SHIN, DN_MB_L_FOOT, DN_MB_US, DN_MB_INERTIAL, DN_MB_TOUCH, DN_MB_FACE, DN_MB_EAR};
string indexToNack[] = {DN_NK_CHEST};
string indexToBalanceSensor[] = {DN_ACCEL_X, DN_ACCEL_Y, DN_ACCEL_Z, DN_ANGLE_X, DN_ANGLE_Y, DN_GYRO_X, DN_GYRO_Y};
string indexToTouchSensor[] = {DN_L_FSR_FL, DN_L_FSR_FR, DN_L_FSR_BL, DN_L_FSR_BR, DN_L_BUMP_L, DN_L_BUMP_R, DN_R_FSR_FL, DN_R_FSR_FR, DN_R_FSR_BL, DN_R_FSR_BR, DN_R_BUMP_L, DN_R_BUMP_R, DN_CHEST_BUTTON};
string indexToBatterySensor[] = {DN_CHARGE, DN_CURRENT, DN_VOLTAGE_MIN, DN_VOLTAGE_MAX, DN_TEMPERATURE};
string indexToDistanceSensor[] = {DN_US_DISTANCE};

/**************************************************************************************
Sensor Class Function Definitions
**************************************************************************************/
// Sensor Class Constructor
Sensors::Sensors(AL::ALPtr<AL::ALBroker> pBroker)
{
#if SENSOR_VERBOSITY > 0
    thelog << "SENSORS: Constructing sensors" << endl;
#endif
    historyvelocityindex = 0;
    historybalanceindex = 0;
#if SENSOR_VERBOSITY > 0
    thelog << "SENSORS: Initialising Time." << endl;
#endif
    dcmTime = (int)alDcm->call<int>("getTime",0);
    dcmStartTime = dcmTime;
    previousDcmTime = dcmStartTime;
    
#if SENSOR_VERBOSITY > 0
    thelog << "SENSORS: Initialising feedback data" << endl;
#endif
    // Initialise sensory feedback arrays to be zero
    for (int i = 0; i < J_NUM_JOINTS; i++)
    {
        jointPositions[i] = 0;
        jointVelocities[i] = 0;
        jointAccelerations[i] = 0;
        previousJointPositions[i] = 0;
        previousJointVelocities[i] = 0;
        previousJointCurrents[i] = 0;
        for (int j = 0; j < VEL_WINDOW_SIZE; j++)
        {
            historyJointVelocities[i][j] = 0;
        }
    }
    walkAmIWalking = false;
    walkPreviousAmIWalking = false;
    
    odometryDeltaX = 0;
    odometryDeltaY = 0;
    odometryDeltaO = 0;
    
    // Initialise balance values (because they need to be filtered)
    for (int i=0; i < B_NUM_SENSORS; i++)
    {
        balanceValues[i] = 0;
        for (int j=0; j < B_WINDOW_SIZE; j++)
        {
            historyBalanceValues[i][j] = 0;
        }
    }
    balanceFallingEnabled = false;
    balanceFalling = false;
    balancePreviousFalling = false;
    balanceFallingForward = false;
    balanceFallingBackward = false;
    balanceFallingLeft = false;
    balanceFallingRight = false;
    balanceFallen = false;
    balancePreviousFallen = false;
    balanceFallenCount = 0;
    
    touchOnGround = false;
    touchPreviousOnGround = false;
    touchLeftFootOnGround = false;
    touchRightFootOnGround = false;
    leftSupportMode = SM_STANCE;
    rightSupportMode = SM_STANCE;
    
    collisionAny = false;
    collisionLeftArm = false;
    collisionLeftArmFront = false;
    collisionLeftArmSide = false;
    collisionLeftArmBack = false;
    collisionRightArm = false;
    collisionRightArmFront = false;
    collisionRightArmSide = false;
    collisionRightArmBack = false;
    collisionLeftFoot = false;
    collisionLeftFootFront = false;
    collisionLeftFootBack = false;
    collisionRightFoot = false;
    collisionRightFootFront = false;
    collisionRightFootBack = false;
    
    // Initialise sonic values (because they need to be filtered)
    for (int i=0; i<S_NUM_SENSORS; i++)
    {
        historysonicindex[i] = 0;
        sonicValues[i] = 0;
        for (int j=0; j<S_WINDOW_SIZE; j++)
            historySonicValues[i][j] = 0;
    }
    
    // ALMemoryFastAccess connection to sensor data
    connectALMemoryFastAccess(pBroker);
    #if SENSOR_VERBOSITY > 0
        thelog << "SENSORS: Getting new sensor data for the first time." << endl;
    #endif
    getSensorData();
   
    #if SENSOR_LOGGING > 0
      createLogs();                 // create logs only if that feature is enabled
    #endif
    
#if SENSOR_ULTRASONIC_ON
    #if SENSOR_VERBOSITY > 0
        thelog << "SENSORS: Constructing ultrasonic thread" << endl;
    #endif
    int err = pthread_create(&SensorUltrasonicThread, NULL, runSonicThread, (void*) this);
    if (err > 0)
    {
        thelog << "SENSORS: ********************* Failed to create sonicthread *********************" << endl;
        thelog << "The error code was: " << err << endl;
    }
#endif
    
    thelog << "SENSORS: Finished constructing sensors" << endl;
    return;
}
// Sensor Class Destructor
Sensors::~Sensors()
{
    #if SENSOR_LOGGING > 0
        finishLogs();
    #endif
    #if SENSOR_ULTRASONIC_ON
        pthread_cancel(SensorUltrasonicThread);
    #endif
    return;
}

/*****************************************
 onNewSensorData()
 - gets lots of sensor data and copies it into organised arrays
 - triggers calculation of soft sensor values too
 - writes to files if datalogging is enabled
 *****************************************/
void Sensors::onNewSensorData()
{
#if SENSOR_VERBOSITY > 2
   thelog << "SENSORS: onNewSensorData()" << endl;
#endif
   
   getSensorData();
   calculateSoftSensors();
   updateSensorHistory();
   
#if SENSOR_LOGGING > 0
   writeSensorData();
#endif
   return;
}


/*****************************************
 getSensorData()
    - gets lots of sensor data and copies it
      into organised arrays
    - triggers calculation of soft sensor values too
*****************************************/
void Sensors::getSensorData()
{
#if SENSOR_VERBOSITY > 2
    thelog << "SENSORS: getSensorData()" << endl;
#endif
    getTimeData();
    getFastMemData();
    return;
}

/*****************************************
 connectALMemoryFastAccess
    - connect all of the useful feedback data (position, current, target, temperature, balance, touch, battery) to the alFastMem
Preconditions: The module named "SensorDataGrabber" exists 
Postconditions: A connection to all of the feedback data is made 
*****************************************/
void Sensors::connectALMemoryFastAccess(AL::ALPtr<AL::ALBroker> pBroker)
{
    thelog << "SENSORS: Connecting to ALMemoryFastAccess." << endl;
    ALValue namelist;           // the list of names to be connected with alFastMem
    namelist.arrayReserve(ALL_NUM_SENSORS);
    
    // add position names
    for (int i=0; i<J_NUM_JOINTS; i++)
    {
        namelist.arrayPush(indexToPositionSensor[i]);
    }
    
    // add current names
    for (int i=0; i<J_NUM_JOINTS; i++)
    {
        namelist.arrayPush(indexToCurrentSensor[i]);
    }
    
    // add target names
    for (int i=0; i<J_NUM_JOINTS; i++)
    {
        namelist.arrayPush(indexToTargetSensor[i]);
    }
    
    // add hardness names
    for (int i=0; i<J_NUM_JOINTS; i++)
    {
        namelist.arrayPush(indexToHardnessSensor[i]);
    }
    
    // add temperature names
    for (int i=0; i<J_NUM_JOINTS; i++)
    {
        namelist.arrayPush(indexToTemperatureSensor[i]);
    }
    
    // add board error names
    for (int i=0; i<MB_NUM_BOARDS; i++)
    {
        namelist.arrayPush(indexToBoardError[i]);
    }
    
    // add nack error names
    for (int i=0; i<NK_NUM_BOARDS; i++)
    {
        namelist.arrayPush(indexToNack[i]);
    }
    
    // add balance names
    for (int i=0; i<B_NUM_SENSORS; i++)
    {
        namelist.arrayPush(indexToBalanceSensor[i]);
    }
    
    // add touch names
    for (int i=0; i<T_NUM_SENSORS; i++)
    {
        namelist.arrayPush(indexToTouchSensor[i]);
    }
    
    // add battery names
    for (int i=0; i<E_NUM_SENSORS; i++)
    {
        namelist.arrayPush(indexToBatterySensor[i]);
    }
    
    // add distance names
    for (int i=0; i<D_NUM_SENSORS; i++)
    {
        namelist.arrayPush(indexToDistanceSensor[i]);
    }
    
    thelog << "SENSORS: Finished creating namelist for ALMemoryFastAccess" << endl;
    thelog << namelist.toString(AL::VerbosityMini) << endl;
    
    alFastMem->ConnectToVariables(pBroker, namelist);
    
    thelog << "SENSORS: Connection to ALMemoryFastAccess established." << endl;
    return;
}

/*****************************************
 get sensor data
*****************************************/
void Sensors::getTimeData()
{
#if SENSOR_VERBOSITY > 2
    thelog << "SENSORS: getTimeData()" << endl;
#endif    
    dcmTime = (int)alDcm->call<int>("getTime", 0);        // use THIS dcmTime when talking to the DCM
    dcmTimeSinceStart = dcmTime - dcmStartTime;
    return;
}

/*****************************************
 getFastMemData()
    - all feedback data is now done through alFastMem
    - note the order in connectALMemoryFastAccess is the order used to extract the information
 Preconditions: the global store
 Postconiditions: the global store of feedback data is updated
*****************************************/
void Sensors::getFastMemData()
{
#if SENSOR_VERBOSITY > 2
    thelog << "SENSORS: getFastMemData()" << endl;
#endif  
    vector<float> fastmemdata;
    alFastMem->GetValues(fastmemdata);
    
    // copy the new data into the global store. This takes negligible processing time. 

    unsigned int fastmemindex = 0;          // the present index into the alldata array
    
    // positions
    for (int i=0; i<J_NUM_JOINTS; i++)
    {
        jointPositions[i] = fastmemdata[fastmemindex];
        fastmemindex++;
    }
    
    // currents
    for (int i=0; i<J_NUM_JOINTS; i++)
    {
        jointCurrents[i] = fastmemdata[fastmemindex];
        fastmemindex++;
    }
    
    // targets
    for (int i=0; i<J_NUM_JOINTS; i++)
    {
        jointTargets[i] = fastmemdata[fastmemindex];
        fastmemindex++;
    }
    
    // hardnesses
    for (int i=0; i<J_NUM_JOINTS; i++)
    {
        jointHardnesses[i] = fastmemdata[fastmemindex];
        fastmemindex++;
    }
    
    // temperatures
    for (int i=0; i<J_NUM_JOINTS; i++)
    {
        jointTemperatures[i] = fastmemdata[fastmemindex];
        fastmemindex++;
    }
    
    // board errors
    for (int i=0; i<MB_NUM_BOARDS; i++)
    {
        boardErrors[i] = fastmemdata[fastmemindex];
        fastmemindex++;
    }
    
    // nacks
    for (int i=0; i<NK_NUM_BOARDS; i++)
    {
        nackErrors[i] = fastmemdata[fastmemindex];
        fastmemindex++;
    }
    
    // balance values
    static float tempbalance[B_NUM_SENSORS];              // I need to do some filtering at the very heart of the get data
    static float sum;
    for (int i=0; i<B_NUM_SENSORS; i++)
    {
        tempbalance[i] = fastmemdata[fastmemindex];
        fastmemindex++;
    }
    sum = sqrt(pow(tempbalance[B_ACCEL_X], 2) + pow(tempbalance[B_ACCEL_Y], 2) + pow(tempbalance[B_ACCEL_Z], 2));
    if (sum < 70)
    {
        for (int i=0; i<B_NUM_SENSORS; i++)
        {
            balanceValues[i] = tempbalance[i];
        }
    }
    
    
    // touch values
    for (int i=0; i<T_NUM_SENSORS; i++)
    {
        touchValues[i] = fastmemdata[fastmemindex];
        fastmemindex++;
    }
    
    // battery values
    for (int i=0; i<E_NUM_SENSORS; i++)
    {
        batteryValues[i] = fastmemdata[fastmemindex];
        fastmemindex++;
    }
    
    // distance values
    for (int i=0; i<D_NUM_SENSORS; i++)
    {
        distanceValues[i] = fastmemdata[fastmemindex];
        fastmemindex++;
    }
    
    return;
}

/*****************************************
 Filter Data
 *****************************************/
void Sensors::filterData()
{
    filterBalanceValues();
    //filterJointVelocities();
}

void Sensors::filterBalanceValues()
{
    // The balance values are filtered by a 8 order minimum phase FIR low pass filter. Passband edge = 6Hz, Passband ripple= 1dB, stopband edge -20dB.
    for (int i = 0; i < B_NUM_SENSORS; i++)
    {
        historyBalanceValues[i][historybalanceindex] = balanceValues[i];
        balanceValues[i] = 0.880663287116484*(-0.068828325795436*historyBalanceValues[i][(B_WINDOW_SIZE + historybalanceindex-8)%B_WINDOW_SIZE] - 0.066584108138720*historyBalanceValues[i][(B_WINDOW_SIZE + historybalanceindex-7)%B_WINDOW_SIZE] - 0.016367546372531*historyBalanceValues[i][(B_WINDOW_SIZE + historybalanceindex-6)%B_WINDOW_SIZE] + 0.099408189849786*historyBalanceValues[i][(B_WINDOW_SIZE + historybalanceindex-5)%B_WINDOW_SIZE] + 0.234571953457501*historyBalanceValues[i][(B_WINDOW_SIZE + historybalanceindex-4)%B_WINDOW_SIZE] + 0.318607970748508*historyBalanceValues[i][(B_WINDOW_SIZE + historybalanceindex-3)%B_WINDOW_SIZE] + 0.307674130346582*historyBalanceValues[i][(B_WINDOW_SIZE + historybalanceindex-2)%B_WINDOW_SIZE] + 0.216321829545804*historyBalanceValues[i][(B_WINDOW_SIZE + historybalanceindex-1)%B_WINDOW_SIZE] + 0.110703670369260*historyBalanceValues[i][historybalanceindex]);
        //DCM 100Hz balanceValues[i] = 0.945625469066873*(-0.059311517098191*historyBalanceValues[i][(B_WINDOW_SIZE + historybalanceindex-8)%B_WINDOW_SIZE] - 0.000856137299458*historyBalanceValues[i][(B_WINDOW_SIZE + historybalanceindex-7)%B_WINDOW_SIZE] + 0.051964222917671*historyBalanceValues[i][(B_WINDOW_SIZE + historybalanceindex-6)%B_WINDOW_SIZE] + 0.127834095351578*historyBalanceValues[i][(B_WINDOW_SIZE + historybalanceindex-5)%B_WINDOW_SIZE] + 0.198715956886153*historyBalanceValues[i][(B_WINDOW_SIZE + historybalanceindex-4)%B_WINDOW_SIZE] + 0.235169385734816*historyBalanceValues[i][(B_WINDOW_SIZE + historybalanceindex-3)%B_WINDOW_SIZE] + 0.222053635052465*historyBalanceValues[i][(B_WINDOW_SIZE + historybalanceindex-2)%B_WINDOW_SIZE] + 0.166603220105332*historyBalanceValues[i][(B_WINDOW_SIZE + historybalanceindex-1)%B_WINDOW_SIZE] + 0.115328266134171*historyBalanceValues[i][historybalanceindex]);
    }
    historybalanceindex = (historybalanceindex + 1)%B_WINDOW_SIZE;
    return;
}

void Sensors::filterJointVelocities()
{
    // The velocity values are filtered by a 8 order minimum phase FIR low pass filter. Passband edge = 10Hz, Passband ripple= 1dB, stopband edge -20dB.
    /*for (int i = 0; i < J_NUM_JOINTS; i++)
    {
        historyJointVelocities[i][historyvelocityindex] = jointVelocities[i];
        //DCM = 50Hz jointVelocities[i] = 0.908471624011480*(0.027275582417591*historyJointVelocities[i][(VEL_WINDOW_SIZE + historyvelocityindex-8)%VEL_WINDOW_SIZE] + 0.075358585052493*historyJointVelocities[i][(VEL_WINDOW_SIZE + historyvelocityindex-7)%VEL_WINDOW_SIZE] + 0.140358120634960*historyJointVelocities[i][(VEL_WINDOW_SIZE + historyvelocityindex-6)%VEL_WINDOW_SIZE] + 0.197351021127837*historyJointVelocities[i][(VEL_WINDOW_SIZE + historyvelocityindex-5)%VEL_WINDOW_SIZE] + 0.220063227285465*historyJointVelocities[i][(VEL_WINDOW_SIZE + historyvelocityindex-4)%VEL_WINDOW_SIZE] + 0.197351021127837*historyJointVelocities[i][(VEL_WINDOW_SIZE + historyvelocityindex-3)%VEL_WINDOW_SIZE] + 0.140358120634960*historyJointVelocities[i][(VEL_WINDOW_SIZE + historyvelocityindex-2)%VEL_WINDOW_SIZE] + 0.075358585052493*historyJointVelocities[i][(VEL_WINDOW_SIZE + historyvelocityindex-1)%VEL_WINDOW_SIZE] + 0.027275582417591*historyJointVelocities[i][historyvelocityindex]);
        jointVelocities[i] = -0.059885673555845*historyJointVelocities[i][(VEL_WINDOW_SIZE + historyvelocityindex-8)%VEL_WINDOW_SIZE] - 0.101746769741238*historyJointVelocities[i][(VEL_WINDOW_SIZE + historyvelocityindex-7)%VEL_WINDOW_SIZE] - 0.026932512852181*historyJointVelocities[i][(VEL_WINDOW_SIZE + historyvelocityindex-6)%VEL_WINDOW_SIZE] + 0.043017527360489*historyJointVelocities[i][(VEL_WINDOW_SIZE + historyvelocityindex-5)%VEL_WINDOW_SIZE] + 0.185076072459385*historyJointVelocities[i][(VEL_WINDOW_SIZE + historyvelocityindex-4)%VEL_WINDOW_SIZE] + 0.264641510204521*historyJointVelocities[i][(VEL_WINDOW_SIZE + historyvelocityindex-3)%VEL_WINDOW_SIZE] + 0.286307728191018*historyJointVelocities[i][(VEL_WINDOW_SIZE + historyvelocityindex-2)%VEL_WINDOW_SIZE] + 0.215337183024144*historyJointVelocities[i][(VEL_WINDOW_SIZE + historyvelocityindex-1)%VEL_WINDOW_SIZE] + 0.136683810811822*historyJointVelocities[i][historyvelocityindex];
    }
    historyvelocityindex = (historyvelocityindex + 1)%VEL_WINDOW_SIZE;
    */
    double sum = 0;
    for (int jnum = 0; jnum < J_NUM_JOINTS; jnum++)
    {
        historyJointVelocities[jnum][historyvelocityindex] = jointVelocities[jnum];          // copy the joint velocity into the average buffer before it is filtered
        
        sum = 0;
        for (int i = 0; i < VEL_WINDOW_SIZE; i++)
        {
            sum += historyJointVelocities[jnum][i];
        }
        
        jointVelocities[jnum] = sum/VEL_WINDOW_SIZE;                                   // now set the jointVelocity to the averaged value
    }
    historyvelocityindex = (historyvelocityindex + 1)%VEL_WINDOW_SIZE;
    return;
}

/*****************************************
 calculate soft sensor data
*****************************************/

void Sensors::calculateSoftSensors()
{
   
    signCurrentData();
    calculateJointVelocities();
    filterBalanceValues();
    calculateJointAccelerations();
   
    // Calculate the velocity and current sums
    jointVelocitySum = 0;
    jointCurrentSum = 0;
    for (unsigned char i=2; i<J_NUM_JOINTS; i++)            // Warning: This assumes that the head is the first two joints, and that its movement is unimportant
    {
        jointVelocitySum += fabs(jointVelocities[i]);
        jointCurrentSum += fabs(jointCurrents[i]);
    }
    
#if SENSOR_EXPORT_TO_AL
    alStm->insertData("Jason/Misc/jointVelocitySum", jointVelocitySum, 0);
    alStm->insertData("Jason/Misc/jointCurrentSum", jointCurrentSum, 0);    
#endif
    
    calculateStiffnessCorrectionFactor();
    calculateFootForceReadings();
    calculateCoP();
    determineSupportMode();
    determineWhetherOnGround();
    determineWhetherWalking();
    determineWhetherFalling();
    determineWhetherColliding();
}

/* Signs the electric current data
 Preconditions: jointTargets, jointPositions and jointCurrents must have been set
 */
void Sensors::signCurrentData()
{
   for (int i=0; i< J_NUM_JOINTS; i++)
   {
      if (jointHardnesses[i] <= 0)                // if the motor hardness is off keep the current sign the same as the previous value
      {
         if (previousJointCurrents[i] < 0)
            jointCurrents[i] = -jointCurrents[i];
      }
      else if (jointTargets[i] < jointPositions[i])
         jointCurrents[i] = -jointCurrents[i];
   }
   return;
}

void Sensors::calculateJointVelocities()
{
   if (dcmTime - previousDcmTime < 5)                // don't try and calculate the velocities; this avoids divide by zero errors
      return;
   
   for (int i=0; i<J_NUM_JOINTS; i++)
      jointVelocities[i] = 1000*(jointPositions[i] - previousJointPositions[i])/(dcmTime - previousDcmTime);
   
   return;
}

void Sensors::calculateJointAccelerations()
{
   if (dcmTime - previousDcmTime < 5)                // don't try and calculate the acceleration
      return;
   
   for (int i = 0; i < J_NUM_JOINTS; i++)
      jointAccelerations[i] = 1000*(jointVelocities[i] - previousJointVelocities[i])/(dcmTime - previousDcmTime);
   return;
}

/* Calculates odometry using Steve's kinematics
 Postconditions: odometryDeltaX, odometryDeltaY, odometryDeltaO are updated with new values
 */
void Sensors::calculateOdometry()
{
    const float turnMultiplier = 1.0;
    const float xMultiplier = -1.25;
    const float yMultiplier = -1.0;
    
    static float previousHipYaw = 0;
    static float previousLeftX = 0;
    static float previousLeftY = 0;
    static float previousRightX = 0;
    static float previousRightY = 0;
    
    Kinematics *kin = &Kinematics::getInstance();
    // Get Feet Positions
    vector<float> leftFootPosition = kin->GetLeftFootPosition();
    vector<float> rightFootPosition = kin->GetRightFootPosition();
    // Get Hip Position
    float hipYaw = jointPositions[J_L_HIP_YAWPITCH];
    
    float leftX = leftFootPosition[0];
    float rightX = rightFootPosition[0]; 
    float leftY = leftFootPosition[1];
    float rightY = rightFootPosition[1];
    
    // Distances moved in the last frame
    float angleDiff = 0.0;     // change in orientation since last frame
    float xDiff = 0.0;         // change in forward direction (these are in metres because Aldebaran is in metres)
    float yDiff = 0.0;         // change in sideways direction
    
    if (touchLeftFootOnGround == true && touchRightFootOnGround == false)     // on left foot
    {
        angleDiff = turnMultiplier*(hipYaw - previousHipYaw);
        xDiff = xMultiplier*(leftX - previousLeftX);         
        yDiff = yMultiplier*(leftY - previousLeftY);
    }
    else if (touchLeftFootOnGround == false && touchRightFootOnGround == true)   // on right foot
    {
        angleDiff = -turnMultiplier*(hipYaw - previousHipYaw);
        xDiff = xMultiplier*(rightX - previousRightX);
        yDiff = yMultiplier*(rightY - previousRightY);
        
    } 
    else if(touchLeftFootOnGround == true && touchRightFootOnGround == true)
    {
        float leftsum = touchValues[T_L_FSR_FL] + touchValues[T_L_FSR_FR] + touchValues[T_L_FSR_BL] + touchValues[T_L_FSR_BR];
        float rightsum = touchValues[T_R_FSR_FL] + touchValues[T_R_FSR_FR] + touchValues[T_R_FSR_BL] + touchValues[T_R_FSR_BR];
        
        if((leftsum - rightsum) > 1500){
            angleDiff = -turnMultiplier*(hipYaw - previousHipYaw);
            xDiff = xMultiplier*(rightX - previousRightX);        
            yDiff = yMultiplier*(rightY - previousRightY);
        }
        else if((rightsum -leftsum) > 1500)
        {
            angleDiff = turnMultiplier*(hipYaw - previousHipYaw); 
            xDiff = xMultiplier*(leftX - previousLeftX);          
            yDiff = yMultiplier*(leftY - previousLeftY);          
        }
        
    }
    
    // Update historic variables   
    previousHipYaw = hipYaw;
    previousLeftX = leftX;
    previousRightX = rightX;
    previousLeftY = leftY;
    previousRightY = rightY;
    
    // copy values to global parameters
    odometryDeltaX = xDiff;
    odometryDeltaY = yDiff;
    odometryDeltaO = angleDiff;
    
    return;
}

/* Calibrates the foot force sensors
 Preconditions: The robot must be standing still
 */
void Sensors::calibrateFootForceSensors()
{
    return;
}

/* 
 */
void Sensors::calculateStiffnessCorrectionFactor()
{
    static float electricCurrentSum = 0;
    static int electricCurrentCount = 0;
    static const float nominalElectricCurrentSum = 23.544;           // the expected long term average of the force under a single foot (Newtons)
    
    electricCurrentSum += jointCurrentSum;
    electricCurrentCount++;
    
    if (electricCurrentCount > 800)
    {   // if the count is getting big; fudge the average so that the count doesn't over flow
        electricCurrentSum = (electricCurrentSum/electricCurrentCount)*400;
        electricCurrentCount = 400;
    }
    
#if SENSOR_EXPORT_TO_AL
    alStm->insertData("Jason/Misc/averageCurrent", (electricCurrentSum/electricCurrentCount), 0);
#endif
}

/* Calculate the centre of pressure (ZMP) under each foot
 Postconditions: touchLeftCoPX, touchLeftCoPY, touchRightCoPX, touchRightCoPY are set to the calculated values in centimetres
 */
void Sensors::calculateFootForceReadings()
{
    static const float m = 8900;                // values determined by experiment for conversion from alderbaran's readings to Newtons
    static const float b = -1.77;
    
    touchPSValues[PS_L_FL] = (1.0/touchValues[T_L_FSR_FL])*m + b;
    touchPSValues[PS_L_FR] = (1.0/touchValues[T_L_FSR_FR])*m + b;
    touchPSValues[PS_L_BL] = (1.0/touchValues[T_L_FSR_BL])*m + b;
    touchPSValues[PS_L_BR] = (1.0/touchValues[T_L_FSR_BR])*m + b;
    
    touchPSValues[PS_L] = touchPSValues[PS_L_FL] + touchPSValues[PS_L_FR] + touchPSValues[PS_L_BL] + touchPSValues[PS_L_BR];
    
    touchPSValues[PS_R_FL] = (1.0/touchValues[T_R_FSR_FL])*m + b;
    touchPSValues[PS_R_FR] = (1.0/touchValues[T_R_FSR_FR])*m + b;
    touchPSValues[PS_R_BL] = (1.0/touchValues[T_R_FSR_BL])*m + b;
    touchPSValues[PS_R_BR] = (1.0/touchValues[T_R_FSR_BR])*m + b;
    touchPSValues[PS_R] = touchPSValues[PS_R_FL] + touchPSValues[PS_R_FR] + touchPSValues[PS_R_BL] + touchPSValues[PS_R_BR];

#if SENSOR_EXPORT_TO_AL
    alStm->insertData("Jason/Touch/touchLeftPS_FL", touchPSValues[PS_L_FL], 0);
    alStm->insertData("Jason/Touch/touchLeftPS_FR", touchPSValues[PS_L_FR], 0);    
    alStm->insertData("Jason/Touch/touchLeftPS_BL", touchPSValues[PS_L_BL], 0);
    alStm->insertData("Jason/Touch/touchLeftPS_BR", touchPSValues[PS_L_BR], 0);
    alStm->insertData("Jason/Touch/touchLeftPS_L", touchPSValues[PS_L], 0);
    
    alStm->insertData("Jason/Touch/touchRightPS_FL", touchPSValues[PS_R_FL], 0);
    alStm->insertData("Jason/Touch/touchRightPS_FR", touchPSValues[PS_R_FR], 0);    
    alStm->insertData("Jason/Touch/touchRightPS_BL", touchPSValues[PS_R_BL], 0);
    alStm->insertData("Jason/Touch/touchRightPS_BR", touchPSValues[PS_R_BR], 0);
    alStm->insertData("Jason/Touch/touchRightPS_R", touchPSValues[PS_R], 0);
#endif
}

/* Calculate the centre of pressure (ZMP) under each foot
 Postconditions: touchLeftCoPX, touchLeftCoPY, touchRightCoPX, touchRightCoPY are set to the calculated values in centimetres
 */
void Sensors::calculateCoP()
{
    touchLeftCoPX = (7.10*touchPSValues[PS_L_FL] + 7.10*touchPSValues[PS_L_FR] - 3.04*touchPSValues[PS_L_BL] - 2.98*touchPSValues[PS_L_BR])/touchPSValues[PS_L];
    touchLeftCoPY = (3.00*touchPSValues[PS_L_FL] - 2.30*touchPSValues[PS_L_FR] + 3.00*touchPSValues[PS_L_BL] - 1.90*touchPSValues[PS_L_BR])/touchPSValues[PS_L];

    touchRightCoPX = (7.10*touchPSValues[PS_R_FL] + 7.10*touchPSValues[PS_R_FR] - 3.04*touchPSValues[PS_R_BL] - 2.98*touchPSValues[PS_R_BR])/touchPSValues[PS_R];
    touchRightCoPY = (2.30*touchPSValues[PS_R_FL] - 3.00*touchPSValues[PS_R_FR] + 1.90*touchPSValues[PS_R_BL] - 3.00*touchPSValues[PS_R_BR])/touchPSValues[PS_R];
   
#if SENSOR_EXPORT_TO_AL
   alStm->insertData("Jason/Touch/touchLeftCoPX", touchLeftCoPX, 0);
   alStm->insertData("Jason/Touch/touchLeftCoPY", touchLeftCoPY, 0);    
   alStm->insertData("Jason/Touch/touchRightCoPX", touchRightCoPX, 0);
   alStm->insertData("Jason/Touch/touchRightCoPY", touchRightCoPY, 0);
#endif
}
/* Determines the support mode while walking.
 Each foot can have the following support modes:
    1. stance: The foot is on the ground
    2. push: transistioning from stance to swing
    3. swing: this foot is not supporting the weight of the robot
    4. impact: transistioning from swing back to stance.
 */
void Sensors::determineSupportMode()
{
// It is easier to write logic to determine the support mode with filtered weights. 
    // I filtered the left and right feet forces, as well as keeping a long term average for calibration purposes
    static const unsigned char historylength = 10;
    static int historyindex = 0;
    static float historyLeftValues[historylength];
    static float historyRightValues[historylength];
    
    // I also want to have this self calibrating so I am going to keep the average of the left and right forces, so that I can scale them.
    float leftAverageForce = 0;
    float rightAverageForce = 0;
    static float leftSumForce = 0;
    static float rightSumForce = 0;
    static int averageCount = 0;
    static const float nominalforce = 23.544;           // the expected long term average of the force under a single foot (Newtons)
    
    historyLeftValues[historyindex] = touchPSValues[PS_L];
    historyRightValues[historyindex] = touchPSValues[PS_R];

    float leftForce = 0;                // filtered version of the total force on the left foot (N).
    float rightForce = 0;               // filtered version of the total force on the right foot (N).
    for (unsigned char i=0; i<historylength; i++)
    {
        leftForce += historyLeftValues[i];
        rightForce += historyRightValues[i]; 
    }
    leftForce /= historylength;
    rightForce /= historylength;
    historyindex = (historyindex + 1)%historylength;
    
    leftSumForce += leftForce;
    rightSumForce += rightForce;
    averageCount++;
    leftAverageForce = leftSumForce/averageCount;
    rightAverageForce = rightSumForce/averageCount;
    
    if (averageCount > 800)
    {   // if the count is getting big; fudge the average so that the count doesn't over flow
        averageCount = 400;
        leftSumForce = leftAverageForce*averageCount;
        rightSumForce = rightAverageForce*averageCount;
    }
    leftForce *= (nominalforce/leftAverageForce);               // use the long term average to calibrate the forces
    rightForce *= (nominalforce/rightAverageForce);
    
// OK. The logic starts done here.
    
    float percentLeftFoot = 100*leftForce/(leftForce + rightForce);
    float percentRightFoot = 100*rightForce/(leftForce + rightForce);
    
    if (walkAmIWalking == true)
    {   // if I am walking then determine what support mode I am in, based on the previous support mode and sensor feedback
        
        if (leftSupportMode == SM_STANCE && rightSupportMode == SM_STANCE)
        {   // if I am in double support mode and stopped
            if (percentLeftFoot < 40)
                leftSupportMode = SM_PUSH;
            else if (percentRightFoot < 40)
                rightSupportMode = SM_PUSH;
        }
        else
        {
            // do the logic for the left foot
            if (leftSupportMode == SM_STANCE)
            {   // We can transistion to SM_PUSH. This happens when:
                // The weight begins to be transfered to the other foot and the other foot is in SM_STANCE or SM_IMPACT
                if (rightSupportMode == SM_STANCE || rightSupportMode == SM_IMPACT)
                    if (percentLeftFoot < 47)
                        leftSupportMode = SM_PUSH;
            }
            else if (leftSupportMode == SM_PUSH)
            {   // We can transition to SM_SWING. This happens when:
                // The weight on this foot is very low, and the other foot is in SM_STANCE
                if (rightSupportMode == SM_STANCE)
                    if (percentLeftFoot < 36)
                        leftSupportMode = SM_SWING;
            }
            else if (leftSupportMode == SM_SWING)
            {   // We can transition to SM_IMPACT. This happens where:
                // The weight comes back to this foot, and the other foot is in SM_STANCE
                if (rightSupportMode == SM_STANCE)
                    if (percentLeftFoot > 36)
                        leftSupportMode = SM_IMPACT;
            }
            else if (leftSupportMode == SM_IMPACT)
            {   // We can transition to SM_STANCE. This happens when:
                // The weight is more on this foot
                if (percentLeftFoot > 47)
                    leftSupportMode = SM_STANCE;
            }
            
            // do the logic for the right foot
            if (rightSupportMode == SM_STANCE)
            {   // We can transistion to SM_PUSH. This happens when:
                // The weight begins to be transfered to the other foot and the other foot is in SM_STANCE or SM_IMPACT
                if (leftSupportMode == SM_STANCE || leftSupportMode == SM_IMPACT)
                    if (percentRightFoot < 47)
                        rightSupportMode = SM_PUSH;
            }
            else if (rightSupportMode == SM_PUSH)
            {   // We can transition to SM_SWING. This happens when:
                // The weight on this foot is very low, and the other foot is in SM_STANCE
                if (leftSupportMode == SM_STANCE)
                    if (percentRightFoot < 36)
                        rightSupportMode = SM_SWING;
            }
            else if (rightSupportMode == SM_SWING)
            {   // We can transition to SM_IMPACT. This happens where:
                // The weight comes back to this foot, and the other foot is in SM_STANCE
                if (leftSupportMode == SM_STANCE)
                    if (percentRightFoot > 36)
                        rightSupportMode = SM_IMPACT;
            }
            else if (rightSupportMode == SM_IMPACT)
            {   // We can transition to SM_STANCE. This happens when:
                // The weight is more on this foot
                if (percentRightFoot > 47)
                    rightSupportMode = SM_STANCE;
            }
        }
            
    }
    else
    {
        leftSupportMode = SM_STANCE;
        rightSupportMode = SM_STANCE;
    }
    
    
#if SENSOR_EXPORT_TO_AL
    alStm->insertData("Jason/Support/leftForce", leftForce, 0);
    alStm->insertData("Jason/Support/rightForce", rightForce, 0);    
    alStm->insertData("Jason/Support/leftPercent", percentLeftFoot, 0);
    alStm->insertData("Jason/Support/rightPercent", percentRightFoot, 0);

    alStm->insertData("Jason/Support/leftMode", (int)leftSupportMode, 0);
    alStm->insertData("Jason/Support/rightMode", (int)rightSupportMode, 0);
#endif
}

/* Determine whether each foot is in contact with the ground
 Postconditions: touchLeftFootOnGround, touchRightFootOnGround and touchOnGround are set to true if on the ground
 */
void Sensors::determineWhetherOnGround()
{
    static unsigned char touchOffGroundCount = 0;
    touchPreviousOnGround = touchOnGround;
    // this actually turned out to be a very easy problem to solve using only the foot sensors
    float leftsum = touchValues[T_L_FSR_FL] + touchValues[T_L_FSR_FR] + touchValues[T_L_FSR_BL] + touchValues[T_L_FSR_BR];
    float rightsum = touchValues[T_R_FSR_FL] + touchValues[T_R_FSR_FR] + touchValues[T_R_FSR_BL] + touchValues[T_R_FSR_BR];
    
    if (leftsum > 9000)
        touchLeftFootOnGround = false;
    else
        touchLeftFootOnGround = true;
    
    if (rightsum > 9000)
        touchRightFootOnGround = false;
    else
        touchRightFootOnGround = true;
    
    if (touchLeftFootOnGround || touchRightFootOnGround)        // if either foot is on the ground then we are on the ground
    {
        touchOffGroundCount = 0;
        touchOnGround = true;
    }
    else
    {
        if (touchOffGroundCount < 250)
            touchOffGroundCount++;
        
        if (touchOffGroundCount > 10)
            touchOnGround = false;
        else
            touchOnGround = true;
    }
    
    #if SENSOR_EXPORT_TO_AL
        alStm->insertData("Jason/Touch/LeftFootOnGround", (int)touchLeftFootOnGround, 0);
        alStm->insertData("Jason/Touch/RightFootOnGround", (int)touchRightFootOnGround, 0);    
        alStm->insertData("Jason/Touch/LeftSum", leftsum, 0);
        alStm->insertData("Jason/Touch/RightSum", rightsum, 0);
    #endif
    
}

/* Determine whether the robot is 'walking'
 Postconditions: walkAmIWalking is set to true if the robot is walking, false if it is still
 */
void Sensors::determineWhetherWalking()
{
    // The robot is not 'walking' if the joint velocity sum has been 'still' for 3 consecutive dcmcycle
    static unsigned char stillcount = 0;          // the number of calls the robot has been still
    
    walkPreviousAmIWalking = walkAmIWalking;
    // I need walkIsActive to be true immediately after a call to walk, so I keep a global variable (walkCyclesSinceCall) which can be reset by anyone
    if (walkCyclesSinceCall < 254)
        walkCyclesSinceCall++;
    
    if (walkCyclesSinceCall < 11)        // if the global variable has been reset by a walk call in the last 10 cycles then walk is active
        walkAmIWalking = true;
    else
    {   // so if there is no recent walk call, I need to check the joint feedback to determine whether it is walking
        if (jointVelocitySum > 1.2)
        {
            walkAmIWalking = true;
            stillcount = 0;
        }
        else
        {
            if (stillcount > 2)
            {
                walkAmIWalking = false;
            }
            else
            {
                stillcount++;
                walkAmIWalking = true;
            }
        }
    }
#if SENSOR_EXPORT_TO_AL
    alStm->insertData("Jason/Walk/walkAmIWalking", (int)walkAmIWalking, 0);
#endif
}

/* Deterimines whether the robot is stable, falling or fallen.
 
 The robot is stable if balanceFallingEnabled == false or the accelerometers indicate the robot is fairly upright.
 The robot is falling if the robot is leaning far enough over that it can not recover.
 The robot has fallen over if it is lying down, or it has been falling for awhile. The transition from falling to fallen, always occurs eventually.
 */
void Sensors::determineWhetherFalling()
{
    static unsigned char fallencount = 0;         // the number of consecutive calls (ie dcm cycles) the robot has been approximately horizontal
    static unsigned char fallingcount = 0;        // the number of dcm cycles since the robot started falling over
    
#if SENSOR_VERBOSITY > 1
    if (balanceFallingEnabled == true)
        thelog << "SENSORS: determineWhetherFalling(): fallencount=" << (int)fallencount << " fallingcount=" << (int)fallingcount << endl;
#endif

    if (balanceFallingEnabled == false)
    {   // if falling control is not enabled, then everything is reset
        balanceFallingForward = false;
        balanceFallingBackward = false;
        balanceFallingLeft = false;
        balanceFallingRight = false;
        //balanceFallen = false;
        
        fallencount = 0;
        fallingcount = 0;
    }
    else
    {
        balancePreviousFalling = balanceFalling;
        balancePreviousFallen = balanceFallen;
        if (fallingcount > 0)
            fallingcount++;
        
        // Firstly determine whether the robot has fallen. 
        if (balancePreviousFallen == false)
        {
            /* There are two ways of detecting whether I have fallen over:
                (a) the torso is approximately horizontal
                (b) I have been 'falling' long enough that I must have fallen over by now
             It is possible to go to (a) without going through the 'falling' state if the robot falls over while the falling control
             is not enabled, or if the robot is booted up lying down.
            */
            if (fabs(balanceValues[B_ANGLE_Y]) > 1.22 || fabs(balanceValues[B_ANGLE_X]) > 1.22)
                fallencount++;
            else
                fallencount = 0;
            
            if (fallencount > 5)
            {   // if the torso has been approximately horizontal for 0.1s then we have fallen over (the count is used to prevent starting a get up too soon after a fall)
                balanceFallen = true;
                fallencount = 0;
            }
            else if (fallingcount > 40)
            {   // if it has been 0.8s since the robot started falling over, then assume we have now fallen to the ground
                balanceFallen = true;
                fallencount = 0;
                fallingcount = 0;
            }
        }
        
        if (balanceFallen == true)
        {   // we can't be 'falling' if we have already fallen
            balanceFallingForward = false;
            balanceFallingBackward = false;
            balanceFallingLeft = false;
            balanceFallingRight = false;
            
            if (balancePreviousFallen == false)
                balanceFallenCount++;
            
            fallingcount = 0;
        }
        else            
        {   // If we haven't fallen, we could be falling; determine whether we are falling, and in what direction(s)
            if (touchOnGround && fallencount == 0)
            {
                if (balanceValues[B_ANGLE_Y] < -0.50)        // we are falling backwards
                    balanceFallingBackward = true;
                
                if (balanceValues[B_ANGLE_Y] > 0.50)    // we are falling forwards
                    balanceFallingForward = true;
                
                if (balanceValues[B_ANGLE_X] < -0.65)   // we are falling left
                    balanceFallingLeft = true;
                
                if (balanceValues[B_ANGLE_X] > 0.65)   // we are falling right
                    balanceFallingRight = true;
            }
        }
    }
    
    // if we are falling in any direction then we ARE falling
    balanceFalling = balanceFallingForward || balanceFallingBackward || balanceFallingLeft || balanceFallingRight;
    
    if (balancePreviousFalling == false && balanceFalling == true)
        fallingcount = 1;
    
#if SENSOR_EXPORT_TO_AL
    alStm->insertData("Jason/Balance/AngleX", balanceValues[B_ANGLE_X], 0);
    alStm->insertData("Jason/Balance/AngleY", balanceValues[B_ANGLE_Y], 0);
    alStm->insertData("Jason/Balance/AccelX", balanceValues[B_ACCEL_X], 0);
    alStm->insertData("Jason/Balance/AccelY", balanceValues[B_ACCEL_Y], 0);
    alStm->insertData("Jason/Balance/AccelZ", balanceValues[B_ACCEL_Z], 0);
    alStm->insertData("Jason/Balance/FallingForward", (int)balanceFallingForward, 0);
    alStm->insertData("Jason/Balance/FallingBacward", (int)balanceFallingBackward, 0);
    alStm->insertData("Jason/Balance/FallingLeft", (int)balanceFallingLeft, 0);
    alStm->insertData("Jason/Balance/FallingRight", (int)balanceFallingRight, 0);
    
    alStm->insertData("Jason/Balance/Falling", (int)balanceFalling, 0);
    alStm->insertData("Jason/Balance/Fallen", (int)balanceFallen, 0);
#endif
}

void Sensors::determineWhetherColliding()
{
    collisionLeftArmFront = false;
    collisionLeftArmSide = false;
    collisionLeftArmBack = false;
    
    collisionRightArmFront = false;
    collisionRightArmSide = false;
    collisionRightArmBack = false;
    
    // Collision left foot front is easy; just use the foot bumpers
    static int bumpercollisiontime = 60;        // the duration of a collision with the foot bumpers
    static int leftfootbumpercount = 0;
    static int rightfootbumpercount = 0;
    if (touchValues[T_L_BUMP_L] == 1 || touchValues[T_L_BUMP_R] == 1)
        leftfootbumpercount = 0;
    else
        leftfootbumpercount++;

    if (leftfootbumpercount < bumpercollisiontime)
        collisionLeftFootFront = true;
    else
        collisionLeftFootFront = false;
    
    // Collision right foot bumper is easy; just use the foot bumpers
    if (touchValues[T_R_BUMP_L] == 1 || touchValues[T_R_BUMP_R] == 1)
        rightfootbumpercount = 0;
    else
        rightfootbumpercount++;
    
    if (rightfootbumpercount < bumpercollisiontime)
        collisionRightFootFront = true;
    else
        collisionRightFootFront = false;
    
    collisionLeftFootBack = false;
    collisionRightFootBack = false;
    
    collisionLeftArm = collisionLeftArmFront || collisionLeftArmSide || collisionLeftArmBack;
    collisionRightArm = collisionRightArmFront || collisionRightArmSide || collisionRightArmBack;
    collisionLeftFoot = collisionLeftFootFront || collisionLeftFootBack;
    collisionRightFoot = collisionRightFootFront || collisionRightFootBack;
    
    collisionAny = collisionLeftArm || collisionRightArm || collisionLeftFoot || collisionRightFoot;
}

void Sensors::updateSensorHistory()
{
    previousDcmTime = dcmTime;
    for (int i = 0; i < J_NUM_JOINTS; i++)
    {
        previousJointPositions[i] = jointPositions[i];
        previousJointVelocities[i] = jointVelocities[i];
        previousJointCurrents[i] = jointCurrents[i];
    }
    return;
}

/*****************************************
 writeSensorData()
 - writes the current sensor data into files
 - one file for each type of sensor data
 *****************************************/
void Sensors::writeSensorData()
{
#if SENSOR_VERBOSITY > 2
    thelog << "SENSORS: writeSensorData()" << endl;
#endif
    writePositionData();
    writeVelocityData();
    writeAccelerationData();
    writeCurrentData();
    writeTargetData();
    writeHardnessData();
    writeTemperatureData();
    writeBalanceData();
    writeTouchData();
    writeBatteryData();
    writeDistanceData();
    return;
}

/*****************************************
 createLogs()
 - creates the sensor log files
 *****************************************/
void Sensors::createLogs()
{
#if SENSOR_LOGGING > 0
    positionLog.open("/var/log/jointPositions.csv", fstream::out);
    writeJointLabels(positionLog);
    velocityLog.open("/var/log/jointVelocities.csv", fstream::out);
    writeJointLabels(velocityLog);
    accelerationLog.open("/var/log/jointAccelerations.csv", fstream::out);
    writeJointLabels(accelerationLog);
    currentLog.open("/var/log/jointCurrents.csv", fstream::out);
    writeJointLabels(currentLog);
    targetLog.open("/var/log/jointTargets.csv", fstream::out);
    writeJointLabels(targetLog);
    hardnessLog.open("/var/log/jointHardnesses.csv", fstream::out);
    writeJointLabels(hardnessLog);
    temperatureLog.open("/var/log/jointTemperatures.csv", fstream::out);
    writeJointLabels(temperatureLog);
    balanceLog.open("/var/log/balanceValues.csv", fstream::out);
    writeBalanceLabels();
    touchLog.open("/var/log/touchValues.csv", fstream::out);
    writeTouchLabels();
    batteryLog.open("/var/log/batteryValues.csv", fstream::out);
    writeBatteryLabels();
    distanceLog.open("/var/log/distanceValues.csv", fstream::out);
    writeDistanceLabels();
#endif
}

/*****************************************
 finishLogs()
 - closes the sensor log files
 *****************************************/
void Sensors::finishLogs()
{
#if SENSOR_LOGGING > 0
    positionLog.close();
    velocityLog.close();
    accelerationLog.close();
    currentLog.close();
    targetLog.close();
    hardnessLog.close();
    temperatureLog.close();
    balanceLog.close();
    touchLog.close();
    batteryLog.close();
    distanceLog.close();
#endif
}

/*****************************************
 write sensor data
*****************************************/
void Sensors::writePositionData()
{
  // Time
  positionLog << dcmTimeSinceStart << ", ";
  for (int i = 0; i < J_NUM_JOINTS; i++)
  {
    positionLog << jointPositions[i] << ", ";
  }
  positionLog << endl;
  return;
}

void Sensors::writeVelocityData()
{
  // Time
  velocityLog << dcmTimeSinceStart << ", ";
  for (int i = 0; i < J_NUM_JOINTS; i++)
  {
    velocityLog << jointVelocities[i] << ", ";
  }
  velocityLog << endl;
  return;
}

void Sensors::writeAccelerationData()
{
  // Time
  accelerationLog << dcmTimeSinceStart << ", ";
  for (int i = 0; i < J_NUM_JOINTS; i++)
  {
    accelerationLog << jointAccelerations[i] << ", ";
  }
  accelerationLog << endl;
  return;
}

void Sensors::writeCurrentData()
{
  // Time
  currentLog << dcmTimeSinceStart << ", ";
  for (int i = 0; i < J_NUM_JOINTS; i++)
  {
    currentLog << jointCurrents[i] << ", ";
  }
  currentLog << endl;
  return;
}

void Sensors::writeTargetData()
{
  targetLog << dcmTimeSinceStart << ", ";
  for (int i = 0; i < J_NUM_JOINTS; i++)
  {
    targetLog << jointTargets[i] << ", ";
  }
  targetLog << endl;
  return;
}

void Sensors::writeHardnessData()
{
    hardnessLog << dcmTimeSinceStart << ", ";
    for (int i = 0; i < J_NUM_JOINTS; i++)
    {
        hardnessLog << jointHardnesses[i] << ", ";
    }
    hardnessLog << endl;
    return;
}

void Sensors::writeBalanceData()
{
  balanceLog << dcmTimeSinceStart << ", ";
  for (int i = 0; i < B_NUM_SENSORS; i++)
  {
    balanceLog << balanceValues[i] << ", ";
  }
  // write the fall control feedback values
  balanceLog << balanceFalling << ", " << balanceFallen << ", ";
  balanceLog << endl; 
  return;
}

void Sensors::writeTouchData()
{
  touchLog << dcmTimeSinceStart << ", ";
  touchLog << touchLeftCoPX << ", " << touchLeftCoPY << ", " << touchRightCoPX << ", " << touchRightCoPY << ", ";    
  for (int i = 0; i < T_NUM_SENSORS; i++)
  {
    touchLog << touchValues[i] << ", ";
  }
  touchLog << endl; 
  return;
}

void Sensors::writeTemperatureData()
{
  temperatureLog << dcmTimeSinceStart << ", ";
  for (int i = 0; i < J_NUM_JOINTS; i++)
  {
    temperatureLog << jointTemperatures[i] << ", ";
  }
  temperatureLog << endl;
  return;
}

void Sensors::writeBatteryData()
{
  batteryLog << dcmTimeSinceStart << ", ";
  for (int i = 0; i < E_NUM_SENSORS; i++)
  {
    batteryLog << batteryValues[i] << ", ";
  }
  batteryLog << endl;
  return;
}

void Sensors::writeDistanceData()
{
    distanceLog << dcmTimeSinceStart << ", ";
    for (int i = 0; i < S_NUM_SENSORS; i++)
    {
        distanceLog << sonicValues[i] << ", ";
    }
    distanceLog << endl;
    return;
}

void Sensors::writeJointLabels(ofstream &logfile)
{    
    logfile << "Time (ms), ";
    logfile << "HEAD_YAW, ";
    logfile << "HEAD_PITCH, ";
    logfile << "L_SHOULDER_ROLL, ";
    logfile << "L_SHOULDER_PITCH, ";
    logfile << "L_ELBOW_YAW, ";
    logfile << "L_ELBOW_ROLL, ";
    logfile << "R_SHOULDER_ROLL, ";
    logfile << "R_SHOULDER_PITCH, ";
    logfile << "R_ELBOW_YAW, ";
    logfile << "R_ELBOW_ROLL, ";
    logfile << "L_HIP_YAW_PITCH, ";
    logfile << "L_HIP_ROLL, ";
    logfile << "L_HIP_PITCH, ";
    logfile << "L_KNEE_PITCH, ";
    logfile << "L_ANKLE_PITCH, ";
    logfile << "L_ANKLE_ROLL, ";
    logfile << "R_HIP_YAW_PITCH, ";
    logfile << "R_HIP_ROLL, ";
    logfile << "R_HIP_PITCH, ";
    logfile << "R_KNEE_PITCH, ";
    logfile << "R_ANKLE_PITCH, ";
    logfile << "R_ANKLE_ROLL, ";
    logfile << endl;
}

void Sensors::writeBalanceLabels()
{
    balanceLog << "Time (ms), ";
    balanceLog << "B_ACCEL_X, ";
    balanceLog << "B_ACCEL_Y, ";
    balanceLog << "B_ACCEL_Z, ";
    balanceLog << "B_ANGLE_X, ";
    balanceLog << "B_ANGLE_Y, ";
    balanceLog << "B_GYRO_X, ";
    balanceLog << "B_GYRO_Y, ";
    balanceLog << "balanceFalling, ";
    balanceLog << "balanceFallen, ";
    balanceLog << endl;
    return;
}

void Sensors::writeTouchLabels()
{
    touchLog << "Time (ms), ";
    touchLog << "LeftCoPX (cm)";
    touchLog << "LeftCoPY (cm)";
    touchLog << "RightCoPX (cm)";
    touchLog << "RightCoPY (cm)";
    touchLog << "T_L_FSR_FL, ";
    touchLog << "T_L_FSR_FR, ";
    touchLog << "T_L_FSR_BL, ";
    touchLog << "T_L_FSR_BR, ";
    touchLog << "T_L_BUMP_L, ";
    touchLog << "T_L_BUMP_R, ";
    touchLog << "T_R_FSR_FL, ";
    touchLog << "T_R_FSR_FR, ";
    touchLog << "T_R_FSR_BL, ";
    touchLog << "T_R_FSR_BR, ";
    touchLog << "T_R_BUMP_L, ";
    touchLog << "T_R_BUMP_R, ";
    touchLog << "T_CHEST_BUTTON, ";
    touchLog << endl;
    return;
}

void Sensors::writeBatteryLabels()
{
    batteryLog << "Time (ms), ";
    batteryLog << "E_CHARGE, ";
    batteryLog << "E_CURRENT, ";
    batteryLog << "E_VOLTAGE_MIN, ";
    batteryLog << "E_VOLTAGE_MAX, ";
    batteryLog << "E_TEMPERATURE, ";
    batteryLog << endl;
    return;
}

void Sensors::writeDistanceLabels()
{
    distanceLog << "Time (ms), ";
    distanceLog << "S_LL, ";
    distanceLog << "S_LR, ";
    distanceLog << "S_RL, ";
    distanceLog << "S_RR, ";
    distanceLog << "I_L, ";
    distanceLog << "I_R, ";
    distanceLog << endl;
    return;
}

/* Sends a command to the aldcm which will trigger an ultrasonic transmission
 @param value       there are 4 transmitter/receiver combinations 0.0 = LL, 1.0 = LR, 2.0 = RL, 3.0 = RR (yes value is a float)
 */
void Sensors::sendUSCommandToDCM(float value)
{
    int dcmOffset = dcmTime;
    ALValue param;      // List of all parameters
    ALValue command;    // A single command.
    ALValue commands;   // The list of all commands
    
    // Initial parameters
    param.arrayPush("US/Actuator/Value");
    param.arrayPush(string("ClearAfter"));               
    
    command.arrayPush(value);
    command.arrayPush(0 + dcmOffset);
    
    commands.arrayPush(command);
    
    param.arrayPush(commands);
#if SENSOR_VERBOSITY > 2
    thelog << param.toString(AL::VerbosityMini) << endl;
#endif
    alDcm->callVoid("set", param);
    return;
}

/* Sleeps the thread it is called in for msec since pnextWakeTime (so that I predetermined frequency of thread execution can be acheived)
 @param pnextWakeTime       the wake thread wake time. Before function call this should be the previous time the thread was executed. After the call
                            it will be the current wake time
 @param msec                the sleep time in milliseconds
 */
void Sensors::sleepMSec(timespec* pnextWakeTime, int msec)
{
    pnextWakeTime->tv_nsec += 1e6*msec;
    if (pnextWakeTime->tv_nsec > 1e9)              // we need to be careful with the nanosecond clock overflowing...
    {
        pnextWakeTime->tv_sec += 1;
        pnextWakeTime->tv_nsec -= 1e9;
    }
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, pnextWakeTime, NULL);   
}

void* runSonicThread(void *arg)
{
    thelog << "Creating SonicThread" << endl;
    
    Sensors* sensors = (Sensors*) arg;
    
    struct timespec nextWakeTime;                       // The absolute time for the sonic thread to wake up
    clock_gettime(CLOCK_REALTIME, &nextWakeTime);       // Initialise the next wake time to be now
    
    while (true)
    {
        #if SENSOR_VERBOSITY > 2
            thelog << "SonicThread is running " << dcmTimeSinceStart << endl;
        #endif
        
        for (int i=0; i<S_NUM_SENSORS; i++)         // There are 4 sonic sensor combinations LL, LR, RL, RR; we will cycle them in that order
        {
            Sensors::sendUSCommandToDCM(i);                  // send command to sonic sensor to initiate reading
            Sensors::sleepMSec(&nextWakeTime, 300);          // wait for the reading 150ms
            
            sonicValues[i] = 100*distanceValues[D_US];
            
            /*sensors->historySonicValues[i][sensors->historysonicindex[i]] = 100*distanceValues[D_US];  
            sensors->historysonicindex[i] = (sensors->historysonicindex[i] + 1) % S_NUM_SENSORS;
            
            float sum = 0;
            for (unsigned char j=0; j<S_WINDOW_SIZE; j++)
            {
                sum += sensors->historySonicValues[i][j];
            }
            sonicValues[i] = sum/S_WINDOW_SIZE;*/
            
            // update sonar obstacle distance (it is just the min)
            unsigned char minindex = 0;
            float mindistance = 600;
            for (unsigned char i=0; i<S_NUM_SENSORS; i++)
            {
                if (sonicValues[i] < mindistance)
                    mindistance = sonicValues[i];
            }
            sonicObstacleDistance = mindistance;

            // export to ALMemory if desired
            #if SENSOR_EXPORT_TO_AL
                alStm->insertData("Jason/US/LL", sonicValues[S_LL], 0);
                alStm->insertData("Jason/US/LR", sonicValues[S_LR], 0);
                alStm->insertData("Jason/US/RL", sonicValues[S_RL], 0);
                alStm->insertData("Jason/US/RR", sonicValues[S_RR], 0);
                alStm->insertData("Jason/US/ObstacleDistance", sonicObstacleDistance, 0);
            #endif
        }
    }
    pthread_exit(NULL);
}
  
