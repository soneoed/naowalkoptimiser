/** A low level wrapper of the alDcm
 * @author Jason Kulk
 *
 * Version : $Id: actuators.cpp,v 1.4 2009/05/24 06:26:27 jason Exp $
 */

#include "jwalk.h"
#include "alproxies.h"
#include "actuators.h"

string indexToPositionActuator[] = {"HeadYaw/Position/Actuator/Value","HeadPitch/Position/Actuator/Value","LShoulderRoll/Position/Actuator/Value","LShoulderPitch/Position/Actuator/Value","LElbowYaw/Position/Actuator/Value","LElbowRoll/Position/Actuator/Value","RShoulderRoll/Position/Actuator/Value","RShoulderPitch/Position/Actuator/Value","RElbowYaw/Position/Actuator/Value","RElbowRoll/Position/Actuator/Value","LHipYawPitch/Position/Actuator/Value","LHipRoll/Position/Actuator/Value","LHipPitch/Position/Actuator/Value","LKneePitch/Position/Actuator/Value","LAnklePitch/Position/Actuator/Value","LAnkleRoll/Position/Actuator/Value","RHipYawPitch/Position/Actuator/Value","RHipRoll/Position/Actuator/Value","RHipPitch/Position/Actuator/Value","RKneePitch/Position/Actuator/Value","RAnklePitch/Position/Actuator/Value","RAnkleRoll/Position/Actuator/Value"};
string indexToHardnessActuator[] = {"HeadYaw/Hardness/Actuator/Value","HeadPitch/Hardness/Actuator/Value","LShoulderRoll/Hardness/Actuator/Value","LShoulderPitch/Hardness/Actuator/Value","LElbowYaw/Hardness/Actuator/Value","LElbowRoll/Hardness/Actuator/Value","RShoulderRoll/Hardness/Actuator/Value","RShoulderPitch/Hardness/Actuator/Value","RElbowYaw/Hardness/Actuator/Value","RElbowRoll/Hardness/Actuator/Value","LHipYawPitch/Hardness/Actuator/Value","LHipRoll/Hardness/Actuator/Value","LHipPitch/Hardness/Actuator/Value","LKneePitch/Hardness/Actuator/Value","LAnklePitch/Hardness/Actuator/Value","LAnkleRoll/Hardness/Actuator/Value","RHipYawPitch/Hardness/Actuator/Value","RHipRoll/Hardness/Actuator/Value","RHipPitch/Hardness/Actuator/Value","RKneePitch/Hardness/Actuator/Value","RAnklePitch/Hardness/Actuator/Value","RAnkleRoll/Hardness/Actuator/Value"};
float jointLimits[][2] = {/*HeadYaw*/{-1.5707963705062866, 1.5707963705062866}, /*HeadPitch*/{-0.78539818525314331, 0.78539818525314331}, /*LShoulderRoll*/{0.0, 1.6580628156661987}, /*LShoulderPitch*/{-2.0943951606750488, 2.0943951606750488}, /*LElbowYaw*/{-2.0943951606750488, 2.0943951606750488}, /*LElbowRoll*/{-1.6580628156661987, 0.0}, /*RShoulderRoll*/{-1.6580628156661987, 0.0}, /*RShoulderPitch*/{-2.0943951606750488, 2.0943951606750488}, /*RElbowYaw*/{-2.0943951606750488, 2.0943951606750488}, /*RElbowRoll*/{0.0, 1.6580628156661987}, /*LHipYawPitch*/{-0.95993107557296753, 0.69813168048858643}, /*LHipRoll*/{-0.43633231520652771, 0.78539818525314331}, /*LHipPitch*/{-1.5707963705062866, 0.52359879016876221}, /*LKneePitch*/{0.0, 2.268928050994873}, /*LAnklePitch*/{-1.2217304706573486, 0.78539818525314331}, /*LAnkleRoll*/{-0.78539818525314331, 0.78539818525314331}, /*RHipYawPitch*/{-0.95993107557296753, 0.69813168048858643}, /*RHipRoll*/{-0.78539818525314331, 0.43633231520652771}, /*RHipPitch*/{-1.5707963705062866, 0.52359879016876221}, /*RKneePitch*/{0.0, 2.268928050994873}, /*RAnklePitch*/{-1.2217304706573486, 0.78539818525314331}, /*RAnkleRoll*/{-0.78539818525314331, 0.78539818525314331}};

float actuatorHardnesses[J_NUM_JOINTS];
float actuatorPositions[J_NUM_JOINTS];

float Actuators::MaxBatteryVoltage = 24654.0;

Actuators::Actuators()
{
    #if (ACTUATORS_VERBOSITY > 0)
        thelog << "ACTUATORS: Constructing actuators" << endl;
    #endif
    for (unsigned char i=0; i<J_NUM_JOINTS; i++)
    {
        actuatorHardnesses[i] = 0;
        actuatorPositions[i] = jointPositions[i];       // initialise the target position to be the initial position
        thelog << actuatorPositions[i] << ", ";
    }
    thelog << endl;
    createHardnessAliases();
    createTargetAliases();
}

/*! Creates aliases for hardness actuators to be used simultaneously.
 The following aliases are created:
 - every hardness actuator (ALIAS_HARDNESS_ALL)
 - every hardness actuator except the head (ALIAS_HARDNESS_NOT_HEAD)

 */
void Actuators::createHardnessAliases()
{
    #if (ACTUATORS_VERBOSITY > 2)
        thelog << "ACTUATORS: createHardnessAliases() Creating Hardness Aliases." << endl;
    #endif
    ALValue param;
    ALValue names;                  // The actuator names to be under this alias
    ALValue result;
    
    // Firstly, the alias for all
    param.arrayPush(ALIAS_HARDNESS_ALL);
    for (int i=0; i<J_NUM_JOINTS; i++)
    {
        names.arrayPush(indexToHardnessActuator[i]);
    }
    param.arrayPush(names);
    
    result = alDcm->call<ALValue>("createAlias", param);
    #if (ACTUATORS_VERBOSITY > 1)
        thelog << "ACTUATORS: createHardnessAliases(): " << result.toString(AL::VerbosityMini) << endl;
    #endif
    
    // Secondly, the alias for all but the head
    param.clear();
    names.clear();
    result.clear();
    param.arrayPush(ALIAS_HARDNESS_NOT_HEAD);
    for (int i=2; i<J_NUM_JOINTS; i++)                          // CAUTION: This relies on the head being the first two joints!!!!
    {
        names.arrayPush(indexToHardnessActuator[i]);
    }
    param.arrayPush(names);
    
    result = alDcm->call<ALValue>("createAlias", param);
    #if (ACTUATORS_VERBOSITY > 1)
        thelog << "ACTUATORS: createHardnessAliases(): " << result.toString(AL::VerbosityMini) << endl;
    #endif
    
    // Thirdly, the alias for just the head
    param.clear();
    names.clear();
    result.clear();
    param.arrayPush(ALIAS_HARDNESS_HEAD);
    for (int i=0; i<ALIAS_HARDNESS_HEAD_LENGTH; i++)
    {
        names.arrayPush(indexToHardnessActuator[i]);
    }
    param.arrayPush(names);
    
    result = alDcm->call<ALValue>("createAlias", param);
#if (ACTUATORS_VERBOSITY > 1)
    thelog << "ACTUATORS: createHardnessAliases(): " << result.toString(AL::VerbosityMini) << endl;
#endif
    
    return;
}

/*! Creates aliases for position actuators to be used simultaneously.
 The following aliases are created:
 - every position actuator (ALIAS_TARGETS_ALL)
 - every position actuator except the head (ALIAS_TARGETS_NOT_HEAD)
 */
void Actuators::createTargetAliases()
{
    #if (ACTUATORS_VERBOSITY > 2)
        thelog << "ACTUATORS: createTargetAliases() Creating Target Aliases." << endl;
    #endif
    ALValue param;
    ALValue names;                  // The actuator names to be under this alias
    ALValue result;
    
    // Firstly, the alias for all
    param.arrayPush(ALIAS_TARGETS_ALL);
    for (int i=0; i<J_NUM_JOINTS; i++)
    {
        names.arrayPush(indexToPositionActuator[i]);
    }
    param.arrayPush(names);
    
    result = alDcm->call<ALValue>("createAlias", param);
    #if (ACTUATORS_VERBOSITY > 1)
        thelog << "ACTUATORS: createTargetAliases(): " << result.toString(AL::VerbosityMini) << endl;
    #endif
    
    // Secondly, the alias for all but the head
    param.clear();
    names.clear();
    result.clear();
    param.arrayPush(ALIAS_TARGETS_NOT_HEAD);
    for (int i=2; i<J_NUM_JOINTS; i++)                          // CAUTION: This relies on the head being the first two joints!!!!
    {
        names.arrayPush(indexToPositionActuator[i]);
    }
    param.arrayPush(names);
    
    result = alDcm->call<ALValue>("createAlias", param);
    #if (ACTUATORS_VERBOSITY > 1)
        thelog << "ACTUATORS: createHardnessAliases(): " << result.toString(AL::VerbosityMini) << endl;
    #endif
    
    // Thirdly, the alias for just the head
    param.clear();
    names.clear();
    result.clear();
    param.arrayPush(ALIAS_TARGETS_HEAD);
    for (int i=0; i<ALIAS_TARGETS_HEAD_LENGTH; i++)
    {
        names.arrayPush(indexToHardnessActuator[i]);
    }
    param.arrayPush(names);
    
    result = alDcm->call<ALValue>("createAlias", param);
#if (ACTUATORS_VERBOSITY > 1)
    thelog << "ACTUATORS: createHardnessAliases(): " << result.toString(AL::VerbosityMini) << endl;
#endif
    return;
}

Actuators::~Actuators()
{
}

/*! Specify a new joint for a single joint
 @param jointindex          the index of the joint eg. J_L_ANKLE_PITCH
 @param position            the new target angle
 @param time                the time to get to the target position
 
 Note. that successive calls to this function can make a 'motion curve' because existing dcm commands are only erased if they are AFTER the new command.
 So goToAngle(0, 0, 100) and goToAngle(0, -1, 300) will make the head goto 0 and then to -1
 */
int Actuators::goToAngle(int jointindex, float position, int time)
{
    sendToActuator(indexToPositionActuator[jointindex], time, position);
    return dcmTime + time;
}

/*! Go to the new positions at the specified velocity (the aldcm does the interpolation)
 @param positions[]         the new target positions (must be of length ALIAS_TARGETS_NOT_HEAD_LENGTH)
 @param time                the time in ms from now to reach the desired positions
 
 @return finishTime         the dcmTime the motion will finish. I stress that this is the dcmTime; the dcmTime the motion will finish; the dcmTime.
 */
int Actuators::goToAnglesNotHead(float positions[], int time)
{
    sendToAliasActuator(ALIAS_TARGETS_NOT_HEAD, time, positions, ALIAS_TARGETS_NOT_HEAD_LENGTH);
    return time + dcmTime;
}

/*! Go to the new positions at the specified velocity (the aldcm does the interpolation)
 @param positions[]         the new target positions (must be of length ALIAS_TARGETS_NOT_HEAD_LENGTH)
 @param velocity            the velocity in rad/s
 
 @return finishTime         the dcmTime the motion will finish. I stress that this is the dcmTime; the dcmTime the motion will finish; the dcmTime.
 */
int Actuators::goToAnglesWithVelocityNotHead(float positions[], float velocity)
{
    int times[ALIAS_TARGETS_NOT_HEAD_LENGTH];
    int maxtime = 0;
    for (unsigned char i=0; i<ALIAS_TARGETS_NOT_HEAD_LENGTH; i++)
    {
        if ((i+2 == J_L_KNEE_PITCH) || (i+2 == J_R_KNEE_PITCH))         // if the joint is a knee it needs to be moved twice as fast! 
            times[i] = (int)1000*(fabs(jointPositions[i+2] - positions[i])/(2*velocity));
        else
            times[i] = (int)1000*(fabs(jointPositions[i+2] - positions[i])/velocity);
        
        if (times[i] > maxtime)
            maxtime = times[i];
    }
    sendToAliasActuator(ALIAS_TARGETS_NOT_HEAD, times, positions, ALIAS_TARGETS_NOT_HEAD_LENGTH);
    return maxtime + dcmTime;
}

/*! Go to the new positions at the specified velocity (the aldcm does the interpolation)
 @param positions[]         the new target positions (must be of length ALIAS_TARGETS_ALL_LENGTH)
 @param time                the time in ms from now to reach the desired positions
 
 @return finishTime         the dcmTime the motion will finish. I stress that this is the dcmTime; the dcmTime the motion will finish; the dcmTime.
 */
int Actuators::goToAnglesAll(float positions[], int time)
{
    sendToAliasActuator(ALIAS_TARGETS_ALL, time, positions, ALIAS_TARGETS_ALL_LENGTH);
    return time + dcmTime;
}

/*! Go to the new positions at the specified velocity (the aldcm does the interpolation)
 @param positions[]         the new target positions (must be of length ALIAS_TARGETS_ALL_LENGTH)
 @param velocity            the velocity in rad/s
 
 @return finishTime         the dcmTime the motion will finish. I stress that this is the dcmTime; the dcmTime the motion will finish; the dcmTime.
 */
int Actuators::goToAnglesWithVelocityAll(float positions[], float velocity)
{
    int times[ALIAS_TARGETS_ALL_LENGTH];
    int maxtime = 0;
    for (unsigned char i=0; i<ALIAS_TARGETS_ALL_LENGTH; i++)
    {
        if ((i == J_L_KNEE_PITCH) || (i == J_R_KNEE_PITCH))         // if the joint is a knee it needs to be moved twice as fast! 
            times[i] = (int)1000*(fabs(jointPositions[i] - positions[i])/(2*velocity));
        else
            times[i] = (int)1000*(fabs(jointPositions[i] - positions[i])/velocity);
        
        if (times[i] > maxtime)
            maxtime = times[i];
    }
    sendToAliasActuator(ALIAS_TARGETS_ALL, times, positions, ALIAS_TARGETS_ALL_LENGTH);
    return maxtime + dcmTime;
}

/*! Go to the new head yaw position
 @param position            the new position in radians
 @param time                the time to get to that position in milliseconds
 
 @return finishtime         the dcmTime the motion will finish. I stress that this is the dcmTime; the dcmTime the motion will finish; the dcmTime.
 */
int Actuators::goToAngleHeadYaw(float position, int time)
{
    sendToActuator(indexToPositionActuator[0], time, position);
    return dcmTime + time;
}

/*! Go to the new head pitch position
 @param position            the new position in radians
 @param time                the time to get to that position in milliseconds
 
 @return finishtime         the dcmTime the motion will finish. I stress that this is the dcmTime; the dcmTime the motion will finish; the dcmTime.
 */
int Actuators::goToAngleHeadPitch(float position, int time)
{
    sendToActuator(indexToPositionActuator[1], time, position);
    return dcmTime + time;
}


/********************************************************************************************************************************************************************
 Voltage stablised hardness settings
 ********************************************************************************************************************************************************************/

/* Adjusts the hardness; correcting for changes in voltage, and asymmetries between left and right
 @param hardness            the hardness value that needs to be adjusted
 @param adjustedhardness    the new hardness value after adjustement
 */
void Actuators::adjustHardness(float hardness, float* adjustedhardness)
{
    // First correct all hardnesses for battery voltage:
    short batteryvoltage = 3*(batteryValues[E_VOLTAGE_MIN] + batteryValues[E_VOLTAGE_MAX]);         // the battery voltage in mV
    (*adjustedhardness) = hardness*(Actuators::MaxBatteryVoltage/batteryvoltage);
    
    return;
}

/* Adjusts the hardnesses; correcting for changes in voltage, and asymmetries between left and right
 @param hardnesses          the hardnesses to be adjusted (this array can not be modified, because it needs to remain at the original/uncorrected value for future iterations)
 @param adjustedhardnesses  an array whose values will be updated with the adjusted hardnesses values
 @param numactuators        the length of hardnesses
 */
void Actuators::adjustHardnesses(float hardnesses[], float adjustedhardnesses[], unsigned char numactuators)
{
    // First correct all hardnesses for battery voltage:
    short batteryvoltage = 3*(batteryValues[E_VOLTAGE_MIN] + batteryValues[E_VOLTAGE_MAX]);         // the battery voltage in mV
    for (unsigned char i=0; i<numactuators; i++)
        adjustedhardnesses[i] = hardnesses[i]*(Actuators::MaxBatteryVoltage/batteryvoltage);
    
    // Now to do left and right compensation I need to know which indices are left and right
    if (numactuators == ALIAS_TARGETS_NOT_HEAD_LENGTH)
    {
    }
    else if (numactuators == ALIAS_TARGETS_ALL_LENGTH)
    {
    }
    return;
}

/*! Set the stiffness (hardness) for a single joint
 @param jointindex      the index of the joint to change the stiffness for (eg. J_L_ANKLE_PITCH)
 @param value           the new hardness value
 */
void Actuators::setStiffness(int jointindex, float value)
{
    float adjustedvalue = 0;
    adjustHardness(value, &adjustedvalue);
    sendToActuator(indexToHardnessActuator[jointindex], 0, adjustedvalue);
}

/*! Set the stiffness (hardness) for the two head joints
 @param values[2]       the new stiffness values
 */
void Actuators::setStiffnessHead(float values[])
{
    static float adjustedvalues[ALIAS_HARDNESS_HEAD_LENGTH];
    adjustHardnesses(values, adjustedvalues, ALIAS_HARDNESS_HEAD_LENGTH);
    sendToAliasActuator(ALIAS_HARDNESS_HEAD, 0, adjustedvalues, ALIAS_HARDNESS_HEAD_LENGTH);
}

/*! Set the stiffness for every joint
 @param values[]        the new hardness values (must be length == ALIAS_HARDNESS_ALL_LENGTH)
 */
void Actuators::setStiffnessAll(float values[])
{
    setStiffnessAll(values, 0);
}

int Actuators::setStiffnessAll(float values[], int time)
{
    static float adjustedvalues[ALIAS_HARDNESS_ALL_LENGTH];
    adjustHardnesses(values, adjustedvalues, ALIAS_HARDNESS_ALL_LENGTH);
    sendToAliasActuator(ALIAS_HARDNESS_ALL, time, adjustedvalues, ALIAS_HARDNESS_ALL_LENGTH);
    return dcmTime + time;
}

/*! Set the stiffness for every joint
 @param values[]        the new hardness values (must be length == ALIAS_HARDNESS_NOT_HEAD_LENGTH)
 */
void Actuators::setStiffnessNotHead(float values[])
{
    static float adjustedvalues[ALIAS_HARDNESS_NOT_HEAD_LENGTH];
    adjustHardnesses(values, adjustedvalues, ALIAS_HARDNESS_NOT_HEAD_LENGTH);
    sendToAliasActuator(ALIAS_HARDNESS_NOT_HEAD, 0, adjustedvalues, ALIAS_HARDNESS_NOT_HEAD_LENGTH);
}

/********************************************************************************************************************************************************************
 Communication with DCM
 ********************************************************************************************************************************************************************/
/*! Clears the targets saved in the dcm
 */
void Actuators::clearDCM()
{
#if (ACTUATORS_VERBOSITY > 2)
    thelog << "ACTUATORS: clearDCM()." << endl;
#endif
    int dcmOffset = dcmTime;
    ALValue param;                          // List of all parameters
    param.arraySetSize(6);                  // The number of parameters for this mode is always 6
    
    ALValue actuatorcommands;               // The list of all commands for a single actuator
    actuatorcommands.arraySetSize(1);       // Here every actuator gets only a single value
    
    ALValue allcommands;                    // The list of all commands for every actuator
    allcommands.arraySetSize(ALIAS_TARGETS_ALL_LENGTH);
    
    ALValue alltimes;                       // The list of all times that every actuator will get a new value
    alltimes.arraySetSize(1);
    
    // Initial parameters
    param[0] = ALIAS_TARGETS_ALL;
    param[1] = string("ClearAll"); 
    param[2] = string("time-separate");              // This mode requires every joint under the alias be specified values at the same time
    param[3] = 0;
    
    // Create time vector
    alltimes[0] = 20 + dcmOffset;
    param[4] = alltimes;
    
    // Create actuator vectors
    for (int i=0; i<ALIAS_TARGETS_ALL_LENGTH; i++)
    {
        actuatorcommands[0] = jointPositions[i];
        allcommands[i] = actuatorcommands;
    }
    
    param[5] = allcommands;
    
#if (ACTUATORS_VERBOSITY > 1)
    thelog << "ACTUATORS: clearDCM() " << param.toString(AL::VerbosityMini) << endl;
#endif
    alDcm->callVoid("setAlias", param);
    return;
}


/*! Send an entire motion frame to the whole body
 
 @param positions[]         the target joint positions for the next dcm cycle (must have length == ALIAS_TARGETS_ALL_LENGTH)
 @param hardnesses[]        the joint hardnesses for the next dcm cycle (must have length == ALIAS_TARGETS_ALL_LENGTH)
 */
void Actuators::sendFrameToBody(float positions[], float hardnesses[])
{
    sendToAliasActuator(ALIAS_TARGETS_ALL, 0, positions, ALIAS_TARGETS_ALL_LENGTH);
    
    static float adjustedhardnesses[ALIAS_HARDNESS_ALL_LENGTH];
    adjustHardnesses(hardnesses, adjustedhardnesses, ALIAS_HARDNESS_ALL_LENGTH);
    sendToAliasActuator(ALIAS_HARDNESS_ALL, 0, adjustedhardnesses, ALIAS_HARDNESS_ALL_LENGTH);
}

/*! Send an entire motion frame to the every joint except the head; the head is for Steve :D
 
 @param positions[]         the target joint positions for the next dcm cycle (must have length == ALIAS_TARGETS_NOT_HEAD_LENGTH)
 @param hardnesses[]        the joint hardnesses for the next dcm cycle (must have length == ALIAS_TARGETS_NOT_HEAD_LENGTH)
 */
void Actuators::sendFrameToNotHead(float positions[], float hardnesses[])
{
    sendToAliasActuator(ALIAS_TARGETS_NOT_HEAD, 0, positions, ALIAS_TARGETS_NOT_HEAD_LENGTH);

    static float adjustedhardnesses[ALIAS_HARDNESS_NOT_HEAD_LENGTH];
    adjustHardnesses(hardnesses, adjustedhardnesses, ALIAS_HARDNESS_NOT_HEAD_LENGTH);  
    sendToAliasActuator(ALIAS_HARDNESS_NOT_HEAD, 0, adjustedhardnesses, ALIAS_HARDNESS_NOT_HEAD_LENGTH);
}

/*! Sends a float value to the specified actuator using the alDcm
 
 @param actuatorname     the aldebaran string name of the actuator (eg. LHipRoll/Hardness/Actuator/Value)
 @param time             the time, from now, in ms to apply the command (Note. The DCM will interpolate the command)
 @param value            the (float) value to be sent to the actuator
 */
void Actuators::sendToActuator(string actuatorname, int time, float value)
{
#if (ACTUATORS_VERBOSITY > 2)
    thelog << "ACTUATORS: sendToActuator() Sending Float Command to DCM" << endl;
#endif
    ALValue param;      // List of all parameters
    ALValue command;    // A single command.
    ALValue commands;   // The list of all commands
    
    // Initial parameters
    param.arrayPush(actuatorname);
#if ACTUATORS_CLEARAFTER == 1
    int dcmOffset = dcmTime + 20;
    param.arrayPush(string("ClearAfter"));                  // be very careful using 'Merge' --- it can very easily result in vibrations
                                                            // be careful using 'ClearAll' --- it is suceptiable to precision errors in dcmTime
#else
    int dcmOffset = dcmTime;
    param.arrayPush(string("ClearAll"));
#endif
    
    command.arrayPush(value);
    command.arrayPush(time + dcmOffset);
    
    commands.arrayPush(command);
    
    param.arrayPush(commands);
#if (ACTUATORS_VERBOSITY > 1)
    thelog << "ACTUATORS: sendToActuator() Actual dcmTime " << (int)alDcm->call<int>("getTime",0) << endl; 
    thelog << "ACTUATORS: sendToActuator() " << param.toString(AL::VerbosityMini) << endl;
#endif
    alDcm->callVoid("set", param);
    return;
}

/*! Sends a single command to each actuator under the specified alias. The time is the same for each command
 
 @param aliasname       the numotion string name for the alias (eg. NUmotion/Hardness/All)
 @param time            the time in ms to apply the single command for each actuator (Note. The DCM will interpolate the command if it is non-zero)
 @param values[]        a float array of a value to be sent to each actuator under the alias
 @param numactuators    the length of the values array, ie. the number of actuators under the alias
 */
void Actuators::sendToAliasActuator(string aliasname, int time, float values[], int numactuators)
{
#if (ACTUATORS_VERBOSITY > 2)
    thelog << "ACTUATORS: sendToAliasActuator() Sending Alias Command to DCM" << endl;
#endif
    ALValue param;                          // List of all parameters
    param.arraySetSize(6);                  // The number of parameters for this mode is always 6
    
    ALValue actuatorcommands;               // The list of all commands for a single actuator
    actuatorcommands.arraySetSize(1);       // Here every actuator gets only a single value
    
    ALValue allcommands;                    // The list of all commands for every actuator
    allcommands.arraySetSize(numactuators);
    
    ALValue alltimes;                       // The list of all times that every actuator will get a new value
    alltimes.arraySetSize(1);
    
    // Initial parameters
    param[0] = aliasname;
    
#if ACTUATORS_CLEARAFTER == 1
    int dcmOffset = dcmTime + 40;
    param[1] = string("ClearAfter");
#else
    int dcmOffset = dcmTime;
    param[1] = string("ClearAll");
#endif
    
    param[2] = string("time-separate");              // This mode requires every joint under the alias be specified values at the same time
    param[3] = 0;
    
    // Create time vector
    alltimes[0] = time + dcmOffset;
    param[4] = alltimes;
    
    // Create actuator vectors
    for (int i=0; i<numactuators; i++)
    {
        actuatorcommands[0] = values[i];
        allcommands[i] = actuatorcommands;
    }
    
    param[5] = allcommands;
    
#if (ACTUATORS_VERBOSITY > 1)
    thelog << "ACTUATORS: sendToAliasActuator() Actual dcmTime " << (int)alDcm->call<int>("getTime",0) << endl; 
    thelog << "ACTUATORS: sendToAliasActuator() " << param.toString(AL::VerbosityMini) << endl;
#endif
    alDcm->callVoid("setAlias", param);
    return;
}

/*! Sends a single command to each actuator under the specified alias. The time is the DIFFERENT for each command
 
 @param aliasname       the numotion string name for the alias (eg. NUmotion/Hardness/All)
 @param times[]         the times in ms to apply the single command for each actuator (Note. The DCM will interpolate the command if it is non-zero)
 @param values[]        a float array of a value to be sent to each actuator under the alias
 @param numactuators    the length of the values array, ie. the number of actuators under the alias
 */
void Actuators::sendToAliasActuator(string aliasname, int times[], float values[], int numactuators)
{
#if (ACTUATORS_VERBOSITY > 2)
    thelog << "ACTUATORS: sendToAliasActuator() Sending Alias Command to DCM" << endl;
#endif
    ALValue param;                          // List of all parameters
    param.arraySetSize(4);                  // The number of parameters for this time-mixed mode is always 4
    
    ALValue command;                        // The command for a single actuator
    command.arraySetSize(3);                // For time-mixed mode the command is [target, time, 0]
    
    ALValue actuatorcommands;               // The list of all commands for a single actuator
    actuatorcommands.arraySetSize(1);       // Here every actuator gets a single command and a time [command, time]
    
    ALValue allcommands;                    // The list of all commands for every actuator
    allcommands.arraySetSize(numactuators);
    
    // Initial parameters
    param[0] = aliasname;
    
#if ACTUATORS_CLEARAFTER == 1
    int dcmOffset = dcmTime + 40;
    param[1] = string("ClearAfter");
#else
    int dcmOffset = dcmTime;
    param[1] = string("ClearAll");
#endif
    
    param[2] = string("time-mixed");        // This mode allows each actuator to receive a command with a different time
    
    // Create actuator vectors
    for (int i=0; i<numactuators; i++)
    {
        command[0] = values[i];                // the command for the actuator
        command[1] = times[i] + dcmOffset;     // the time for the command         
        command[2] = 0;                        // the importance level for the command
        actuatorcommands[0] = command;
        allcommands[i] = actuatorcommands;
    }
    
    param[3] = allcommands;
    
#if (ACTUATORS_VERBOSITY > 1)
    thelog << "ACTUATORS: sendToAliasActuator() Actual dcmTime " << (int)alDcm->call<int>("getTime",0) << endl; 
    thelog << "ACTUATORS: sendToAliasActuator() " << param.toString(AL::VerbosityMini) << endl;
#endif
    alDcm->callVoid("setAlias", param);
    return;
}

/*! Sends a sequence of commands to a single actuator
 WARNING: Untested/Unused function!
 @param actuatorname    the aldebaran string name of the actuator (eg. LHipRoll/Hardness/Actuator/Value)
 @param starttime       the start time in ms (from now) at which point to start applying the curve (Unfortunatly, if this is non-zero the DCM will interpolate the first command) 
 @param values[]        a float array of values to be sent to the actuator at each dcm cycle
 @param numvalues       the length of the values array
 */
void Actuators::sendCurveToActuator(string actuatorname, int starttime, float value[], int numvalues)
{
#if (ACTUATORS_VERBOSITY > 2)
    thelog << "ACTUATORS: sendCurveToActuator() Sending float curve to alDcm" << endl;
#endif
    for (int i=0; i<numvalues; i++)
    {
        sendToActuator(actuatorname, starttime + i*ALDCM_CYCLETIME, value[i]);
    }
}

/*! Sends a sequence of command to each actuator under the specified alias. The time each member of the sequence is sent is the same for each actuator under the alias.
 
 Essentially, this function is used to send a motion curve for the entire body to the dcm, all at once. The motion curve (values[][]) specifies new target positions for each
 of the (numvalues) subsequent dcm cycles.
 
 @param aliasname       the numotion string name for the alias (eg. NUmotion/Hardness/All)
 @param starttime       the start time in ms (from now) at which point to start applying the curve. Unfortunatly, if this is non-zero the DCM will interpolate the first command :( 
 @param values[][]      a float matrix of a values to be sent to each actuator at each cycle under the alias
 @param numvalues       the number of rows in the values matrix, ie the number of future dcm cycles worth of motion in values[][]
 */
void Actuators::sendCurveToAliasActuator(string aliasname, int starttime, float values[/*numvalues*/][ALIAS_TARGETS_ALL_LENGTH], int numvalues)
{
#if (ACTUATORS_VERBOSITY > 2)
    thelog << "ACTUATORS: sendCurveToAliasActuator() Sending curve to alDcm" << endl;
#endif
    ALValue param;                          // List of all parameters
    param.arraySetSize(6);                  // The number of parameters for this mode is always 6
    
    ALValue actuatorcommands;                       // The list of all commands for a single actuator
    actuatorcommands.arraySetSize(numvalues);       // Here every actuator gets numvalues commands
    
    ALValue allcommands;                    // The list of all commands for every actuator
    allcommands.arraySetSize(ALIAS_TARGETS_ALL_LENGTH);
    
    ALValue alltimes;                       // The list of all times that every actuator will get a new value
    alltimes.arraySetSize(numvalues);
    
    // Initial parameters
    param[0] = aliasname;
    
#if ACTUATORS_CLEARAFTER == 1
    int dcmOffset = dcmTime + 20;
    param[1] = string("ClearAfter");
#else
    int dcmOffset = dcmTime;
    param[1] = string("ClearAll");
#endif
    
    param[2] = string("time-separate");              // This mode requires every joint under the alias be specified values at the same times
    param[3] = 0;
    
    // Create time vector
    for (int i=0; i<numvalues; i++)
        alltimes[i] = starttime + i*ALDCM_CYCLETIME + dcmOffset;
    param[4] = alltimes;
    
    // Create actuator vectors
    for (int i=0; i<ALIAS_TARGETS_ALL_LENGTH; i++)              // for each actuator 
    {
        for (int j=0; j<numvalues; j++)              // make a list of each target 
        {
            actuatorcommands[j] = values[j][i];
        }
        allcommands[i] = actuatorcommands;
    }
    
    param[5] = allcommands;
    
#if (ACTUATORS_VERBOSITY > 1)
    thelog << "ACTUATORS: sendCurveToAliasActuator() Actual dcmTime " << (int)alDcm->call<int>("getTime",0) << endl; 
    thelog << "ACTUATORS: sendCurveToAliasActuator() " << param.toString(AL::VerbosityMini) << endl;
#endif
    alDcm->callVoid("setAlias", param);
    return;
}


/*! Sends a sequence of command to each actuator under the specified alias. The time each member of the sequence is sent is the same for each actuator under the alias.
 
 Essentially, this function is used to send a motion curve for the entire body to the dcm, all at once. The motion curve (values[][]) specifies new target positions for each
 of the (numvalues) subsequent dcm cycles.
 
 @param aliasname       the numotion string name for the alias (eg. NUmotion/Hardness/All)
 @param starttime       the start time in ms (from now) at which point to start applying the curve. Unfortunatly, if this is non-zero the DCM will interpolate the first command :( 
 @param values[][]      a float matrix of a values to be sent to each actuator at each cycle under the alias
 @param numvalues       the number of rows in the values matrix, ie the number of future dcm cycles worth of motion in values[][]
 */
void Actuators::sendCurveToAliasActuator(string aliasname, int starttime, float values[/*numvalues*/][ALIAS_TARGETS_NOT_HEAD_LENGTH], int numvalues)
{
#if (ACTUATORS_VERBOSITY > 2)
    thelog << "ACTUATORS: sendCurveToAliasActuator() Sending curve to alDcm" << endl;
#endif
    ALValue param;                          // List of all parameters
    param.arraySetSize(6);                  // The number of parameters for this mode is always 6
    
    ALValue actuatorcommands;                       // The list of all commands for a single actuator
    actuatorcommands.arraySetSize(numvalues);       // Here every actuator gets numvalues commands
    
    ALValue allcommands;                    // The list of all commands for every actuator
    allcommands.arraySetSize(ALIAS_TARGETS_NOT_HEAD_LENGTH);
    
    ALValue alltimes;                       // The list of all times that every actuator will get a new value
    alltimes.arraySetSize(numvalues);
    
    // Initial parameters
    param[0] = aliasname;
    
#if ACTUATORS_CLEARAFTER == 1
    int dcmOffset = dcmTime + 20;
    param[1] = string("ClearAfter");
#else
    int dcmOffset = dcmTime;
    param[1] = string("ClearAll");
#endif
    
    param[2] = string("time-separate");              // This mode requires every joint under the alias be specified values at the same times
    param[3] = 0;
    
    // Create time vector
    for (int i=0; i<numvalues; i++)
        alltimes[i] = starttime + i*ALDCM_CYCLETIME + dcmOffset;
    param[4] = alltimes;
    
    // Create actuator vectors
    for (int i=0; i<ALIAS_TARGETS_NOT_HEAD_LENGTH; i++)              // for each actuator 
    {
        for (int j=0; j<numvalues; j++)              // make a list of each target 
        {
            actuatorcommands[j] = values[j][i];
        }
        allcommands[i] = actuatorcommands;
    }
    
    param[5] = allcommands;
    
#if (ACTUATORS_VERBOSITY > 1)
    thelog << "ACTUATORS: sendCurveToAliasActuator() Actual dcmTime " << (int)alDcm->call<int>("getTime",0) << endl; 
    thelog << "ACTUATORS: sendCurveToAliasActuator() " << param.toString(AL::VerbosityMini) << endl;
#endif
    alDcm->callVoid("setAlias", param);
}

