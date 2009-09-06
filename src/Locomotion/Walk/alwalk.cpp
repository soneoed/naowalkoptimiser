/*
 *  alwalk.cpp
 *  jwalk
 *
 *  Created by jason on 6/04/09.
 *  Copyright 2009 UoN. All rights reserved.
 *
 */

#include "alwalk.h"

ALWalk::ALWalk()
{
#if ALWALK_VERBOSITY > 0
    thelog << "ALWALK: Initialising." << endl;
#endif
    
    readConfigs();
    initSelf();

#if ALWALK_VERBOSITY > 0
    thelog << "ALWALK: Initialised." << endl;
#endif
}

void ALWalk::initSelf()
{
    alWalkGoDirection = ALWALK_DIRECTION_UNDEFINED;     // we start with an undefined direction
    configALMotion(ALWALK_PRIMITIVE_FORWARD);
}

ALWalk::~ALWalk()
{
}

/********************************************************************************************************************************************************************
 Simple Movement Interface
 ********************************************************************************************************************************************************************/

void ALWalk::setStiffnessNotHead(float values[])
{
#if JWALK_ALMOTION
    static vector<float> larm (6, 0.0);
    static vector<float> rarm (6, 0.0);
    static vector<float> lleg (6, 0.0);
    static vector<float> rleg (6, 0.0);
    
    // Aldebaran's "LArm" chain:    "LShoulderPitch"    "LShoulderRoll"    "LElbowYaw"    "LElbowRoll"    "LWristYaw" = 0    "LHand" = 0
    // My order is:                 "LShoulderRoll",    "LShoulderPitch"   "LElbowYaw"    "LElbowRoll"
    // I can't be fucked changing 'my order' to match that of Aldebaran's, so...
    larm[0] = values[1];        // LShoulderPitch
    larm[1] = values[0];        // LShoulderRol
    larm[2] = values[2];        // LElbowYaw matches
    larm[3] = values[3];        // RElbowRoll matches
    // leave the rest at zero
    
    rarm[0] = values[1 + JOINT_OFFSET_RARM];    // RShoulderPitch
    rarm[1] = values[0 + JOINT_OFFSET_RARM];    // RShoulderRoll
    rarm[2] = values[2 + JOINT_OFFSET_RARM];    // RElbowYaw matches
    rarm[3] = values[3 + JOINT_OFFSET_RARM];
    // leave the rest at zero
    
    alMotion->post.gotoChainStiffnesses(string("LArm"), larm, 0.1, 0);
    alMotion->post.gotoChainStiffnesses(string("RArm"), rarm, 0.1, 0);
    
    // Aldebaran's "LLeg" chain    "LHipYawPitch"    "LHipRoll"    "LHipPitch"    "LKneePitch"    "LAnklePitch"    "LAnkleRoll"
    // My order is:                "LHipYawPitch"    "LHipRoll"    "LHipPitch"    "LKneePitch"    "LAnklePitch"    "LAnkleRoll"
    // So, they match, woot!
    for (unsigned char i=0; i<6; i++)
    {
        lleg[i] = values[i + JOINT_OFFSET_LLEG];
        rleg[i] = values[i + JOINT_OFFSET_RLEG];
    }
    
    alMotion->post.gotoChainStiffnesses(string("LLeg"), lleg, 0.1, 0);
    alMotion->post.gotoChainStiffnesses(string("RLeg"), rleg, 0.1, 0);
#endif
}

int ALWalk::goToAnglesNotHead(float positions[], int time)
{
#if JWALK_ALMOTION
    static vector<float> larm (6, 0.0);
    static vector<float> rarm (6, 0.0);
    static vector<float> lleg (6, 0.0);
    static vector<float> rleg (6, 0.0);
    
    // Aldebaran's "LArm" chain:    "LShoulderPitch"    "LShoulderRoll"    "LElbowYaw"    "LElbowRoll"    "LWristYaw" = 0    "LHand" = 0
    // My order is:                 "LShoulderRoll",    "LShoulderPitch"   "LElbowYaw"    "LElbowRoll"
    // I can't be fucked changing 'my order' to match that of Aldebaran's, so...
    larm[0] = positions[1];        // LShoulderPitch
    larm[1] = positions[0];        // LShoulderRol
    larm[2] = positions[2];        // LElbowYaw matches
    larm[3] = positions[3];        // RElbowRoll matches
    // leave the rest at zero
    
    rarm[0] = positions[1 + JOINT_OFFSET_RARM];    // RShoulderPitch
    rarm[1] = positions[0 + JOINT_OFFSET_RARM];    // RShoulderRoll
    rarm[2] = positions[2 + JOINT_OFFSET_RARM];    // RElbowYaw matches
    rarm[3] = positions[3 + JOINT_OFFSET_RARM];
    // leave the rest at zero
    
    // Aldebaran's "LLeg" chain    "LHipYawPitch"    "LHipRoll"    "LHipPitch"    "LKneePitch"    "LAnklePitch"    "LAnkleRoll"
    // My order is:                "LHipYawPitch"    "LHipRoll"    "LHipPitch"    "LKneePitch"    "LAnklePitch"    "LAnkleRoll"
    // So, they match woot!
    for (unsigned char i=0; i<6; i++)
    {
        lleg[i] = positions[i + JOINT_OFFSET_LLEG];
        rleg[i] = positions[i + JOINT_OFFSET_RLEG];
    }
    
    float seconds = time/1000.0;
    alMotion->post.gotoChainAngles(string("LArm"), larm, seconds, 1);
    alMotion->post.gotoChainAngles(string("RArm"), rarm, seconds, 1);
    alMotion->post.gotoChainAngles(string("LLeg"), lleg, seconds, 1);
    alMotion->post.gotoChainAngles(string("RLeg"), rleg, seconds, 1);
    return dcmTime + time;
#else
    return dcmTime;
#endif
}

/*! Go to the new positions at the specified velocity (the almotion does the interpolation)
 @param positions[]         the new target positions (must be of length ALIAS_TARGETS_NOT_HEAD_LENGTH)
 @param velocity            the velocity in rad/s
 
 @return finishTime         the dcmTime the motion will finish. I stress that this is the dcmTime; the dcmTime the motion will finish; the dcmTime.
 */
int ALWalk::goToAnglesWithVelocityNotHead(float positions[], float velocity)
{
#if JWALK_ALMOTION
    // calculate the finishing times for the given velocity
    float times[ALIAS_TARGETS_NOT_HEAD_LENGTH];                 // the time in seconds
    float maxtime = 0;
    for (unsigned char i=0; i<ALIAS_TARGETS_NOT_HEAD_LENGTH; i++)
    {
        if ((i+2 == J_L_KNEE_PITCH) || (i+2 == J_R_KNEE_PITCH))         // if the joint is a knee it needs to be moved twice as fast! 
            times[i] = fabs(jointPositions[i+2] - positions[i])/(2*velocity);
        else
            times[i] = fabs(jointPositions[i+2] - positions[i])/velocity;
        
        if (times[i] > maxtime)
            maxtime = times[i];
    }
    
    for (unsigned char i=0; i<ALIAS_TARGETS_NOT_HEAD_LENGTH; i++)
        alMotion->post.gotoAngle(indexToName[i+2], positions[i], times[i], 1);
    return dcmTime + (int) (1000*maxtime);
#else
    return dcmTime;
#endif
}

/********************************************************************************************************************************************************************
 Walk Interface
    This interface has several limitations:
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
void ALWalk::walkStraight(float distance)
{
#if JWALK_ALMOTION
    if (walkIsActive())
    {
        return;
    }
#if ALWALK_VERBOSITY > 1
    thelog << "ALWALK: walkStraight(" << distance << ")" << endl;
#endif
    
    distance = distance/100.0;
    if (distance >= 0)
    {
        configALMotion(ALWALK_PRIMITIVE_FORWARD);
        alMotion->post.walkStraight(distance, alWalkConfigs[ALWALK_PRIMITIVE_FORWARD].CyclesPerStep);
    }
    else
    {
        configALMotion(ALWALK_PRIMITIVE_BACKWARD);
        alMotion->post.walkStraight(distance, alWalkConfigs[ALWALK_PRIMITIVE_BACKWARD].CyclesPerStep);
    }
#endif
}

/*! Walk along a arc of radius (cm) for an angle (radians)
 @param angle           the angle of the arc (radians)
 @param radius          the radius of the circle the arc is on (cm)
 
Note. Nothing happens if the robot is already walking
 */
void ALWalk::walkArc(float angle, float radius)
{
#if JWALK_ALMOTION
    if (walkIsActive())
        return;
    
#if ALWALK_VERBOSITY > 1
    thelog << "ALWALK: walkArc(" << angle << ", " << radius << ")" << endl;
#endif
    
    radius = radius/100.0;              // convert the radius to metres as required by almotion
    configALMotion(ALWALK_PRIMITIVE_ARC);
    alMotion->post.walkArc(angle, radius, alWalkConfigs[ALWALK_PRIMITIVE_ARC].CyclesPerStep);
#endif
}

/*! Walk sideways 'distance' in centimetres
 @param distance        the distance to walk in cm (left is positive)
 
Note. Nothing happens if the robot is already walking
 */
void ALWalk::walkSideways(float distance)
{
#if JWALK_ALMOTION
    if (walkIsActive())
        return;
    
#if ALWALK_VERBOSITY > 1
    thelog << "ALWALK: walkSideways(" << distance << ")" << endl;
#endif
    
    distance = distance/100.0;          // convert to metres as required by almotion
    configALMotion(ALWALK_PRIMITIVE_SIDEWAYS);
    alMotion->post.walkSideways(distance, alWalkConfigs[ALWALK_PRIMITIVE_SIDEWAYS].CyclesPerStep);
#endif
}

/*! Turn 'angle' radians on the spot
 @param angle       the angle to turn in radians (left/anticlockwise is postive)
 
Note. Nothing happens if the robot is already walking
 */
void ALWalk::turn(float angle)
{
#if JWALK_ALMOTION
    if (walkIsActive())
        return;
    
#if ALWALK_VERBOSITY > 1
    thelog << "ALWALK: turn(" << angle << ")" << endl;
#endif
    
    configALMotion(ALWALK_PRIMITIVE_TURN);
    alMotion->post.turn(angle, alWalkConfigs[ALWALK_PRIMITIVE_TURN].CyclesPerStep);
#endif
}

/*! Returns true if there is an active alWalk task. Returns false otherwise
 */
bool ALWalk::walkIsActive()
{
#if JWALK_ALMOTION
    if (balanceFalling == true || balanceFallen == true)        // the walk can not be active if I have fallen over
        return false;
    else if (walkAmIWalking == true)                            // if sensors thinks I am walking then I am definitely walking
        return true;
    else if (alMotion->walkIsActive() == true)
        return true;
    else
        return false;
#else
    return false;
#endif
}

/*! Blocks until along of the pending walk tasks have been completed.
 This could take some time if there are many steps in the queue
 */
void ALWalk::waitUntilFinished()
{
#if JWALK_ALMOTION
    while (walkIsActive())
    {
        sleep(0.05);
    }
    //alMotion->waitUntilWalkIsFinished();  //<---- This function fails, typical!
#endif
}

/*! Safely stops the current walk as soon as possible (it will finish its current step, and then take a stopping step to bring the robot to rest)
 
 This is not a blocking function, ie. when it returns the robot will still be walking
 */
void ALWalk::stop()
{
#if JWALK_ALMOTION
    alMotion->clearFootsteps();
    alWalkGoDirection = ALWALK_DIRECTION_UNDEFINED;
#endif
}

/********************************************************************************************************************************************************************
 Go Walk Interface
 
 This is a slightly thicker wrapper of almotion's walk. It is designed to ease the implementation of scripted behaviours, and makes the walk a bit more responsive.
 
 This interface is sort of designed to implement scripted paths, where a component of the path is completed in a 'closed-loop' sense. 
 For example, a dodge maneuver would call gLeft() and then later when the obstacle has passed call stop. Or maybe, the search for the ball would just be gTurnLeft while 
 moving the head, when the ball is found just call gStop() (or alWalkStop) 
 
 ********************************************************************************************************************************************************************/

/*! Walk forward until another direction is called, or the walk is stopped (whether explicitly or implicitly)
 */
void ALWalk::goForward()
{
#if JWALK_ALMOTION
    if (walkIsActive())
    {
        if (alWalkGoDirection == ALWALK_DIRECTION_FORWARD)          // if we are already walking forward don't reissue the command
            return;
        else
            stop();                                                 // otherwise change direction
    }
    alWalkGoDirection = ALWALK_DIRECTION_FORWARD;
    configALMotion(ALWALK_PRIMITIVE_FORWARD);
    alMotion->post.walkStraight(6, alWalkConfigs[ALWALK_PRIMITIVE_FORWARD].CyclesPerStep);
#endif
}

/*! Walk backward until another direction is called, or the walk is stopped (whether explicitly or implicitly)
 */
void ALWalk::goBackward()
{
#if JWALK_ALMOTION
    if (walkIsActive())
    {
        if (alWalkGoDirection == ALWALK_DIRECTION_BACKWARD)          // if we are already walking forward don't reissue the command
            return;
        else
            stop();                                                 // otherwise change direction
    }
    alWalkGoDirection = ALWALK_DIRECTION_BACKWARD;
    configALMotion(ALWALK_PRIMITIVE_BACKWARD);
    alMotion->post.walkStraight(-6, alWalkConfigs[ALWALK_PRIMITIVE_BACKWARD].CyclesPerStep);
#endif
}

/*! Walk along an arc in the specified direction until the another direction is called, or the walk is stopped (whether explicitly or implicitly)
 */
void ALWalk::goArc(float angle)
{
#if JWALK_ALMOTION
    float radius = 0;
    
    if (walkIsActive())
    {
        stop();         // I have no choice with walkArc; I must stop and start walking on the new direction
    }
    
    if (angle < 0.05)       // if the angle is small, then I really want to walk forward instead
    {
        alWalkGoDirection = ALWALK_DIRECTION_FORWARD;
        configALMotion(ALWALK_PRIMITIVE_FORWARD);
        alMotion->post.walkStraight(6, alWalkConfigs[ALWALK_PRIMITIVE_FORWARD].CyclesPerStep);
    }
    else
    {
        alWalkGoDirection = ALWALK_DIRECTION_ARC;
        radius = fabs(4*alWalkConfigs[ALWALK_PRIMITIVE_ARC].StepLength/angle);
        configALMotion(ALWALK_PRIMITIVE_ARC);
        alMotion->post.walkArc(angle, radius, alWalkConfigs[ALWALK_PRIMITIVE_ARC].CyclesPerStep);
    }
#endif
}

/*! Walk left (sideways) until the another direction is called, or the walk is stopped (whether explicitly or implicitly)
 */
void ALWalk::goLeft()
{
#if JWALK_ALMOTION
    if (walkIsActive())
    {
        if (alWalkGoDirection == ALWALK_DIRECTION_LEFT)          // if we are already walking left don't reissue the command
            return;
        else
            stop();                                                 // otherwise change direction
    }
    alWalkGoDirection = ALWALK_DIRECTION_LEFT;
    configALMotion(ALWALK_PRIMITIVE_SIDEWAYS);
    alMotion->post.walkSideways(6, alWalkConfigs[ALWALK_PRIMITIVE_SIDEWAYS].CyclesPerStep);
#endif
}

/*! Walk right (sideways) until the another direction is called, or the walk is stopped (whether explicitly or implicitly)
 */
void ALWalk::goRight()
{
#if JWALK_ALMOTION
    if (walkIsActive())
    {
        if (alWalkGoDirection == ALWALK_DIRECTION_RIGHT)          // if we are already walking forward don't reissue the command
            return;
        else
            stop();                                                 // otherwise change direction
    }
    alWalkGoDirection = ALWALK_DIRECTION_RIGHT;
    configALMotion(ALWALK_PRIMITIVE_SIDEWAYS);
    alMotion->post.walkSideways(-6, alWalkConfigs[ALWALK_PRIMITIVE_SIDEWAYS].CyclesPerStep);
#endif
}

/*! Turn left (rotate anticlockwise) on the spot until another direction is called, or the walk is stopped (whether explicitly or implicitly)
 
 This turn is significantly slower (ie the turn velocity is smaller ;)) than other turns.
 */
void ALWalk::goTurnLeft()
{
#if JWALK_ALMOTION
    if (walkIsActive())
    {
        if (alWalkGoDirection == ALWALK_DIRECTION_TURN_LEFT)          // if we are already walking forward don't reissue the command
            return;
        else
            stop();                                                 // otherwise change direction
    }
    alWalkGoDirection = ALWALK_DIRECTION_TURN_LEFT;
    configALMotion(ALWALK_PRIMITIVE_TURN);
    alMotion->post.turn(15*3.14, alWalkConfigs[ALWALK_PRIMITIVE_TURN].CyclesPerStep);
#endif
}

/*! Turn right (rotate clockwise) on the spot until another direction is called, or the walk is stopped (whether explicitly or implicitly)
 
 This turn is significantly slower (ie the turn velocity is smaller ;)) than other turns.
 */
void ALWalk::goTurnRight()
{
#if JWALK_ALMOTION
    if (walkIsActive())
    {
        if (alWalkGoDirection == ALWALK_DIRECTION_TURN_RIGHT)          // if we are already walking forward don't reissue the command
            return;
        else
            stop();                                                 // otherwise change direction
    }
    alWalkGoDirection = ALWALK_DIRECTION_TURN_RIGHT;
    configALMotion(ALWALK_PRIMITIVE_TURN);
    alMotion->post.turn(-15*3.14, alWalkConfigs[ALWALK_PRIMITIVE_TURN].CyclesPerStep);
#endif
}

/*! Returns true if there is an active g task. Returns false otherwise
 
 Note. While the robot takes a stopping step, gWalk will not be 'active', however the robot will still be walking.
 */
bool ALWalk::gIsActive()
{
#if JWALK_ALMOTION
    if (alWalkGoDirection == ALWALK_DIRECTION_UNDEFINED)            // if there isn't a defined direction then g is not 'active'
        return false;
    else
        return walkIsActive();
#else
    return false;
#endif
}

/* Send the gait primitive configuration to almotion
 @param primitive       the primitive for which almotion will be configured to perform
 */
void ALWalk::configALMotion(ALWalkPrimitiveEnum primitive)
{
#if JWALK_ALMOTION
    alwalkconfig_t* config = &alWalkConfigs[primitive];
    // Firstly I need to 'copy' the primitives hardnesses to the public 
    alWalkHardnesses = config->Hardnesses;
    
    // walkConfig:
    alMotion->setWalkConfig(config->StepLength, config->StepHeight, config->StepSide, config->StepTurn, config->ZMPX, config->ZMPY);
    
    // walkExtraConfig
    alMotion->setWalkExtraConfig(config->LHipBacklash, config->RHipBacklash, config->HipHeight, config->TorsoOrientation);
#endif
}

/********************************************************************************************************************************************************************
 File-based configuration of alMotion gait primitives
 ********************************************************************************************************************************************************************/

/* Reads the configuration files for each of the almotion walk primitives
 */
void ALWalk::readConfigs()
{
#if JWALK_ALMOTION
#if ALWALK_VERBOSITY > 0
    thelog << "ALWALK: Reading configurations for almotion's walk primitives." << endl;
#endif
    readConfig(string("Forward"), &alWalkConfigs[ALWALK_PRIMITIVE_FORWARD]);
    readConfig(string("Backward"), &alWalkConfigs[ALWALK_PRIMITIVE_BACKWARD]);
    readConfig(string("Arc"), &alWalkConfigs[ALWALK_PRIMITIVE_ARC]);
    readConfig(string("Sideways"), &alWalkConfigs[ALWALK_PRIMITIVE_SIDEWAYS]);
    readConfig(string("Turn"), &alWalkConfigs[ALWALK_PRIMITIVE_TURN]);
    
#if ALWALK_VERBOSITY > 0
    for (int i=0; i<ALWALK_NUM_PRIMITIVES; i++)
    {
        thelog << "ALWALK: Configuration for primitive " << i << endl;
        for (int j=0; j<ALIAS_TARGETS_NOT_HEAD_LENGTH; j++)
            thelog << alWalkConfigs[i].Hardnesses[j] << ", ";
        thelog << endl;
        
        thelog << alWalkConfigs[i].CyclesPerStep << endl;
        thelog << alWalkConfigs[i].StepLength << ", " << alWalkConfigs[i].StepHeight << ", " << alWalkConfigs[i].StepSide << ", " << alWalkConfigs[i].StepTurn << ", " << alWalkConfigs[i].ZMPX << ", " << alWalkConfigs[i].ZMPY << endl;
        thelog << alWalkConfigs[i].LHipBacklash << ", " << alWalkConfigs[i].RHipBacklash << ", " << alWalkConfigs[i].HipHeight << ", " << alWalkConfigs[i].TorsoOrientation << endl;
        thelog << alWalkConfigs[i].DistanceCommandFactor << ", " << alWalkConfigs[i].AngleCommandFactor << ", " << alWalkConfigs[i].DistanceOdometryFactor << ", "<<  alWalkConfigs[i].AngleOdometryFactor << endl;
    }
#endif
#endif
}

/* Reads the almotion walk primitive configuration file, and saves the data into the config struct
 @param name        the name of the primitive to be loaded (in fact the filename of the file will be 'name.csv')
 @param *config     the struct that will contain the parameters found in the file
 */
void ALWalk::readConfig(string name, alwalkconfig_t* config)
{
#if JWALK_ALMOTION
#if ALWALK_VERBOSITY > 1
    thelog << "ALWALK: Reading config file for " << name << endl;
#endif
    
    string linebuffer;
    float valuebuffer[50];
    int numvalues = 0;
    
    string filepath = CONFIG_LOCATION + name + ".csv";
    ifstream file;
    file.open(filepath.c_str());
    
#if ALWALK_VERBOSITY > 2
    thelog << "ALWALK: Config location " << filepath << endl;
#endif
    
    // the first line is the hardnesses
    getline(file, linebuffer);
    numvalues = lineToFloats(linebuffer, valuebuffer);
    if (numvalues != ALIAS_TARGETS_NOT_HEAD_LENGTH)
        thelog << "ALWALK: Incorrect number of hardnesses specified in config file for " << name << " There were " << numvalues << ", I wanted " << ALIAS_TARGETS_NOT_HEAD_LENGTH << endl;
    else
    {
        for (int i=0; i<ALIAS_TARGETS_NOT_HEAD_LENGTH; i++)
            config->Hardnesses[i] = valuebuffer[i];
    }
    
    // the second line is just the cyclesperstep
    getline(file, linebuffer);
    numvalues = lineToFloats(linebuffer, valuebuffer);
    if (numvalues != 1)
        thelog << "ALWALK: The CyclesPerStep line was incorrect in the configuration file for " << name << endl;
    else
        config->CyclesPerStep = (int)valuebuffer[0];
    
    // the walk config 
    getline(file, linebuffer);
    numvalues = lineToFloats(linebuffer, valuebuffer);
    if (numvalues != 6)
        thelog << "ALWALK: The WalkConfig line was incorrect in the configuration file for " << name << endl;
    else
    {
        config->StepLength = valuebuffer[0];
        config->StepHeight = valuebuffer[1];
        config->StepSide = valuebuffer[2];
        config->StepTurn = valuebuffer[3];
        config->ZMPX = valuebuffer[4];
        config->ZMPY = valuebuffer[5];
    }
    
    // the walk extra config
    getline(file, linebuffer);
    numvalues = lineToFloats(linebuffer, valuebuffer);
    if (numvalues != 4)
        thelog << "ALWALK: The WalkExtraConfig line was incorrect in the configuration file for " << name << endl;
    else
    {
        config->LHipBacklash = valuebuffer[0];
        config->RHipBacklash = valuebuffer[1];
        config->HipHeight = valuebuffer[2];
        config->TorsoOrientation = valuebuffer[3];
    }
    
    // the four correction factors
    getline(file, linebuffer);
    numvalues = lineToFloats(linebuffer, valuebuffer);
    if (numvalues != 4)
        thelog << "ALWALK: The CorrectionFactors line was incorrect in the configuration file for " << name << endl;
    else
    {
        config->DistanceCommandFactor = valuebuffer[0];
        config->AngleCommandFactor = valuebuffer[1];
        config->DistanceOdometryFactor = valuebuffer[2];
        config->AngleOdometryFactor = valuebuffer[3];
    }
    
    file.close();
    
#if ALWALK_VERBOSITY > 9000
    thelog << "ALWALK: Configuration for " << name << endl;
    for (int i=0; i<ALIAS_TARGETS_NOT_HEAD_LENGTH; i++)
        thelog << config->Hardnesses[i] << ", ";
    thelog << endl;
    
    thelog << config->CyclesPerStep << endl;
    thelog << config->StepLength << ", " << config->StepHeight << ", " << config->StepSide << ", " << config->StepTurn << ", " << config->ZMPX << ", " << config->ZMPY << endl;
    thelog << config->LHipBacklash << ", " << config->RHipBacklash << ", " << config->HipHeight << ", " << config->TorsoOrientation << endl;
    thelog << config->DistanceCommandFactor << ", " << config->AngleCommandFactor << ", " << config->DistanceOdometryFactor << ", " << config->AngleOdometryFactor << endl;
#endif
#endif
}

/* Converts a line (string) of a comma separated file into an array of floats
 @param line        a string with floats separated by a comma
 @param values      the array that is updated with the converted floats
 */
int ALWalk::lineToFloats(string line, float values[])
{
    int column = 0;
    int currentindex = 0;
    int startindex = 0;
    int endindex = 0;
    
    // the first entry is special (it is not preceded by a comma)
    endindex = line.find(",", currentindex);
    currentindex = endindex;
    if (column >= 50)
        thelog << "ALWALK: There are too many columns in the file." << line << endl;
    else
        values[column] = atof(line.substr(startindex, endindex).c_str());
    column++;
    
    // scan through the file looking for floats sandwitched between ", " 
    while (currentindex != string::npos)
    {
        startindex = line.find(",", currentindex) + 1;
        currentindex = startindex;
        endindex = line.find(",", currentindex);
        currentindex = endindex;
        if (startindex != string::npos)
        {
            if (column >= 50)
                thelog << "ALWALK: There are too many columns in the file." << line << endl;
            else
                values[column] = atof(line.substr(startindex, endindex).c_str());
            column++;
        }
    }
    return column;
}

/********************************************************************************************************************************************************************
 Test Functions
 ********************************************************************************************************************************************************************/
void ALWalk::innerTest()
{
    // test the walk interface 
    walkStraight(25);
    thelog << "ALWALK: innerTest. Waiting for walk to finish." << endl;
    waitUntilFinished();
    thelog << "ALWALK: innerTest. Finished waiting for walk to finish." << endl;

    walkArc(0.5, 75);
    sleep(3);
    stop();
    thelog << "ALWALK: innerTest. Waiting for walk to finish." << endl;
    waitUntilFinished();
    thelog << "ALWALK: innerTest. Finished waiting for walk to finish." << endl;
    
    walkSideways(5);
    sleep(2);
    stop();
    thelog << "ALWALK: innerTest. Waiting for walk to finish." << endl;
    waitUntilFinished();
    thelog << "ALWALK: innerTest. Finished waiting for walk to finish." << endl;
    
    walkSideways(-5);
    sleep(1);
    thelog << "ALWALK: innerTest. Waiting for walk to finish." << endl;
    waitUntilFinished();
    thelog << "ALWALK: innerTest. Finished waiting for walk to finish." << endl;

    turn(0.5);
}


