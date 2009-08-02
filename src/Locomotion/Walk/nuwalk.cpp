/*
 *  nuwalk.cpp
 *  jwalk
 *
 *  Created by jason on 6/04/09.
 *  Copyright 2009 UoN. All rights reserved.
 *
 */

#include "nuwalk.h"
#include "walkprimitive.h"
#include "jwalk.h"
#include "../../Globals.h"    // I want PI

NUWalk::NUWalk(JWalk* pjwalk)
{
#if NUWALK_VERBOSITY > 0
    thelog << "NUWALK: Initialising." << endl;
#endif
    jwalk = pjwalk;
    actuators = jwalk->JWalkActuators;
    
    CurrentStep = NULL;
    PreviousStep = NULL;
    
    WalkR = 0;
    WalkBearing = 0;
    WalkFinalOrientation = 0;
    WalkSpeed = 0;
    WalkDodge = false;
    
    GWalkDirection = GWALK_DIRECTION_UNDEFINED;
    
    WalkEnabled = false;
    WalkWaitingToStop = false;
    WalkStopped = true;
    
    // Path planning
    PathStraightSpeed = 13.5;           // cm/s
    PathBackwardSpeed = 6.2;            // cm/s (approx guess)
    PathArcSpeed = 11.0;                // cm/s 
    PathSidewardSpeed = 5.0;            // cm/s (approximate guess)
    PathTurnSpeed = 0.7;                // rad/s
    
    PathStopTime = 0.5;
    
    PathMinArcRadius = 10;
    PathMaxArcRadius = 50;
    
    PathPositionAccuracy = 7;           // a rough estimate of the noise in cm (This can be considered the 'accuracy' of the calculated paths)
    PathBearingAccuracy = atan(PathPositionAccuracy/24.0);
    PathInfPathTime = 600;              // infinite path time in seconds (ie the path will never reach the target)
    
    initPrimitives();
    // I will use the forward walk pose as the default pose (this will extended to every default pose in the universe)
    Primitives[TYPE_FORWARD]->getInitialPose(&DefaultPositions, &DefaultHardnesses);
    
    CurrentPositions = DefaultPositions;
    CurrentHardnesses = DefaultHardnesses;
    
#if NUWALK_VERBOSITY > 0
    thelog << "NUWALK: Initialised." << endl;
#endif
}

NUWalk::~NUWalk()
{
    // delete all of the Primitives
    for (unsigned char i=0; i<TYPE_NUM_TYPES; i++)
        delete Primitives[i];
}

void NUWalk::initPrimitives()
{
    for (unsigned char i=0; i<TYPE_NUM_TYPES; i++)
        Primitives[i] = new WalkPrimitive((StepTypeEnum)i);
}

/* Enables the NUWalk engine. Every walk call will need to enable the walk.
 Be sure that this is done after valid walk distance, bearing and final orientation have been set.
 */
void NUWalk::enableWalk()
{
    WalkEnabled = true;
    WalkStopped = false;
}

void NUWalk::onBearing(float bearing, bool dodge)
{
    WalkR = -1;               // I use -1 to distinguish between onBearing and toPoint, which will have R > 0
    WalkBearing = bearing;
    WalkDodge = dodge;
    enableWalk();
}

void NUWalk::toPoint(float distance, float bearing, bool dodge)
{
    WalkR = fabs(distance);
    WalkBearing = bearing;
    WalkFinalOrientation = 1001;
    WalkDesiredOrientation = 1001;
    WalkDodge = dodge;
    enableWalk();
}

void NUWalk::toPointWithOrientation(float distance, float bearing, float finalorientation, bool dodge)
{
    WalkR = fabs(distance);
    WalkBearing = bearing;
    WalkFinalOrientation = finalorientation;
    WalkDesiredOrientation = 1001;
    WalkDodge = dodge;
    enableWalk();
}

void NUWalk::toPointWithMaintainOrientation(float distance, float bearing, float desiredorientation, bool dodge)
{
    WalkR = fabs(distance);
    WalkBearing = bearing;
    WalkFinalOrientation = 1001;
    WalkDesiredOrientation = desiredorientation;
    WalkDodge = dodge;
    enableWalk();
}

// GMode commands
void NUWalk::goForward()
{
    GWalkDirection = GWALK_DIRECTION_FORWARD;
    enableWalk();
}

void NUWalk::goBackward()
{
    GWalkDirection = GWALK_DIRECTION_BACKWARD;
    enableWalk();
}

void NUWalk::goLeft()
{
    GWalkDirection = GWALK_DIRECTION_LEFT;
    enableWalk();
}

void NUWalk::goRight()
{
    GWalkDirection = GWALK_DIRECTION_RIGHT;
    enableWalk();
}

void NUWalk::goTurnLeft()
{
    #if NUWALK_VERBOSITY > 1
        thelog << "NUWALK: goTurnLeft()." << endl;
    #endif
    GWalkDirection = GWALK_DIRECTION_TURN_LEFT;
    enableWalk();
}

void NUWalk::goTurnRight()
{
    GWalkDirection = GWALK_DIRECTION_TURN_RIGHT;
    enableWalk();
}

void NUWalk::doWalk()
{
#if NUWALK_VERBOSITY > 2
    thelog << "NUWALK: doWalk()" << endl;
#endif
    if (WalkEnabled == true and WalkStopped == false)
    {
        #if NUWALK_VERBOSITY > 2
            thelog << "NUWALK: doWalk(). nuWalk is enabled and not stopped." << endl;
        #endif
        if (CurrentStep == NULL)
        {
            #if NUWALK_VERBOSITY > 1
                thelog << "NUWALK: doWalk(). CurrentStep is NULL." << endl;
            #endif
            determineNextStep();
        }
        
        if (CurrentStep != NULL)
        {
            #if NUWALK_VERBOSITY > 2
                thelog << "NUWALK: doWalk(). CurrentStep: " << CurrentStep->Name << endl;
            #endif
            CurrentStep->getNextFrame(&CurrentPositions, &CurrentHardnesses);
            actuators->sendFrameToNotHead(CurrentPositions, CurrentHardnesses);
            
            if (CurrentStep->hasEnded())
            {
                #if NUWALK_VERBOSITY > 1
                    thelog << "NUWALK: doWalk(). CurrentStep: " << CurrentStep->Name << " has ended." << endl;
                #endif
                PreviousStep = CurrentStep;
                if (WalkWaitingToStop == true)
                    doStopStep();
                else
                    determineNextStep();
            }
        }
    }
}

void NUWalk::determineNextStep()
{
    if (jwalk->JWalkMode == JWALK_JMODE)
    {
        action_t nextaction;
        
        #if NUWALK_VERBOSITY > 1
            thelog << "NUWALK: determineNextStep(). JMode. R: " << WalkR << " theta: " << WalkBearing << " orientation: " << WalkFinalOrientation << " speed: " << WalkSpeed << endl;
        #endif
        
        // Calculate the best action to get to the target
        if (WalkR < 0)
            getActionOnBearing(&nextaction, WalkBearing);
        else if (WalkFinalOrientation > 1000 and WalkDesiredOrientation > 1000)
            getActionToPoint(&nextaction, WalkR, WalkBearing);
        else if (WalkDesiredOrientation > 1000)
            getActionToPointWithOrientation(&nextaction, WalkR, WalkBearing, WalkFinalOrientation);
        else
            getActionToPointWithMaintainOrientation(&nextaction, WalkR, WalkBearing, WalkDesiredOrientation);
        
        #if NUWALK_VERBOSITY > 1
            thelog << "NUWALK: determineNextStep(). nextaction.Type = " << typeToTypeName[nextaction.Type] << " nextaction.Direction: " << nextaction.Direction << endl;
        #endif

        if (WalkDodge == true)
        {
            #if NUWALK_VERBOSITY > 1
                thelog << "NUWALK: determineNextStep(). Dodging!" << endl;
            #endif
            if (collisionAny == true)
                avoidCollisions(&nextaction);
            else
                avoidObstacles(&nextaction);
            
            #if NUWALK_VERBOSITY > 1
                thelog << "NUWALK: determineNextStep(). nextaction.Type = " << typeToTypeName[nextaction.Type] << " nextaction.Direction: " << nextaction.Direction << endl;
            #endif
        }
        
        // Get the next step that 'best implements' the 'best action'
        selectNextStep(nextaction);
    }
    else if (jwalk->JWalkMode == JWALK_GMODE)
    {
        #if NUWALK_VERBOSITY > 1
            thelog << "NUWALK: determineNextStep(). GMode. Direction: " << GWalkDirection << endl;
        #endif
        
        selectNextGStep();
    }
    else
    {
        #if NUWALK_VERBOSITY > 1
            thelog << "NUWALK: determineNextStep(). OtherMode: " << jwalk->JWalkMode << endl;
        #endif
    }
    
    #if NUWALK_VERBOSITY > 1
        if (CurrentStep != NULL)
            thelog << "NUWALK: determineNextStep(): " << CurrentStep->Name << endl;
        else
            thelog << "NUWALK: determineNextStep(): Selected a NULL step. This could be bad." << endl;
    #endif
    /*    if (CurrentStep == NULL)
     {
     thelog << "NUWALK: selectNextGStep(): ERROR: I have selected a NULL step. This is bad :(" << endl;
     CurrentStep = PreviousStep->NaturalNext;
     }*/
}

void NUWalk::getActionOnBearing(action_t* action, float bearing)
{
    if (fabs(bearing) < 0.19)
    {
        action->Type = TYPE_FORWARD;
        action->Direction = 0;
    }
    else if (fabs(bearing) < 1.2)
    {
        action->Type = TYPE_ARC;
        action->Direction = 2*6.5/bearing;
    }
    else
    {
        action->Type = TYPE_TURN;
        action->Direction = bearing;
    }
}

/********************************************************************************************************************************************************************
 getActionToPoint()
 ********************************************************************************************************************************************************************/
/*! Update action so that it walks to the specied (distance, bearing) as fast as possible
 @param action, action->Type and action->Direction will be set
 @param distance, the distance to target (cm)
 @param bearing, the heading to the target (radians)
 */
void NUWalk::getActionToPoint(action_t* action, float distance, float bearing)
{
    #if NUWALK_VERBOSITY > 1
        thelog << "NUWALK: getActionToPoint()" << endl;
    #endif
    TargetDistance = distance;
    TargetBearing = wrapAngle(bearing);
    TargetRelativeX = distance*cos(bearing);
    TargetRelativeY = distance*sin(bearing);

    // Firstly, calculate the path times for each of the 6 possible paths to the point
    const static unsigned char numpaths = 6;
    #if NUWALK_VERBOSITY > 0
        static string indextopathname[numpaths] = {string("Straight"), string("Arc"), string("Backward"), string("TurnBackward"), string("Sideward"), string("TurnSideward")};
    #endif
    float pathtimes[numpaths]; 
    pathtimes[0] = calculateStraightPathTime();
    pathtimes[1] = calculateArcPathTime();
    pathtimes[2] = calculateBackwardPathTime();
    pathtimes[3] = calculateTurnStopBackwardPathTime();     
    pathtimes[4] = calculateSidewardPathTime();
    pathtimes[5] = calculateTurnStopSidewardPathTime();

    // Now decide which of those paths is the fastest
    int minpathindex = 0;
    float minpathtime = PathInfPathTime + 10;
    for (unsigned char i = 0; i < numpaths; i++)
    {
        if (pathtimes[i] < minpathtime)
        {
            minpathindex = i;
            minpathtime = pathtimes[i];
        }
    }
    
    #if NUWALK_VERBOSITY > 2
        thelog << "NUWALK: getActionToPoint(). Path times: ";
        for (unsigned char i =0; i < numpaths; i++)
        {
            thelog << pathtimes[i] << ", ";
        }
        thelog << endl;
    #endif
    
    // Now implement the fastest path. That is set action.bearing and action.type.
    if (minpathindex == 0)
        setStraightAction(action);
    else if (minpathindex == 1)
        setArcAction(action);
    else if (minpathindex == 2)
        setBackwardAction(action);
    else if (minpathindex == 3)
        setTurnStopBackwardAction(action);
    else if (minpathindex == 4)
        setSidewardAction(action);
    else if (minpathindex == 5)
        setTurnStopSidewardAction(action);
    
#if NUWALK_VERBOSITY > 0
    thelog << "NUWALK: getActionToPoint() Path: " << indextopathname[minpathindex] << " action: " << action->Type << " " << action->Direction;
#endif

}

/*! Returns the time to reach the target in seconds. If the target can not be reached then PathInfPathTime is returned
TargetDistance, the distance to the target in cm
TargetBearing, the bearing to the target in radians
TargetRelativeX, the x distance in cm (x-axis is forwards)
TargetRelativeY, the y distance in cm (y-axis is left) */
float NUWalk::calculateStraightPathTime()
{
    #if NUWALK_VERBOSITY > 2
        thelog << "NUWALK: calculateStraightPathTime()" << endl;
    #endif
    float pathtime = 0;
    if ((TargetRelativeX > 0 && fabs(TargetRelativeY) < PathPositionAccuracy) || fabs(TargetBearing) < PathBearingAccuracy)
    {   // if the target is ahead and less than the PositionNoise from the centre or if the bearing to the target is small then I can just go forward
        if (walkIsActive() && !(PreviousStep == NULL || PreviousStep->StepType == TYPE_FORWARD || PreviousStep->StepType == TYPE_ARC))
            pathtime += PathStopTime;

        pathtime += TargetRelativeX/PathStraightSpeed;
    }
    else
    {   // if the target is not directly ahead of me I need to turn to it
        // I am going to be a little clever here, and only turn if I need to turn really sharp, otherwise do an arc
        if (fabs(TargetBearing) < 2*6.5/PathMinArcRadius)
        {   // so we don't need to turn too sharp, so do the usual walk on bearing type arc
            if (walkIsActive() && !(PreviousStep == NULL || PreviousStep->StepType == TYPE_FORWARD || PreviousStep->StepType == TYPE_ARC))
                pathtime += PathStopTime;
            float l = 2*6.5; // (by design) the arc is two steps long
            pathtime += fabs(l)/PathArcSpeed;
        }
        else
        {
            if (walkIsActive() && !(PreviousStep == NULL || (PreviousStep->StepType == TYPE_TURN && sameSign(PreviousStep->StepDirection, TargetBearing))))
                pathtime += PathStopTime;

            pathtime += fabs(TargetBearing)/PathTurnSpeed;
            pathtime += PathStopTime;
            pathtime += fabs(TargetDistance)/PathStraightSpeed;
        }
    }
    #if NUWALK_VERBOSITY > 2
        thelog << "NUWALK: calculateStraightPathTime(): Final Time" << pathtime << endl;
    #endif
    return pathtime;
}

/*! Returns the time to reach the target in seconds. If the target can not be reached then PathInfPathTime is returned
 TargetDistance, the distance to the target in cm
 TargetBearing, the bearing to the target in radians
 TargetRelativeX, the x distance in cm (x-axis is forwards)
 TargetRelativeY, the y distance in cm (y-axis is left) */
float NUWalk::calculateArcPathTime()
{
    #if NUWALK_VERBOSITY > 2
        thelog << "NUWALK: calculateArcPathTime()" << endl;
    #endif
    float pathtime = 0;
    float R = 0;                            // the radius of the arc in cm
    if (fabs(TargetBearing) < 0.001 || fabs(fabs(TargetBearing) - PI) < 0.001)
        R = PathMaxArcRadius;
    else
        R = TargetDistance/(2*sin(TargetBearing));
    float l = fabs(R*2*TargetBearing);      // the length of the arc
    pathtime += fabs(l)/PathArcSpeed;       // the time taken to walk on the arc
    
    // apply stop penalty if necessary
    if (walkIsActive() && !(PreviousStep == NULL || PreviousStep->StepType == TYPE_FORWARD || PreviousStep->StepType == TYPE_ARC))
        pathtime += PathStopTime;

    return pathtime;
}

/*! Returns the time to reach the target in seconds. If the target can not be reached then PathInfPathTime is returned
 TargetDistance, the distance to the target in cm
 TargetBearing, the bearing to the target in radians
 TargetRelativeX, the x distance in cm (x-axis is forwards)
 TargetRelativeY, the y distance in cm (y-axis is left) */
float NUWalk::calculateBackwardPathTime()
{
    #if NUWALK_VERBOSITY > 2
        thelog << "NUWALK: calculateBackwardPathTime()" << endl;
    #endif
    float pathtime = 0;
    if ((TargetRelativeX < 0 && fabs(TargetRelativeY) < 5) || fabs(TargetBearing - PI) < PathBearingAccuracy || fabs(TargetBearing + PI) < PathBearingAccuracy)
        pathtime += fabs(TargetRelativeX)/PathBackwardSpeed;
    else
        return PathInfPathTime;

    if (walkIsActive() && !(PreviousStep == NULL || PreviousStep->StepType == TYPE_BACKWARD))
        pathtime += PathStopTime;

    return pathtime;
}

/*! Returns the time to reach the target in seconds. If the target can not be reached then PathInfPathTime is returned
 TargetDistance, the distance to the target in cm
 TargetBearing, the bearing to the target in radians
 TargetRelativeX, the x distance in cm (x-axis is forwards)
 TargetRelativeY, the y distance in cm (y-axis is left) */
float NUWalk::calculateTurnStopBackwardPathTime()
{
    #if NUWALK_VERBOSITY > 2
        thelog << "NUWALK: calculateTurnStopBackwardPathTime()" << endl;
    #endif
    float pathtime = 0;
    float turnangle = 0;        // the turn angle required to walk backwards into the target
    if (TargetBearing > 0)
        turnangle = TargetBearing - PI;
    else
        turnangle = TargetBearing + PI;
    
    if (walkIsActive() && !(PreviousStep == NULL || (PreviousStep->StepType == TYPE_TURN && sameSign(PreviousStep->StepDirection, turnangle))))
        pathtime += PathStopTime;

    pathtime += fabs(turnangle)/PathTurnSpeed;
    pathtime += PathStopTime;
    pathtime += TargetDistance/PathBackwardSpeed;

    return pathtime;
}

/*! Returns the time to reach the target in seconds. If the target can not be reached then PathInfPathTime is returned
 TargetDistance, the distance to the target in cm
 TargetBearing, the bearing to the target in radians
 TargetRelativeX, the x distance in cm (x-axis is forwards)
 TargetRelativeY, the y distance in cm (y-axis is left) */
float NUWalk::calculateSidewardPathTime()
{
    #if NUWALK_VERBOSITY > 2
        thelog << "NUWALK: calculateSidewardPathTime()" << endl;
    #endif
    float pathtime = 0;
    if (fabs(TargetRelativeX) < PathPositionAccuracy || fabs(TargetBearing - PI/2.0) < PathBearingAccuracy || fabs(TargetBearing + PI/2.0) < PathBearingAccuracy)
        pathtime += fabs(TargetRelativeY)/PathSidewardSpeed;
    else
        return PathInfPathTime;

    if (walkIsActive() && !(PreviousStep == NULL || (PreviousStep->StepType == TYPE_SIDEWARD && sameSign(PreviousStep->StepDirection, TargetRelativeY))))
        pathtime += PathStopTime;

    return pathtime;
}

/*! Returns the time to reach the target in seconds. If the target can not be reached then PathInfPathTime is returned
 TargetDistance, the distance to the target in cm
 TargetBearing, the bearing to the target in radians
 TargetRelativeX, the x distance in cm (x-axis is forwards)
 TargetRelativeY, the y distance in cm (y-axis is left) */
float NUWalk::calculateTurnStopSidewardPathTime()
{
    #if NUWALK_VERBOSITY > 2
        thelog << "NUWALK: calculateTurnStopSidewardPathTime()" << endl;
    #endif
    
    float pathtime = 0;
    float turnangle = 0;            // the turn angle to walk sidewards into the target (rad)
    if (TargetBearing > 0)
        turnangle = TargetBearing - PI/2;
    else
        turnangle = TargetBearing + PI/2;
    
    if (walkIsActive() && !(PreviousStep == NULL || (PreviousStep->StepType == TYPE_TURN && sameSign(PreviousStep->StepDirection, turnangle))))
        pathtime += PathStopTime;
    
    pathtime += fabs(turnangle)/PathTurnSpeed;
    pathtime += PathStopTime;
    pathtime += TargetDistance/PathSidewardSpeed;

    return pathtime;
}

/*! Sets action->Type and action->Direction to implement the path
 @param action, the action structure to be updated
 
 TargetDistance, the distance to the target in cm
 TargetBearing, the bearing to the target in radians
 TargetRelativeX, the x distance in cm (x-axis is forwards)
 TargetRelativeY, the y distance in cm (y-axis is left) 
 */
void NUWalk::setStraightAction(action_t* action)
{   
    #if NUWALK_VERBOSITY > 2
        thelog << "NUWALK: setStraightAction()" << endl;
    #endif
    
    if ((TargetRelativeX > 0 && fabs(TargetRelativeY) < PathPositionAccuracy) || fabs(TargetBearing) < PathBearingAccuracy)
    {   // if the target is ahead and less than the PositionNoise from the centre or if the bearing to the target is small then I can just go forward
        action->Type = TYPE_FORWARD;
        action->Direction = 0;
    }
    else
    {   // if the target is not directly ahead of me I need to turn to it
        // I am going to be a little clever here, and only turn if I need to turn really sharp, otherwise do an arc
        if (fabs(TargetBearing) < 2*6.5/PathMinArcRadius)
        {   // so we don't need to turn too sharp, so do the usual walk on bearing type arc
            action->Type = TYPE_ARC;
            action->Direction = 2*6.5/TargetBearing;
        }
        else
        {
            action->Type = TYPE_TURN;
            action->Direction = TargetBearing;
        }
    }
}

/*! Sets action->Type and action->Direction to implement the path
 @param action, the action structure to be updated
 
 TargetDistance, the distance to the target in cm
 TargetBearing, the bearing to the target in radians
 TargetRelativeX, the x distance in cm (x-axis is forwards)
 TargetRelativeY, the y distance in cm (y-axis is left) 
 */
void NUWalk::setArcAction(action_t* action)
{
    #if NUWALK_VERBOSITY > 2
        thelog << "NUWALK: setArcAction()" << endl;
    #endif
    
    action->Type = TYPE_ARC;
    if (fabs(TargetBearing) < 0.001 or fabs(fabs(TargetBearing) - PI) < 0.001)
        action->Direction = PathMaxArcRadius;
    else
        action->Direction = TargetDistance/(2*sin(TargetBearing));
}

/*! Sets action->Type and action->Direction to implement the path
 @param action, the action structure to be updated
 
 TargetDistance, the distance to the target in cm
 TargetBearing, the bearing to the target in radians
 TargetRelativeX, the x distance in cm (x-axis is forwards)
 TargetRelativeY, the y distance in cm (y-axis is left) 
 */
void NUWalk::setBackwardAction(action_t* action)
{          
    #if NUWALK_VERBOSITY > 2
        thelog << "NUWALK: setBackwardAction()" << endl;
    #endif
    
    action->Type = TYPE_BACKWARD;
    action->Direction = 0;
}

/*! Sets action->Type and action->Direction to implement the path
 @param action, the action structure to be updated
 
 TargetDistance, the distance to the target in cm
 TargetBearing, the bearing to the target in radians
 TargetRelativeX, the x distance in cm (x-axis is forwards)
 TargetRelativeY, the y distance in cm (y-axis is left) 
 */
void NUWalk::setTurnStopBackwardAction(action_t* action)
{
    #if NUWALK_VERBOSITY > 2
        thelog << "NUWALK: setTurnStopBackwardAction()" << endl;
    #endif
    
    if ((TargetRelativeX < 0 && fabs(TargetRelativeY) < 5) || fabs(TargetBearing - PI) < PathBearingAccuracy || fabs(TargetBearing + PI) < PathBearingAccuracy)
    {
        action->Type = TYPE_BACKWARD;
        action->Direction = 0;
    }
    else
    {
        action->Type = TYPE_TURN;
        if (TargetBearing > 0)
            action->Direction = (TargetBearing - PI);
        else
            action->Direction = (TargetBearing + PI);
    }
}

/*! Sets action->Type and action->Direction to implement the path
 @param action, the action structure to be updated
 
 TargetDistance, the distance to the target in cm
 TargetBearing, the bearing to the target in radians
 TargetRelativeX, the x distance in cm (x-axis is forwards)
 TargetRelativeY, the y distance in cm (y-axis is left) 
 */
void NUWalk::setSidewardAction(action_t* action)
{
    #if NUWALK_VERBOSITY > 2
        thelog << "NUWALK: setSidewardAction()" << endl;
    #endif
    
    action->Type = TYPE_SIDEWARD;
    if (TargetBearing < 0) 
        action->Direction = -1.0;
    else
        action->Direction = 1.0;
}

/*! Sets action->Type and action->Direction to implement the path
 @param action, the action structure to be updated
 
 TargetDistance, the distance to the target in cm
 TargetBearing, the bearing to the target in radians
 TargetRelativeX, the x distance in cm (x-axis is forwards)
 TargetRelativeY, the y distance in cm (y-axis is left) 
 */
void NUWalk::setTurnStopSidewardAction(action_t* action)
{
    #if NUWALK_VERBOSITY > 2
        thelog << "NUWALK: setTurnStopSidewardAction()" << endl;
    #endif
    
    if (fabs(TargetRelativeX) < PathPositionAccuracy || fabs(TargetBearing - PI/2.0) < PathBearingAccuracy || fabs(TargetBearing + PI/2.0) < PathBearingAccuracy)
    {
        action->Type = TYPE_SIDEWARD;
        if (TargetBearing < 0) 
            action->Direction = -1.0;
        else
            action->Direction = 1.0;
    }
    else
    {
        action->Type = TYPE_TURN;
        if (TargetBearing > 0)
            action->Direction = (TargetBearing - PI/2);
        else
            action->Direction = (TargetBearing + PI/2);
    }
}


/********************************************************************************************************************************************************************
 getActionToPointWithOrientation()
 ********************************************************************************************************************************************************************/
/*! Update action so that it walks to the specied (distance, bearing) as fast as possible
 @param action, action->Type and action->Direction will be set
 @param distance, the distance to target (cm)
 @param bearing, the heading to the target (radians)
 @param finalorientation, the relative final orientation (radians)
 */
void NUWalk::getActionToPointWithOrientation(action_t* action, float distance, float bearing, float finalorientation)
{
    #if NUWALK_VERBOSITY > 1
        thelog << "NUWALK: getActionToPoint()" << endl;
    #endif
    
    TargetDistance = distance;
    TargetBearing = bearing;
    TargetRelativeX = distance*cos(bearing);
    TargetRelativeY = distance*sin(bearing);
    TargetRelativeOrientation = wrapAngle(finalorientation);
    
    const static unsigned char numpaths = 5;
    #if NUWALK_VERBOSITY > 0
        static const string indextopathname[numpaths] = {string("ArcArc"),string("StraightWOrientation"), string("Turn"), string("BackwardWOrientation"), string("SidewardWOrientation")};
    #endif
    
    // Firstly, calculate the path times
    float pathtimes[numpaths];
    pathtimes[0] = calculateArcArcPathTime();                   // fancy path
    pathtimes[1] = calculateStraightWOrientationPathTime();     // forwards
    pathtimes[2] = calculateTurnPathTime();                     // turn. This is basically a fail-safe, if we have reached the target then turn to the target orientation dammit
    pathtimes[3] = calculateBackwardWOrientationPathTime();     // backwards
    pathtimes[4] = calculateSidewardWOrientationPathTime();     // sidewards

    // Now decide which of those paths is the fastest
    int minpathindex = 0;
    float minpathtime = PathInfPathTime + 10;
    for (unsigned char i = 0; i < numpaths; i++)
    {
        if (pathtimes[i] < minpathtime)
        {
            minpathindex = i;
            minpathtime = pathtimes[i];
        }
    }
    
    #if NUWALK_VERBOSITY > 2
        thelog << "NUWALK: getActionToPointWithOrientation(). Path times: ";
        for (unsigned char i =0; i < numpaths; i++)
        {
            thelog << pathtimes[i] << ", ";
        }
        thelog << endl;
    #endif


    // now that I have picked the fastest path calculate the self.action and self.direction to best implement it
    if (minpathindex == 0)
        setArcArcAction(action);
    else if (minpathindex == 1)
        setStraightWOrientationAction(action);
    else if (minpathindex == 2)
        setTurnAction(action);
    else if (minpathindex == 3)
        setBackwardWOrientationAction(action);
    else if (minpathindex == 4)
        setSidewardWOrientationAction(action);
    
    #if NUWALK_VERBOSITY > 0
        thelog << "NUWALK: getActionToPointWithOrientation: (" << TargetDistance << ", " << TargetBearing << "), (" << TargetRelativeX << ", " << TargetRelativeY << ", " <<  TargetRelativeOrientation << ")" << endl;
        thelog << "NUWALK: getActionToPointWithOrientation: Path: " << indextopathname[minpathindex] << " action: " << action->Type << " " << action->Direction << endl;
    #endif    
}

/* Returns the time to reach the target in seconds. If the target can not be reached then PathInfPathTime is returned
 TargetDistance, the distance to the target in cm
 TargetBearing, the bearing to the target in radians
 TargetRelativeOrientation, the relative orientation desired at target
 TargetRelativeX, the x distance in cm (x-axis is forwards)
 TargetRelativeY, the y distance in cm (y-axis is left) 
 */
float NUWalk::calculateArcArcPathTime()
{
    float pathtime = 0;

    float ymin, ymax;       // the relative y-limits that require only a single arc
    if (TargetRelativeOrientation > 0)
    {
        ymin = PathMinArcRadius*(1-cos(TargetRelativeOrientation));
        ymax = PathMaxArcRadius*(1-cos(TargetRelativeOrientation));
    }
    else
    {
        ymax = -PathMinArcRadius*(1-cos(TargetRelativeOrientation));
        ymin = -PathMaxArcRadius*(1-cos(TargetRelativeOrientation));
    }

    if (TargetRelativeY > ymin && TargetRelativeY < ymax && TargetDistance < PathMinArcRadius + PathPositionAccuracy)
    {   // if the y position of the target is between these limits we are on course!
        float requiredradius = 0;               // the required radius of the connecting arc
        float requiredstraightdistance = 0;     // the required connecting straight segment
        if (fabs(TargetRelativeOrientation) < 0.001 || fabs((fabs(TargetRelativeOrientation) - PI)) < 0.001)
            requiredradius = PathMaxArcRadius + 10;
        else
            requiredradius = fabs(TargetRelativeY/(1 - cos(TargetRelativeOrientation)));
        requiredstraightdistance = TargetDistance*cos(TargetBearing) - requiredradius*sin(TargetRelativeOrientation);

        if (requiredstraightdistance < 3)
        {   // if the connecting straight segment is small or negative; then it is time to arc
            if (walkIsActive() && !(PreviousStep == NULL || PreviousStep->StepType == TYPE_FORWARD || PreviousStep->StepType == TYPE_ARC))
                pathtime += PathStopTime;

            float R = 0;        // the radius of the required arc to the TARGET
            if (fabs(TargetBearing) < 0.001 || fabs(fabs(TargetBearing) - PI) < 0.001)
                R = PathMaxArcRadius;
            else
                R = TargetDistance/(2*sin(TargetBearing));
            float l = R*TargetRelativeOrientation;
            pathtime += fabs(l)/PathArcSpeed;
        }
        else
        {   // if the connecting straight segment is long, then a straight segment is required
            if (walkIsActive() && !(PreviousStep == NULL || PreviousStep->StepType == TYPE_FORWARD || PreviousStep->StepType == TYPE_ARC))
                pathtime += PathStopTime;
            
            // Firstly consider the arc segment
            float R = 0;        // the required arc radius to the ORIENTATION
            if (fabs(TargetRelativeOrientation) < 0.001 or fabs(fabs(TargetRelativeOrientation) - PI) < 0.001)
                R = PathMaxArcRadius;
            else
                R = TargetRelativeY/(1 - cos(TargetRelativeOrientation));     // the radius of the arc
            float l = R*TargetRelativeOrientation;                            // length of the arc
            pathtime += fabs(l)/PathArcSpeed;

            // now the connecting straight segment
            float s = TargetDistance*cos(TargetBearing) - R*sin(TargetRelativeOrientation);
            pathtime += fabs(s)/PathStraightSpeed;
        }
    }
    else
    {   // so we are not on course, I need an initial arc to bring the target into line
        float desiredy = 0;         // the relative y coordinate I would like the target to be at
        if (fabs(TargetRelativeOrientation) > 0.9*PI)
            desiredy = -(PathMinArcRadius + 7)*(1-cos(TargetRelativeOrientation));
        else if (TargetRelativeOrientation > 0)
            desiredy = (PathMinArcRadius + 7)*(1-cos(TargetRelativeOrientation));
        else
            desiredy = -(PathMinArcRadius + 7)*(1-cos(TargetRelativeOrientation));

        // Now consider the inital arc/turn to get the target at the desiredy
        float phi = atan2(TargetRelativeY - desiredy, TargetRelativeX);    // the angle I need to turn so that the target is at the desiredy
        // I am going to be a little clever here, and only turn if I need to turn really sharp, otherwise do an arc
        if (fabs(phi) < 3*6.5/PathMinArcRadius)
        {   // so we don't need to turn too sharp, so do the usual walk on bearing type arc
            if (walkIsActive() && !(PreviousStep == NULL || PreviousStep->StepType == TYPE_FORWARD || PreviousStep->StepType == TYPE_ARC))
                pathtime += PathStopTime;
            
            pathtime+= 3*6.5/PathArcSpeed;
        }
        else
        {   // so I need to turn sharp, I better stop and turn
            if (walkIsActive() && !(PreviousStep == NULL || (PreviousStep->StepType == TYPE_TURN && sameSign(PreviousStep->StepDirection, phi))))
                pathtime += PathStopTime;
            
            pathtime = fabs(phi)/PathTurnSpeed;
        }

        // Now consider the final arc around to the final orientation
        float R = 0;
        if (fabs(TargetRelativeOrientation) < 0.001 || fabs(fabs(TargetRelativeOrientation) - PI) < 0.001)
            R = PathMaxArcRadius;
        else
            R = desiredy/(1 - cos(TargetRelativeOrientation));         // the radius of the arc
            
        float l = R*TargetRelativeOrientation;                     // length of the arc
        pathtime += fabs(l)/PathArcSpeed;

        // now the connecting straight segment
        float s = TargetDistance*cos(TargetBearing) - R*sin(TargetRelativeOrientation);
        pathtime += fabs(s)/PathStraightSpeed;
    }

    return pathtime;
}

/* Returns the time to reach the target in seconds. If the target can not be reached then PathInfPathTime is returned
 TargetDistance, the distance to the target in cm
 TargetBearing, the bearing to the target in radians
 TargetRelativeOrientation, the relative orientation desired at target
 TargetRelativeX, the x distance in cm (x-axis is forwards)
 TargetRelativeY, the y distance in cm (y-axis is left) 
 */
float NUWalk::calculateStraightWOrientationPathTime()
{   // Turn 
    float pathtime = 0;
    
    // I need to turn to the target if it is not 'directly ahead me'
    if ((TargetRelativeX > 0 && fabs(TargetRelativeY) <= PathPositionAccuracy) || (fabs(TargetBearing) < PathBearingAccuracy))
    {   // so the target is directly ahead me, just walk forward then turn to orientation
        if (walkIsActive() && !(PreviousStep == NULL || PreviousStep->StepType == TYPE_FORWARD || PreviousStep->StepType == TYPE_ARC))
            pathtime += PathStopTime;

        pathtime += fabs(TargetRelativeX)/PathStraightSpeed;

        // if I need to turn when I reach the target do so; this is easy because the orientation doesn't change
        if (fabs(TargetRelativeOrientation) > PathBearingAccuracy)
        {
            pathtime += PathStopTime;
            pathtime += 2*fabs(TargetRelativeOrientation)/PathTurnSpeed;
        }
    }
    else
    {   // so the target is not directly ahead of me; I need to turn to the target
        
        // Firstly, consider the initial turn to target
        // I am going to be a little clever here, and only turn if I need to turn really sharp, otherwise do an arc
        if (fabs(TargetBearing) < 2*6.5/PathMinArcRadius)
        {   // so we don't need to turn too sharp, so do the usual walk on bearing type arc
            if (walkIsActive() && !(PreviousStep == NULL || PreviousStep->StepType == TYPE_FORWARD || PreviousStep->StepType == TYPE_ARC))
                pathtime += PathStopTime;
            float l = 2*6.5;             // (by design) the arc is two steps long
            pathtime += fabs(l)/PathArcSpeed;
        }
        else
        {
            if (walkIsActive() && !(PreviousStep == NULL || (PreviousStep->StepType == TYPE_TURN && sameSign(PreviousStep->StepDirection, TargetBearing))))
                pathtime += PathStopTime;

            pathtime += fabs(TargetBearing)/PathTurnSpeed;
            pathtime += PathStopTime;
        }

        // Now the walk straight
        pathtime += fabs(TargetDistance)/PathStraightSpeed;

        // if I need to turn when I reach the target do so;
        if (fabs(TargetRelativeOrientation - TargetBearing) > PathBearingAccuracy)
        {
            pathtime += PathStopTime;
            pathtime += 2*fabs(TargetRelativeOrientation - TargetBearing)/PathTurnSpeed;
        }
    }
    return pathtime;
}

/* Returns the time to reach the target in seconds. If the target can not be reached then PathInfPathTime is returned
TargetDistance, the distance to the target in cm
TargetBearing, the bearing to the target in radians
TargetRelativeOrientation, the relative orientation desired at target
TargetRelativeX, the x distance in cm (x-axis is forwards)
TargetRelativeY, the y distance in cm (y-axis is left) 
*/
float NUWalk::calculateTurnPathTime()
{
    // ok when I am close to the target, but only the final orientation differs then I need only turn
    float pathtime = 0;
    // if the target is too far away then I can't just turn
    if (TargetDistance > PathPositionAccuracy || fabs(TargetRelativeOrientation) < 0.01)
        return PathInfPathTime;

    float turnangle = wrapAngle(TargetRelativeOrientation);
    pathtime += fabs(turnangle/PathTurnSpeed);
        
    if (walkIsActive() && !(PreviousStep == NULL || (PreviousStep->StepType == TYPE_TURN && sameSign(PreviousStep->StepDirection, turnangle))))
        pathtime += PathStopTime;

    return pathtime;
}

/* Returns the time to reach the target in seconds. If the target can not be reached then PathInfPathTime is returned
 TargetDistance, the distance to the target in cm
 TargetBearing, the bearing to the target in radians
 TargetRelativeOrientation, the relative orientation desired at target
 TargetRelativeX, the x distance in cm (x-axis is forwards)
 TargetRelativeY, the y distance in cm (y-axis is left) 
 */
float NUWalk::calculateBackwardWOrientationPathTime()
{
    float pathtime = 0;

    // I need to turn to the target if it is not 'directly behind me'
    if ((TargetRelativeX < 0 and fabs(TargetRelativeY) <= PathPositionAccuracy))
    {
        // so the target is directly behind me
        if (walkIsActive() && !(PreviousStep == NULL || PreviousStep->StepType == TYPE_BACKWARD))
            pathtime += PathStopTime;
            
        pathtime += fabs(TargetRelativeX)/PathBackwardSpeed;
            
        // if I need to turn when I reach the target do so; this is easy because I the orientation doesn't change
        if (fabs(TargetRelativeOrientation) > PathBearingAccuracy)
        {
            pathtime += PathStopTime;
            pathtime += fabs(TargetRelativeOrientation)/PathTurnSpeed;
        }
    }
    else
    {
        // so the target is not directly behind me; I need to turn
        float turnangle = 0;
        if (TargetBearing > 0)
            turnangle = TargetBearing - PI;
        else
            turnangle = TargetBearing + PI;
            
        if (walkIsActive() && !(PreviousStep == NULL || (PreviousStep->StepType == TYPE_TURN && sameSign(PreviousStep->StepDirection, turnangle))))
            pathtime += PathStopTime;
        
        pathtime += fabs(turnangle)/PathTurnSpeed;
        pathtime += PathStopTime;
            
        // then walk backwards
        pathtime += fabs(TargetDistance)/PathBackwardSpeed;
            
        // if I need to turn when I reach the target do so; this is easy because I the orientation doesn't change
        if (fabs(TargetRelativeOrientation - turnangle) > PathBearingAccuracy)
        {
            pathtime += PathStopTime;
            pathtime += fabs(TargetRelativeOrientation - turnangle)/PathTurnSpeed;
        }
    }
    return pathtime;
}

/* Returns the time to reach the target in seconds. If the target can not be reached then PathInfPathTime is returned
 TargetDistance, the distance to the target in cm
 TargetBearing, the bearing to the target in radians
 TargetRelativeOrientation, the relative orientation desired at target
 TargetRelativeX, the x distance in cm (x-axis is forwards)
 TargetRelativeY, the y distance in cm (y-axis is left) 
 */
float NUWalk::calculateSidewardWOrientationPathTime()
{
    float pathtime = 0;

    // I need to turn to the target if it is not 'directly beside me'
    if (fabs(TargetRelativeX) <= PathPositionAccuracy)
    {   // so the target is directly beside me
        if (walkIsActive() && !(PreviousStep == NULL || (PreviousStep->StepType == TYPE_SIDEWARD && sameSign(PreviousStep->StepDirection, TargetRelativeY))))
            pathtime += PathStopTime;
            
        pathtime += fabs(TargetRelativeY)/PathSidewardSpeed;
            
        // if I need to turn when I reach the target do so; this is easy because the orientation doesn't change
        if (fabs(TargetRelativeOrientation) > PathBearingAccuracy)
        {
            pathtime += PathStopTime;
            pathtime += fabs(TargetRelativeOrientation)/PathTurnSpeed;
        }
    }
    else
    {   // so the target is not directly behind me; I need to turn
        float turnangle = 0;
        if (TargetBearing > 0)
            turnangle = TargetBearing - PI/2.0;
        else
            turnangle = TargetBearing + PI/2.0;
            
        if (walkIsActive() && !(PreviousStep == NULL || (PreviousStep->StepType == TYPE_SIDEWARD && sameSign(PreviousStep->StepDirection, TargetRelativeY))))
            pathtime += PathStopTime;
            
        pathtime += fabs(turnangle)/PathTurnSpeed;
        pathtime += PathStopTime;
            
        // then walk sidewards
        pathtime += fabs(TargetDistance)/PathSidewardSpeed;
            
        // if I need to turn when I reach the target do so; this is easy because I the orientation doesn't change
        if (fabs(TargetRelativeOrientation - turnangle) > PathBearingAccuracy)
        {
            pathtime += PathStopTime;
            pathtime += fabs(TargetRelativeOrientation - turnangle)/PathTurnSpeed;
        }
    }
    return pathtime;
}

/* Sets action to implement the selected path
 @param action, action->Type and action->Direction are updated
 
 TargetDistance, the distance to the target in cm
 TargetBearing, the bearing to the target in radians
 TargetRelativeOrientation, the relative orientation desired at target
 TargetRelativeX, the x distance in cm (x-axis is forwards)
 TargetRelativeY, the y distance in cm (y-axis is left) 
 */
void NUWalk::setArcArcAction(action_t* action)
{

    float ymin, ymax;       // the relative y-limits that require only a single arc
    if (TargetRelativeOrientation > 0)
    {
        ymin = PathMinArcRadius*(1-cos(TargetRelativeOrientation));
        ymax = PathMaxArcRadius*(1-cos(TargetRelativeOrientation));
    }
    else
    {
        ymax = -PathMinArcRadius*(1-cos(TargetRelativeOrientation));
        ymin = -PathMaxArcRadius*(1-cos(TargetRelativeOrientation));
    }
    
    if (TargetRelativeY > ymin && TargetRelativeY < ymax && TargetDistance < PathMinArcRadius + PathPositionAccuracy)
    {   // if the y position of the target is between these limits we are on course!
        float requiredradius = 0;               // the required radius of the connecting arc
        float requiredstraightdistance = 0;     // the required connecting straight segment
        if (fabs(TargetRelativeOrientation) < 0.001 || fabs((fabs(TargetRelativeOrientation) - PI)) < 0.001)
            requiredradius = PathMaxArcRadius + 10;
        else
            requiredradius = fabs(TargetRelativeY/(1 - cos(TargetRelativeOrientation)));
        requiredstraightdistance = TargetDistance*cos(TargetBearing) - requiredradius*sin(TargetRelativeOrientation);
        
        if (requiredstraightdistance < 3)
        {   // if the connecting straight segment is small or negative; then it is time to arc
            if (fabs(TargetRelativeOrientation) < 0.001 or fabs(fabs(TargetRelativeOrientation) - PI) < 0.001)
            {
                action->Type = TYPE_FORWARD;
                action->Direction = 0;
            }
            else
            {
                action->Type = TYPE_ARC;
                action->Direction = TargetDistance/(2*sin(TargetRelativeOrientation));
            }
        }
        else
        {   // I can walk straight and have the path completed with an arc
            action->Type = TYPE_FORWARD;
            action->Direction = 0;
        }
    }
    else
    {
        float desiredy = 0;         // the relative y coordinate I would like the target to be at
        if (fabs(TargetRelativeOrientation) > 0.9*PI)
            desiredy = -(PathMinArcRadius + 7)*(1-cos(TargetRelativeOrientation));
        else if (TargetRelativeOrientation > 0)
            desiredy = (PathMinArcRadius + 7)*(1-cos(TargetRelativeOrientation));
        else
            desiredy = -(PathMinArcRadius + 7)*(1-cos(TargetRelativeOrientation));
        
        // Now consider the inital arc/turn to get the target at the desiredy
        float phi = atan2(TargetRelativeY - desiredy, TargetRelativeX);    // the angle I need to turn so that the target is at the desiredy
        
        // I am going to be a little clever here, and only turn if I need to turn really sharp, otherwise do an arc
        if (fabs(phi) < 3*6.5/PathMinArcRadius)
        {
            action->Type = TYPE_ARC;
            action->Direction = 3*6.5/phi;
        }
        else
        {
            action->Type = TYPE_TURN;
            action->Direction = phi;
        }
    }
}

/* Sets action to implement the selected path
 @param action, action->Type and action->Direction are updated
 
 TargetDistance, the distance to the target in cm
 TargetBearing, the bearing to the target in radians
 TargetRelativeOrientation, the relative orientation desired at target
 TargetRelativeX, the x distance in cm (x-axis is forwards)
 TargetRelativeY, the y distance in cm (y-axis is left) 
 */
void NUWalk::setStraightWOrientationAction(action_t* action)
{
    if (TargetDistance < PathPositionAccuracy)
    {   // if we are already at the target just face the orientation
        action->Type = TYPE_TURN;
        action->Direction = wrapAngle(TargetRelativeOrientation);
    }
    else
    {   // I need to turn to the target if it is not 'directly ahead me'
        if ((TargetRelativeX > 0 && fabs(TargetRelativeY) <= PathPositionAccuracy) || (fabs(TargetBearing) < PathBearingAccuracy))
        {   // so the target is directly ahead me
            action->Type = TYPE_FORWARD;
            action->Direction = 0;
        }
        else
        {   // so the target is not directly ahead of me; I need to turn
            if (fabs(TargetBearing) < 2*6.5/PathMinArcRadius)
            {
                action->Type = TYPE_ARC;
                action->Direction = 2*6.5/TargetBearing; // (by design) the arc is two steps long
            }
            else
            {
                action->Type = TYPE_TURN;
                action->Direction = TargetBearing;
            }
        }
    }
}

/* Sets action to implement the selected path
 @param action, action->Type and action->Direction are updated
 
 TargetDistance, the distance to the target in cm
 TargetBearing, the bearing to the target in radians
 TargetRelativeOrientation, the relative orientation desired at target
 TargetRelativeX, the x distance in cm (x-axis is forwards)
 TargetRelativeY, the y distance in cm (y-axis is left) 
 */
void NUWalk::setTurnAction(action_t* action)
{
    action->Type = TYPE_TURN;
    action->Direction = wrapAngle(TargetRelativeOrientation);
}

/* Sets action to implement the selected path
 @param action, action->Type and action->Direction are updated
 
 TargetDistance, the distance to the target in cm
 TargetBearing, the bearing to the target in radians
 TargetRelativeOrientation, the relative orientation desired at target
 TargetRelativeX, the x distance in cm (x-axis is forwards)
 TargetRelativeY, the y distance in cm (y-axis is left) 
 */
void NUWalk::setBackwardWOrientationAction(action_t* action)
{
    // I need to turn to the target if it is not 'directly behind me'
    if ((TargetRelativeX < 0 && fabs(TargetRelativeY) <= PathPositionAccuracy))
    {   // so the target is directly behind me
        if (TargetDistance > PathPositionAccuracy)
        {
            action->Type = TYPE_BACKWARD;
            action->Direction = 0;
        }
        else
        {
            action->Type = TYPE_TURN;
            action->Direction = TargetRelativeOrientation;
        }
    }
    else
    {   // so the target is not directly behind me; I need to turn
        if (TargetDistance > 1.5*PathPositionAccuracy)
        {   // if I am not at the target, turn to it
            float turnangle = 0;
            if (TargetBearing > 0)
                turnangle = TargetBearing - PI;
            else
                turnangle = TargetBearing + PI;
                
            action->Type = TYPE_TURN;
            action->Direction = turnangle;
        }
        else
        {   // I am already at the target, turn to face the orientation
            action->Type = TYPE_TURN;
            action->Direction = TargetRelativeOrientation;
        }
    }
}

/* Sets action to implement the selected path
 @param action, action->Type and action->Direction are updated
 
 TargetDistance, the distance to the target in cm
 TargetBearing, the bearing to the target in radians
 TargetRelativeOrientation, the relative orientation desired at target
 TargetRelativeX, the x distance in cm (x-axis is forwards)
 TargetRelativeY, the y distance in cm (y-axis is left) 
 */
void NUWalk::setSidewardWOrientationAction(action_t* action)
{
    if (fabs(TargetRelativeX) <= PathPositionAccuracy)
    {    // so the target is directly beside me
        if (TargetDistance < PathPositionAccuracy)
        {   // if we are at the target then turn to face the orientation
            action->Type = TYPE_TURN;
            action->Direction = TargetRelativeOrientation;
        }
        else
        {
            action->Type = TYPE_SIDEWARD;
            if (TargetRelativeY > 0)
                action->Direction = 1.0;
            else
                action->Direction = -1.0;
        }
    }
    else
    {   // so the target is not directly behind me; I need to turn
        if (TargetDistance < PathPositionAccuracy)
        {   // if we are at the target then turn to face the orientation
            action->Type = TYPE_TURN;
            action->Direction = TargetRelativeOrientation;
        }
        else
        {
            float turnangle = 0;
            if (TargetBearing > 0)
                turnangle = TargetBearing - PI/2.0;
            else
                turnangle = TargetBearing + PI/2.0;
            action->Type = TYPE_TURN;
            action->Direction = turnangle;
        }
    }
}


/********************************************************************************************************************************************************************
 getActionToPointWithMaintainOrientation()
 ********************************************************************************************************************************************************************/
void NUWalk::getActionToPointWithMaintainOrientation(action_t* action, float distance, float bearing, float desiredorientation)
{
#if NUWALK_VERBOSITY > 1
    thelog << "NUWALK: getActionToPoint()" << endl;
#endif
    
    TargetDistance = distance;
    TargetBearing = bearing;
    TargetRelativeX = distance*cos(bearing);
    TargetRelativeY = distance*sin(bearing);
    TargetRelativeOrientation = wrapAngle(desiredorientation);
    
    /* 
     */
    
    if (fabs(TargetRelativeOrientation) > PI/4.0)
    {
        // turn to face orientation
        action->Type = TYPE_TURN;
        action->Direction = TargetRelativeOrientation;
    }
    else
    {
        // select best step to go to point
        if (fabs(TargetRelativeY) >= fabs(TargetRelativeX))
        {   // walk sideways
            action->Type = TYPE_SIDEWARD;
            if (TargetRelativeY > 0)
            {   // walk left
                action->Direction = 1.0;
            }
            else
            {   // walk right
                action->Direction = -1.0;                
            }
        }
        else
        {   // walk straight
            if (TargetRelativeX > 0)
            {   // walk forwards
                action->Type = TYPE_FORWARD;
                action->Direction = 0;
            }
            else
            {   // walk backwards
                action->Type = TYPE_BACKWARD;
                action->Direction = 0;
            }
        }
    }
    return;
/* 
    const static unsigned char numpaths = 4;
#if NUWALK_VERBOSITY > 0
    static const string indextopathname[numpaths] = {string("StraightWMaintain"),string("Turn"), string("BackwardWMaintain"), string("SidewardWMaintain")};
#endif
    
    // Firstly, calculate the path times
    float pathtimes[numpaths];
    pathtimes[0] = calculateStraightWMaintainPathTime();
    pathtimes[1] = calculateTurnPathTime();
    pathtimes[2] = calculateBackwardWMaintainPathTime();             
    pathtimes[3] = calculateSidewardWMaintainPathTime();
    
    // Now decide which of those paths is the fastest
    int minpathindex = 0;
    float minpathtime = PathInfPathTime + 10;
    for (unsigned char i = 0; i < numpaths; i++)
    {
        if (pathtimes[i] < minpathtime)
        {
            minpathindex = i;
            minpathtime = pathtimes[i];
        }
    }
    
    #if NUWALK_VERBOSITY > 2
        thelog << "NUWALK: getActionToPointWithMaintainOrientation(). Path times: ";
        for (unsigned char i =0; i < numpaths; i++)
        {
            thelog << pathtimes[i] << ", ";
        }
        thelog << endl;
    #endif
    
    // now that I have picked the fastest path calculate the self.action and self.direction to best implement it
    if (minpathindex == 0)
        setStraightWMaintainAction(action);
    else if (minpathindex == 1)
        setTurnAction(action);
    else if (minpathindex == 2)
        setBackwardWMaintainAction(action);
    else if (minpathindex == 3)
        setSidewardWMaintainAction(action);
*/
    #if NUWALK_VERBOSITY > 0
        thelog << "NUWALK: getActionToPointWithMaintainOrientation: (" << TargetDistance << ", " << TargetBearing << "), (" << TargetRelativeX << ", " << TargetRelativeY << ", " <<  TargetRelativeOrientation << ")" << endl;
        thelog << "NUWALK: getActionToPointWithMaintainOrientation: " << " action: " << action->Type << " " << action->Direction << endl;
    #endif    
}

/* Returns the time to reach the target in seconds. If the target can not be reached then PathInfPathTime is returned
 TargetDistance, the distance to the target in cm
 TargetBearing, the bearing to the target in radians
 TargetRelativeOrientation, the relative orientation desired at target
 TargetRelativeX, the x distance in cm (x-axis is forwards)
 TargetRelativeY, the y distance in cm (y-axis is left) 
 */
float NUWalk::calculateStraightWMaintainPathTime()
{
    float pathtime = 0;

    if (fabs(TargetBearing - TargetRelativeOrientation) > PI/4.0)
        return PathInfPathTime;

    // I need to turn to the target if it is not 'directly ahead me'
    if ((TargetRelativeX > 0 && fabs(TargetRelativeY) <= PathPositionAccuracy) || (fabs(TargetBearing) < PathBearingAccuracy || fabs(TargetBearing) < PathBearingAccuracy))
    {   // so the target is directly ahead me, I can just walk forward into it
        if (walkIsActive() && !(PreviousStep == NULL || PreviousStep->StepType == TYPE_FORWARD || PreviousStep->StepType == TYPE_ARC))
            pathtime += PathStopTime;

        pathtime += fabs(TargetRelativeX)/PathStraightSpeed;

        // if I need to turn when I reach the target do so; this is easy because the orientation doesn't change
        if (fabs(TargetRelativeOrientation) > PathBearingAccuracy)
        {
            pathtime += PathStopTime;
            pathtime += fabs(TargetRelativeOrientation)/PathTurnSpeed;
        }
    }
    else
    {
        // so the target is not directly ahead of me; I need to turn to the target
        float turnangle = TargetBearing;

        if ((fabs(TargetBearing) < 2*6.5/PathMinArcRadius))
        {
            if (walkIsActive() && !(PreviousStep == NULL || PreviousStep->StepType == TYPE_FORWARD || PreviousStep->StepType == TYPE_ARC))
                pathtime += PathStopTime;
            float l = 2*6.5; // (by design) the arc is two steps long
            pathtime += 3*fabs(l)/PathArcSpeed;
        }
        else
        {
            if (walkIsActive() && !(PreviousStep == NULL || (PreviousStep->StepType == TYPE_TURN && sameSign(PreviousStep->StepDirection, turnangle))))
                pathtime += PathStopTime;

            pathtime += 3*fabs(turnangle)/PathTurnSpeed;
            pathtime += PathStopTime;
        }

        // then walk straight
        pathtime += fabs(TargetDistance)/PathStraightSpeed;

        // if I need to turn when I reach the target do so;
        if (fabs(TargetRelativeOrientation - turnangle) > PathBearingAccuracy)
        {
            pathtime += PathStopTime;
            pathtime += fabs(TargetRelativeOrientation - turnangle)/PathTurnSpeed;
        }
    }

    return pathtime;
}

/* Returns the time to reach the target in seconds. If the target can not be reached then PathInfPathTime is returned
 TargetDistance, the distance to the target in cm
 TargetBearing, the bearing to the target in radians
 TargetRelativeOrientation, the relative orientation desired at target
 TargetRelativeX, the x distance in cm (x-axis is forwards)
 TargetRelativeY, the y distance in cm (y-axis is left) 
 */
float NUWalk::calculateBackwardWMaintainPathTime()
{
    float pathtime = 0;

    if (fabs(fabs(TargetBearing - TargetRelativeOrientation) - PI) > PI/4.0)
        return PathInfPathTime;
    // Turn (optional) Backward Turn (optional)

    // I need to turn to the target if it is not 'directly behind me'
    if (TargetRelativeX < 0 && fabs(TargetRelativeY) <= PathPositionAccuracy)
    {   // so the target is directly behind me
        if (walkIsActive() && !(PreviousStep == NULL || PreviousStep->StepType == TYPE_BACKWARD))
            pathtime += PathStopTime;
    
        pathtime += fabs(TargetRelativeX)/PathBackwardSpeed;
    
        // if I need to turn when I reach the target do so; this is easy because I the orientation doesn't change
        if (fabs(TargetRelativeOrientation) > PathBearingAccuracy)
        {
            pathtime += PathStopTime;
            pathtime += fabs(TargetRelativeOrientation)/PathTurnSpeed;
        }
    }
    else
    {
    // so the target is not directly behind me; I need to turn
        float turnangle = 0;
        if (TargetBearing > 0)
            turnangle = TargetBearing - PI;
        else
            turnangle = TargetBearing + PI;
    
        if (walkIsActive() && !(PreviousStep == NULL || (PreviousStep->StepType == TYPE_TURN && sameSign(PreviousStep->StepDirection, turnangle))))
        pathtime += PathStopTime;
    
        pathtime += fabs(turnangle)/PathTurnSpeed;
        pathtime += PathStopTime;
    
        // then walk backwards
        pathtime += fabs(TargetDistance)/PathBackwardSpeed;
    
        // if I need to turn when I reach the target do so; this is easy because I the orientation doesn't change
        if (fabs(TargetRelativeOrientation - turnangle) > PathBearingAccuracy)
        {
            pathtime += PathStopTime;
            pathtime += fabs(TargetRelativeOrientation - turnangle)/PathTurnSpeed;
        }
    }
    return pathtime;
}

/* Returns the time to reach the target in seconds. If the target can not be reached then PathInfPathTime is returned
 TargetDistance, the distance to the target in cm
 TargetBearing, the bearing to the target in radians
 TargetRelativeOrientation, the relative orientation desired at target
 TargetRelativeX, the x distance in cm (x-axis is forwards)
 TargetRelativeY, the y distance in cm (y-axis is left) 
 */
float NUWalk::calculateSidewardWMaintainPathTime()
{
    float pathtime = 0;

    if (fabs(fabs(TargetBearing - TargetRelativeOrientation) - PI/2) > PI/4.0)
        return PathInfPathTime;

    // Turn (optional) Sideward Turn (optional)
    // I need to turn to the target if it is not 'directly beside me' and I am not facing the right way
    
    /* So, Turn Sideward Turn. The initial turn needs to turn to PI/2 of the TargetBearing!
     */
    
    if (fabs(TargetRelativeX) <= 2*PathPositionAccuracy)
    {
        // so the target is directly beside me
        if (walkIsActive() && !(PreviousStep == NULL || (PreviousStep->StepType == TYPE_SIDEWARD && sameSign(PreviousStep->StepDirection, TargetRelativeY))))
            pathtime += PathStopTime;
            
        pathtime += fabs(TargetRelativeY)/PathSidewardSpeed;
            
        // if I need to turn when I reach the target do so; this is easy because the orientation doesn't change
        if (fabs(TargetRelativeOrientation) > PathBearingAccuracy)
        {
            pathtime += PathStopTime;
            pathtime += fabs(TargetRelativeOrientation)/PathTurnSpeed;
        }
    }
    else
    {
        // so the target is not directly behind me; I need to turn
        float turnangle = 0;
        if (TargetBearing > 0)
            turnangle = TargetBearing - PI/2;
        else
            turnangle = TargetBearing + PI/2;
            
        if (walkIsActive() && !(PreviousStep == NULL || (PreviousStep->StepType == TYPE_TURN && sameSign(PreviousStep->StepDirection, turnangle))))
            pathtime += PathStopTime;
        
        pathtime += fabs(turnangle)/PathTurnSpeed;
        pathtime += PathStopTime;
            
        // then walk sidewards
        pathtime += fabs(TargetDistance)/PathSidewardSpeed;
            
            // if I need to turn when I reach the target do so; this is easy because I the orientation doesn't change
        if (fabs(TargetRelativeOrientation - turnangle) > PathBearingAccuracy)
        {
            pathtime += PathStopTime;
            pathtime += fabs(TargetRelativeOrientation - turnangle)/PathTurnSpeed;
        }
    }
    return pathtime;
}

/* Sets action to implement the selected path
 @param action, action->Type and action->Direction are updated
 
 TargetDistance, the distance to the target in cm
 TargetBearing, the bearing to the target in radians
 TargetRelativeOrientation, the relative orientation desired at target
 TargetRelativeX, the x distance in cm (x-axis is forwards)
 TargetRelativeY, the y distance in cm (y-axis is left) 
 */
void NUWalk::setStraightWMaintainAction(action_t* action)
{
    // if we are already at the target just face the orientation
    if (TargetDistance < PathPositionAccuracy && fabs(TargetRelativeOrientation) > 0.01)
    {
        action->Type = TYPE_TURN;
        action->Direction = TargetRelativeOrientation;
    }
    else
    {
        // Turn (optional) Straight Turn (optional)
        // I need to turn to the target if it is not 'directly ahead me'
        if ((TargetRelativeX > 0 && fabs(TargetRelativeY) <= PathPositionAccuracy) || (fabs(TargetBearing) < PathBearingAccuracy))
        {   // so the target is directly ahead me
            action->Type = TYPE_FORWARD;
            action->Direction = 0;
        }
        else
        {   // so the target is not directly ahead of me; I need to turn
            if (fabs(TargetBearing) < 2*6.5/PathMinArcRadius)
            {
                action->Type = TYPE_ARC;
                action->Direction = 2*6.5/TargetBearing; // (by design) the arc is two steps long
            }
            else
            {
                action->Type = TYPE_TURN;
                action->Direction = TargetBearing;
            }
        }
    }
}
            
/* Sets action to implement the selected path
 @param action, action->Type and action->Direction are updated
 
 TargetDistance, the distance to the target in cm
 TargetBearing, the bearing to the target in radians
 TargetRelativeOrientation, the relative orientation desired at target
 TargetRelativeX, the x distance in cm (x-axis is forwards)
 TargetRelativeY, the y distance in cm (y-axis is left) 
 */
void NUWalk::setBackwardWMaintainAction(action_t* action)
{
    // Turn (optional) Backward Turn (optional)

    // I need to turn to the target if it is not 'directly behind me'
    if (TargetRelativeX < 0 && fabs(TargetRelativeY) <= PathPositionAccuracy)
    {   // so the target is directly behind me
        if (TargetDistance > PathPositionAccuracy || fabs(TargetRelativeOrientation) < 0.01)
        {
            action->Type = TYPE_BACKWARD;
            action->Direction = 0;
        }
        else
        {
            action->Type = TYPE_TURN;
            action->Direction = TargetRelativeOrientation;
        }
    }
    else
    {   // so the target is not directly behind me; I need to turn
        if (TargetDistance > PathPositionAccuracy)
        {
            float turnangle = 0;
            if (TargetBearing > 0)
                turnangle = TargetBearing - PI;
            else
                turnangle = TargetBearing + PI;

            action->Type = TYPE_TURN;
            action->Direction = turnangle;
        }
        else
        {
            action->Type = TYPE_TURN;
            action->Direction = TargetRelativeOrientation;
        }
    }
}
            
/* Sets action to implement the selected path
 @param action, action->Type and action->Direction are updated
 
 TargetDistance, the distance to the target in cm
 TargetBearing, the bearing to the target in radians
 TargetRelativeOrientation, the relative orientation desired at target
 TargetRelativeX, the x distance in cm (x-axis is forwards)
 TargetRelativeY, the y distance in cm (y-axis is left) 
 */
void NUWalk::setSidewardWMaintainAction(action_t* action)
{
    // Turn (optional) Sideward Turn (optional)

    // I need to turn to the target if it is not 'directly beside me'
    if (fabs(TargetRelativeX) <= PathPositionAccuracy)
    {   // so the target is directly beside me
        if (TargetDistance < PathPositionAccuracy && fabs(TargetRelativeOrientation) > 0.01)
        {   // if we are at the target then turn to face the orientation
            action->Type = TYPE_TURN;
            action->Direction = TargetRelativeOrientation;
        }
        else
        {
            action->Type = TYPE_SIDEWARD;
            if (TargetRelativeY > 0)
                action->Direction = 1.0;
            else
                action->Direction = -1.0;
        }
    }
    else
    {   // so the target is not directly behind me; I need to turn
        if (TargetDistance < PathPositionAccuracy && fabs(TargetRelativeOrientation) > 0.01)
        {   // if we are at the target then turn to face the orientation
            action->Type = TYPE_TURN;
            action->Direction = TargetRelativeOrientation;
        }
        else
        {
            float turnangle = 0;
            if (TargetBearing > 0)
                turnangle = TargetBearing - PI/2.0;
            else
                turnangle = TargetBearing + PI/2.0;
                
            action->Type = TYPE_TURN;
            action->Direction = turnangle;
        }
    }
}

/* Uses PreviousStep to determine the direction of the last step and returns it
 */
float NUWalk::getPreviousDirection()
{
    float previousdirection = 0;
    // calculate direction (in radians) of previous step
    if (PreviousStep == NULL)       // careful if the previous step was NULL!
        previousdirection = 0;
    else if (PreviousStep->StepType == TYPE_FORWARD)
        previousdirection = 0;
    else if (PreviousStep->StepType == TYPE_BACKWARD)
        previousdirection = PI;
    else if (PreviousStep->StepType == TYPE_ARC)
        previousdirection = 13.0/PreviousStep->StepDirection;
    else if (PreviousStep->StepType == TYPE_TURN)
        previousdirection = PreviousStep->StepDirection;
    else if (PreviousStep->StepType == TYPE_SIDEWARD)
        previousdirection = PreviousStep->StepDirection*(PI/2.0);
    
    return previousdirection;
}

/* Uses the data stored in action to determine the direction action wants to go 
 */
float NUWalk::getDesiredDirection(action_t* action)
{
    float desireddirection = 0;
    // calculate direction (in radians) of previous step
    if (action == NULL)       // careful if the previous step was NULL!
        desireddirection = 0;
    else if (action->Type == TYPE_FORWARD)
        desireddirection = 0;
    else if (action->Type == TYPE_BACKWARD)
        desireddirection = PI;
    else if (action->Type == TYPE_ARC)
        desireddirection = 13.0/action->Direction;
    else if (action->Type == TYPE_TURN)
        desireddirection = action->Direction;
    else if (action->Type == TYPE_SIDEWARD)
        desireddirection = action->Direction*(PI/2.0);
    
    return desireddirection;
}


/* Returns true if the angles have the same sign */
bool NUWalk::sameSign(float angle1, float angle2)
{
    if ((angle1 == 0) || (angle2 == 0))
        return true;
    else if ((angle1 < 0 && angle2 < 0) || (angle1 > 0 && angle2 > 0))
        return true;
    else
        return false;
}

/* Returns a wrapped version of the angle in range +-PI*/
float NUWalk::wrapAngle(float angle)
{
    if (fabs(angle) < PI)
        return angle;
    else
    {
        // I need to add or subtract multiples of 2*pi until I get it between +-pi
        if (angle > 0)
            return angle - 2*PI*int((angle + PI)/(2*PI));
        else
            return angle - 2*PI*int((angle - PI)/(2*PI));
    }
}

/********************************************************************************************************************************************************************
 Higher-level step selection
 ********************************************************************************************************************************************************************/
void NUWalk::selectNextStep(action_t nextaction)
{
    #if NUWALK_VERBOSITY > 1
        thelog << "NUWALK: selectNextStep(). CurrentStep:" << CurrentStep << " PreviousStep:" << PreviousStep << endl;
    #endif
    
    if (CurrentStep == NULL || PreviousStep == NULL || PreviousStep->StepClass == CLASS_FSTOP || PreviousStep->StepClass == CLASS_NSTOP)
        selectStartStep(nextaction);
    else
    {
        if (nextaction.Type == TYPE_SIDEWARD || nextaction.Type == TYPE_TURN)
        {   // if the next action is NOT a 'forward' walk type
            char signofnextdirection = (char) (nextaction.Direction/fabs(nextaction.Direction));
            char signofpreviousdirection = (char) (PreviousStep->StepDirection/fabs(PreviousStep->StepDirection));
            if (nextaction.Type == PreviousStep->StepType && signofnextdirection == signofpreviousdirection)
            {   // if the type and direction are the same then I can try and link the steps
                
                if (PreviousStep->StepType == TYPE_SIDEWARD) 
                {   // if we are stepping sideways, we can only go to the natural next step
                    CurrentStep = PreviousStep->NaturalNext;
                }
                else
                {   // we are turning, so we need to link potentially different turn magnitiudes
                    // can only link a left turn when the previous step was a right (ie a left turn must lead with the left foot)
                    CurrentStep = Primitives[nextaction.Type]->getTurnStep(PreviousStep, nextaction.Direction);
                }
            }
            else
            {   // if I need to change type or direction here, so I might need to stop first
                if ((PreviousStep->StepDirection > 0 && PreviousStep->StepLeft == false) || (PreviousStep->StepDirection < 0 && PreviousStep->StepLeft == true))
                {
                    if (nextaction.Type == TYPE_SIDEWARD)
                        CurrentStep = Primitives[nextaction.Type]->getSideStep(PreviousStep, nextaction.Direction);
                    else
                        CurrentStep = Primitives[nextaction.Type]->getTurnStep(PreviousStep, nextaction.Direction);
                }
                else
                {
                    stop();
                    doStopStep();
                }
            }
        }
        else if (nextaction.Type == TYPE_BACKWARD)
        {
            if (PreviousStep->StepType == TYPE_BACKWARD)
                CurrentStep = PreviousStep->NaturalNext;
            else
            {
                stop();
                doStopStep();
            }
        }
        else
        {   // if the next action is a 'forward' walk type; I should always be able to link the steps if the previous step was also forward!
            if (PreviousStep->StepType == TYPE_SIDEWARD || PreviousStep->StepType == TYPE_TURN)
            {   // if the previous step is a sidestep or turn then I might need to stop first
                if ((PreviousStep->StepDirection > 0 && PreviousStep->StepLeft == false) || (PreviousStep->StepDirection < 0 && PreviousStep->StepLeft == true))
                    CurrentStep = Primitives[nextaction.Type]->getForwardStep(PreviousStep, nextaction.Direction, Primitives[TYPE_FORWARD]);
                else
                {
                    stop();
                    doStopStep();
                }
            }
            else if (PreviousStep->StepType == TYPE_BACKWARD)
            {   // if the previous step is a backwards then I need to stop
                stop();
                doStopStep();
            }
            else
                CurrentStep = Primitives[nextaction.Type]->getForwardStep(PreviousStep, nextaction.Direction, Primitives[TYPE_FORWARD]);
        }
    }
    #if NUWALK_VERBOSITY > 1
        if (CurrentStep != NULL)
            thelog << "NUWALK: selectNextStep(): " << CurrentStep->Name << endl;
        else
            thelog << "NUWALK: selectNextStep(): Selected a NULL step. This could be bad." << endl;
    #endif
    /*    if (CurrentStep == NULL)
     {
     thelog << "NUWALK: selectNextGStep(): ERROR: I have selected a NULL step. This is bad :(" << endl;
     CurrentStep = PreviousStep->NaturalNext;
     }*/
}

void NUWalk::selectStartStep(action_t action)
{   // this is a fairly easy problem, just pick the right walk type and direction
    CurrentStep = Primitives[action.Type]->getStartStep(action.Direction);
    #if NUWALK_VERBOSITY > 1
        thelog << "NUWALK: selectStartStep(): " << CurrentStep->Name << endl;
    #endif
}

void NUWalk::selectNextGStep()
{   
    #if NUWALK_VERBOSITY > 1
        thelog << "NUWALK: selectNextGStep(). CurrentStep:" << CurrentStep << " PreviousStep:" << PreviousStep << endl;
    #endif
    
    if (CurrentStep == NULL || PreviousStep == NULL || PreviousStep->StepClass == CLASS_FSTOP || PreviousStep->StepClass == CLASS_NSTOP)
        selectStartGStep();
    else
    {
        if (GWalkDirection == GWALK_DIRECTION_LEFT || GWalkDirection == GWALK_DIRECTION_RIGHT)
        {   // If I want to walk sideways, look at the previous step. If it was sideward and in the same direction continue, otherwise stop
            if (PreviousStep->StepType == TYPE_SIDEWARD)
            {
                if (PreviousStep->StepDirection > 0 && GWalkDirection == GWALK_DIRECTION_LEFT)
                    CurrentStep = PreviousStep->NaturalNext;
                else if (PreviousStep->StepDirection < 0 && GWalkDirection == GWALK_DIRECTION_RIGHT)
                    CurrentStep = PreviousStep->NaturalNext;
                else
                {
                    stop();
                    doStopStep();
                }
            }
            else
            {
                stop();
                doStopStep();
            }
        }
        else if (GWalkDirection == GWALK_DIRECTION_TURN_LEFT || GWalkDirection == GWALK_DIRECTION_TURN_RIGHT)
        {   // If I want to turn, look at the previous step. If it was a turn in the same direction continue, otherwise stop
            if (PreviousStep->StepType == TYPE_TURN)
            {
                if (PreviousStep->StepDirection > 0 && GWalkDirection == GWALK_DIRECTION_TURN_LEFT)
                    CurrentStep = PreviousStep->NaturalNext;
                else if (PreviousStep->StepDirection < 0 && GWalkDirection == GWALK_DIRECTION_TURN_RIGHT)
                    CurrentStep = PreviousStep->NaturalNext;
                else
                {
                    stop();
                    doStopStep();
                }
            }
            else
            {
                stop();
                doStopStep();
            }
        }
        else if (GWalkDirection == GWALK_DIRECTION_BACKWARD)
        {   // if I want to walk backwards, look at the previous step if it was backward continue, otherwise stop
            if (PreviousStep->StepType == TYPE_BACKWARD)
                CurrentStep = PreviousStep->NaturalNext;
            else
            {
                stop();
                doStopStep();
            }
        }
        else if (GWalkDirection == GWALK_DIRECTION_FORWARD)
        {   // if want to walk 'forward' I should always be able to link the steps if the previous step was also forward!
            if (PreviousStep->StepType == TYPE_SIDEWARD || PreviousStep->StepType == TYPE_TURN || PreviousStep->StepType == TYPE_BACKWARD)
            {   // if the previous step is not forwards then I need to stop first
                stop();
                doStopStep();
            }
            else
                CurrentStep = Primitives[TYPE_FORWARD]->getForwardStep(PreviousStep, 0, Primitives[TYPE_FORWARD]);
        }
    }
    
    #if NUWALK_VERBOSITY > 1
        if (CurrentStep != NULL)
            thelog << "NUWALK: selectNextGStep(): " << CurrentStep->Name << endl;
        else
            thelog << "NUWALK: selectNextGStep(): Selected a NULL step. This could be bad." << endl;
    #endif
    /*    if (CurrentStep == NULL)
     {
     thelog << "NUWALK: selectNextGStep(): ERROR: I have selected a NULL step. This is bad :(" << endl;
     CurrentStep = PreviousStep->NaturalNext;
     }*/
}
    
void NUWalk::selectStartGStep()
{
    // I need to translate gDirections into step types
    StepTypeEnum type;
    switch (GWalkDirection) {
        case GWALK_DIRECTION_FORWARD:
            type = TYPE_FORWARD;
            break;
        case GWALK_DIRECTION_BACKWARD:
            type = TYPE_BACKWARD;
            break;
        case GWALK_DIRECTION_LEFT:
        case GWALK_DIRECTION_RIGHT:
            type = TYPE_SIDEWARD;
            break;
        case GWALK_DIRECTION_TURN_LEFT:
        case GWALK_DIRECTION_TURN_RIGHT:
            type = TYPE_TURN;
            break;
        default:
            type = TYPE_FORWARD;
            break;
    }
    
    // I need to know whether the g is left or right directed; set direction positive for left and negative for right
    float direction = 0;
    if (GWalkDirection == GWALK_DIRECTION_FORWARD || GWalkDirection == GWALK_DIRECTION_BACKWARD)
        direction = 0;
    else if (GWalkDirection == GWALK_DIRECTION_LEFT || GWalkDirection == GWALK_DIRECTION_TURN_LEFT)
        direction = 1;
    else if (GWalkDirection == GWALK_DIRECTION_RIGHT || GWalkDirection == GWALK_DIRECTION_TURN_RIGHT)
        direction = -1;
    
    CurrentStep = Primitives[type]->getStartGStep(direction);
#if NUWALK_VERBOSITY > 1
    thelog << "NUWALK: selectStartStep(): " << CurrentStep->Name << endl;
#endif
}


void NUWalk::avoidObstacles(action_t* action)
{
    switch (action->Type) 
    {
        case TYPE_FORWARD:
        case TYPE_ARC:
            avoidObstaclesForwardWalk(action);
            break;
        case TYPE_BACKWARD:
            avoidObstaclesBackwardWalk(action);
            break;
        case TYPE_TURN:
            return;     // no dodge while turning
            break;
        case TYPE_SIDEWARD:
            avoidObstaclesSidewardWalk(action);
            break;
        default:
            break;
    }
    return;
}


void NUWalk::avoidObstaclesForwardWalk(action_t* action)
{
    /* When walking forwards I would like to keep walking forwards as much as possible
       So, the primary aim is to calculate an arc radius right and an arc radius left that will avoid the obstacle
       Then pick the path which dodges the least, and is closest to the desired direction
     */

    #if NUWALK_VERBOSITY > 2
        thelog << "NUWALK: avoidObstaclesForwardWalk()" << endl;
    #endif
    
    static const float distancethreshold = 65.0;            // distance threshold before an obstable is detected
    static const float naowidth = 20.0;                     // width of the nao (cm)
    
    static const float outerlimit = 1.15;                   // outer limit of ultrasonic range (radians)
    static const float innerlimit = 0.32;                   // inner limit of ultrasonic range (radians)
    
    static const float k_memory = 0.6;
    static const float k_inertia = 0.2;
    
    static float previousdirection = 0;                 // the current robot direction in radians (ie PreviousStep->StepDirection)
    static float desireddirection = 0;                  // the direction I want to go if there is no obstacle (radians)
    
    previousdirection = getPreviousDirection();
    desireddirection = getDesiredDirection(action);
    
    float ll = sonicValues[S_LL];
    float lr = sonicValues[S_LR];
    float rl = sonicValues[S_RL];
    float rr = sonicValues[S_RR];
    
    float dodgeleftangle = 0;           // calculated angle to dodge left
    float dodgerightangle = 0;          // calculated angle to dodge right
    float dodgeangle = 0;               // the angle we are going to dodge to
    
    // I need a special case; when all sensors are on;
    float mininner = 0;                 // minimum distance between the two centre sonars
    float minouter = 0;                 // minimum distance between the two outer sonars
    float dodgedistance = 0;            // the distance to the obstacle we are dodging
    if (lr < rl)
        mininner = lr;
    else
        mininner = rl;
    
    if (ll < rr)
        minouter = ll;
    else
        minouter = rr;
    
    if (fabs(minouter - mininner) > 7)
    {   // if the difference between the inner and outer pairs is greater than 7cm, dodge the closest obstacle

        // go around from left to right, and go to the left of the left most obstacle
        if (ll < distancethreshold)
        {
            dodgeleftangle = outerlimit + atan2(naowidth, ll);
        }
        else if (lr < distancethreshold || rl < distancethreshold)
        {
            if (lr < rl)
                dodgeleftangle = atan2(naowidth, lr);
            else
                dodgeleftangle = atan2(naowidth, rl);
        }
        else if (rr < distancethreshold)
        {
            dodgeleftangle = -innerlimit + atan2(naowidth, rr);
        }
        else
        {
            dodgeleftangle = -PI;
        }
        
        // go around from right to left, and go to the right of the right most obstacle
        if (rr < distancethreshold)
        {
            dodgerightangle = -outerlimit - atan2(naowidth, rr);
        }
        else if (rl < distancethreshold || lr < distancethreshold)
        {
            if (rl < lr)
                dodgerightangle = -atan2(naowidth, rl);
            else
                dodgerightangle = -atan2(naowidth, lr);
        }
        else if (ll < distancethreshold)
        {
            dodgerightangle = innerlimit - atan2(naowidth, ll);
        }
        else
        {
            dodgerightangle = PI;
        }
    }
    else
    {   // if all sensors have a approximately the same value, assume that there is only a single obstacle in front of the robot
        dodgeleftangle = innerlimit + atan2(naowidth, lr);
        dodgerightangle = -innerlimit - atan2(naowidth, rl);
    }
    
    #if NUWALK_VERBOSITY > 1
        thelog << "NUWALK: avoidObstaclesForwardWalk(). Dodge left: " << dodgeleftangle << " Dodge right: " << dodgerightangle << " Desired Direction: " << desireddirection << " Previous Direction: " << previousdirection << endl;
    #endif
    
    // first question: do I need to dodge
    if (desireddirection >= dodgeleftangle || desireddirection <= dodgerightangle)
    {
        #if NUWALK_VERBOSITY > 1
            thelog << "NUWALK: avoidObstaclesForwardWalk(). Decided dodge was unnecessary!" << endl;
        #endif
        return;
    }
    else
    {
        // compute the costs for the two directions, and pick the one with the least cost
        float leftcost = fabs(desireddirection - dodgeleftangle) + k_memory*fabs(previousdirection - dodgeleftangle) + k_inertia*fabs(dodgeleftangle);
        float rightcost = fabs(desireddirection - dodgerightangle) + k_memory*fabs(previousdirection - dodgerightangle) + k_inertia*fabs(dodgerightangle);
     
        #if NUWALK_VERBOSITY > 1
            thelog << "NUWALK: avoidObstaclesForwardWalk(). Decided dodge was necessary!" << " leftcost: " << leftcost << " rightcost: " << rightcost << endl;
        #endif
        
        if (leftcost < rightcost)
        {   // dodge left
            dodgeangle = dodgeleftangle;
        }
        else
        {   // dodge right
            dodgeangle = dodgerightangle;
        }
        
        if (sonicObstacleDistance <= 25 || fabs(dodgeangle) > 0.85)
        {   // I want to always sidestep when the obstacle is very close or the dodge angle is large
            action->Type = TYPE_SIDEWARD;
            if (dodgeangle >= 0)
                action->Direction = 1.0;
            else
                action->Direction = -1.0;
        }
        else
        {   // I want to sidestep if the dodge angle is too big to arc along
            
            action->Type = TYPE_ARC;
            action->Direction = 2*6.5/dodgeangle;
        }
        
        #if NUWALK_VERBOSITY > 1
            thelog << "NUWALK: avoidObstaclesForwardWalk(). Computed path: " << action->Type << " " << action->Direction << endl;
        #endif
    }
}

void NUWalk::avoidObstaclesBackwardWalk(action_t* action)
{
    return;
}

void NUWalk::avoidObstaclesSidewardWalk(action_t* action)
{
    return;
}

void NUWalk::avoidCollisions(action_t* action)
{
    if (collisionLeftFootFront == true || collisionRightFootFront == true)
    {
        action->Type = TYPE_BACKWARD;
        action->Direction = 0;
    }
        
    // Rules.
    // if kicking something with left foot, step backwards, 
    // if larm collision and not rarm then go right
    // if rarm collision and not larm then go left
    // if RR is 'clear' then step right sidewards until I clear the variable, otherwise go left
    // if kicking something with right foot, step backwards, 
    // if LL is 'clear' sidewards until I clear the variable
    return;
}

void NUWalk::doStopStep()
{
    if (PreviousStep->StepClass == CLASS_FSTOP || PreviousStep->StepClass == CLASS_NSTOP || ((PreviousStep->StepType == TYPE_SIDEWARD || PreviousStep->StepType == TYPE_TURN) && ((PreviousStep->StepDirection > 0 && PreviousStep->StepLeft == false) || (PreviousStep->StepDirection < 0 && PreviousStep->StepLeft == true))))
    {   // if the last step was a stop step, then we are now stopped or
        // if sideward or turn and the right direction then we are also stopped
        WalkStopped = true;
        WalkWaitingToStop = false;
        CurrentStep = NULL;     
    }
    else
        selectStopStep();
}

void NUWalk::selectStopStep()
{
    if (PreviousStep->StepType == TYPE_FORWARD || PreviousStep->StepType == TYPE_BACKWARD)
    {   // we can stop straight steps at anytime
        CurrentStep = PreviousStep->StopNext;
    }
    else if (PreviousStep->StepDirection < 0 && PreviousStep->StepLeft == false)
    {   // however, we can only stop a step moving RIGHT with a LEFT step, so the previous step must have been RIGHT
        CurrentStep = PreviousStep->StopNext;        // the stopping step depends only on the previous step, so the stopping step is selected on init
    }
    else if (PreviousStep->StepDirection > 0 && PreviousStep->StepLeft == true)
    {   // similarily, we can only stop moving LEFT if the previous step was LEFT
        CurrentStep = PreviousStep->StopNext;        // the stopping step depends only on the previous step, so the stopping step is selected on init
    }
    else
    {   // otherwise, keeping walking and stop later
        CurrentStep = PreviousStep->NaturalNext;
    }
#if NUWALK_VERBOSITY > 1
    thelog << "NUWALK: selectStopStep(): " << CurrentStep->Name << endl;
#endif
}

bool NUWalk::walkIsActive()
{
    if (WalkStopped == true)
        return false;
    else
        return true;
}

void NUWalk::stop()
{
    if (walkIsActive())
        WalkWaitingToStop = true;
}

void NUWalk::emergencyStop()
{
    WalkEnabled = false;
    WalkWaitingToStop = false;
    WalkStopped = true;
    GWalkDirection = GWALK_DIRECTION_UNDEFINED;
    CurrentStep = NULL;
    PreviousStep = NULL;
}

/*
 void NUWalk::getActionToPoint(action_t* action, float radius, float bearing)
 {
 // this logic needs to be updated very badly!
 if (fabs(bearing) < 0.19)
 {
 action->Type = TYPE_FORWARD;
 action->Direction = 0;
 }
 else if (fabs(bearing) < 1.2)
 {
 action->Type = TYPE_ARC;
 action->Direction = 2*6.5/bearing;
 }
 else
 {
 if (radius < 10)
 {
 action->Type = TYPE_SIDEWARD;
 if (bearing < 0)
 action->Direction = -1.0;
 else
 action->Direction = 1.0;
 }
 else
 {
 action->Type = TYPE_TURN;
 action->Direction = bearing;
 }
 }
 }*/

