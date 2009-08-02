/*
 *  stepplanner.h
 *  jwalk
 *
 *  Created by jason on 6/04/09.
 *  Copyright 2009 UoN. All rights reserved.
 *
 */

#ifndef NUWALK_H
#define NUWALK_H

#include "jwalkincludes.h"
#include "step.h"

class Actuators;
class JWalk;
//class Step;
class WalkPrimitive;

struct action_t
{
    StepTypeEnum Type;
    float Direction;
};

enum GWalkDirectionEnum
{
    GWALK_DIRECTION_UNDEFINED,
    GWALK_DIRECTION_FORWARD,
    GWALK_DIRECTION_BACKWARD,
    GWALK_DIRECTION_ARC,           // this is a tricky because it is the only on with a continuous input
    GWALK_DIRECTION_LEFT,
    GWALK_DIRECTION_RIGHT,
    GWALK_DIRECTION_TURN_LEFT,
    GWALK_DIRECTION_TURN_RIGHT,
    GWALK_NUM_DIRECTIONS
};

#define NUWALK_VERBOSITY       3

class NUWalk
{
    public:
        NUWalk(JWalk* pjwalk);
        ~NUWalk();
        
        // JMode commands
        void onBearing(float bearing, bool dodge=false);
        void toPoint(float distance, float bearing, bool dodge=false);
        void toPointWithOrientation(float distance, float bearing, float finalorientation, bool dodge=false);
        void toPointWithMaintainOrientation(float distance, float bearing, float desiredorientation, bool dodge=false);
    
        // GMode commands
        void goForward();
        void goBackward();
        void goLeft();
        void goRight();
        void goTurnLeft();
        void goTurnRight();
    
        void doWalk();
    
        bool walkIsActive();
        void stop();
        void emergencyStop();
    private:
        // Initialisation
        void initPrimitives();
    
        void enableWalk();
        void determineNextStep();
        void selectNextStep(action_t action);
        void selectStartStep(action_t action);
        void selectNextGStep();
        void selectStartGStep();
    
        void avoidObstacles(action_t* action);
        void avoidObstaclesForwardWalk(action_t* action);
        void avoidObstaclesBackwardWalk(action_t* action);
        void avoidObstaclesSidewardWalk(action_t* action);
    
        void avoidCollisions(action_t* action);
    
        void doStopStep();
        void selectStopStep();
    
        // Path Planning
        void getActionOnBearing(action_t* action, float bearing);
    
        void getActionToPoint(action_t* action, float distance, float bearing);
        float calculateStraightPathTime();
        float calculateArcPathTime();
        float calculateBackwardPathTime();
        float calculateTurnStopBackwardPathTime();
        float calculateSidewardPathTime();
        float calculateTurnStopSidewardPathTime();
        void setStraightAction(action_t* action);
        void setArcAction(action_t* action);
        void setBackwardAction(action_t* action);
        void setTurnStopBackwardAction(action_t* action);
        void setSidewardAction(action_t* action);
        void setTurnStopSidewardAction(action_t* action);
    
        void getActionToPointWithOrientation(action_t* action, float distance, float bearing, float finalorientation);
        float calculateArcArcPathTime();
        float calculateStraightWOrientationPathTime();
        float calculateTurnPathTime();
        float calculateBackwardWOrientationPathTime();
        float calculateSidewardWOrientationPathTime();
        void setArcArcAction(action_t* action);
        void setStraightWOrientationAction(action_t* action);
        void setTurnAction(action_t* action);
        void setBackwardWOrientationAction(action_t* action);
        void setSidewardWOrientationAction(action_t* action);
    
        void getActionToPointWithMaintainOrientation(action_t* action, float distance, float bearing, float desiredorientation);
        float calculateStraightWMaintainPathTime();
        float calculateBackwardWMaintainPathTime();             
        float calculateSidewardWMaintainPathTime();
        void setStraightWMaintainAction(action_t* action);
        void setBackwardWMaintainAction(action_t* action);
        void setSidewardWMaintainAction(action_t* action);
    
        float getPreviousDirection();
        float getDesiredDirection(action_t* action);
        bool sameSign(float angle1, float angle2);
        float wrapAngle(float angle);
    public:
        float* DefaultPositions;
        float* DefaultHardnesses;
    private:
        JWalk* jwalk;
        Actuators* actuators;
    
        WalkPrimitive* Primitives[TYPE_NUM_TYPES];
        Step* CurrentStep;
        Step* PreviousStep;
    
        float WalkR;
        float WalkBearing;
        float WalkFinalOrientation;
        float WalkDesiredOrientation;
        float WalkSpeed;
        bool WalkDodge;
    
        float TargetDistance;
        float TargetBearing;
        float TargetRelativeX;
        float TargetRelativeY;
        float TargetRelativeOrientation;
    
        float PathStraightSpeed;
        float PathBackwardSpeed;
        float PathArcSpeed;
        float PathSidewardSpeed;
        float PathTurnSpeed;
        float PathStopTime;
        
        float PathMinArcRadius;
        float PathMaxArcRadius;
        float PathPositionAccuracy;
        float PathBearingAccuracy;
        float PathInfPathTime;
    
        GWalkDirectionEnum GWalkDirection;
    
        float* CurrentPositions;
        float* CurrentHardnesses;

        bool WalkEnabled;
        bool WalkWaitingToStop;
        bool WalkStopped;
};

#endif

