/*
 *  alwalk.h
 *  jwalk
 *
 *  Created by jason on 6/04/09.
 *  Copyright 2009 UoN. All rights reserved.
 *
 */

#include "jwalkincludes.h"
#include "actuators.h"

#define ALWALK_VERBOSITY        3

#define CONFIG_LOCATION         string("/home/root/jasondata/alwalkconfig/")

enum ALWalkPrimitiveEnum
{
    ALWALK_PRIMITIVE_FORWARD,
    ALWALK_PRIMITIVE_BACKWARD,
    ALWALK_PRIMITIVE_ARC,
    ALWALK_PRIMITIVE_SIDEWAYS,
    ALWALK_PRIMITIVE_TURN,
    ALWALK_NUM_PRIMITIVES
};

enum ALWalkDirectionEnum
{
    ALWALK_DIRECTION_UNDEFINED,
    ALWALK_DIRECTION_FORWARD,
    ALWALK_DIRECTION_BACKWARD,
    ALWALK_DIRECTION_ARC,           // this is a tricky because it is the only on with a continuous input
    ALWALK_DIRECTION_LEFT,
    ALWALK_DIRECTION_RIGHT,
    ALWALK_DIRECTION_TURN_LEFT,
    ALWALK_DIRECTION_TURN_RIGHT,
    ALWALK_NUM_DIRECTIONS
};

struct alwalkconfig_t
{
    float Hardnesses[ALIAS_TARGETS_NOT_HEAD_LENGTH];
    int CyclesPerStep;
    float StepLength;                   // the step length
    float StepHeight;                   // the step height
    float StepSide;                     // the size of sideways step
    float StepTurn;                     // the angle of a turn step
    float ZMPX;                         // ZMP in the forwards direction
    float ZMPY;                         // ZMP in the backwards direction
    float LHipBacklash;     
    float RHipBacklash;
    float HipHeight;
    float TorsoOrientation;             // Torso orientation; positive is forwards
    float DistanceCommandFactor;        // such that alMotion((desired distance) * DistanceCommandFactor)
    float AngleCommandFactor;           // such that alMotion((desired angle) * AngleCommandFactor)
    float DistanceOdometryFactor;       // such that actual distance = DistanceOdometryFactor * alMotionOdometry()
    float AngleOdometryFactor;          // such that actual angle = AngleOdometryFactor * alMotionOdometry()
};

class ALWalk
{
    public:
        ALWalk();
        ~ALWalk();
        
        void setStiffnessNotHead(float values[]);
        int goToAnglesNotHead(float positions[], int time);
        int goToAnglesWithVelocityNotHead(float positions[], float velocity);
    
        void walkStraight(float distance);
        void walkArc(float angle, float radius);
        void walkSideways(float distance);
        void turn(float angle);
    
        void goForward();
        void goBackward();
        void goArc(float angle);
        void goLeft();
        void goRight();
        void goTurnLeft();
        void goTurnRight();
        bool gIsActive();
    
        bool walkIsActive();
        void waitUntilFinished();
        void stop();
    
        void innerTest();

    private:
        // Initialisation
        void initSelf();
        void readConfigs();
        void readConfig(string name, alwalkconfig_t* config);
        int lineToFloats(string line, float values[]);
    
        void configALMotion(ALWalkPrimitiveEnum primitive);
    public:
        float *alWalkHardnesses;
    private:
        alwalkconfig_t alWalkConfigs[ALWALK_NUM_PRIMITIVES];
        ALWalkDirectionEnum alWalkGoDirection;
    
        
};



