/*
 *  walkprimitive.h
 *  jwalk
 *
 *  Created by jason on 28/04/09.
 *  Copyright 2009 UoN. All rights reserved.
 *
 */

#ifndef WALKPRIMITIVE_H
#define WALKPRIMITIVE_H

#define WALKPRIMITIVE_VERBOSITY         3

#include "step.h"
#include "nuwalk.h"

struct primitive_t 
{
    Step* lfirst;
    Step* rfirst;
    Step* lfollow;
    Step* rfollow;
    Step* lnormal;
    Step* rnormal;
    Step* lfstop;
    Step* rfstop;
    Step* lnstop;
    Step* rnstop;
};

class WalkPrimitive
{
    public:
        WalkPrimitive(StepTypeEnum steptype);
        ~WalkPrimitive();
    
        void getInitialPose(float** positions, float** hardnesses);
    
        Step* getStartStep(float direction);
        Step* getStartGStep(float direction);
        Step* getForwardStep(Step* previousstep, float direction, WalkPrimitive* forwardprimitives);
        Step* getTurnStep(Step* previousstep, float direction);
        Step* getSideStep(Step* previousstep, float direction);
    
    private:
        // Initialisation functions
        void loadSteps(string primitivename, unsigned char numvalues, string valuenames[]);
        void loadSubPrimitive(unsigned char index, string subprimitivename);
        void loadForwardSteps();
        void loadArcSteps();
        void loadDiagonalSteps();
        void loadSidewardSteps();
        void loadTurnSteps();
        void loadBackwardSteps();
    
        primitive_t* getSubStepFromDirection(float direction);
    
    public:
        primitive_t* Steps;         // This is an array of primitive_t whose length can only be determined on initialisation
        unsigned char StepsLength;
    private:
        StepTypeEnum StepType;
    
        float StepLeftDirectionToIndex;         // index = (int) StepLeftDirectionToIndex * direction + StepsLeftDirectionToIndexOffset.
        float StepRightDirectionToIndex;
        unsigned char StepLeftDirectionToIndexOffset, StepRightDirectionToIndexOffset;
        unsigned char StepsFirstLeftDirection, StepsLastLeftDirection;          // the index of the first and last left directed sub primitives
        unsigned char StepsFirstRightDirection, StepsLastRightDirection;
};

#endif
