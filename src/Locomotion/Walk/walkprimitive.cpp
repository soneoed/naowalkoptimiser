/*
 *  walkprimitive.cpp
 *  jwalk
 *
 *  Created by jason on 28/04/09.
 *  Copyright 2009 UoN. All rights reserved.
 *
 */

#include "walkprimitive.h"

#define NUM_FORWARD_PRIMITIVES          1
string typeForwardValueNames[NUM_FORWARD_PRIMITIVES] = {string("")};

#define NUM_ARC_PRIMITIVES              40
string typeArcValueNames[NUM_ARC_PRIMITIVES] = {string("Left005"), string("Left010"), string("Left015"), string("Left020"), string("Left025"), string("Left030"), string("Left035"), string("Left040"), string("Left045"), string("Left050"), string("Left055"), string("Left060"), string("Left065"), string("Left070"), string("Left075"), string("Left080"), string("Left085"), string("Left090"), string("Left095"), string("Left100"),      //eg. string("Left025") the radius in cm
                                                string("Right005"), string("Right010"), string("Right015"), string("Right020"), string("Right025"), string("Right030"), string("Right035"), string("Right040"), string("Right045"), string("Right050"), string("Right055"), string("Right060"), string("Right065"), string("Right070"), string("Right075"), string("Right080"), string("Right085"), string("Right090"), string("Right095"), string("Right100")};

#define NUM_DIAGONAL_PRIMITIVES         2
string typeDiagonalValueNames[NUM_DIAGONAL_PRIMITIVES] = {  string("Left"),            // eg. string("Left0.5") the amount of sideways in MILLIMETRES
                                                            string("Right")};

#define NUM_SIDEWARD_PRIMITIVES         2
string typeSidewardValueNames[NUM_SIDEWARD_PRIMITIVES] = {  string("Left"), 
                                                            string("Right")};

#define NUM_TURN_PRIMITIVES             42
string typeTurnValueNames[NUM_TURN_PRIMITIVES] = {  string("Left0.2"), string("Left0.3"), string("Left0.5"), string("Left0.6"), string("Left0.8"), string("Left0.9"), string("Left1.1"), string("Left1.3"),  string("Left1.4"),  string("Left1.6"), string("Left1.7"), string("Left1.9"), string("Left2.0"), string("Left2.2"), string("Left2.4"), string("Left2.5"), string("Left2.7"), string("Left2.8"),  string("Left3.0"),  string("Left3.1"),
                                                    string("Right0.2"), string("Right0.3"), string("Right0.5"), string("Right0.6"), string("Right0.8"), string("Right0.9"), string("Right1.1"), string("Right1.3"),  string("Right1.4"),  string("Right1.6"), string("Right1.7"), string("Right1.9"), string("Right2.0"), string("Right2.2"), string("Right2.4"), string("Right2.5"), string("Right2.7"), string("Right2.8"),  string("Right3.0"),  string("Right3.1"),
                                                    string("Leftggg"), string("Rightggg")};

#define NUM_BACKWARD_PRIMITIVES         1
string typeBackwardValueNames[NUM_BACKWARD_PRIMITIVES] = {string("")};

/* Now I want to modify this so that there are two more WalkTurn primitive_t. I imagine this will serve as a template for future hacks!
 
 Option 1:
    add to typeTurnValueNames and increase NUM_TURN_PRIMITIVES
        - this will make loading the steps easy. If I make them the last two, then it won't hurt anything
 */


/*! A Class that encapsulates a walk TYPE.
 
 The central member of this class is Steps. He is an array of several/many primitive_t structures.
 Each primitive_t contains all of the Steps for a single TYPE + VALUE (ie start, follow, normal, fstop and nstop steps)
 Each Step contains the frames (both position and hardness) for that Step.
 
 We access Steps using 
    [0] for WalkForward and WalkBackward, because there is only a single primitive_t
    [0] or [1] for WalkSideward where '0' will be Left and '1' will be right
    [(int) StepLeftDirectionToIndex * direction + StepsLeftDirectionToIndexOffset] for WalkArc and WalkTurn
 
 @param steptype, the type of primitive (eg. TYPE_FORWARD, TYPE_TURN etc)
 */
WalkPrimitive::WalkPrimitive(StepTypeEnum steptype)
{
#if WALKPRIMITIVE_VERBOSITY > 0
    thelog << "WALKPRIMITIVE: Creating walkprimitive: " << typeToTypeName[steptype] << endl;
#endif
    StepType = steptype;
    
    StepsFirstLeftDirection = 254;
    StepsLastLeftDirection = 0;          // the index of the first and last left directed sub primitives
    StepsFirstRightDirection = 254; 
    StepsLastRightDirection = 0;
    
    switch (StepType)
    {
        case TYPE_FORWARD:
            loadForwardSteps();
            break;
        case TYPE_ARC:
            loadArcSteps();
            break;
        /*case TYPE_DIAGONAL:
            loadDiagonalSteps();
            break;*/
        case TYPE_SIDEWARD:
            loadSidewardSteps();
            break;
        case TYPE_TURN:
            loadTurnSteps();
            break;
        case TYPE_BACKWARD:
            loadBackwardSteps();
            break;
        default:
            break;
    }
}

WalkPrimitive::~WalkPrimitive()
{
    // delete all of the steps
    for (unsigned char i=0; i<StepsLength; i++)
    {
        delete Steps[i].lfirst;
        delete Steps[i].rfirst;
        delete Steps[i].lfollow;
        delete Steps[i].rfollow;
        delete Steps[i].lnormal;
        delete Steps[i].rnormal;
        delete Steps[i].lfstop;
        delete Steps[i].rfstop;
        delete Steps[i].lnstop;
        delete Steps[i].rnstop;
    }
}

/*! Updates positions and hardnesses to point to their 'default' values
 */
void WalkPrimitive::getInitialPose(float** positions, float** hardnesses)
{
    Steps[0].lfirst->getInitialPose(positions, hardnesses);
}

/*! Returns a pointer to the start step in the specified direction
 @param direction, the direction to take the start step. 
 
 For forwards and backwards the direction is ignored, because it has no meaning.
 For sideward steps only the sign of the direction is used.
 For a turn the direction is the desired turn angle in radians
 For an arc the direction is the desired arc radius in cm
 
 */
Step* WalkPrimitive::getStartStep(float direction)
{
    #if WALKPRIMITIVE_VERBOSITY > 0
        thelog << "WALKPRIMITIVE: getStartStep()" << endl;
    #endif
    if (StepType == TYPE_FORWARD || StepType == TYPE_BACKWARD)
        return Steps[0].rfirst;
    else if (StepType == TYPE_ARC || StepType == TYPE_TURN)
    {
        primitive_t* substep = getSubStepFromDirection(direction);
        if (direction > 0)
            return substep->lfirst;
        else
            return substep->rfirst;
    }
    else if (StepType == TYPE_SIDEWARD)
    {
        if (direction > 0)
            return Steps[0].lfirst;
        else
            return Steps[1].rfirst;
    }
    else
        return Steps[0].rfirst;;
}

/*! Returns a pointer to the start gstep of THIS primitive
 @param direction, a positive value will produce g in the left direction and a negative value will produce g in the right direction
 
 Important: It is assumed that special gSteps are placed at the end of the Steps array. The second last one is left directed and the last one is right directed.
 */
Step* WalkPrimitive::getStartGStep(float direction)
{
#if WALKPRIMITIVE_VERBOSITY > 0
    thelog << "WALKPRIMITIVE: getStartGStep()" << endl;
#endif
    if (StepType == TYPE_FORWARD || StepType == TYPE_BACKWARD)
        return Steps[0].lfirst;
    else if (StepType == TYPE_ARC || StepType == TYPE_TURN)
    {
        if (direction > 0)
            return Steps[StepsLength-2].lfirst;
        else
            return Steps[StepsLength-1].rfirst;
        
    }
    else if (StepType == TYPE_SIDEWARD)
    {
        if (direction > 0)
            return Steps[0].lfirst;
        else
            return Steps[1].rfirst;
    }
    else
        return Steps[0].lfirst;;
}

/*! Returns a pointer to the turn step that should follow the previous step
 @param previousstep, a pointer to the step that has just completed
 @param direction, the desired turn angle in radians
 */
Step* WalkPrimitive::getTurnStep(Step* previousstep, float direction)
{
#if WALKPRIMITIVE_VERBOSITY > 0
    thelog << "WALKPRIMITIVE: getTurnStep()" << endl;
#endif
    /* The logic for getting a turn step is as follows:
        if the previous step was a stop step, then select the starting step in the desired direction
        if the previous step was not a stop step, then we need to take care that we start with the correct step at the correct time
            a left turn can only start with a left step
      
    */
    if (previousstep->StepClass == CLASS_FSTOP || previousstep->StepClass == CLASS_NSTOP)
        return getStartStep(direction);
    else if ((previousstep->StepType == TYPE_TURN || previousstep->StepType == TYPE_SIDEWARD) && ((previousstep->StepDirection < 0 && direction > 0) || (previousstep->StepDirection > 0 && direction < 0)))
        return getStartStep(direction);
        
    
    // I need to trigger a start step if there is a sudden change in direction
    
    if (previousstep->StepLeft == true)
    {   // if the last step was a left, then this one has to be a RIGHT STEP
        if (direction > 0)      
            return previousstep->NaturalNext;       // you can't lead a left turn with a right step
        else
        {
            primitive_t* substep = getSubStepFromDirection(direction);
            
            if (previousstep->StepClass == CLASS_START)
            {
                if (substep->rfollow == NULL)
                    return substep->rfstop;
                else
                    return substep->rfollow;
            }
            else if (previousstep->StepClass == CLASS_FSTOP || previousstep->StepClass == CLASS_NSTOP)
                return substep->rfirst;
            else
            {
                if (substep->rnormal == NULL)
                    return substep->rnstop;
                else
                    return substep->rnormal;
            }
        }
    }
    else
    {   // if the last step was a right, then this one has to be a LEFT STEP
        if (direction < 0)      
            return previousstep->NaturalNext;       // you can't lead a right turn with a left step
        else
        {
            primitive_t* substep = getSubStepFromDirection(direction);
            
            if (previousstep->StepClass == CLASS_START)
            {
                if (substep->lfollow == NULL)
                    return substep->lfstop;
                else
                    return substep->lfollow;
            }
            else if (previousstep->StepClass == CLASS_FSTOP || previousstep->StepClass == CLASS_NSTOP)
                return substep->lfirst;
            else
            {
                if (substep->lnormal == NULL)
                    return substep->lnstop;
                else
                    return substep->lnormal;
            }
        }
    }
}

Step* WalkPrimitive::getSideStep(Step* previousstep, float direction)
{
    if (previousstep->StepClass == CLASS_FSTOP || previousstep->StepClass == CLASS_NSTOP)
        return getStartStep(direction);
    else if ((previousstep->StepType == TYPE_TURN || previousstep->StepType == TYPE_SIDEWARD) && ((previousstep->StepDirection < 0 && direction > 0) || (previousstep->StepDirection > 0 && direction < 0)))
        return getStartStep(direction);
    else
        return previousstep->NaturalNext;
}


/*! Returns a pointer to the forward step that should follow the previous step
 @param previousstep, a pointer to the step that has just completed
 @param direction, for WalkArc steps this is the desired arc radius in cm, for WalkForward steps this is ignored
 @param forwardprimitives, a pointer to the WalkPrimitive containing the WalkForward primitive_t
 */
Step* WalkPrimitive::getForwardStep(Step* previousstep, float direction, WalkPrimitive* forwardprimitives)
{
#if WALKPRIMITIVE_VERBOSITY > 0
    thelog << "WALKPRIMITIVE: getForwardStep()" << endl;
#endif
    if (previousstep->StepLeft == true)
    {   // if the last step was a left, then this one has to be a RIGHT STEP
        
        // if the previous step was not in the forward direction, then we need to be careful to only end movements in the RIGHT direction
        if (previousstep->StepType != TYPE_FORWARD && previousstep->StepDirection > 0)
        {   // so the previous step was in the LEFT direction; it hasn't finished if the previous step was LEFT
            return previousstep->NaturalNext;
        }
        else
        {   // so the previous step was either in the RIGHT direction, or STRAIGHT; it is possible to move to the new direction :)
            if (StepType == TYPE_FORWARD)
            {   // if I want to go WalkForward I can at anytime
                if (previousstep->StepType == TYPE_TURN || previousstep->StepType == TYPE_SIDEWARD)
                    return Steps[0].rfirst;
                else if (previousstep->StepClass == CLASS_START)
                    return Steps[0].rfollow;
                else if (previousstep->StepClass == CLASS_FSTOP || previousstep->StepClass == CLASS_NSTOP)
                    return Steps[0].rfirst;
                else
                    return Steps[0].rnormal;
            }
            else if (direction > 0)
            {   // I can only start a movement in the LEFT direction with a left step, so I need to take a forward step before turning left :D
                if (previousstep->StepType == TYPE_TURN || previousstep->StepType == TYPE_SIDEWARD)
                    return forwardprimitives->Steps[0].rfirst;
                else if (previousstep->StepClass == CLASS_START)
                    return forwardprimitives->Steps[0].rfollow;
                else if (previousstep->StepClass == CLASS_FSTOP || previousstep->StepClass == CLASS_NSTOP)
                    return forwardprimitives->Steps[0].rfirst;
                else
                    return forwardprimitives->Steps[0].rnormal;
            }
            else
            {   // I can start a RIGHT movement with a RIGHT step, so :D
                primitive_t* substep = getSubStepFromDirection(direction);
                
                // select the right class of step based on the previous step's class
                if (previousstep->StepType == TYPE_TURN || previousstep->StepType == TYPE_SIDEWARD)
                    return substep->rfirst;
                else if (previousstep->StepClass == CLASS_START)
                    return substep->rfollow;
                else if (previousstep->StepClass == CLASS_FSTOP || previousstep->StepClass == CLASS_NSTOP)
                    return substep->rfirst;
                else
                    return substep->rnormal;
            }
        }
    }
    else
    {   // if the last step was a right, then this one has to be a LEFT STEP
        
        // if the previous step was not in the forward direction, then we need to be careful to only end movements in the LEFT direction
        if (previousstep->StepType != TYPE_FORWARD && previousstep->StepDirection < 0)
        {   // so the previous step was in the RIGHT direction; it hasn't finished if the previous step was RIGHT
            return previousstep->NaturalNext;
        }
        else
        {   // so the previous step was either in the LEFT direction, or STRAIGHT; it is possible to move to the new direction :)
            if (StepType == TYPE_FORWARD)
            {   // if I want to go WalkForward I can
                if (previousstep->StepType == TYPE_TURN || previousstep->StepType == TYPE_SIDEWARD)
                    return Steps[0].lfirst;
                else if (previousstep->StepClass == CLASS_START)
                    return Steps[0].lfollow;
                else if (previousstep->StepClass == CLASS_FSTOP || previousstep->StepClass == CLASS_NSTOP)
                    return Steps[0].lfirst;
                else
                    return Steps[0].lnormal;
            }
            else if (direction < 0)
            {   // I can only start a movement in the RIGHT direction with a RIGHT step, so I need to take a forward step before turning RIGHT :D
                if (previousstep->StepType == TYPE_TURN || previousstep->StepType == TYPE_SIDEWARD)
                    return forwardprimitives->Steps[0].lfirst;
                else if (previousstep->StepClass == CLASS_START)
                    return forwardprimitives->Steps[0].lfollow;
                else if (previousstep->StepClass == CLASS_FSTOP || previousstep->StepClass == CLASS_NSTOP)
                    return forwardprimitives->Steps[0].lfirst;
                else
                    return forwardprimitives->Steps[0].lnormal;
            }
            else
            {   // I can start a LEFT movement with a LEFT step, so :D
                primitive_t* substep = getSubStepFromDirection(direction);
                
                // select the right class of step based on the previous step's class
                if (previousstep->StepType == TYPE_TURN || previousstep->StepType == TYPE_SIDEWARD)
                    return substep->lfirst;
                else if (previousstep->StepClass == CLASS_START)
                    return substep->lfollow;
                else if (previousstep->StepClass == CLASS_FSTOP || previousstep->StepClass == CLASS_NSTOP)
                    return substep->lfirst;
                else
                    return substep->lnormal;
            }
        }        
    }
}

/* Returns a pointer to the step closest to the specified direction.
 Preconditions: StepLeftDirectionToIndex, StepRightDirectionToIndex, StepLeftDirectionToIndexOffset, StepRightDirectionToIndexOffset are set
 
 */
primitive_t* WalkPrimitive::getSubStepFromDirection(float direction)
{
    unsigned char index = 0;
    if (fabs(direction) < 0.01)
    {
        index = 0;
    }
    else if (direction > 0)
    {   // if the direction is positive then we select a left step
        index = (unsigned char) (StepLeftDirectionToIndex*direction + StepLeftDirectionToIndexOffset);
    
        // clip the index so that we don't get index out of bounds errors
        if (index > StepsLastLeftDirection)
            index = StepsLastLeftDirection;
    }
    else
    {   // if the direction is negative then we select a right step
        index = (unsigned char) (StepRightDirectionToIndex*direction + StepRightDirectionToIndexOffset);
        
        // clip the index so that we don't get index out of bounds errors
        if (index > StepsLastRightDirection)
            index = StepsLastRightDirection;
    }
    
    return &Steps[index];
}

/* Loads all of the steps associated with the specified primitive
 @param primitivename, the TYPE of the primitive, it will be the prefix for all filenames
 @param numvalues, the number of steps to be loaded
 @param valuenames[], the VALUE for each step (eg. "Left200" or "Right1.2")
 */
void WalkPrimitive::loadSteps(string primitivename, unsigned char numvalues, string valuenames[])
{
#if WALKPRIMITIVE_VERBOSITY > 1
    thelog << "WALKPRIMITIVE: Loading type: " << primitivename << endl;
#endif
    Steps = new primitive_t[numvalues];
    StepsLength = numvalues;
    
    for (unsigned char i=0; i<StepsLength; i++)
    {
        loadSubPrimitive(i, primitivename + valuenames[i]);
    }
    
    // I need to calculate direction to index constants here (assuming each sub primitive is equally spaced)
    if (StepType == TYPE_FORWARD || StepType == TYPE_BACKWARD)
    {
        StepLeftDirectionToIndex = 0;
        StepRightDirectionToIndex = 0;
    }
    else if (StepType == TYPE_SIDEWARD)
    {
        StepLeftDirectionToIndex = 0;
        StepRightDirectionToIndex = 1;
    }
    else
    {
        StepLeftDirectionToIndex = (StepsLastLeftDirection - StepsFirstLeftDirection)/(Steps[StepsLastLeftDirection].lfirst->StepDirection - Steps[StepsFirstLeftDirection].lfirst->StepDirection);
        StepRightDirectionToIndex = (StepsLastRightDirection - StepsFirstRightDirection)/(Steps[StepsLastRightDirection].lfirst->StepDirection - Steps[StepsFirstRightDirection].lfirst->StepDirection);
        StepLeftDirectionToIndexOffset = StepsFirstLeftDirection;
        StepRightDirectionToIndexOffset = StepsFirstRightDirection;
        
        #if WALKPRIMITIVE_VERBOSITY > 1
            thelog << "WALKPRIMITIVE: Calculating Arc/Turn Index Limits: left:" << StepLeftDirectionToIndex << ", " << (int)StepLeftDirectionToIndexOffset << " right:" << StepRightDirectionToIndex << ", " << (int)StepRightDirectionToIndexOffset << endl;
            thelog << "WALKPRIMITIVE: First left:" << Steps[StepLeftDirectionToIndexOffset].lfirst->Name << " Right first:" << Steps[StepRightDirectionToIndexOffset].rfirst->Name;
        #endif
    }
}

/* The work horse for loading steps. All steps for a single TYPE + VALUE are loaded here.
 
 Steps is an array of primitive_t structures, and each primitive_t contains all of the Steps for a single TYPE + VALUE
    So, what needs to happen here is the populating of the Steps[index] primitive_t with Steps.
 
 This is mostly straightforward, the only catch is primitive_t s with less than the usual number of files.
    Every primitive_t has a 'LeftStart' and 'RightStart'
    A primitive_t with only two Steps will not have a 'Follow', normal or a 'NStop'; I set these to NULL
    A primitive_t with only four Steps will not have either a Left (normal) or a Right (normal) depending on the VALUE
    A primitive_t with six steps has a complete set of Steps
 
 Additionally, this function sets the NaturalNext and StopNext for each Step in the primitive_t (care must be taken with primitive_t that are missing Steps)
 
 Finally, the four variables StepsFirstLeftDirection, StepsLastLeftDirection and StepsFirstRightDirection, StepsLastRightDirection are set
 
 @param index, the index into Steps where the steps will be stored
 @param subprimitivename, TYPE + VALUE of the subprimitive, so it will be something like WalkForward, WalkTurnLeft1.1, WalkArcLeft030 etc
 */
void WalkPrimitive::loadSubPrimitive(unsigned char index, string subprimitivename)
{
#if WALKPRIMITIVE_VERBOSITY > 1
    thelog << "WALKPRIMITIVE: Loading sub primitive: " << subprimitivename << " into " << (int)index << endl;
#endif

    // Create the steps
    Steps[index].lfirst = new Step(subprimitivename + LEFT_NAME + CLASS_START_NAME);
    Steps[index].rfirst = new Step(subprimitivename + RIGHT_NAME + CLASS_START_NAME);
    Steps[index].lfollow = new Step(subprimitivename + LEFT_NAME + CLASS_FOLLOW_NAME);
    Steps[index].rfollow = new Step(subprimitivename + RIGHT_NAME + CLASS_FOLLOW_NAME);
    
    if (Steps[index].lfollow->StepLength == 0 || Steps[index].rfollow->StepLength == 0)
    {   // then this primitive only has '2' steps (6 files, 4 are useless), so there are no follow, normal or nstop steps
        Steps[index].lfollow = NULL;
        Steps[index].rfollow = NULL;
        Steps[index].lnormal = NULL;
        Steps[index].rnormal = NULL;
        Steps[index].lfstop = new Step(subprimitivename + LEFT_NAME + CLASS_FSTOP_NAME);
        Steps[index].rfstop = new Step(subprimitivename + RIGHT_NAME + CLASS_FSTOP_NAME);
        Steps[index].lnstop = NULL;
        Steps[index].rnstop = NULL;
        
        // Now link them all together
        Steps[index].lfirst->NaturalNext = Steps[index].rfstop;
        Steps[index].lfirst->StopNext = Steps[index].rfstop;
        Steps[index].rfirst->NaturalNext = Steps[index].lfstop;
        Steps[index].rfirst->StopNext = Steps[index].lfstop;
       
        Steps[index].lfstop->NaturalNext = Steps[index].rfirst;
        Steps[index].rfstop->NaturalNext = Steps[index].lfirst;
    }
    else
    {
        // then this primitive has at least '4' steps, so there will be lfollow and rfollow
        Steps[index].lnormal = new Step(subprimitivename + LEFT_NAME + CLASS_NORMAL_NAME);          // Note. that this step could be absent
        Steps[index].rnormal = new Step(subprimitivename + RIGHT_NAME + CLASS_NORMAL_NAME);         // Note. that this step could be absent
        Steps[index].lfstop = new Step(subprimitivename + LEFT_NAME + CLASS_FSTOP_NAME);
        Steps[index].rfstop = new Step(subprimitivename + RIGHT_NAME + CLASS_FSTOP_NAME);
        Steps[index].lnstop = new Step(subprimitivename + LEFT_NAME + CLASS_NSTOP_NAME);
        Steps[index].rnstop = new Step(subprimitivename + RIGHT_NAME + CLASS_NSTOP_NAME);
        
        // Now link them all together
        // Linking the starting steps is the same for both '4' and '6' step turns
        Steps[index].lfirst->NaturalNext = Steps[index].rfollow;
        Steps[index].lfirst->StopNext = Steps[index].rfstop;
        Steps[index].rfirst->NaturalNext = Steps[index].lfollow;
        Steps[index].rfirst->StopNext = Steps[index].lfstop;
        
        // However, linking the follow and normal steps of '4' and '6' step turns is different.
        
        if (Steps[index].lnormal->StepLength == 0)
        {   // if there is no lnormal, that is because we have 4 steps; rstart, lfollow, rnormal, lnstop
            Steps[index].lnormal = NULL;
            
            // Firstly do the follow steps
            Steps[index].lfollow->NaturalNext = Steps[index].rnormal;
            Steps[index].lfollow->StopNext = Steps[index].rnstop;
            Steps[index].rfollow->NaturalNext = Steps[index].lnstop;
            Steps[index].rfollow->StopNext = Steps[index].lnstop;
            
            // Now link the normal steps
            Steps[index].rnormal->NaturalNext = Steps[index].lnstop;
            Steps[index].rnormal->StopNext = Steps[index].lnstop;
            
        } 
        else if (Steps[index].rnormal->StepLength == 0)
        {   // if there is no rnormal, that is because we have 4 steps; lstart, rfollow, lnormal, rnstop
            Steps[index].rnormal = NULL;
            
            // Firstly do the follow steps
            Steps[index].lfollow->NaturalNext = Steps[index].rnstop;
            Steps[index].lfollow->StopNext = Steps[index].rnstop;
            Steps[index].rfollow->NaturalNext = Steps[index].lnormal;
            Steps[index].rfollow->StopNext = Steps[index].lnstop;
            
            // Now link the normal steps
            Steps[index].lnormal->NaturalNext = Steps[index].rnstop;
            Steps[index].lnormal->StopNext = Steps[index].rnstop;
        }
        else
        {   // there are 6 steps, so all steps are present
            // Firstly do the follow steps
            Steps[index].lfollow->NaturalNext = Steps[index].rnormal;
            Steps[index].lfollow->StopNext = Steps[index].rnstop;
            Steps[index].rfollow->NaturalNext = Steps[index].lnormal;
            Steps[index].rfollow->StopNext = Steps[index].lnstop;
            
            // Now link the normal steps
            Steps[index].lnormal->NaturalNext = Steps[index].rnormal;
            Steps[index].lnormal->StopNext = Steps[index].rnstop;
            Steps[index].rnormal->NaturalNext = Steps[index].lnormal;
            Steps[index].rnormal->StopNext = Steps[index].lnstop;
        }

        Steps[index].lfstop->NaturalNext = Steps[index].rfirst;
        Steps[index].rfstop->NaturalNext = Steps[index].lfirst;
        Steps[index].lnstop->NaturalNext = Steps[index].rfirst;
        Steps[index].rnstop->NaturalNext = Steps[index].lfirst;
    }
    
    // keep track of the first and last index in each direction
    if (fabs(Steps[index].lfirst->StepDirection) < LEFT_INF - 1)
    {   // only count steps that have directions less than infinity. I will make sure that the infinite direction steps come after the right direction steps
        if (Steps[index].lfirst->StepDirection > 0)
        {
            StepsLastLeftDirection = index;
            if (index < StepsFirstLeftDirection)
                StepsFirstLeftDirection = index;
        }
        else
        {
            StepsLastRightDirection = index;
            if (index < StepsFirstRightDirection)
                StepsFirstRightDirection = index;
        }
    }
    
}

/* Loads all of the WalkForward steps into Steps[]
 */
void WalkPrimitive::loadForwardSteps()
{
    loadSteps(TYPE_FORWARD_NAME, NUM_FORWARD_PRIMITIVES, typeForwardValueNames);
}

/* Loads all of the WalkArc steps into Steps[]
 */
void WalkPrimitive::loadArcSteps()
{
    loadSteps(TYPE_ARC_NAME, NUM_ARC_PRIMITIVES, typeArcValueNames);
}

/* Loads all of the WalkDiagonal steps into Steps[]
 */
void WalkPrimitive::loadDiagonalSteps()
{
    /*loadSteps(TYPE_DIAGONAL_NAME, NUM_DIAGONAL_PRIMITIVES, typeDiagonalValueNames);*/
}

/* Loads all of the WalkSideward steps into Steps[]
 */
void WalkPrimitive::loadSidewardSteps()
{
    loadSteps(TYPE_SIDEWARD_NAME, NUM_SIDEWARD_PRIMITIVES, typeSidewardValueNames);
}

/* Loads all of the WalkTurn steps into Steps[]
 */
void WalkPrimitive::loadTurnSteps()
{
    loadSteps(TYPE_TURN_NAME, NUM_TURN_PRIMITIVES, typeTurnValueNames);
}

/* Loads all of the WalkBacward steps into Steps[]
 */
void WalkPrimitive::loadBackwardSteps()
{
    loadSteps(TYPE_BACKWARD_NAME, NUM_BACKWARD_PRIMITIVES, typeBackwardValueNames);
}


