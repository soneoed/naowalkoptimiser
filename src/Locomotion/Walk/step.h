/** A Step class
 * @author Jason Kulk
 *
 * Version : $Id: step.h,v 1.2 2009/05/31 03:52:53 jason Exp $
 */

#ifndef STEP_H
#define STEP_H

#include "jwalkincludes.h"
#include "actuators.h"

#define STEP_VERBOSITY                      0

#define STEP_LOCATION                       string("/home/root/jasondata/")
#define STEP_MAX_LENGTH                     255

extern string typeToTypeName[];
extern string classToClassName[];

// These definitions are used to determine the filenames of each step
// My naming format is as follows:
// TYPE    VALUE   (Left/Right)    CLASS.csv
//      where   TYPE is the name of the step eg. WalkForward, WalkTurn, etc
//              VALUE is the 3 digit value associated with the step. 
//                  For arcs it is the radius LeftXXX in cm. For turns it is the turn angle RightX.X in radians
//              Left/Right is whether the step is a Left or Right step
//              CLASS is the class of step eg. Start, Follow, FStop, NStop
#define TYPE_NONE_NAME              string("NoStep")
#define TYPE_FORWARD_NAME           string("WalkForward")
#define TYPE_ARC_NAME               string("WalkArc")
#define TYPE_DIAGONAL_NAME          string("WalkDiagonal")
#define TYPE_SIDEWARD_NAME          string("WalkSideward")
#define TYPE_TURN_NAME              string("WalkTurn")
#define TYPE_BACKWARD_NAME          string("WalkBackward")

#define CLASS_START_NAME            string("Start")
#define CLASS_FOLLOW_NAME           string("Follow")
#define CLASS_NORMAL_NAME           string("")
#define CLASS_FSTOP_NAME            string("FStop")
#define CLASS_NSTOP_NAME            string("NStop")

#define LEFT_NAME                   string("Left")
#define RIGHT_NAME                  string("Right")

// Special gWalks will have StepDirections set to +/- 1000
#define LEFT_INF                    1000
#define RIGHT_INF                   -1000

enum StepClassEnum 
{
    CLASS_START,
    CLASS_FOLLOW,
    CLASS_NORMAL,
    CLASS_FSTOP,
    CLASS_NSTOP,
    CLASS_NUM_CLASSES
};

enum StepTypeEnum 
{
    TYPE_NONE,      // that is don't take a step
    TYPE_FORWARD,
    TYPE_ARC,
    //TYPE_DIAGONAL,
    TYPE_SIDEWARD,
    TYPE_TURN,
    TYPE_BACKWARD,
    TYPE_NUM_TYPES
};

class Step
{
    public:
        Step(string name);
        ~Step();
    
        void getNextFrame(float** positions, float** hardnesses);
        bool hasEnded();
        void getInitialPose(float** positions, float** hardnesses);
    
    private:
        void setStepProperties();
        void resetStep();
        void loadStep();                                    // load step and pose
        void readCSV(ifstream& file, float data[STEP_MAX_LENGTH][ALIAS_TARGETS_NOT_HEAD_LENGTH]);       // read step file
        void readCSV(ifstream& file, float data[ALIAS_TARGETS_NOT_HEAD_LENGTH]);                        // read pose file
    
    public:
        string Name;                        // the step's name (this will be used by higher levels to plan and connect steps together)
        float StepPositions[STEP_MAX_LENGTH][ALIAS_TARGETS_NOT_HEAD_LENGTH];
        float StepHardnesses[STEP_MAX_LENGTH][ALIAS_HARDNESS_NOT_HEAD_LENGTH];
        unsigned char StepLength;           // the step length in dcm cycles
        
        float StepDirection;                // the direction of the step (this is obtained from the 'Value' field in the filename, and has different meanings depending on the Type
        StepClassEnum StepClass;            // the class (Start, Follow, Normal, FStop, NStop)
        StepTypeEnum StepType;              // the type (Forward, Arc, Diagonal, Sideward, Turn, Backward)
        bool StepLeft;                      // true if this is a left step
        Step* NaturalNext;                  // the step which naturally follows this one
        Step* StopNext;                     // the stopping step which would follow this on if I wanted to stop 
    
    private:
        unsigned char StepCurrentIndex;     // the current index into StepPositions and StepHardnesses
        
        float StepInitialPose[ALIAS_TARGETS_NOT_HEAD_LENGTH];
};

#endif
