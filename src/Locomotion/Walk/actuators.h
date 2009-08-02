/** A low level wrapper of the alDcm
 * @author Jason Kulk
 *
 * Version : $Id: actuators.h,v 1.5 2009/05/31 03:58:58 jason Exp $
 */

#ifndef ACTUATORS_H
#define ACTUATORS_H

#include "jwalkincludes.h"
#include "sensors.h"

#define ACTUATORS_VERBOSITY                 0
#define ACTUATORS_CLEARAFTER                0           // use this to switch between the aldcm's ClearAfter and ClearAll modes

// These offsets only work for 'not head' data, add 2 to work with 'all' data
#define JOINT_OFFSET_LARM                   0
#define JOINT_OFFSET_RARM                   4           // so that jointPositions[JOINT_OFFSET_RARM] is the first joint in the arm
#define JOINT_OFFSET_LLEG                   8
#define JOINT_OFFSET_RLEG                   14

#define ALIAS_HARDNESS_ALL                  string("Actuators/Hardness/All")
#define ALIAS_HARDNESS_ALL_LENGTH           J_NUM_JOINTS
#define ALIAS_HARDNESS_NOT_HEAD             string("Actuators/Hardness/NotHead")
#define ALIAS_HARDNESS_NOT_HEAD_LENGTH      J_NUM_JOINTS - 2
#define ALIAS_HARDNESS_HEAD                 string("Actuators/Hardness/Head")
#define ALIAS_HARDNESS_HEAD_LENGTH          2

#define ALIAS_TARGETS_ALL                   string("Actuators/Targets/All")
#define ALIAS_TARGETS_ALL_LENGTH            J_NUM_JOINTS
#define ALIAS_TARGETS_NOT_HEAD              string("Actuators/Targets/NotHead")
#define ALIAS_TARGETS_NOT_HEAD_LENGTH       J_NUM_JOINTS - 2
#define ALIAS_TARGETS_HEAD                 string("Actuators/Targets/Head")
#define ALIAS_TARGETS_HEAD_LENGTH          2

// Convert the index defined in sensors.h to the position actuator name ie J_L_ANKLE_PITCH => "LAnklePitch/Position/Actuator/Value"
extern string indexToPositionActuator[];
extern string indexToHardnessActuator[];
extern float jointLimits[][2];

extern float actuatorHardnesses[];
extern float actuatorPositions[];

class Actuators
{
    public:
        Actuators();
        ~Actuators();
    
        // Voltage stabilised hardness
        void setStiffness(int jointindex, float value);
        void setStiffnessHead(float value[]);
        void setStiffnessAll(float values[]);
        int setStiffnessAll(float values[], int time);
        void setStiffnessNotHead(float values[]);
    
        // Whole body position and hardness values for the next dcm cycle
        void clearDCM();    
        void sendFrameToBody(float positions[], float hardnesses[]);
        void sendFrameToNotHead(float positions[], float hardnesses[]);
    
        // Whole body positions with time
        int goToAngle(int jointindex, float position, int time);
        int goToAnglesAll(float positions[], int time);
        int goToAnglesWithVelocityAll(float positions[], float velocity);
        int goToAnglesNotHead(float positions[], int time);
        int goToAnglesWithVelocityNotHead(float positions[], float velocity);
        
        // Head positions
        int goToAngleHeadYaw(float position, int time);
        int goToAngleHeadPitch(float position, int time);
    
    public:
        static float MaxBatteryVoltage;

    private:
        // Initialisation functions
        void createHardnessAliases();
        void createTargetAliases();
    
        // Stiffness adjustments
        void adjustHardness(float hardness, float* adjustedhardness);
        void adjustHardnesses(float hardnesses[], float adjustedhardnesses[], unsigned char numactuators);
        
        // Communication with the alDcm
        void sendToActuator(string actuatorname, int time, float value);                                    // single actuator, time and target
        void sendToAliasActuator(string aliasname, int time, float values[], int numactuators);             // many actuators, single time, a target for each actuator
        void sendToAliasActuator(string aliasname, int times[], float values[], int numactuators);          // many actuators, single different time for each target
    
        void sendCurveToActuator(string actuatorname, int starttime, float value[], int numvalues);         // single actuator, numvalues target, a start time
        void sendCurveToAliasActuator(string aliasname, int starttime, float values[/*numvalues*/][ALIAS_TARGETS_ALL_LENGTH], int numvalues);   // many actuators, many targets
        void sendCurveToAliasActuator(string aliasname, int starttime, float values[/*numvalues*/][ALIAS_TARGETS_NOT_HEAD_LENGTH], int numvalues);   // many actuators, many targets
};

#endif
