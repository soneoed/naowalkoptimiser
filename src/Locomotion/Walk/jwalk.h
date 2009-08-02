/**
 * @author Jason Kulk
 *
 * Version : $Id: jwalk.h,v 1.8 2009/07/01 18:26:56 jason Exp $
 */

#ifndef JWALK_H
#define JWALK_H

#include "sensors.h"
#include "actuators.h"
#include "nuwalk.h"
#include "alwalk.h"

#include "../script.h"

using namespace std;

namespace AL
{
    class ALBroker;
}

#define JWALK_VERBOSITY         3
#define JWALK_STANDALONE        1               // set this to 1, to make JWalk a standalone AL::ALModule
#define JWALK_OPTIMISER         1

#define JWalk_VERSION_MAJOR     "0"
#define JWalk_VERSION_MINOR     "0"

#if JWALK_STANDALONE
    class JWalk : public AL::ALModule
#else
    class JWalk
#endif
{
    public:
        #if JWALK_STANDALONE
            JWalk(AL::ALPtr<AL::ALBroker> pBroker, const string& pName);
        #else
            JWalk();
        #endif
        virtual ~JWalk();
    
        void initWalk();
    
        // Head movement functions
        bool canUseHead();
        bool usingDCM();
        bool modeUsesDCM(JWalkModeEnum mode);
        int setHeadYaw(float angle, int time);         // time in ms like everything else
        int setHeadPitch(float angle, int time);
        void setHeadStiffness(float values[]);
    
        // Universal walk functions
        bool walkIsActive();   
        void waitUntilFinished();
        void stop();
        void emergencyStop();
        void braceForImpact();
        void getUp();
        int doScript(script scripttorun, bool scripturgent);

        // 'Omni-directional' walk interface
        void nuWalkOnBearing(float bearing, unsigned char speed, bool dodge=false);
        void nuWalkToPoint(float distance, float bearing, bool dodge=false);
        void nuWalkToPointWithOrientation(float distance, float bearing, float finalorientation, bool dodge=false);
        void nuWalkToPointWithMaintainOrientation(float distance, float bearing, float desiredorientation, bool dodge=false);
    
        
    
        bool nuWalkIsActive();
        void nuWalkStop();
        
        // 'do this until I tell you to stop' walk interface
        void gForward();
        void gBackward();
        void gArc(float direction);
        void gLeft();
        void gRight();
        void gTurnLeft();
        void gTurnRight();
        bool gIsActive();
        void gStop();
    
        // 'alMotion-like' walk interface
        void alWalkStraight(float distance);
        void alWalkArc(float angle, float radius);
        void alWalkSideways(float distance);
        void alWalkTurn(float angle);
        bool alWalkIsActive();
        void alWalkWaitUntilFinished();
        void alWalkStop();
    
        int getInCommonPose();
        bool inCommonPose();
        bool inBackwardBracePose();
        void doPreviousWalkCall();
    
        #if JWALK_STANDALONE        // These functions need to be implemented when inheriting from ALModule
            string version();
            bool innerTest();
        #endif
    public:
        Sensors* volatile JWalkSensors;
        Actuators* volatile JWalkActuators;
        NUWalk* volatile nuWalk;
        ALWalk* volatile alWalk;
        
        JWalkModeEnum JWalkPreviousMode;
        JWalkModeEnum JWalkMode;
        JWalkModeEnum JWalkNextMode;
    
        bool JWalkWaitingForFinish;
        JWalkFunctionIDEnum JWalkFunctionID;
        float JWalkParam1, JWalkParam2, JWalkParam3;
    
        float* JWalkCommonPositions;
        float* JWalkCommonHardnesses;
    private:
        void initSelf();
        void initAldebaranProxies();
        void initThreads();
        void onDCMPostProcess();
        
        void changeModes(JWalkModeEnum nextmode);
};

#endif
