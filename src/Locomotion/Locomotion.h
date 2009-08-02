#ifndef LOCOMOTION_H_DEFINED
#define LOCOMOTION_H_DEFINED

//#include <almotionproxy.h>
#include "Walk/jwalk.h"
#include "Walk/sensors.h"
#include "../Globals.h"
#include <vector>
#include <fstream>
#include <iostream>
#include "alproxy.h"
#include "alloggerproxy.h"
#include "almotionproxy.h"
#include "dcmproxy.h"
#include "script.h"

class Locomotion
{
	public:
    bool stiffnessSet;
    script getUpFront, getUpBack, getUpBackFall;
    script leftKick, leftWideKick, rightKick, rightWideKick;
    script leftInsideKick, rightInsideKick, leftInsideKickClose, rightInsideKickClose;
    script leftBackKick, rightBackKick;
    script leftSaveClose, rightSaveClose, centreSave;

// New Panning.
    int currentMotionEndTime;
    int generateZigZagSearchPattern(AL::ALValue& pitchCommands, AL::ALValue& yawCommands, float maxYaw, float minPitch, float maxPitch, int numTiltLevels, int timePerPan);
    int generateNod(AL::ALValue& pitchCommands, AL::ALValue& yawCommands, float headyaw, float minPitch, float maxPitch, int timePerNod);
    int numPanSequences;
//      
    int numPanSweeps;
    int ALMotionLastHeadTaskID;
    bool HeadMotionUsingDCM;   // Head motion always has to go through jwalk
    int yawTask, pitchTask;
    int framesSinceMoved;
    int m_callCount;
    JWalk *m_Walk;
    // Odometry Variables
    double previousHipYaw;
    //int previousSupportMode;
    float previousLeftX;
    float previousRightX;
    float previousLeftY;
    float previousRightY;
    //double x, y, distance, theta;       // Temporary odometry variables
    ofstream odometryFile;
    
public:
    Locomotion();
    ~Locomotion();
    void RunLocomotion();
    void updateOdometry();
    int MoveHead(float newYaw, float newPitch, int timeMs = 0);
    int MoveHeadALMotion(float yaw, float pitch, int timeMs);
    int MoveHeadDCM(float yaw, float pitch, int timeMs);
    void LocalisationPan(bool autoSetCamera = true);
    int ALMotionPan(float maxYaw, float minPitch, float maxPitch, float numPitchLevels, int timePerPan);
    bool isPanning();
    void nodPan(float headyaw, bool autoSetCamera = true);
    void nodPanDCM(float headyaw);
    void nodPanALMotion(float headyaw);
    void SearchPan(bool autoSetCamera = true);
    void CloseBallSearchPan(bool autoSetCamera = true);
    void Initialise();
    void TrackPoint(double x, double y);
    vector<float> NextFrameTargets;
    
    int doKick(float kickyposition);
    int doSave(float saveyposition);
};

#endif
