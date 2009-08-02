#include "Locomotion.h"
#include "../Nao.h"
#include "../Kinematics/Kinematics.h"
#include "../CameraControl.h"

#include <vector>
#include <math.h>
#include <time.h>

using namespace std;
using namespace AL;

#define LOCOMOTION_VERBOSITY    3

// At 0.8 radians, the bottom camera starts to get blocked by the shoulder pads.
#define BOTTOM_CAMERA_MAX_YAW_MAG 0.8f
#define BOTTOM_CAMERA_MAX_PITCH 0.5f

enum{
MIN_ANGLE,
MAX_ANGLE,
MAX_CHANGE
};

Locomotion::Locomotion(){
  std::cout << "Initialising Locomotion." << endl;  
  m_Walk = new JWalk();
  std::cout << "JWalk Created." << endl;  
    
  m_callCount = 0;
  stiffnessSet = false;
  yawTask = 0;
  pitchTask = 0;
  ALMotionLastHeadTaskID = 0;
  framesSinceMoved = 0;
  numPanSequences = 0;
  odomForward = 0;
  odomLeft = 0;
  odomTurn = 0; 
  HeadMotionUsingDCM = false;
  // Initialise odometry variables
  /*x = 0;        // forward movement
  y = 0;        // backward movement
  distance = 0; // total movement
  theta = 0;    // orientation (left is positive)*/
  std::cout << "Loading files..." << endl;
  odometryFile.open("/var/log/odometry.csv");
  getUpFront.loadFile("/home/root/StandUpFront.csv");
  getUpBack.loadFile("/home/root/StandUpBack.csv");
  getUpBackFall.loadFile("/home/root/StandUpFromBackFall.csv");

  // load forward kicks
  leftKick.loadFile("/home/root/LeftKick.csv");
  leftWideKick.loadFile("/home/root/LeftWideKick.csv");
  rightKick.loadFile("/home/root/RightKick.csv");
  rightWideKick.loadFile("/home/root/RightWideKick.csv");
  
  // load sideward kicks
  leftInsideKick.loadFile("/home/root/LeftKickInside.csv");
  leftInsideKickClose.loadFile("/home/root/LeftKickInsideClose.csv");
  rightInsideKick.loadFile("/home/root/RightKickInside.csv");
  rightInsideKickClose.loadFile("/home/root/RightKickInsideClose.csv");
  
  // load backward kicks
  leftBackKick.loadFile("/home/root/LeftKickBack.csv");
  rightBackKick.loadFile("/home/root/RightKickBack.csv");
    
  // load goal keeper saves  
  leftSaveClose.loadFile("/home/root/LeftSaveClose.csv");
  rightSaveClose.loadFile("/home/root/RightSaveClose.csv");
  centreSave.loadFile("/home/root/CentreSave.csv");
  std::cout << "Files Loaded." << endl;  
 
  std::cout << "Locomotion Initialised." << endl;
  return;
}

Locomotion::~Locomotion(){
  odometryFile.close();
}

void Locomotion::RunLocomotion(){
    m_callCount++;
    return;
}

void Locomotion::Initialise(){
  std::cout << "Locomotion Initialising..." << endl;
  std::cout << "Finding current time..." << endl;
  
  const bool useDCM = false;

  currentMotionEndTime = NAO->DCM_Proxy->getTime(0);
  if(NAO->DCM_Proxy && useDCM)
  {
    HeadMotionUsingDCM = true;
  }
  else
  {
    HeadMotionUsingDCM = false;
  }

  if(!stiffnessSet){
    //NAO->ALMotion_Proxy->gotoChainStiffness((string)"Head", (float)0.5, (float)1.0, (int)0);
      m_Walk->initWalk();
      stiffnessSet = true;
  }

  std::cout << "Finding foot positions..." << endl;
  // Get initial foot positions for odometery
  Kinematics *kinematics = &Kinematics::getInstance();
  // Get Feet positions
  // Kinematics has not been run yet, so must calculate these ourselves.
  vector<float> leftFootPosition = kinematics->CalculateLeftFootPosition( jointPositions[J_L_HIP_YAWPITCH], 
                                                                          jointPositions[J_L_HIP_ROLL], 
                                                                          jointPositions[J_L_HIP_PITCH], 
                                                                          jointPositions[J_L_KNEE_PITCH], 
                                                                          jointPositions[J_L_ANKLE_PITCH], 
                                                                          jointPositions[J_L_ANKLE_ROLL]);

  vector<float> rightFootPosition = kinematics->CalculateRightFootPosition( jointPositions[J_R_HIP_YAWPITCH], 
                                                                            jointPositions[J_R_HIP_ROLL], 
                                                                            jointPositions[J_R_HIP_PITCH], 
                                                                            jointPositions[J_R_KNEE_PITCH], 
                                                                            jointPositions[J_R_ANKLE_PITCH], 
                                                                            jointPositions[J_R_ANKLE_ROLL]);

  previousLeftX = leftFootPosition[0];
  previousRightX = rightFootPosition[0]; 
  previousLeftY = leftFootPosition[1];
  previousRightY = rightFootPosition[1];

  previousHipYaw = jointPositions[J_L_HIP_YAWPITCH]; //(double)NAO->ALMemory_Proxy->getData(string("Device/SubDeviceList/LHipYawPitch/Position/Sensor/Value"),0);
  
  std::cout << "Getting Support Mode..." << endl;
  // Get Support Mode
  //previousSupportMode =  NAO->ALMotion_Proxy->getSupportMode();

  std::cout << "Locomotion Initialisation Complete!" << endl;
}

void Locomotion::updateOdometry()
{
  // get Sensor data
  // Time
//  if(!dcm) return;
//  int timenow = nao->DCM->call<int>("getTime",(int)0);
  // Yaw

  const float turnMultiplier = 1.0;
  const float xMultiplier = 1.25; // 2.5;
  const float yMultiplier = -1.0;
  float hipYaw = jointPositions[J_L_HIP_YAWPITCH]; // (double)NAO->ALMemory_Proxy->getData(string("Device/SubDeviceList/LHipYawPitch/Position/Sensor/Value"),0);
  
  // Get Support Mode
  //int supportMode =  NAO->ALMotion_Proxy->getSupportMode();
  Kinematics *kin = &Kinematics::getInstance();
  // Get Feet positions
//  vector<float> leftFootPosition = NAO->ALMotion_Proxy->getPosition(std::string("LLeg"), (int)0);
  vector<float> leftFootPosition = kin->GetLeftFootPosition();

//  vector<float> rightFootPosition = NAO->ALMotion_Proxy->getPosition(std::string("RLeg"), (int)0);
  vector<float> rightFootPosition = kin->GetRightFootPosition();

  float leftX = leftFootPosition[0];
  float rightX = rightFootPosition[0]; 
  float leftY = leftFootPosition[1];
  float rightY = rightFootPosition[1];

//  cout << "leftX ALMotion: " << leftFootPosition[0]*100.0 << " Mine: " << newleftFootPosition[0] << endl;
//  cout << "rightX ALMotion: " << rightFootPosition[0]*100.0 << " Mine: " << newrightFootPosition[0] << endl;
//  cout << "leftY ALMotion: " << leftFootPosition[1]*100.0 << " Mine: " << newleftFootPosition[1] << endl;
//  cout << "rightY ALMotion: " << rightFootPosition[1]*100.0 << " Mine: " << newrightFootPosition[1] << endl;
  // Distances moved in the last frame
  float angleDiff = 0.0;     // change in orientation since last frame
  float xDiff = 0.0;         // change in forward direction (these are in metres because Aldebaran is in metres)
  float yDiff = 0.0;         // change in sideways direction
  
  if (touchLeftFootOnGround == true && touchRightFootOnGround == false)     // on left foot
  {
//    angleDiff = 1.39*(hipYaw - previousHipYaw);    // 1.39 determined through experiment
    angleDiff = turnMultiplier*(hipYaw - previousHipYaw);    // 1.39 determined through experiment
    xDiff = xMultiplier*(leftX - previousLeftX);          // 2.5 determined through experiment
    yDiff = yMultiplier*(leftY - previousLeftY);          // uncalibrated
  }
  else if (touchLeftFootOnGround == false && touchRightFootOnGround == true)   // on right foot
  {
    angleDiff = -turnMultiplier*(hipYaw - previousHipYaw);    // 1.39 determined through experiment
    xDiff = xMultiplier*(rightX - previousRightX);          // 2.5 determined through experiment
    yDiff = yMultiplier*(rightY - previousRightY);

  } else if(touchLeftFootOnGround == true && touchRightFootOnGround == true){

    float leftsum = touchValues[T_L_FSR_FL] + touchValues[T_L_FSR_FR] + touchValues[T_L_FSR_BL] + touchValues[T_L_FSR_BR];
    float rightsum = touchValues[T_R_FSR_FL] + touchValues[T_R_FSR_FR] + touchValues[T_R_FSR_BL] + touchValues[T_R_FSR_BR];

    if( (leftsum - rightsum) > 500){
      angleDiff = -turnMultiplier*(hipYaw - previousHipYaw);    // 1.39 determined through experiment
      xDiff = xMultiplier*(rightX - previousRightX);          // 2.5 determined through experiment
      yDiff = yMultiplier*(rightY - previousRightY);
    } else if( (rightsum -leftsum) > 500){
      angleDiff = turnMultiplier*(hipYaw - previousHipYaw);    // 1.39 determined through experiment
      xDiff = xMultiplier*(leftX - previousLeftX);          // 2.5 determined through experiment
     yDiff = yMultiplier*(leftY - previousLeftY);          // uncalibrated
    }

  }
  
  // Maintain position relative to start
  /*theta = theta + angleDiff;
  distance = distance *xDiff;
  x = x + xDiff*cos(theta) + yDiff*sin(theta);
  y = y + xDiff*sin(theta) + yDiff*cos(theta);*/
    
  // Update historic variables   
  previousHipYaw = hipYaw;
  previousLeftX = leftX;
  previousRightX = rightX;
  previousLeftY = leftY;
  previousRightY = rightY;
  //previousSupportMode = supportMode;

  odomForward = xDiff;
  odomLeft = yDiff;
  odomTurn = angleDiff;
  
  // Save odometry to a file for debug and testing
//  odometryFile << timenow << ", " << distance << ", " << theta << ", " << x << ", " << y << endl;
  /*odometryFile << gx << ", " << gy << ", ";
  odometryFile << l1 << ", " << l2 << ", " << l3 << ", " << l4 << ", ";
  odometryFile << r1 << ", " << r2 << ", " << r3 << ", " << r4 << endl;
  odometryFile << hipYaw << ", " << leftHipPitch << ", " << leftHipRoll << ", ";
  odometryFile << rightHipPitch << ", " << rightHipRoll << ", ";
  odometryFile << leftAnklePitch << ", " << leftAnkleRoll << ", ";
  odometryFile << leftKneePitch << ", " << rightKneePitch << ", ";
  odometryFile << rightAnklePitch << ", " << rightAnkleRoll << ", " */
  return;
}

void Locomotion::TrackPoint(double x, double y)
{
    CameraControl* camera =  &CameraControl::getInstance();

    Kinematics *kin = &Kinematics::getInstance();

    float currYaw = kin->GetHeadYaw();
    float currPitch = kin->GetHeadPitch();

    float radsPerPixelHoriz = FOVx/IMAGE_WIDTH;
    float radsPerPixelVert = FOVy/IMAGE_HEIGHT;

    float cx = (IMAGE_WIDTH/2); // Center X -> desired position
    float cy = (IMAGE_HEIGHT/2); // Center Y -> desired position

    float xError = cx - x;
    float yError = cy - y;

    float alphaX = 0.8; //0.33;
    float alphaY = 0.8; //0.5;

    float angErrX = alphaX*xError*radsPerPixelHoriz;
    float angErrY = alphaY*yError*radsPerPixelVert;

    float newYaw = currYaw + angErrX;
    float newPitch = currPitch - angErrY;

    const float CAMERA_TOP_MAX_TILT = 0.75f;
    const float CAMERA_BOTTOM_MIN_TILT = -0.2f;
//  const float CAMERA_TOP_MAX_TILT = 0.5f;
//  const float CAMERA_BOTTOM_MIN_TILT = -0.5f;

    if(camera->getCameraInUse() == CAMERA_TOP){
    // If the top camera will need to look too high to track into the position
    // Switch to the bottom camera and try to point that camera to this point.
        if(newPitch > CAMERA_TOP_MAX_TILT){
            float temp = newPitch;
            bool ok = camera->setCameraInUse(CAMERA_BOTTOM);
            if(ok){
                newPitch -= DEG_TO_RAD(40.0);
            }
        }
    }
    else if(camera->getCameraInUse() == CAMERA_BOTTOM){
    // If the bottom camera will need to look too low to track into the position
    // Switch to the bottom camera and try to point that camera to this point.
        if(newPitch < CAMERA_BOTTOM_MIN_TILT){
            float temp = newPitch;      
            bool ok = camera->setCameraInUse(CAMERA_TOP);
            if(ok){
                newPitch += DEG_TO_RAD(40.0);
            }
        }
    }
    
    if(camera->getCameraInUse() == CAMERA_BOTTOM){ 
        // if using the bottom camera, we want to crop the angle so that the view is not blocked by the shoulderpads.
        newYaw = CROP(newYaw, -BOTTOM_CAMERA_MAX_YAW_MAG, BOTTOM_CAMERA_MAX_YAW_MAG);
        if(newPitch > BOTTOM_CAMERA_MAX_PITCH){
            newPitch = BOTTOM_CAMERA_MAX_PITCH;
        }
    }


    bool worked = MoveHead(newYaw,newPitch);
    return;
}

int Locomotion::MoveHead(float newYaw, float newPitch, int timeMs)
{
    int motionEndTimeDcm = dcmTime;

    if ( m_Walk->canUseHead() ) {
        #if LOCOMOTION_VERBOSITY > 1  
            thelog << "LOCOMOTION: MoveHead()." << endl;
        #endif

        if( m_Walk->usingDCM() ) {
            NAO->ALMotion_Proxy->stop(ALMotionLastHeadTaskID);
            motionEndTimeDcm = MoveHeadDCM(newYaw, newPitch, timeMs);
        }
        else {
            motionEndTimeDcm = MoveHeadALMotion(newYaw, newPitch, timeMs);
        }
    }
    return motionEndTimeDcm;
}

int Locomotion::MoveHeadDCM(float newYaw, float newPitch, int timeMS)
{
    int endTime = dcmTime;
    if (m_Walk->canUseHead())          // don't move the head if you have fallen, or are falling
    {
        if((!NAO->DCM_Proxy) || (!m_Walk->usingDCM()))
            return endTime;
        
        #if LOCOMOTION_VERBOSITY > 0  
            thelog << "LOCOMOTION: MoveHeadDCM()." << endl;
        #endif
        
        // Apply Changes to joints
        ALValue yawCommand,pitchCommand;
        const int timeBetweenFrames = 1000 / 15; // Milliseconds.
        endTime = endTime + MAX(timeBetweenFrames, timeMS);

        yawCommand.arraySetSize(3);
        yawCommand[0] = string("HeadYaw/Position/Actuator/Value");
        yawCommand[1] = string("ClearAll");
        yawCommand[2].arraySetSize(1);
        yawCommand[2][0].arraySetSize(2);
        yawCommand[2][0][0] = newYaw;
        yawCommand[2][0][1] = endTime;

        pitchCommand.arraySetSize(3);
        pitchCommand[0] = string("HeadPitch/Position/Actuator/Value");
        pitchCommand[1] = string("ClearAll");
        pitchCommand[2].arraySetSize(1);
        pitchCommand[2][0].arraySetSize(2);
        pitchCommand[2][0][0] = newPitch;
        pitchCommand[2][0][1] = endTime;

        NAO->DCM_Proxy->set(pitchCommand);
        NAO->DCM_Proxy->set(yawCommand);
        currentMotionEndTime = endTime;
    }
    return endTime;
}


void Locomotion::nodPan(float headyaw, bool autoSetCamera)
{
    // if we aren't already noding, then start noding, else monitor when to switch cameras
    if (m_Walk->canUseHead())
    {
        if(isPanning() == false)        // If last sequence has not finished, don't add another.
        {
            #if LOCOMOTION_VERBOSITY > 1  
                thelog << "LOCOMOTION: nodPan()." << endl;
            #endif
            if(m_Walk->usingDCM())
            {
                NAO->ALMotion_Proxy->stop(ALMotionLastHeadTaskID);
                nodPanDCM(headyaw);
            }
            else 
                nodPanALMotion(headyaw);
        }
        else if (autoSetCamera)
        {
            CameraControl* camera =  &CameraControl::getInstance();
            if (jointPositions[J_HEAD_PITCH] > DEG_TO_RAD(15) && camera->getCameraInUse() == CAMERA_TOP)
                camera->setCameraInUse(CAMERA_BOTTOM);
            else if (jointPositions[J_HEAD_PITCH] < DEG_TO_RAD(22) && camera->getCameraInUse() == CAMERA_BOTTOM)
                camera->setCameraInUse(CAMERA_TOP);
        }
    }
    return;
    
}

void Locomotion::nodPanDCM(float headyaw)
{
    if (!m_Walk->canUseHead())
        return;
    
    if((!NAO->DCM_Proxy) || (!m_Walk->usingDCM()))
        return;
    
    #if LOCOMOTION_VERBOSITY > 0  
        thelog << "LOCOMOTION: nodPanDCM()." << endl;
    #endif
    
    const int panTime = 1000;
    int currTime = dcmTime;
    
    if(isPanning() == false) // If last sequence has finished, add another.
    {
        ALValue pitch, yaw;
       currentMotionEndTime = generateNod(pitch, yaw, headyaw, 0, DEG_TO_RAD(45), panTime);
//       currentMotionEndTime = generateNod(pitch, yaw, headyaw, -0.5, DEG_TO_RAD(45), panTime);
        NAO->DCM_Proxy->set(pitch);
        NAO->DCM_Proxy->set(yaw);
    }
    return;
}

int Locomotion::generateNod(AL::ALValue& pitchCommands, AL::ALValue& yawCommands, float headyaw, float minPitch, float maxPitch, int timePerNod)
{
    if(!NAO->DCM_Proxy) return 0;
	int startTime = dcmTime + 20; 

    if (timePerNod < 100)
        timePerNod = 100;
    
    pitchCommands.clear();
    pitchCommands.arraySetSize(3);
    yawCommands.clear();
    yawCommands.arraySetSize(3);
    
    pitchCommands[0] = string("HeadPitch/Position/Actuator/Value");
    pitchCommands[1] = string("ClearAll");
    pitchCommands[2].arraySetSize(2);
    
    pitchCommands[2][0].arraySetSize(2);
    pitchCommands[2][0][0] = maxPitch;
    pitchCommands[2][0][1] = startTime + timePerNod/2;
    
    pitchCommands[2][1].arraySetSize(2);
    pitchCommands[2][1][0] = minPitch;
    pitchCommands[2][1][1] = startTime + timePerNod;
    
    yawCommands[0] = string("HeadYaw/Position/Actuator/Value");
    yawCommands[1] = string("ClearAll");
    yawCommands[2].arraySetSize(1);
    
    yawCommands[2][0].arraySetSize(2);
    yawCommands[2][0][0] = headyaw;
    yawCommands[2][0][1] = startTime + 100;
    
    
    return startTime + timePerNod; 
}




void Locomotion::nodPanALMotion(float headyaw)
{
    if (!m_Walk->canUseHead())
        return;
    
    if(!NAO->ALMotion_Proxy || m_Walk->usingDCM()) 
        return;
    
    #if LOCOMOTION_VERBOSITY > 0  
        thelog << "LOCOMOTION: nodPanALMotion()." << endl;
    #endif
    
    // Pan parameters
    const float maxYaw = DEG_TO_RAD(90.0);
    const float minPitch = 0.75;
    const float maxPitch = -0.3;
    const int numPitchLevels = 3;
    const int timePerPan = 2.0f;
    ALMotionPan(maxYaw, minPitch, maxPitch, numPitchLevels, timePerPan);
}




int Locomotion::MoveHeadALMotion(float yaw, float pitch, int timeMs)
{
    const int timeBetweenFrames = 1000 / 15; // Milliseconds.
    int endTime = dcmTime;
    float endTimeFromNowMs = MAX(timeMs, timeBetweenFrames);
    float endTimeSec = endTimeFromNowMs / 1000.0;
    if (m_Walk->canUseHead())
    {
        float clippedYaw, clippedPitch;
        if((!NAO->ALMotion_Proxy) || m_Walk->usingDCM())
           return endTime;
        
        #if LOCOMOTION_VERBOSITY > 0  
            thelog << "LOCOMOTION: MoveHeadALMotion()." << endl;
        #endif
        
        NAO->ALMotion_Proxy->stop(ALMotionLastHeadTaskID);
        clippedYaw = CROP(yaw, (float)DEG_TO_RAD(-120), (float)DEG_TO_RAD(120));
//        if(clippedYaw != yaw) cout << "Locomotion: yaw clipped from " << yaw << " Radians to " << clippedYaw << " Radians." << endl;
        clippedPitch = CROP(pitch, (float)DEG_TO_RAD(-45), (float)DEG_TO_RAD(45));
//        if(clippedPitch != pitch) cout << "Locomotion: pitch clipped from " << pitch << " Radians to " << clippedPitch << " Radians." << endl;
        NAO->ALMotion_Proxy->post.gotoAngle("HeadYaw",clippedYaw, endTimeSec, 0);
        NAO->ALMotion_Proxy->post.gotoAngle("HeadPitch",clippedPitch, endTimeSec, 0);
        endTime = endTime + endTimeFromNowMs;
        currentMotionEndTime = endTime;
    }
    return endTime;
}

void Locomotion::LocalisationPan(bool autoSetCamera){
    if (m_Walk->canUseHead())
    {
        if(isPanning() == true) return;// If last sequence has not finished, don't add another.
        #if LOCOMOTION_VERBOSITY > 1  
            thelog << "LOCOMOTION: MoveHeadALMotion()." << endl;
        #endif

        // Pan parameters
        const float maxYaw = DEG_TO_RAD(120.0);
        const float minPitch = DEG_TO_RAD(40.0);
        const float maxPitch = DEG_TO_RAD(5.0);
        const int numPitchLevels = 2;
        const int timePerPan = 1750;

        if(autoSetCamera){
            // Set to top camera
            bool ok = CameraControl::getInstance().setCameraInUse(CAMERA_TOP);
        }

        if(m_Walk->usingDCM())
        {
            NAO->ALMotion_Proxy->stop(ALMotionLastHeadTaskID);
            ALValue pitch, yaw;
            int endTime = generateZigZagSearchPattern(pitch, yaw, maxYaw, minPitch, maxPitch, numPitchLevels, timePerPan);
            NAO->DCM_Proxy->set(pitch);
            NAO->DCM_Proxy->set(yaw);
            currentMotionEndTime = endTime;
            numPanSequences++;
        }
        else {
            if(!NAO->ALMotion_Proxy) return;
            int endTime = ALMotionPan(maxYaw, minPitch, maxPitch, numPitchLevels, timePerPan);
            currentMotionEndTime = endTime;
            numPanSequences++;
        }
    }
    return;
}

void Locomotion::SearchPan(bool autoSetCamera)
{
    if (m_Walk->canUseHead())
    {
        if(isPanning() == true) return;// If last sequence has not finished, don't add another.
        #if LOCOMOTION_VERBOSITY > 1  
            thelog << "LOCOMOTION: SearchPan()." << endl;
        #endif

        // Pan parameters
        const float maxYaw = BOTTOM_CAMERA_MAX_YAW_MAG; // 0.8 radians is the limit before the bottom camera will be blocked by shoulder pads.
        const float minPitch = 0.45;
        const float maxPitch = -0.3;
        const int numPitchLevels = 3;
        const int timePerPan = 1750; // in milliseconds

        if(autoSetCamera){
            // Set to use bottom camera
            bool ok = CameraControl::getInstance().setCameraInUse(CAMERA_BOTTOM);
        }

        if(m_Walk->usingDCM())
        {
            NAO->ALMotion_Proxy->stop(ALMotionLastHeadTaskID);
            ALValue pitch, yaw;
            int endTime = generateZigZagSearchPattern(pitch, yaw, maxYaw, minPitch, maxPitch, numPitchLevels, timePerPan);
            NAO->DCM_Proxy->set(pitch);
            NAO->DCM_Proxy->set(yaw);
            currentMotionEndTime = endTime;
            numPanSequences++;
        }
        else {
            if(!NAO->ALMotion_Proxy) return;
            int endTime = ALMotionPan(maxYaw, minPitch, maxPitch, numPitchLevels, timePerPan);
            currentMotionEndTime = endTime;
            numPanSequences++;
        }
    }
    return;
}



void Locomotion::CloseBallSearchPan(bool autoSetCamera)
{
    if (m_Walk->canUseHead()){
        if(isPanning() == true) return;// If last sequence has not finished, don't add another.
        #if LOCOMOTION_VERBOSITY > 1  
            thelog << "LOCOMOTION: CloseBallSearch()." << endl;
        #endif

        if(autoSetCamera){
            // Set to use bottom camera
            bool ok = CameraControl::getInstance().setCameraInUse(CAMERA_BOTTOM);
        }


        // Pan parameters
        const float maxYaw = DEG_TO_RAD(45.0);
        const float headPitch = 0.4;
        const int timePerPan = 1000; // in milliseconds

        float targetYaw = 0.0;

        if(jointPositions[J_HEAD_YAW] > 0.0f){
            targetYaw = -maxYaw;
        } 
        else {
            targetYaw = maxYaw;             
        }

        if(m_Walk->usingDCM())
        {
            cout << "DCM Mode" << endl;
            int startTime = dcmTime + 20; 
            NAO->ALMotion_Proxy->stop(ALMotionLastHeadTaskID);
            ALValue pitch, yaw;

            pitch.arraySetSize(3);
            pitch[0] = string("HeadPitch/Position/Actuator/Value");
            pitch[1] = string("ClearAll");
            pitch[2].arraySetSize(1);
        
            pitch[2][0].arraySetSize(2);
            
            pitch[2][0][0] = headPitch;
            pitch[2][0][1] = startTime;

            yaw.arraySetSize(3);
            yaw[0] = string("HeadYaw/Position/Actuator/Value");
            yaw[1] = string("ClearAll");
            yaw[2].arraySetSize(1);
        
            yaw[2][0].arraySetSize(2);
            
            yaw[2][0][0] = targetYaw;
            yaw[2][0][1] = startTime + timePerPan;

            NAO->DCM_Proxy->set(pitch);
            NAO->DCM_Proxy->set(yaw);

            currentMotionEndTime = startTime + timePerPan;
            numPanSequences++;
        }
        else {
            cout << "ALMotion Mode" << endl;
            if(!NAO->ALMotion_Proxy) return;
            NAO->ALMotion_Proxy->stop(ALMotionLastHeadTaskID);
            NAO->ALMotion_Proxy->post.setAngle("HeadPitch",headPitch);
            ALMotionLastHeadTaskID = NAO->ALMotion_Proxy->post.gotoAngle("HeadYaw",targetYaw, (float)timePerPan/1000.0, 0);
            currentMotionEndTime = dcmTime + timePerPan;
            numPanSequences++;
        }
    }
    return;
}


int Locomotion::ALMotionPan(float maxYaw, float minPitch, float maxPitch, float numPitchLevels, int timePerPan){
    if (!m_Walk->canUseHead())
        return dcmTime;
      
    if(!NAO->ALMotion_Proxy || m_Walk->usingDCM()) 
        return dcmTime;
    
    #if LOCOMOTION_VERBOSITY > 0  
        thelog << "LOCOMOTION: ALMotionPan()." << endl;
    #endif
    float panTimeSec = timePerPan / 1000.0f; // Convert to seconds
    const float startTime = 0.0f; // Seconds
    float deltaPitch = (maxPitch - minPitch) / (numPitchLevels - 1); // Amount to increment between each tilt level.
    float numPoints = 2 * numPitchLevels; // Total number of positions in sequence.
    float currPointTime;

    AL::ALValue motors, allAngles, allTimes, yawAngles, pitchAngles, yawTimes, pitchTimes;

    yawAngles.arraySetSize(numPoints);
    pitchAngles.arraySetSize(numPoints);
    yawTimes.arraySetSize(numPoints);
    pitchTimes.arraySetSize(numPoints);

    motors.arrayPush("HeadYaw");
    motors.arrayPush("HeadPitch");

    // Add upward
    for (int point = 0; point < numPoints; point++)
    {
        currPointTime = startTime + (point + 1)* panTimeSec;
        yawAngles[point] = (1 - 2*(point%2)) * maxYaw;
        yawTimes[point] = currPointTime;

        pitchAngles[point] = minPitch + (int)(point/2) * deltaPitch;
        pitchTimes[point] = currPointTime;
    }

    // Move back to center.
    currPointTime = currPointTime + 0.5f;
    yawAngles.arrayPush(0.0f);
    yawTimes.arrayPush(currPointTime);

    pitchAngles.arrayPush(0.0f);
    pitchTimes.arrayPush(currPointTime);

    allAngles.arrayPush(yawAngles);
    allAngles.arrayPush(pitchAngles);
    allTimes.arrayPush(yawTimes);
    allTimes.arrayPush(pitchTimes);
    NAO->ALMotion_Proxy->stop(ALMotionLastHeadTaskID); // Stop the previous motion if it is still active.
    ALMotionLastHeadTaskID = NAO->ALMotion_Proxy->post.doMove(motors,allAngles,allTimes,0); // Create new motion.

    int endTime = dcmTime + (int)(currPointTime*1000); // convert time back to ms
    return endTime;
}

// Returns the time that the pattern will end.
int Locomotion::generateZigZagSearchPattern(AL::ALValue& pitchCommands, AL::ALValue& yawCommands, float maxYaw, float minPitch, float maxPitch, int numPitchLevels, int timePerPan)
{
    if(!NAO->DCM_Proxy) return 0;
    int startTime = dcmTime + 20; 
    int pitchPoint;
    int currPointTime = startTime;
  
    float deltaPitch = (maxPitch - minPitch) / (numPitchLevels - 1); // Amount to increment between each pitch level.
    float numPoints = 2 * numPitchLevels; // Total number of positions in sequence.

    pitchCommands.clear();
    pitchCommands.arraySetSize(3);
    yawCommands.clear();
    yawCommands.arraySetSize(3);

    pitchCommands[0] = string("HeadPitch/Position/Actuator/Value");
    pitchCommands[1] = string("ClearAll");
    pitchCommands[2].arraySetSize(numPoints);
      
    yawCommands[0] = string("HeadYaw/Position/Actuator/Value");
    yawCommands[1] = string("ClearAll");
    yawCommands[2].arraySetSize(numPoints);
  
  // Add upward
  for (int point = 0; point < numPoints; point++)
  {
    currPointTime = startTime + (point + 1)* timePerPan;
    yawCommands[2][point].arraySetSize(2);
    yawCommands[2][point][0] = (1 - 2*(point%2)) * maxYaw;
    yawCommands[2][point][1] = currPointTime;

    pitchCommands[2][point].arraySetSize(2);
    pitchCommands[2][point][0] = minPitch + (int)(point/2) * deltaPitch;
    pitchCommands[2][point][1] = currPointTime;
  }
  return currPointTime; 
}

bool Locomotion::isPanning()
{   
   return (dcmTime <= currentMotionEndTime);
}

int Locomotion::doKick(float kickyposition)
{
    if (kickyposition < -6)
        return m_Walk->doScript(rightWideKick, false);       // rightwidekick
    else if (kickyposition < 0)
        return m_Walk->doScript(rightKick, false);       // rightkick
    else if (kickyposition < 6)
        return m_Walk->doScript(leftKick, false);       // leftkick
    else
        return m_Walk->doScript(leftWideKick, false);       // leftwidekick
}

int Locomotion::doSave(float saveyposition)
{
    if (saveyposition < -20)
        return m_Walk->doScript(rightSaveClose, true);       
    else if (saveyposition < 20)
        return m_Walk->doScript(centreSave, true);       
    else
        return m_Walk->doScript(leftSaveClose, true);
}




