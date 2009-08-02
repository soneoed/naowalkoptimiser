/**

 * @author Jason Kulk

 * NUbots (c) 2008 All Rights Reserved - This file is confidential.\n

 *
 * This file contains globals and sensor data access functions.

 * Version : $Id: sensors.h,v 1.4 2009/07/02 09:37:12 jason Exp $

 */
#ifndef SENSORS_H
#define SENSORS_H

#include "jwalkincludes.h"

#define SENSOR_VERBOSITY        0
    
#define SENSOR_LOGGING          0               // enable logging of all data by setting this to 1
#define SENSOR_ULTRASONIC_ON    1               // enable the ultrasonic sensors by setting this to 1
#define SENSOR_EXPORT_TO_AL     0               // set this to 1 to export additional soft sensors to almemory


/**************************************************************************************
Sensor Device Names
**************************************************************************************/
extern string indexToName[];

// Position
extern string indexToPositionSensor[];
#define DN_HEAD_YAW_POSITION          std::string("Device/SubDeviceList/HeadYaw/Position/Sensor/Value")
#define DN_HEAD_PITCH_POSITION        std::string("Device/SubDeviceList/HeadPitch/Position/Sensor/Value")
#define DN_L_SHOULDER_ROLL_POSITION   std::string("Device/SubDeviceList/LShoulderRoll/Position/Sensor/Value")
#define DN_L_SHOULDER_PITCH_POSITION  std::string("Device/SubDeviceList/LShoulderPitch/Position/Sensor/Value")
#define DN_L_ELBOW_YAW_POSITION       std::string("Device/SubDeviceList/LElbowYaw/Position/Sensor/Value") 
#define DN_L_ELBOW_ROLL_POSITION      std::string("Device/SubDeviceList/LElbowRoll/Position/Sensor/Value")
#define DN_R_SHOULDER_ROLL_POSITION   std::string("Device/SubDeviceList/RShoulderRoll/Position/Sensor/Value")
#define DN_R_SHOULDER_PITCH_POSITION  std::string("Device/SubDeviceList/RShoulderPitch/Position/Sensor/Value")
#define DN_R_ELBOW_YAW_POSITION       std::string("Device/SubDeviceList/RElbowYaw/Position/Sensor/Value") 
#define DN_R_ELBOW_ROLL_POSITION      std::string("Device/SubDeviceList/RElbowRoll/Position/Sensor/Value")
#define DN_L_HIP_YAWPITCH_POSITION    std::string("Device/SubDeviceList/LHipYawPitch/Position/Sensor/Value") 
#define DN_L_HIP_ROLL_POSITION        std::string("Device/SubDeviceList/LHipRoll/Position/Sensor/Value") 
#define DN_L_HIP_PITCH_POSITION       std::string("Device/SubDeviceList/LHipPitch/Position/Sensor/Value")
#define DN_L_KNEE_PITCH_POSITION      std::string("Device/SubDeviceList/LKneePitch/Position/Sensor/Value")
#define DN_L_ANKLE_PITCH_POSITION     std::string("Device/SubDeviceList/LAnklePitch/Position/Sensor/Value") 
#define DN_L_ANKLE_ROLL_POSITION      std::string("Device/SubDeviceList/LAnkleRoll/Position/Sensor/Value")
#define DN_R_HIP_YAWPITCH_POSITION    std::string("Device/SubDeviceList/LHipYawPitch/Position/Sensor/Value")
#define DN_R_HIP_ROLL_POSITION        std::string("Device/SubDeviceList/RHipRoll/Position/Sensor/Value") 
#define DN_R_HIP_PITCH_POSITION       std::string("Device/SubDeviceList/RHipPitch/Position/Sensor/Value")
#define DN_R_KNEE_PITCH_POSITION      std::string("Device/SubDeviceList/RKneePitch/Position/Sensor/Value")
#define DN_R_ANKLE_PITCH_POSITION     std::string("Device/SubDeviceList/RAnklePitch/Position/Sensor/Value") 
#define DN_R_ANKLE_ROLL_POSITION      std::string("Device/SubDeviceList/RAnkleRoll/Position/Sensor/Value")

// Current
extern string indexToCurrentSensor[];
#define DN_HEAD_YAW_CURRENT          std::string("Device/SubDeviceList/HeadYaw/ElectricCurrent/Sensor/Value")
#define DN_HEAD_PITCH_CURRENT        std::string("Device/SubDeviceList/HeadPitch/ElectricCurrent/Sensor/Value")
#define DN_L_SHOULDER_ROLL_CURRENT   std::string("Device/SubDeviceList/LShoulderRoll/ElectricCurrent/Sensor/Value")
#define DN_L_SHOULDER_PITCH_CURRENT  std::string("Device/SubDeviceList/LShoulderPitch/ElectricCurrent/Sensor/Value")
#define DN_L_ELBOW_YAW_CURRENT       std::string("Device/SubDeviceList/LElbowYaw/ElectricCurrent/Sensor/Value") 
#define DN_L_ELBOW_ROLL_CURRENT      std::string("Device/SubDeviceList/LElbowRoll/ElectricCurrent/Sensor/Value")
#define DN_R_SHOULDER_ROLL_CURRENT   std::string("Device/SubDeviceList/RShoulderRoll/ElectricCurrent/Sensor/Value")
#define DN_R_SHOULDER_PITCH_CURRENT  std::string("Device/SubDeviceList/RShoulderPitch/ElectricCurrent/Sensor/Value")
#define DN_R_ELBOW_YAW_CURRENT       std::string("Device/SubDeviceList/RElbowYaw/ElectricCurrent/Sensor/Value") 
#define DN_R_ELBOW_ROLL_CURRENT      std::string("Device/SubDeviceList/RElbowRoll/ElectricCurrent/Sensor/Value")
#define DN_L_HIP_YAWPITCH_CURRENT    std::string("Device/SubDeviceList/LHipYawPitch/ElectricCurrent/Sensor/Value") 
#define DN_L_HIP_ROLL_CURRENT        std::string("Device/SubDeviceList/LHipRoll/ElectricCurrent/Sensor/Value") 
#define DN_L_HIP_PITCH_CURRENT       std::string("Device/SubDeviceList/LHipPitch/ElectricCurrent/Sensor/Value")
#define DN_L_KNEE_PITCH_CURRENT      std::string("Device/SubDeviceList/LKneePitch/ElectricCurrent/Sensor/Value")
#define DN_L_ANKLE_PITCH_CURRENT     std::string("Device/SubDeviceList/LAnklePitch/ElectricCurrent/Sensor/Value") 
#define DN_L_ANKLE_ROLL_CURRENT      std::string("Device/SubDeviceList/LAnkleRoll/ElectricCurrent/Sensor/Value")
#define DN_R_HIP_YAWPITCH_CURRENT    std::string("Device/SubDeviceList/LHipYawPitch/ElectricCurrent/Sensor/Value")
#define DN_R_HIP_ROLL_CURRENT        std::string("Device/SubDeviceList/RHipRoll/ElectricCurrent/Sensor/Value") 
#define DN_R_HIP_PITCH_CURRENT       std::string("Device/SubDeviceList/RHipPitch/ElectricCurrent/Sensor/Value")
#define DN_R_KNEE_PITCH_CURRENT      std::string("Device/SubDeviceList/RKneePitch/ElectricCurrent/Sensor/Value")
#define DN_R_ANKLE_PITCH_CURRENT     std::string("Device/SubDeviceList/RAnklePitch/ElectricCurrent/Sensor/Value") 
#define DN_R_ANKLE_ROLL_CURRENT      std::string("Device/SubDeviceList/RAnkleRoll/ElectricCurrent/Sensor/Value")

// Actuator Target
extern string indexToTargetSensor[];
#define DN_HEAD_YAW_TARGET          std::string("Device/SubDeviceList/HeadYaw/Position/Actuator/Value")
#define DN_HEAD_PITCH_TARGET        std::string("Device/SubDeviceList/HeadPitch/Position/Actuator/Value")
#define DN_L_SHOULDER_ROLL_TARGET   std::string("Device/SubDeviceList/LShoulderRoll/Position/Actuator/Value")
#define DN_L_SHOULDER_PITCH_TARGET  std::string("Device/SubDeviceList/LShoulderPitch/Position/Actuator/Value")
#define DN_L_ELBOW_YAW_TARGET       std::string("Device/SubDeviceList/LElbowYaw/Position/Actuator/Value") 
#define DN_L_ELBOW_ROLL_TARGET      std::string("Device/SubDeviceList/LElbowRoll/Position/Actuator/Value")
#define DN_R_SHOULDER_ROLL_TARGET   std::string("Device/SubDeviceList/RShoulderRoll/Position/Actuator/Value")
#define DN_R_SHOULDER_PITCH_TARGET  std::string("Device/SubDeviceList/RShoulderPitch/Position/Actuator/Value")
#define DN_R_ELBOW_YAW_TARGET       std::string("Device/SubDeviceList/RElbowYaw/Position/Actuator/Value") 
#define DN_R_ELBOW_ROLL_TARGET      std::string("Device/SubDeviceList/RElbowRoll/Position/Actuator/Value")
#define DN_L_HIP_YAWPITCH_TARGET    std::string("Device/SubDeviceList/LHipYawPitch/Position/Actuator/Value") 
#define DN_L_HIP_ROLL_TARGET        std::string("Device/SubDeviceList/LHipRoll/Position/Actuator/Value") 
#define DN_L_HIP_PITCH_TARGET       std::string("Device/SubDeviceList/LHipPitch/Position/Actuator/Value")
#define DN_L_KNEE_PITCH_TARGET      std::string("Device/SubDeviceList/LKneePitch/Position/Actuator/Value")
#define DN_L_ANKLE_PITCH_TARGET     std::string("Device/SubDeviceList/LAnklePitch/Position/Actuator/Value") 
#define DN_L_ANKLE_ROLL_TARGET      std::string("Device/SubDeviceList/LAnkleRoll/Position/Actuator/Value")
#define DN_R_HIP_YAWPITCH_TARGET    std::string("Device/SubDeviceList/LHipYawPitch/Position/Actuator/Value")
#define DN_R_HIP_ROLL_TARGET        std::string("Device/SubDeviceList/RHipRoll/Position/Actuator/Value") 
#define DN_R_HIP_PITCH_TARGET       std::string("Device/SubDeviceList/RHipPitch/Position/Actuator/Value")
#define DN_R_KNEE_PITCH_TARGET      std::string("Device/SubDeviceList/RKneePitch/Position/Actuator/Value")
#define DN_R_ANKLE_PITCH_TARGET     std::string("Device/SubDeviceList/RAnklePitch/Position/Actuator/Value") 
#define DN_R_ANKLE_ROLL_TARGET      std::string("Device/SubDeviceList/RAnkleRoll/Position/Actuator/Value")

// Actuator Target
extern string indexToHardnessSensor[];
#define DN_HEAD_YAW_HARDNESS          std::string("Device/SubDeviceList/HeadYaw/Hardness/Actuator/Value")
#define DN_HEAD_PITCH_HARDNESS        std::string("Device/SubDeviceList/HeadPitch/Hardness/Actuator/Value")
#define DN_L_SHOULDER_ROLL_HARDNESS   std::string("Device/SubDeviceList/LShoulderRoll/Hardness/Actuator/Value")
#define DN_L_SHOULDER_PITCH_HARDNESS  std::string("Device/SubDeviceList/LShoulderPitch/Hardness/Actuator/Value")
#define DN_L_ELBOW_YAW_HARDNESS       std::string("Device/SubDeviceList/LElbowYaw/Hardness/Actuator/Value") 
#define DN_L_ELBOW_ROLL_HARDNESS      std::string("Device/SubDeviceList/LElbowRoll/Hardness/Actuator/Value")
#define DN_R_SHOULDER_ROLL_HARDNESS   std::string("Device/SubDeviceList/RShoulderRoll/Hardness/Actuator/Value")
#define DN_R_SHOULDER_PITCH_HARDNESS  std::string("Device/SubDeviceList/RShoulderPitch/Hardness/Actuator/Value")
#define DN_R_ELBOW_YAW_HARDNESS       std::string("Device/SubDeviceList/RElbowYaw/Hardness/Actuator/Value") 
#define DN_R_ELBOW_ROLL_HARDNESS      std::string("Device/SubDeviceList/RElbowRoll/Hardness/Actuator/Value")
#define DN_L_HIP_YAWPITCH_HARDNESS    std::string("Device/SubDeviceList/LHipYawPitch/Hardness/Actuator/Value") 
#define DN_L_HIP_ROLL_HARDNESS        std::string("Device/SubDeviceList/LHipRoll/Hardness/Actuator/Value") 
#define DN_L_HIP_PITCH_HARDNESS       std::string("Device/SubDeviceList/LHipPitch/Hardness/Actuator/Value")
#define DN_L_KNEE_PITCH_HARDNESS      std::string("Device/SubDeviceList/LKneePitch/Hardness/Actuator/Value")
#define DN_L_ANKLE_PITCH_HARDNESS     std::string("Device/SubDeviceList/LAnklePitch/Hardness/Actuator/Value") 
#define DN_L_ANKLE_ROLL_HARDNESS      std::string("Device/SubDeviceList/LAnkleRoll/Hardness/Actuator/Value")
#define DN_R_HIP_YAWPITCH_HARDNESS    std::string("Device/SubDeviceList/LHipYawPitch/Hardness/Actuator/Value")
#define DN_R_HIP_ROLL_HARDNESS        std::string("Device/SubDeviceList/RHipRoll/Hardness/Actuator/Value") 
#define DN_R_HIP_PITCH_HARDNESS       std::string("Device/SubDeviceList/RHipPitch/Hardness/Actuator/Value")
#define DN_R_KNEE_PITCH_HARDNESS      std::string("Device/SubDeviceList/RKneePitch/Hardness/Actuator/Value")
#define DN_R_ANKLE_PITCH_HARDNESS     std::string("Device/SubDeviceList/RAnklePitch/Hardness/Actuator/Value") 
#define DN_R_ANKLE_ROLL_HARDNESS      std::string("Device/SubDeviceList/RAnkleRoll/Hardness/Actuator/Value")

// Temperature
extern string indexToTemperatureSensor[];
#define DN_HEAD_YAW_TEMPERATURE          std::string("Device/SubDeviceList/HeadYaw/Temperature/Sensor/Value")
#define DN_HEAD_PITCH_TEMPERATURE        std::string("Device/SubDeviceList/HeadPitch/Temperature/Sensor/Value")
#define DN_L_SHOULDER_ROLL_TEMPERATURE   std::string("Device/SubDeviceList/LShoulderRoll/Temperature/Sensor/Value")
#define DN_L_SHOULDER_PITCH_TEMPERATURE  std::string("Device/SubDeviceList/LShoulderPitch/Temperature/Sensor/Value")
#define DN_L_ELBOW_YAW_TEMPERATURE       std::string("Device/SubDeviceList/LElbowYaw/Temperature/Sensor/Value") 
#define DN_L_ELBOW_ROLL_TEMPERATURE      std::string("Device/SubDeviceList/LElbowRoll/Temperature/Sensor/Value")
#define DN_R_SHOULDER_ROLL_TEMPERATURE   std::string("Device/SubDeviceList/RShoulderRoll/Temperature/Sensor/Value")
#define DN_R_SHOULDER_PITCH_TEMPERATURE  std::string("Device/SubDeviceList/RShoulderPitch/Temperature/Sensor/Value")
#define DN_R_ELBOW_YAW_TEMPERATURE       std::string("Device/SubDeviceList/RElbowYaw/Temperature/Sensor/Value") 
#define DN_R_ELBOW_ROLL_TEMPERATURE      std::string("Device/SubDeviceList/RElbowRoll/Temperature/Sensor/Value")
#define DN_L_HIP_YAWPITCH_TEMPERATURE    std::string("Device/SubDeviceList/LHipYawPitch/Temperature/Sensor/Value") 
#define DN_L_HIP_ROLL_TEMPERATURE        std::string("Device/SubDeviceList/LHipRoll/Temperature/Sensor/Value") 
#define DN_L_HIP_PITCH_TEMPERATURE       std::string("Device/SubDeviceList/LHipPitch/Temperature/Sensor/Value")
#define DN_L_KNEE_PITCH_TEMPERATURE      std::string("Device/SubDeviceList/LKneePitch/Temperature/Sensor/Value")
#define DN_L_ANKLE_PITCH_TEMPERATURE     std::string("Device/SubDeviceList/LAnklePitch/Temperature/Sensor/Value") 
#define DN_L_ANKLE_ROLL_TEMPERATURE      std::string("Device/SubDeviceList/LAnkleRoll/Temperature/Sensor/Value")
#define DN_R_HIP_YAWPITCH_TEMPERATURE    std::string("Device/SubDeviceList/LHipYawPitch/Temperature/Sensor/Value")
#define DN_R_HIP_ROLL_TEMPERATURE        std::string("Device/SubDeviceList/RHipRoll/Temperature/Sensor/Value") 
#define DN_R_HIP_PITCH_TEMPERATURE       std::string("Device/SubDeviceList/RHipPitch/Temperature/Sensor/Value")
#define DN_R_KNEE_PITCH_TEMPERATURE      std::string("Device/SubDeviceList/RKneePitch/Temperature/Sensor/Value")
#define DN_R_ANKLE_PITCH_TEMPERATURE     std::string("Device/SubDeviceList/RAnklePitch/Temperature/Sensor/Value") 
#define DN_R_ANKLE_ROLL_TEMPERATURE      std::string("Device/SubDeviceList/RAnkleRoll/Temperature/Sensor/Value")

// Balance
extern string indexToBalanceSensor[];
#define DN_ACCEL_X                  std::string("Device/SubDeviceList/InertialSensor/AccX/Sensor/Value")
#define DN_ACCEL_Y                  std::string("Device/SubDeviceList/InertialSensor/AccY/Sensor/Value") 
#define DN_ACCEL_Z                  std::string("Device/SubDeviceList/InertialSensor/AccZ/Sensor/Value")
#define DN_ANGLE_X                  std::string("Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value")
#define DN_ANGLE_Y                  std::string("Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value")
#define DN_GYRO_X                   std::string("Device/SubDeviceList/InertialSensor/GyrX/Sensor/Value")
#define DN_GYRO_Y                   std::string("Device/SubDeviceList/InertialSensor/GyrY/Sensor/Value") 

// Touch
extern string indexToTouchSensor[];
#define DN_L_FSR_FL                 std::string("Device/SubDeviceList/LFoot/FSR/FrontLeft/Sensor/Value")
#define DN_L_FSR_FR                 std::string("Device/SubDeviceList/LFoot/FSR/FrontRight/Sensor/Value")
#define DN_L_FSR_BL                 std::string("Device/SubDeviceList/LFoot/FSR/RearLeft/Sensor/Value")
#define DN_L_FSR_BR                 std::string("Device/SubDeviceList/LFoot/FSR/RearRight/Sensor/Value")
#define DN_L_BUMP_L                 std::string("Device/SubDeviceList/LFoot/Bumper/Left/Sensor/Value")
#define DN_L_BUMP_R                 std::string("Device/SubDeviceList/LFoot/Bumper/Right/Sensor/Value")
#define DN_R_FSR_FL                 std::string("Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/Value")
#define DN_R_FSR_FR                 std::string("Device/SubDeviceList/RFoot/FSR/FrontRight/Sensor/Value")
#define DN_R_FSR_BL                 std::string("Device/SubDeviceList/RFoot/FSR/RearLeft/Sensor/Value")
#define DN_R_FSR_BR                 std::string("Device/SubDeviceList/RFoot/FSR/RearRight/Sensor/Value")
#define DN_R_BUMP_L                 std::string("Device/SubDeviceList/RFoot/Bumper/Left/Sensor/Value")
#define DN_R_BUMP_R                 std::string("Device/SubDeviceList/RFoot/Bumper/Right/Sensor/Value")
#define DN_CHEST_BUTTON             std::string("Device/SubDeviceList/ChestBoard/Button/Sensor/Value")
#define DN_SIMPLE_CLICK             std::string("ALWatchDog/SimpleClickOccured")
#define DN_DOUBLE_CLICK             std::string("ALWatchDog/DoubleClickOccured")
#define DN_TRIPLE_CLICK             std::string("ALWatchDog/TripleClickOccured")

// Battery
extern string indexToBatterySensor[];
#define DN_CHARGE                   std::string("Device/SubDeviceList/Battery/Charge/Sensor/Value")
#define DN_CURRENT                  std::string("Device/SubDeviceList/Battery/Current/Sensor/Value")
#define DN_VOLTAGE_MIN              std::string("Device/SubDeviceList/Battery/Charge/Sensor/CellVoltageMin")
#define DN_VOLTAGE_MAX              std::string("Device/SubDeviceList/Battery/Charge/Sensor/CellVoltageMax")
#define DN_TEMPERATURE              std::string("Device/SubDeviceList/Battery/Temperature/Sensor/Value")

// Distance (ultrasonic and infrared sensors)
extern string indexToDistanceSensor[];
#define DN_US_DISTANCE              std::string("Device/SubDeviceList/US/Sensor/Value")
//#define DN_L_IR                     std::string("Device/SubDeviceList/LIR/Sensor/Value")
//#define DN_R_IR                     std::string("Device/SubDeviceList/RIR/Sensor/Value")

/**************************************************************************************
Sensor Array Indices
**************************************************************************************/

// Joint sensors: Position, Target, Hardness, Current, Temperature indicies
enum{
    J_HEAD_YAW,
    J_HEAD_PITCH,
    J_L_SHOULDER_ROLL,
    J_L_SHOULDER_PITCH,
    J_L_ELBOW_YAW,
    J_L_ELBOW_ROLL,
    J_R_SHOULDER_ROLL,
    J_R_SHOULDER_PITCH,
    J_R_ELBOW_YAW,
    J_R_ELBOW_ROLL,
    J_L_HIP_YAWPITCH,
    J_L_HIP_ROLL,
    J_L_HIP_PITCH,
    J_L_KNEE_PITCH,
    J_L_ANKLE_PITCH,
    J_L_ANKLE_ROLL,
    J_R_HIP_YAWPITCH,
    J_R_HIP_ROLL,
    J_R_HIP_PITCH,
    J_R_KNEE_PITCH,
    J_R_ANKLE_PITCH,
    J_R_ANKLE_ROLL,
    J_NUM_JOINTS
};

// Balance sensors: accelerometer, gyro and Aldebaran's angle
enum{
  // Accelerometers
    B_ACCEL_X, 
    B_ACCEL_Y, 
    B_ACCEL_Z,
  // Angles
    B_ANGLE_X,
    B_ANGLE_Y,
  // Gyrometers
    B_GYRO_X,
    B_GYRO_Y,
    B_NUM_SENSORS
};

// Touch sensors: Feet FSR, feet bumpers, chest button
enum{
  // Left Foot Touch Sensors
    // Pressure Sensors
    T_L_FSR_FL,
    T_L_FSR_FR,
    T_L_FSR_BL,
    T_L_FSR_BR,
    // Bumpers
    T_L_BUMP_L,
    T_L_BUMP_R,
  // Right Foot Touch Sensors
    // Pressure Sensors
    T_R_FSR_FL,
    T_R_FSR_FR,
    T_R_FSR_BL,
    T_R_FSR_BR,
    // Bumpers
    T_R_BUMP_L,
    T_R_BUMP_R,
  // Other Touch Sensors
    T_CHEST_BUTTON,
    T_NUM_SENSORS
};

// Energy sensors: battery
enum{
    E_CHARGE,
    E_CURRENT,
    E_VOLTAGE_MIN,
    E_VOLTAGE_MAX,
    E_TEMPERATURE,
    E_NUM_SENSORS
};

// Distance sensors (just the ultrasonic sensors)
enum{
    D_US,
    //D_L_IR,
    //D_R_IR,
    D_NUM_SENSORS
};

#define ALL_NUM_SENSORS             J_NUM_JOINTS*4 + B_NUM_SENSORS + T_NUM_SENSORS + E_NUM_SENSORS + D_NUM_SENSORS

// Sonic sensors
enum{
    S_LL,
    S_LR,
    S_RL,
    S_RR,
    S_NUM_SENSORS
};

/**************************************************************************************
Sensor Arrays
**************************************************************************************/

extern int dcmTime;
extern int dcmTimeSinceStart;

// Proprioception Feedback Data. Velocities and Accelerations are measured by soft sensors
extern float jointPositions[J_NUM_JOINTS];
extern float jointVelocities[J_NUM_JOINTS];
extern float jointVelocitySum;
extern float jointAccelerations[J_NUM_JOINTS];
extern float jointCurrents[J_NUM_JOINTS];
extern float jointCurrentSum;
extern float jointTargets[J_NUM_JOINTS];
extern float jointHardnesses[J_NUM_JOINTS];
extern unsigned char walkCyclesSinceCall;   // I need this so that walkIsActive is true immediately after a call to walk!
extern bool walkAmIWalking;
extern bool walkPreviousAmIWalking;         // I need this one to detect when the robot stops

// Thermoception Feedback Data.
extern float jointTemperatures[J_NUM_JOINTS];

// Balance Feedback Data. Accelerometers and Gyros
extern float balanceValues[B_NUM_SENSORS];
extern bool balanceFallingEnabled;
extern bool balanceFalling;                 // the robot is presently falling
extern bool balancePreviousFalling;
extern bool balanceFallingForward;
extern bool balanceFallingBackward;
extern bool balanceFallingLeft;
extern bool balanceFallingRight;
extern bool balanceFallen;                  // the robot has finished falling, and is now fallen (you can start getting up now)
extern bool balancePreviousFallen;

// Touch Feedback Data. Foot Pressure sensors, foot bump sensors, and chest buttons
extern float touchValues[T_NUM_SENSORS];
extern float touchLeftCoPX;
extern float touchLeftCoPY;
extern float touchRightCoPX;
extern float touchRightCoPY;
extern bool touchOnGround;
extern bool touchPreviousOnGround;
extern bool touchLeftFootOnGround;
extern bool touchRightFootOnGround;

extern bool collisionAny;
extern bool collisionLeftArm;
extern bool collisionLeftArmFront;
extern bool collisionLeftArmSide;
extern bool collisionLeftArmBack;
extern bool collisionRightArm;
extern bool collisionRightArmFront;
extern bool collisionRightArmSide;
extern bool collisionRightArmBack;
extern bool collisionLeftFoot;
extern bool collisionLeftFootFront;
extern bool collisionLeftFootBack;
extern bool collisionRightFoot;
extern bool collisionRightFootFront;
extern bool collisionRightFootBack;

// Battery Feedback Data. 
extern float batteryValues[E_NUM_SENSORS];

// Distance Feedback Data. (centimetres)
//extern float distanceValues[D_NUM_SENSORS];                 // caution: these values are only to be used by me, use the ones below for useful sensor data
extern float sonicValues[S_NUM_SENSORS];
extern float sonicObstacleDistance;

#define VEL_WINDOW_SIZE     16
#define B_WINDOW_SIZE       9
#define S_WINDOW_SIZE       1

/**************************************************************************************
Sensor Array Class
**************************************************************************************/

class Sensors
{
  public:
    Sensors(AL::ALPtr<AL::ALBroker> pBroker);
    virtual ~Sensors();
    void onNewSensorData();
    void getSensorData();
    void writeSensorData();
    void createLogs();
    void finishLogs();
    
    static void sendUSCommandToDCM(float value);
    static void sleepMSec(timespec* nextWakeTime, int msec);
    
  private:
    void connectALMemoryFastAccess(AL::ALPtr<AL::ALBroker> pBroker);
    
    void getTimeData();
    void getFastMemData();
    
    void signCurrentData();
    void calculateJointVelocities();
    void calculateJointAccelerations();
    
    void filterData();
    void filterBalanceValues();
    void filterJointVelocities();
    
    void calculateSoftSensors();
    void calculateCoP();
    void determineWhetherOnGround();
    void determineWhetherWalking();
    void determineWhetherFalling();
    void determineWhetherColliding();
    
    void updateSensorHistory();
    
    // Data logging functions
    void writePositionData();
    void writeVelocityData();
    void writeAccelerationData();
    void writeCurrentData();
    void writeTargetData();
    void writeHardnessData();
    void writeTemperatureData();
    void writeBalanceData();
    void writeTouchData();
    void writeBatteryData();
    void writeDistanceData();
    
    void writeJointLabels(ofstream &logfile);
    void writeBalanceLabels();
    void writeTouchLabels();
    void writeBatteryLabels();
    void writeDistanceLabels();

  public:
    int historysonicindex[S_NUM_SENSORS];                                          // this has to be public because the data is shared with a thread
    float historySonicValues[S_NUM_SENSORS][S_WINDOW_SIZE];
    
  private:
    int dcmStartTime;         // The dcm time when Sensors was created
    // Historical sensor data
    int previousDcmTime;
    float previousJointPositions[J_NUM_JOINTS];
    float previousJointVelocities[J_NUM_JOINTS];
    float previousJointCurrents[J_NUM_JOINTS];
    
    int historybalanceindex;
    float historyBalanceValues[B_NUM_SENSORS][B_WINDOW_SIZE];
    
    int historyvelocityindex;
    float historyJointVelocities[J_NUM_JOINTS][VEL_WINDOW_SIZE];
    
    // file streams to write sensor data into
//#if SENSOR_LOGGING
    ofstream positionLog;
    ofstream velocityLog;
    ofstream accelerationLog;
    ofstream currentLog;
    ofstream targetLog;
    ofstream hardnessLog;
    ofstream temperatureLog;
    ofstream balanceLog;
    ofstream touchLog;
    ofstream batteryLog;
    ofstream distanceLog;
//#endif
    
    pthread_t SensorUltrasonicThread;
};

void *runSonicThread(void* arg);            // it is too hard to make a thread function a member of a class

#endif
