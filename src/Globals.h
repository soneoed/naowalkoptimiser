#ifndef GLOBALS_H_DEFINED
#define GLOBALS_H_DEFINED  

#define robot_console_printf printf
#include <stdio.h>

#include <math.h>
#include "Network/GamePacketDefs.h"

const double PI = 4.0*atan(1.0);

template <class T>
inline T ABS(T x){
  if(x > 0) return x;
  else return -x;
}

template <class T>
inline T MAX(T x, T y){
  if(x > y) return x;
  else return y;
}

template <class T>
inline T MIN(T x, T y){
  if(x < y) return x;
  else return y;
}

template <class T>
inline T NORMALISE(T theta){
    return atan2(sin(theta), cos(theta));
}

template <class T>
inline T CROP(T num, T low, T high){
  if (num < low) num = low;
  else if (num > high) num = high;
  return num;
}

template <class T>
inline T SIGN(T x){
  return (x / ABS(x));
}


inline double RAD_TO_DEG(double x){
  return ((x)*180.0)/PI;
}

inline double DEG_TO_RAD(double x){
  return ((x)*PI)/180;
}

//ROBOT
//const double FOVx = DEG_TO_RAD(46.4); // Calculated as 58 degrees * 4/5
//const double FOVx = DEG_TO_RAD(47.8); // Calculated as 2* atan(tan(58deg /2) * 4/5)
const double FOVx = DEG_TO_RAD(45.0f); // Calculated value from measurements on the forum.
//ROBOT 
//const double FOVy = DEG_TO_RAD(34.8); // Calculated as 58 degrees * 3/5
//const double FOVy = DEG_TO_RAD(36.8); // Calculated as 2* atan(tan(58deg /2) * 3/5)
const double FOVy = DEG_TO_RAD(34.45f); // Calculated value from measurements on the forum.

//SIMULATOR const double FOVx = 0.7854;

//SIMULATOR const double FOVy = FOVx * 240.0 / 320.0;

extern int IMAGE_WIDTH; // 160 for simulator, 640 for robot 
extern int IMAGE_HEIGHT; // 120 for simulator, 480 for robot

extern int g_fps;

extern double EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS;

enum {
  c_UNDEFINED = 0,
  c_WHITE = 1,
  c_GREEN = 2,
  c_SHADOW_OBJECT= 3,
  c_RED = 4,
  c_RORANGE= 5,
  c_ORANGE = 6,
  c_YORANGE = 7,
  c_YELLOW = 8,
  c_BLUE = 9,
  c_SHBLUE= 10,
  NUM_COLOURS = 11
};

double CalculateBearing(double cx);
double CalculateElevation(double cy);

// Stuff we have heard from game controller ... 
const int NUM_ROBOTS = 3;

extern int framesSinceGameController;  

extern int packetsSent;

extern double odomForward;
extern double odomLeft;
extern double odomTurn;
extern double odomReliability; 

#define WM_BUFFERSIZE 6

#define BVR_BUFFERSIZE 6

extern StoredGamePackets latestReceivedGamePackets[NUM_ROBOTS];

#endif
