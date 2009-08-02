#include "Globals.h"
#include <iostream>

using namespace std;

int IMAGE_WIDTH = 160; // 160 for simulator, 640 for robot 
int IMAGE_HEIGHT = 120; // 120 for simulator, 480 for robot
double EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS = (0.5*IMAGE_WIDTH) / (tan(0.5*FOVx));

int g_fps = 15;

int framesSinceGameController;  

int packetsSent;

double odomForward;
double odomLeft;
double odomTurn;
double odomReliability;

StoredGamePackets latestReceivedGamePackets[NUM_ROBOTS];


double CalculateBearing(double cx){
  return atan( (IMAGE_WIDTH/2-cx) / ( (IMAGE_WIDTH/2) / (tan(FOVx/2.0)) ) );
}


double CalculateElevation(double cy){
  return atan( (IMAGE_HEIGHT/2-cy) / ( (IMAGE_HEIGHT/2) / (tan(FOVy/2.0)) ) );
}
