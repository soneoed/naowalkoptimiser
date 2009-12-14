#ifndef H_KINEMATICS_H_DEFINED
#define H_KINEMATICS_H_DEFINED
//#include "Horizon.h"
#include <vector>
class Kinematics
{
	public:
    static Kinematics& getInstance()
    {
      static Kinematics kinematicsInstance;
      return kinematicsInstance;
    }
    /*float bodyPitch;
    float bodyRoll;
    float headYaw;
    float headPitch;
    int cameraNumber;
    Horizon horizonLine;
    void Update(int cameraNumber);
    void TransformPosition(double distance,double bearing,double elevation, double *transformedDistance,double *transformedBearing,double *transformedElevation);
    std::vector<float> TransformPosition(double distance,double bearing,double elevation);
    float CalculateHeightOfOrigin();

    float GetBodyPitch(){ return bodyPitch; };
    float GetBodyRoll(){ return bodyRoll; };
    float GetHeadYaw(){ return headYaw; };
    float GetHeadPitch(){ return headPitch; };
    int GetCameraNumber(){ return cameraNumber; };
    */
    std::vector<float> GetRightFootPosition();
    std::vector<float> GetLeftFootPosition();
    std::vector<float> CalculateRightFootPosition(float thetaHipYawPitch, float thetaHipRoll, float thetaHipPitch, float thetaKneePitch, float thetaAnklePitch, float thetaAnkleRoll);
    std::vector<float> CalculateLeftFootPosition(float thetaHipYawPitch, float thetaHipRoll, float thetaHipPitch, float thetaKneePitch, float thetaAnklePitch, float thetaAnkleRoll);

// Nao measurement constants
    static const float cameraTopOffsetX;
    static const float cameraTopOffsetZ;
    static const float cameraTopOffsetAngle;
    static const float cameraBottomOffsetX;
    static const float cameraBottomOffsetZ;
    static const float cameraBottomOffsetAngle;

    static const float neckOffsetZ;
    static const float hipOffsetY;
    static const float hipOffsetZ;
    static const float thighLength;
    static const float tibiaLength;
    static const float footHeight;

  private:
    std::vector<float> leftFootPosition;
    std::vector<float> rightFootPosition;

// Hidden for static class
    Kinematics();                             // Private constructor
    Kinematics(const Kinematics&);            // Prevent copy constructor
    Kinematics& operator=(const Kinematics&); // Prevent assignment
};

#endif

