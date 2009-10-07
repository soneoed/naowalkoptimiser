/*
 *  optimiser.h
 *  walkoptimiser
 *
 *  A class that implements an optimisation algorithm
 *
 *  Created by jason on 7/09/09.
 *  Copyright 2009 UoN. All rights reserved.
 *
 */

#include "Locomotion/Walk/step.h"

class Optimiser
{
    public:
        Optimiser();
        ~Optimiser();
    
        void doOptimisation(Step* currentstep, float currentspeed);
    private:
        void tickOptimiser(float speed, float power);
        void mutateBestParameters();
    
        void initBestParameters();
        void copyToBestParameters();
    
        float normalDistribution(float mean, float sigma);
    
        void assessParameters(Step* currentstep, float currentspeed);
    public:
        float BestSpeed;            // the speed in cm/s of the best set of parameters
        float BestCost;             // the cost of transport in J/(Ncm)
        float BestParameters[SM_NUM_MODES][SH_NUM_JOINTS];              // the best set of parameters
        float BestDeltaParameters[SM_NUM_MODES][SH_NUM_JOINTS];         // the difference BestParameters - PreviousBestParameters
    
        float CurrentParameters[SM_NUM_MODES][SH_NUM_JOINTS];
    private:
        Step* LeftStep;
        Step* RightStep;
    
        int SpeedCount;             // the number of speeds received with the current settings
        float SpeedSum;             // the cumulative sum of the received speeds 
        int SpeedCountLimit;        // the number of speeds required before progressing the optimisation
       
        float SpeedImprovement;     // the last speed improvement
        float SpeedPreviousImprovement;     // the previous speed improvement
        float CostImprovement;
        float CostPreviousImprovement;
    
        float PowerSum;             // the cumulative sums of the power
    
        float Alpha;
        int ResetLimit;
        int CountSinceLastImprovement;
    
        int AssessSpeedCount;       // the number of speeds received with the current settings that will be used to assess the speed accurately
        float AssessSpeedSum;       // the sum
        float AssessPowerSum;
        int AssessSpeedCountLimit;  // the number of speeds required before an assessment is reported.
};


