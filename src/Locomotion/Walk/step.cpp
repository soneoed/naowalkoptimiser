/** A Step class
 * @author Jason Kulk
 *
 * Version : $Id: step.cpp,v 1.2 2009/05/31 03:54:58 jason Exp $
 */

#include "step.h"
#include "sensors.h"
#include "actuators.h"

string typeToTypeName[] = {TYPE_NONE_NAME, TYPE_FORWARD_NAME, TYPE_ARC_NAME, /*TYPE_DIAGONAL_NAME,*/ TYPE_SIDEWARD_NAME, TYPE_TURN_NAME, TYPE_BACKWARD_NAME};
string classToClassName[] = {CLASS_START_NAME, CLASS_FOLLOW_NAME, CLASS_NORMAL_NAME, CLASS_FSTOP_NAME, CLASS_NSTOP_NAME};

/*! Create a simple container for the step
 @param name, the name will be used to locate the file storing the step, and to configure properties of the step itself
              the name should be something like WalkForwardLeftStart where the format is WalkTypeValueLeftClass (Value is optional)
 */
Step::Step(string name)
{
    Name = name;
    setStepProperties();
    loadStep();
    StepCurrentIndex = 0;
    StepWaitCount = 0;
    StepLastCallTime = dcmTime;
}

/* Sets all of the step properties (ie the class, type, and left/right) based on the Name
 Preconditions: Name must contain the full name of the step (Type Value Left Class)
 Postconditions: StepClass, StepType and StepLeft will be updated.
 */
void Step::setStepProperties()
{
#if STEP_VERBOSITY > 2
    thelog << "STEP: setStepProperties()" << Name << endl;
#endif
    // determine the step's class (ie start, follow, normal, fstop or nstop)
    if (Name.find(CLASS_START_NAME) != string::npos)
        StepClass = CLASS_START;
    else if (Name.find(CLASS_FOLLOW_NAME) != string::npos)
        StepClass = CLASS_FOLLOW;
    else if (Name.find(CLASS_FSTOP_NAME) != string::npos)
        StepClass = CLASS_FSTOP;
    else if (Name.find(CLASS_NSTOP_NAME) != string::npos)
        StepClass = CLASS_NSTOP;
    else
        StepClass = CLASS_NORMAL;
    
    // determine the step's type
    if (Name.find(TYPE_FORWARD_NAME) != string::npos)
        StepType = TYPE_FORWARD;
    else if (Name.find(TYPE_ARC_NAME) != string::npos)
        StepType = TYPE_ARC;
    else if (Name.find(TYPE_DIAGONAL_NAME) != string::npos)
        StepType = TYPE_FORWARD;    //TYPE_DIAGONAL;
    else if (Name.find(TYPE_SIDEWARD_NAME) != string::npos)
        StepType = TYPE_SIDEWARD;
    else if (Name.find(TYPE_TURN_NAME) != string::npos)
        StepType = TYPE_TURN;
    else if (Name.find(TYPE_BACKWARD_NAME) != string::npos)
        StepType = TYPE_BACKWARD;
    else
        StepType = TYPE_FORWARD;
    
    // determine whether the step is left or right
    string lkey = LEFT_NAME;
    string rkey = RIGHT_NAME;
    size_t lfound = Name.rfind(lkey);
    size_t rfound = Name.rfind(rkey);
    
    if (lfound != string::npos)
    {
        if (rfound == string::npos)         // only Left was found
            StepLeft = true;                // so it must be a left step
        else
        {                                   // The Name contains both Left and Right
            if (lfound > rfound)            // But Left was last
                StepLeft = true;
            else                            // But Right was last
                StepLeft = false;
        }
    }
    else                                    // only Right was found
        StepLeft = false;                   // so it must be a right step
    
    // I need to determine the StepDirection
    // for arcs, diagonals, and turns this is a 3 digit number
    
    if (StepType == TYPE_ARC)
    {   // Typical value format for arc: Left120
        size_t lfound = Name.find(LEFT_NAME);
        size_t rfound = Name.find(RIGHT_NAME);
        if (lfound < rfound)
            StepDirection = (int)atof(Name.substr(lfound + LEFT_NAME.size(), lfound + LEFT_NAME.size() + 3).c_str());
        else
            StepDirection = -(int)atof(Name.substr(rfound + RIGHT_NAME.size(), rfound + RIGHT_NAME.size() + 3).c_str());
    }
    /*else if (StepType == TYPE_DIAGONAL)
    {
    }*/
    else if (StepType == TYPE_TURN)
    {   // Typical value format Left0.2
        size_t lfound = Name.find(LEFT_NAME);
        size_t rfound = Name.find(RIGHT_NAME);
        if (lfound < rfound)
        {
            StepDirection = atof(Name.substr(lfound + LEFT_NAME.size(), lfound + LEFT_NAME.size() + 3).c_str());
            if (fabs(StepDirection) < 0.001)
            {   // if the StepDirection is small then no valid conversion was performed and we can assume that it is a ggg Walk
                StepDirection = LEFT_INF;
            }
        }
        else
        {
            StepDirection = -atof(Name.substr(rfound + RIGHT_NAME.size(), rfound + RIGHT_NAME.size() + 3).c_str());
            if (fabs(StepDirection) < 0.001)
            {   // if the StepDirection is small then no valid conversion was performed and we can assume that it is a ggg Walk
                StepDirection = RIGHT_INF;
            }
        }
    }
    else if (StepType == TYPE_SIDEWARD)
    {
        size_t lfound = Name.find(LEFT_NAME);
        size_t rfound = Name.find(RIGHT_NAME);
        if (lfound < rfound)
            StepDirection = 1.0;
        else
            StepDirection = -1.0;
    }
    else
        StepDirection = 0;
            
    NaturalNext = NULL;         // this is set externally :)
    StopNext = NULL;            // this is set externally too :P
    
#if STEP_VERBOSITY > 1
    thelog << "STEP: setStepProperties()" << Name << " Type: " << typeToTypeName[StepType] << " Class: " << classToClassName[StepClass] << " Left: " << (int)StepLeft << " Direction: " << StepDirection << endl;
#endif
}

/* Reset the step so that when higher level decides to use this step again, it starts from the beginning
 Call this function after the step has ended
 */
void Step::resetStep()
{
    StepCurrentIndex = 0;
    StepWaitCount = 0;
    StepSwingCount = 0;
    StepPushCount = 0;
}

/* Load the stop from file(s) saved in memory
 StepPositions is updated to contain the joint targets specified by STEP_LOCATION + 'Position' + Name + '.csv'
 Similarly, StepHardnesses is updated to contain the joint hardnesses specified in STEP_LOCATION + 'Hardness' + Name + '.csv'
 */
void Step::loadStep()
{
    // the location of the step will be STEP_LOCATION + Name + .csv (the name will need to be fairly verbose)
    
    // firstly the position file
    string filepath = STEP_LOCATION + "Position" + Name + ".csv";
    ifstream file;
    file.open(filepath.c_str());
    if (!file.is_open())            // if the file doesn't open properly
    {
        StepLength = 0;
        return;
    }
    #if STEP_VERBOSITY > 2
        thelog << "STEP: " << Name << " loading file " << filepath << endl;
    #endif
    readCSV(file, StepPositions);
    file.close();
    
    // make a copy of the step positions just in case I change them
    for (unsigned char i=0; i<StepLength; i++)
        for (unsigned char j=0; j<ALIAS_TARGETS_NOT_HEAD_LENGTH; j++)
            StepOriginalPositions[i][j] = StepPositions[i][j];
    
    // then the hardness file
    filepath = STEP_LOCATION + "Hardness" + Name + ".csv";
    file.open(filepath.c_str());
#if STEP_VERBOSITY > 2
    thelog << "STEP: " << Name << " loading file " << filepath << endl;
#endif
    readCSV(file, StepHardnesses);
    loadSupportHardnesses();
    file.close();
    
    // make a copy of the step positions just in case I change them
    for (unsigned char i=0; i<StepLength; i++)
        for (unsigned char j=0; j<ALIAS_TARGETS_NOT_HEAD_LENGTH; j++)
            StepOriginalHardnesses[i][j] = StepHardnesses[i][j];
    
    // Now if applicable load the intial pose associated with this step
    if (Name.find(CLASS_START_NAME) != string::npos)
    {
        // I need to remove the last 'Left' or 'Right' from the name because both left and right steps must have the same initial pose.
        filepath = STEP_LOCATION + "Pose" + Name + ".csv";
        
        // look for the last left or right
        string lkey = string(LEFT_NAME);
        string rkey = string(RIGHT_NAME);
        size_t lfound = filepath.rfind(lkey);
        size_t rfound = filepath.rfind(rkey);
        
        if (lfound != string::npos)
        {
            if (rfound == string::npos)         // only Left was found
                filepath.erase(lfound, lkey.length());
            else
            {
                if (lfound > rfound)            // Left was last
                    filepath.erase(lfound, lkey.length());
                else                            // Right was last
                    filepath.erase(rfound, rkey.length());
            }
        }
        else                                    // only Right was found
            filepath.erase(rfound, rkey.length());

        #if STEP_VERBOSITY > 2
            thelog << "STEP: " << Name << " loading initial pose file " << filepath << endl;
        #endif
        file.open(filepath.c_str());
        readCSV(file, StepInitialPose);
        file.close();
    }
}

void Step::loadSupportHardnesses()
{
    // TODO: Don't hard code the support hardnesses
                                                                // Yaw,HRoll,HPitch,KPitch,APitch,ARoll
    float tempSupportHardnesses[SM_NUM_MODES][SH_NUM_JOINTS] = {{0.70, 0.26, 0.55, 0.25, 0.24, 0.28},          // SM_STANCE
                                                                {0.70, 0.26, 0.55, 0.25, 0.30, 0.28},          // SM_PUSH
                                                                {0.70, 0.26, 0.55, 0.25, 0.24, 0.28},          // SM_SWING
                                                                {0.70, 0.26, 0.55, 0.25, 0.24, 0.28}};         // SM_IMPACT
    
    for (unsigned char i=0; i<SM_NUM_MODES; i++)
    {
        for (unsigned char j=0; j<SH_NUM_JOINTS; j++)
        {
            StepSupportHardnesses[i][j] = tempSupportHardnesses[i][j];
        }
    }
    
    // Original Parameters: 14.52 cm/s 0 falls
    /*float tempSupportHardnesses[SM_NUM_MODES][SH_NUM_JOINTS] = { {0.70, 0.26, 0.55, 0.25, 0.24, 0.28},          // SM_STANCE
        {0.70, 0.26, 0.55, 0.25, 0.30, 0.28},          // SM_PUSH
        {0.70, 0.26, 0.55, 0.25, 0.24, 0.28},          // SM_SWING
        {0.70, 0.26, 0.55, 0.25, 0.24, 0.28}};         // SM_IMPACT*/
    // I dub thee parameter set 1. 15.81cm/s 2 falls
    /*    float tempSupportHardnesses[SM_NUM_MODES][SH_NUM_JOINTS] =  {{0.98104,0.229553,0.574826,0.301256,0.265263,0.329965},
     {0.944944,0.227637,0.357773,0.203263,0.361033,0.259898},
     {0.765265,0.170768,0.551094,0.290503,0.361697,0.280994},
     {0.832776,0.245527,0.462208,0.236142,0.275438,0.238185}};*/
    // Parameter set 2 (aggressive) 15.68cm/s 2 falls
    /*    float tempSupportHardnesses[SM_NUM_MODES][SH_NUM_JOINTS] =  {{1.00000,0.175095,0.691917,0.328562,0.246158,0.397768},
     {1.00000,0.17426,0.243639,0.214631,0.320585,0.266417},
     {0.835204,0.177197,0.56878,0.264094,0.395472,0.275922},
     {0.855421,0.250112,0.449961,0.240087,0.28294,0.238268}};*/
    // Parameter Set 3: 16.1 cm/s, 0.0235. 7 falls 
    /*    float tempSupportHardnesses[SM_NUM_MODES][SH_NUM_JOINTS] = {{1.48641,0.188189,0.601593,0.41919,0.274744,0.33676},
     {0.49211,0.1007,0.351099,0.238197,0.293742,0.17116},
     {0.576903,0.208073,0.379698,0.161446,0.317679,0.254671},
     {0.872576,0.257933,0.494881,0.260146,0.232316,0.426483}};*/
    // Parameter Set 4: 16.25 cm/s 0.0216 6 falls
    /*    float tempSupportHardnesses[SM_NUM_MODES][SH_NUM_JOINTS] = {{1.84945,0.149649,0.699574,0.485594,0.264979,0.357034},
     {0.586952,0.0618059,0.302333,0.217231,0.272252,0.162651},
     {0.562834,0.191856,0.453009,0.142515,0.374402,0.217592},
     {1.15279,0.286017,0.502982,0.258765,0.235281,0.449806}};*/
}

/* Read a CSV file into the data matrix (used for loading a single step)
 @param file, the ifstream from which to extract the data
 @param data[][], the matrix that will be updated to contain the values that were in the stream
 */
void Step::readCSV(ifstream& file, float data[STEP_MAX_LENGTH][ALIAS_TARGETS_NOT_HEAD_LENGTH])
{
    string linebuffer;
    int currentindex, startindex, endindex;
    unsigned char column = 0;
    unsigned char row = 0;
    
    // discard the labels, this shit is hardcoded
    getline(file, linebuffer);
    
    while (!file.eof())
    {
        // grab a new line from the file
        column = 0;
        currentindex = 0;
        startindex = 0;
        getline(file, linebuffer);
        
        // the first entry is special (it is not preceded by a comma)
        endindex = linebuffer.find(",", currentindex);
        currentindex = endindex;
        if (column >= J_NUM_JOINTS)
            thelog << "STEP: " << Name << " there are too many columns in the file" << endl;
        else
            data[row][column] = atof(linebuffer.substr(startindex, endindex).c_str());
        column++;
        
        // scan through the file looking for floats sandwitched between ", " 
        while (currentindex != string::npos)
        {
            startindex = linebuffer.find(",", currentindex) + 1;
            currentindex = startindex;
            endindex = linebuffer.find(",", currentindex);
            currentindex = endindex;
            if (startindex != string::npos)
            {
                if (column >= J_NUM_JOINTS)
                    thelog << "STEP: " << Name << " there are too many columns in the file" << endl;
                else
                    data[row][column] = atof(linebuffer.substr(startindex, endindex).c_str());
                column++;
            }
        }
        row++;
        if (StepLength != 0 && row > StepLength + 1)            // this will clip the hardnesses to be the same length as the position
        {
            thelog << "STEP: " << Name << " Warning. The file contains more rows than the previous one(s). Ignoring all subsequent rows." << endl;
            return;
        }
    }
    StepLength = row - 1;
    #if STEP_VERBOSITY > 1
        thelog << "STEP: " << Name << " loaded step with " << (int)StepLength << " rows." <<  endl;
        for (unsigned char i=0; i<StepLength; i++)
        {
            for (unsigned char j=0; j<ALIAS_TARGETS_NOT_HEAD_LENGTH; j++)
                thelog << data[i][j] << ", ";
            thelog << endl;
        }
    #endif
}

/* Read a CSV file into the data array (used for loading a single pose)
 @param file, the ifstream from which to extract the data
 @param data[], the array that will be updated to contain the values that were in the stream
 */
void Step::readCSV(ifstream& file, float data[ALIAS_TARGETS_NOT_HEAD_LENGTH])
{
    string linebuffer;
    int currentindex = 0;
    int startindex = 0;
    int endindex = 0;
    unsigned char column = 0;
    
    // discard the labels, this shit is hardcoded
    getline(file, linebuffer);
    
    // Now the line of data
    getline(file, linebuffer);
        
    // the first entry is special (it is not preceded by a comma)
    endindex = linebuffer.find(",", currentindex);
    currentindex = endindex;
    data[column] = atof(linebuffer.substr(startindex, endindex).c_str());
    column++;
        
    // scan through the line looking for floats sandwitched between ", " 
    while (currentindex != string::npos)
    {
        startindex = linebuffer.find(",", currentindex) + 1;
        currentindex = startindex;
        endindex = linebuffer.find(",", currentindex);
        currentindex = endindex;
        if (currentindex != string::npos)
        {
            if (column >= J_NUM_JOINTS)
                thelog << "STEP: " << Name << " there are too many columns in the file" << endl;
            else
                data[column] = atof(linebuffer.substr(startindex, endindex).c_str());
            column++;
        }
    }

#if STEP_VERBOSITY > 1
    thelog << "STEP: " << Name << " loaded initial pose." <<  endl;
    for (unsigned char j=0; j<ALIAS_TARGETS_NOT_HEAD_LENGTH; j++)
        thelog << data[j] << ", ";
    thelog << endl;
#endif
}

/*! Get the next frame of positions and hardnesses for this step
 @param positions, a pointer to the array where the positions will be stored
 @param hardnesses, a pointer to the array where the hardnesses will be stored
 Pointers to arrays are used so that no data actually needs to be transferred, I just set the pointer to the next frame
 */
void Step::getNextFrame(float** positions, float** hardnesses)
{
    if (dcmTime - StepLastCallTime > 120 && StepCurrentIndex != 0)
    {   // this is one horrible hack; when the robot falls nothing resets the step, so when it is called again all hell breaks loose
        thelog << "STEP: getNextFrame: reseting step because the robot has fallen or similar" << endl;
        resetStep();
    }
    if (StepCurrentIndex >= StepLength)
    {   // this is another fairly horrible hack; now that I used a closed loop support mode, the step can overrun its end. As a patch I just REUSE the last entry
        StepCurrentIndex = StepLength - 1;
        StepWaitCount++;
    }
    
    // to support the worst hack ever, I need to copy from the original values to the desired values (because the desired values can now be changed)
    for (unsigned char i=0; i<StepLength; i++)
        for (unsigned char j=0; j<ALIAS_TARGETS_NOT_HEAD_LENGTH; j++)
            StepPositions[i][j] = StepOriginalPositions[i][j];
    
    // A proportional controller on the body orientation
    /*if (StepType == TYPE_FORWARD)
    {
        const float gain = 0.4;            // 0.18 is what I was using for stance with accelerometers
        const float desiredtilt = 0;
        static float error = 0;
        error = desiredtilt - balanceValues[B_ANGLE_Y];
        
        if (fabs(error) > 0.1)
        {
            StepPositions[StepCurrentIndex][J_L_HIP_PITCH - 2] -= (gain/1.88)*error;
            StepPositions[StepCurrentIndex][J_L_ANKLE_PITCH - 2] -= gain*error;
            StepPositions[StepCurrentIndex][J_R_HIP_PITCH - 2] -= (gain/1.88)*error;
            StepPositions[StepCurrentIndex][J_R_ANKLE_PITCH - 2] -= gain*error;
            
            if (leftSupportMode == SM_STANCE || leftSupportMode == SM_PUSH)
            {
                StepPositions[StepCurrentIndex][J_L_HIP_PITCH - 2] -= (gain/1.88)*error;
                StepPositions[StepCurrentIndex][J_L_ANKLE_PITCH - 2] -= gain*error;
            }
            if (rightSupportMode == SM_STANCE || rightSupportMode == SM_PUSH)
            {
                StepPositions[StepCurrentIndex][J_R_HIP_PITCH - 2] -= (gain/1.88)*error;
                StepPositions[StepCurrentIndex][J_R_ANKLE_PITCH - 2] -= gain*error;
            }
        }
    }*/
    
    *positions = StepPositions[StepCurrentIndex];
    *hardnesses = StepHardnesses[StepCurrentIndex];
    if (StepType == TYPE_FORWARD)
        useSupportHardnesses(*hardnesses);
    StepCurrentIndex++;
    StepLastCallTime = dcmTime;
}

/* Use the support hardness values instead of those in the files for the legs
 */
void Step::useSupportHardnesses(float hardnesses[])
{   
    for (unsigned char i=0; i<SH_NUM_JOINTS; i++)
    {
        hardnesses[JOINT_OFFSET_LLEG + i] = StepSupportHardnesses[leftSupportMode][i];
        hardnesses[JOINT_OFFSET_RLEG + i] = StepSupportHardnesses[rightSupportMode][i];
    }
}

/*! Returns true if this step has finished, false otherwise
 Postconditions: This step is also reset, so that when it is used again we start from the begining
 */
bool Step::hasEnded()
{
    // Hack: I want the step to end when the supportMode for the current step has been SM_SWING for 2 calls
    const unsigned char swingThreshold = 1;
    const unsigned char pushThreshold = 3;
    
    if ((StepType == TYPE_FORWARD || StepType == TYPE_ARC) && (StepClass == CLASS_NORMAL))
    {
        if (StepLeft == true)
        {   // if this is a left step we are looking for a transition to leftSupportMode == SM_SWING
            if (leftSupportMode == SM_SWING)
                StepSwingCount++;
            else
                StepSwingCount = 0;
        }
        else
        {   // if this is a right step we are looking for a transition to rightSupportMode == SM_SWING
            if (rightSupportMode == SM_SWING)
                StepSwingCount++;
            else
                StepSwingCount = 0;
        }
        
        if (StepSwingCount > swingThreshold || StepPushCount > pushThreshold || StepWaitCount > 5)
        {
            resetStep();
            return true;
        }
        else
            return false;
    }
    else
    {   // Don't mess with the stopping steps, you will leave the robot standing on one leg!!!!!!
        // Don't mess with the starting steps either
        if (StepCurrentIndex >= StepLength)
        {
            resetStep();
            return true;
        }
        else
            return false;
    }
}

/*! Get the initial pose for this step. Note that you will get a rubbish pose if this step is not a starting step!
 @param, a pointer to an array where the positions of the initial pose are stored
 @param, a pointer to an array that will be updated to point to where the hardnesses are stored
 */
void Step::getInitialPose(float** positions, float** hardnesses)
{
    *positions = StepInitialPose;
    *hardnesses = StepHardnesses[0];
}

Step::~Step()
{
}
