/** A Step class
 * @author Jason Kulk
 *
 * Version : $Id: step.cpp,v 1.2 2009/05/31 03:54:58 jason Exp $
 */

#include "step.h"
#include "sensors.h"
#include "actuators.h"

string typeToTypeName[] = {TYPE_FORWARD_NAME, TYPE_ARC_NAME, /*TYPE_DIAGONAL_NAME,*/ TYPE_SIDEWARD_NAME, TYPE_TURN_NAME, TYPE_BACKWARD_NAME};
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
    
    // then the hardness file
    filepath = STEP_LOCATION + "Hardness" + Name + ".csv";
    file.open(filepath.c_str());
#if STEP_VERBOSITY > 2
    thelog << "STEP: " << Name << " loading file " << filepath << endl;
#endif
    readCSV(file, StepHardnesses);
    file.close();
    
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
    *positions = StepPositions[StepCurrentIndex];
    *hardnesses = StepHardnesses[StepCurrentIndex];
    StepCurrentIndex++;
}

/*! Returns true if this step has finished, false otherwise
 Postconditions: This step is also reset, so that when it is used again we start from the begining
 */
bool Step::hasEnded()
{
    if (StepCurrentIndex >= StepLength)
    {
        resetStep();
        return true;
    }
    else
        return false;
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
