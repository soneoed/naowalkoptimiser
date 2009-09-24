#include "script.h"
#include "Walk/jwalk.h"


#define SCRIPT_USE_HEAD         0

using namespace std;

static inline bool cropJunk(std::string &input)
{
  bool result = false;
  input.erase( std::remove(input.begin(),input.end(),'\r') , input.end() ); // Remove any '\r values inserted by windows.
  input.erase( std::remove(input.begin(),input.end(),'\n') , input.end() ); // Remove any '\n' values.
  if( (input[0] == '"') && (input[input.size()-1] == '"') )                 // Remove any '"' values around the name.
  {
    input = input.substr(1,input.size()-2);
    result = true;
  }
  return result;
}

script::script()
{
    m_fileName = "";
    m_scriptLength = 0;
    m_hasScript = false;
    m_jointNames.clear();
    m_jointPositions.clear();
    m_timePoints.clear();
}

script::script(std::string file, JWalk* pjwalk)
{
  script();
  jwalk = pjwalk;
  loadFile(file);
}

bool script::loadFile(std::string file)
{
  int pos, jointID;
  float tempValue;
  std::string lineBuffer;
  std::string valueBuffer;
  fstream scriptFile(file.c_str(), fstream::in);
  
  // Clear out any existsing data.
  m_jointNames.clear();
  m_jointPositions.clear();
  m_timePoints.clear();

  if(scriptFile.is_open() == false)
  {
    m_hasScript = false;
    return false;  
  }

  // Read in Names
  if(getline(scriptFile,lineBuffer))
  {
    while( (pos = lineBuffer.find(',')) >= 0)
    {
      valueBuffer = lineBuffer.substr(0,pos);
      lineBuffer = lineBuffer.substr(pos + 1);
      if(valueBuffer == std::string("")) continue;
      cropJunk(valueBuffer);
      m_jointNames.push_back(valueBuffer);
    }
    // Get the last collumn
    valueBuffer = lineBuffer;
    if(valueBuffer != std::string(""))
    {
      cropJunk(valueBuffer);
      m_jointNames.push_back(valueBuffer);
    }
  }

  // Read in values.
  while(getline(scriptFile,lineBuffer))
  {
    // Get Time Value
    pos = lineBuffer.find(',');
    valueBuffer = lineBuffer.substr(0,pos);
    lineBuffer = lineBuffer.substr(pos + 1);
    tempValue = 0.0;
    sscanf(valueBuffer.c_str(), "%f", &tempValue);
    m_timePoints.push_back(tempValue * 1000);
    jointID = 0;
    while( (pos = lineBuffer.find(',')) >= 0)
    {
      valueBuffer = lineBuffer.substr(0,pos);
      lineBuffer = lineBuffer.substr(pos + 1);
      if(valueBuffer == std::string("")) continue;
      tempValue = 0.0;
      sscanf(valueBuffer.c_str(), "%f", &tempValue);
      while( (m_jointPositions.size() > jointID) == false) m_jointPositions.push_back(std::vector<float>());
      m_jointPositions[jointID].push_back(tempValue);
      jointID++;
    }
    // Get the last collumn
    valueBuffer = lineBuffer;
    if(valueBuffer != std::string(""))
    {
      tempValue = 0.0;
      int matched = sscanf(valueBuffer.c_str(), "%f", &tempValue);
      while( (m_jointPositions.size() > jointID) == false) m_jointPositions.push_back(std::vector<float>());
      m_jointPositions[jointID].push_back(tempValue);
    }
  }

  // Check that these values all make sense and can be used.
  bool dataGood = true;
  if(m_jointPositions.size() != m_jointNames.size())
  {
    dataGood = false; // If there are not as many names as actuators something is wrong.
    std::cout << "Error loading script: " << file << ", " << m_jointNames.size() << " actuators labeled, yet positions provided for " << m_jointPositions.size() << " actuators." << std::endl;
  }
  int numTimePoints = m_timePoints.size();
  // Either set variables or reset them depending on result of data check.
  for(int j = 0; j < m_jointPositions.size(); j++)
  {  
    if( numTimePoints != m_jointPositions[j].size() )
    { 
      dataGood = false;
      std::cout << "Error loading script: " << file << ", " << numTimePoints << " time points given, yet actuator " << j << " has data for " << m_jointPositions[j].size() << std::endl;
    }
  }
  if(dataGood)
  {
    m_scriptLength = m_timePoints.size();
    m_hasScript = true;
    m_fileName = file;
    print();
  } else {
    m_scriptLength = 0;
    m_hasScript = false;
    m_fileName = "";
    m_jointNames.clear();
    m_jointPositions.clear();
    m_timePoints.clear();
  }
  return dataGood;
}

void script::print()
{
  std::cout << "Script Available = " << m_hasScript << std::endl;
  if(m_hasScript == false) return;
  
  std::cout << "Script: " << m_fileName << ":" << std::endl;
  std::cout << "Length = " << getNumSteps() << " Joints = " << m_jointNames.size() << " Running Time = " << getRunningTime() << "ms." << std::endl;
/*
  std::cout << "Time(ms)";
  for (int jointID = 0; jointID < m_jointNames.size(); jointID++)
  {
    std::cout << "\t" << m_jointNames[jointID];
  }
  std::cout << std::endl;
  for(int step = 0; step < m_timePoints.size(); step++)
  {
    std::cout << m_timePoints[step];
    for(int jointID = 0; jointID < m_jointPositions.size(); jointID++)
    {
      std::cout << "\t" << m_jointPositions[jointID][step];
    }
    std::cout << std::endl;
  }
*/
}

int script::play(bool block)
{
  bool result = true;
  const std::string updateKind = "ClearAfter";
  int currTime = 0;
  currTime = dcmTime; //NAO->DCM_Proxy->getTime(0);
  if( (m_hasScript == false) || (getNumSteps() <= 0) ) 
  {
      thelog << "script: " << m_fileName << " is invalid: hasscript: " << m_hasScript << " numsteps: " << getNumSteps() << endl;
      return currTime;
  }
    
  ALValue instruction;            // list of all parameters for set [Name, Kind [[target, time],....,[targetN, timeN]]
  ALValue timeCommand;            // a single [target, time] pair
  ALValue commands;               // list of all timed commands for a SINGLE JOINT

  setStiffness(0.8);

  instruction.arraySetSize(3);
  timeCommand.arraySetSize(2);

  for(int jointID = 0; jointID < m_jointNames.size(); jointID++)  
  {
    if (m_jointNames[jointID] == std::string("RHipYawPitch"))
      continue;
      
    instruction[0] = m_jointNames[jointID] + "/Position/Actuator/Value";
    instruction[1] = updateKind;
      
    commands.clear();
    // I need to change the final pose of any script to match the walk stance
    for(int instrNum = 0; instrNum < getNumSteps() - 1; instrNum++)
    {
      timeCommand[0] = m_jointPositions[jointID][instrNum];
      timeCommand[1] = currTime + (int)m_timePoints[instrNum];
      commands.arrayPush(timeCommand);
    }
      
    if (jointID < 2)        // if the joint ID is of the head, then it is OK to use the value in the script
        timeCommand[0] = m_jointPositions[jointID][getNumSteps() - 1];
    else                    // for the rest of the body we need to go to the walk pose
        timeCommand[0] = jwalk->JWalkCommonPositions[jointID - 2];      // this assumes that the joints are in the same order
    timeCommand[1] = currTime + (int)m_timePoints[getNumSteps() - 1];  
    commands.arrayPush(timeCommand);
    
    
    instruction[2] = commands;
    alDcm->callVoid("set", instruction);
    //thelog << "script: " << m_fileName << " play() " << instruction.toString(AL::VerbosityMini) << endl;
    
  }
  if(block) usleep( 1000*getRunningTime() );
  
  return currTime + getRunningTime();
}

bool script::setStiffness(float newStiffness)
{
  bool result = true;
  const std::string updateKind = "ClearAfter";

  ALValue instruction,commands,timeCommand;
  int currTime = 0;
  currTime = dcmTime;

  instruction.arraySetSize(3);
  timeCommand.arraySetSize(2);

  for(int jointID = 0; jointID < m_jointNames.size(); jointID++)  
  {
    if (m_jointNames[jointID] == std::string("RHipYawPitch"))
      continue;
    
    instruction[0] = m_jointNames[jointID] + "/Hardness/Actuator/Value";
    instruction[1] = updateKind;
    commands.clear();

    timeCommand[0] = newStiffness;
#if SCRIPT_USE_HEAD == 0
    #warning Script is not using the head. Make sure the head is broken
    if (m_jointNames[jointID] == std::string("HeadYaw") || m_jointNames[jointID] == std::string("HeadPitch"))
      timeCommand[0] = 0;
#endif
    timeCommand[1] = currTime;
    commands.arrayPush(timeCommand);
    instruction[2] = commands;
    alDcm->callVoid("set", instruction);
      
    //thelog << "script: play() " << instruction.toString(AL::VerbosityMini) << endl;
      
  }
  return result;
}

float script::getRunningTime()
{
  if(m_timePoints.size() > 0)
    return m_timePoints.back();
  else 
    return 0.0;
}
