#ifndef SCRIPT_H
#define SCRIPT_H
#include <string>
#include <vector>
#include "dcmproxy.h"
#include "alvalue.h"
class script 
{
  public:
    script();
    script(std::string file);
    bool loadFile(std::string file);
    int play(bool block);
    std::string getFileName() {return m_fileName;}
    int getNumSteps() {return m_scriptLength;} 
    bool hasScript() {return m_hasScript;} 
    float getRunningTime();
    void print();
    bool setStiffness(float newStiffness);
  private:
    bool m_hasScript;
    std::string m_fileName;
    std::vector<std::string> m_jointNames;
    std::vector<float> m_timePoints;
    std::vector< std::vector<float> > m_jointPositions;
    int m_scriptLength;
};

#endif
