#ifndef SCRIPT_H
#define SCRIPT_H

class JWalk;        // hacking; need a forward declaration
#include "Walk/jwalkincludes.h"

class script 
{
  public:
    script();
    script(std::string file, JWalk* pjwalk);
    bool loadFile(std::string file);
    int play(bool block);
    std::string getFileName() {return m_fileName;}
    int getNumSteps() {return m_scriptLength;} 
    bool hasScript() {return m_hasScript;} 
    float getRunningTime();
    void print();
    bool setStiffness(float newStiffness);
  private:
    JWalk* jwalk;
    bool m_hasScript;
    std::string m_fileName;
    std::vector<std::string> m_jointNames;
    std::vector<float> m_timePoints;
    std::vector< std::vector<float> > m_jointPositions;
    int m_scriptLength;
};

#endif
