/**
 * @author Jason Kulk
 *
 * Version : $Id: jwalkincludes.h,v 1.3 2009/06/23 07:06:56 jason Exp $
 */

#ifndef JWALKINLCUDES_H
#define JWALKINLCUDES_H

// Aldebaran's Includes
#include "alxplatform.h"
#include <fstream>
#include <sstream>
#include <albroker.h>
#include <almodule.h>
#include <altools.h>
#include "altypes.h"
#include "alptr.h"
#include "albrokermanager.h"

// Aldebaran Proxies
#include "alproxies.h"

// Jason's Includes
#include <semaphore.h>
#include <pthread.h>
#include <time.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
using namespace std;

namespace AL
{
    class ALBroker;
}

#define DEBUG_TO_FILE           1                   // controls the location of the debug information (a file, or cout)
#if DEBUG_TO_FILE
    #define thelog              jasonLog
    extern std::ofstream thelog;
#else
    #define thelog              cout
#endif

#define JWALK_REALTIME          1
#define JWALK_REALTIME_PRIORITY 40
#define ALDCM_CYCLETIME         20                // the alDcm cycle time in milliseconds

// The modes the walk engine can be in
enum JWalkModeEnum 
{   
    JWALK_TRANSITION,           // this indicates that the walk engine is in the process of switching between modes
    JWALK_JMODE,                // custom walk mode (ie jWalk)
    JWALK_GMODE,                
    JWALK_ALMODE,
    JWALK_SMODE,                // script mode
    JWALK_NUM_MODES
};

// The functions in the walk engine
enum JWalkFunctionIDEnum
{
    ID_NUWALKONBEARING,
    ID_NUWALKTOPOINT,
    ID_NUWALKTOPOINTWITHORIENTATION,
    ID_NUWALKTOPOINTWITHMAINTAINORIENTATION,
    ID_GFORWARD,
    ID_GBACKWARD,
    ID_GARC,
    ID_GLEFT,
    ID_GRIGHT,
    ID_GTURNLEFT,
    ID_GTURNRIGHT,
    ID_ALSTRAIGHT,
    ID_ALARC,
    ID_ALSIDEWAYS,
    ID_ALTURN,
    ID_UNDEFINED,
    ID_NUM_IDS
};

#endif
