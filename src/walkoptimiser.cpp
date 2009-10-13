/*! \mainpage
    @author Jason Kulk
    @version 1.2
 
 \brief  The purpose of this software is to autonomously adjust walk parameters for Aldebaran’s NAO robot.
        At present, a laser scanner is used to track, and remotely control the NAO. The measured speed is sent to the NAO, where an EHCLS algorithm is used to minimise the cost of transport.
        The walk parameters adjusted are joint stiffness values. A gait cycle is split into 4 phases; stance, push, swing and impact. The joint stiffness of each leg joint is specified for each gait phase. Thus, there are 24 parameters under optimisation.
 
 Long description.

    Copyright (C) 2009  Jason Kulk, NUbots, University of Newcastle, Australia

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _WIN32
#include <signal.h>
#endif

#include "altypes.h"
#include "alxplatform.h"
#include "walkoptimiser.h"
#include "alptr.h"
#include "albroker.h"
#include "almodule.h"
#include "albrokermanager.h"

using namespace std;
using namespace AL;

//<EXE_INCLUDE> don't remove this comment
#include "Locomotion/Walk/jwalk.h"

//</EXE_INCLUDE> don't remove this comment

//<ODECLAREINSTANCE> don't remove this comment

//</ODECLAREINSTANCE> don't remove this comment


#ifdef WALKOPTIMISER_IS_REMOTE_OFF

#ifdef _WIN32
    #error You can not compile jason for Windows
#else
#define ALCALL
#endif

#ifdef __cplusplus
extern "C"
{
#endif


ALCALL int _createModule( ALPtr<ALBroker> pBroker )
{      
  // init broker with the main broker inctance 
  // from the parent executable
  ALBrokerManager::setInstance(pBroker->fBrokerManager.lock());
  ALBrokerManager::getInstance()->addBroker(pBroker);

    
  // create modules instance
//<OGETINSTANCE> don't remove this comment
ALModule::createModule<JWalk>(pBroker,"JWalk" );

//</OGETINSTANCE> don't remove this comment 

  return 0;
}

ALCALL int _closeModule(  )
{
  // Delete module instance
//<OKILLINSTANCE> don't remove this comment

//</OKILLINSTANCE> don't remove this comment
 
  return 0;
}

# ifdef __cplusplus
}
# endif

#else
    #error You can not compile jason as remote
#endif

