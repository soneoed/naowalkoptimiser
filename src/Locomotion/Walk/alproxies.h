/**
 * @author Jason Kulk
 *
 * Version : $Id: alproxies.h,v 1.1 2009/05/06 02:42:29 jason Exp $
 */

#ifndef ALPROXIES_H
#define ALPROXIES_H

// Aldebaran Proxies
#include "alptr.h"
#include "alproxy.h"
#include "almotionproxy.h"
#include "almemoryproxy.h"
#include "almemoryfastaccess.h"
#include "dcmproxy.h"
//extern AL::ALPtr<AL::ALMotionProxy> alMotion;
extern AL::ALPtr<AL::ALMemoryProxy> alStm;
extern ALMemoryFastAccess* alFastMem;              // The super secret almemoryfastaccess. Use this for very fast access to variables on a regular basis (ie sensor feedback)
extern AL::ALPtr<AL::ALProxy> alDcm;

#endif
