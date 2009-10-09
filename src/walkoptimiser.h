/*
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

#ifndef WALKOPTIMISER_H
#define WALKOPTIMISER_H

// ..::: Headers ::
// Add it here to be sure every file includes it
#include "alxplatform.h"
#include "config.h"
#include <fstream>
#include <sstream>
#include <albroker.h>
#include <almodule.h>
#include <altools.h>


#define ALVALUE_STRING( val ) ((val.getType() == ALValue::TypeString) ? std::string(val) : std::string("") )
#define ALVALUE_DOUBLE( val ) ((val.getType() == ALValue::TypeDouble || val.getType() == ALValue::TypeInt) ? double(val) : 0.0 )
#define ALVALUE_INT( val ) ((val.getType() == ALValue::TypeInt || val.getType() == ALValue::TypeDouble) ? int(val) : 0)

#endif // WALKOPTIMISER_H
