/* Copyright (C) 2004-2009 MBSim Development Team
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 *  
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * Lesser General Public License for more details. 
 *  
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: rzander@users.berlios.de
 */

#ifndef _ROTATIONALSPRINGDAMPER_H_
#define _ROTATIONALSPRINGDAMPER_H_

#include "mbsim/link_mechanics.h"
#include "mbsim/frame.h"
#include <fmatvec/function.h>

#ifdef HAVE_OPENMBVCPPINTERFACE
namespace OpenMBV {
  class CoilSpring;
}
#endif

namespace MBSim {

  class RigidBody;
  /** \brief A spring damper force law between .
   * This class connects two frames and applies a torque in it, which depends in the
   * relative rotation and velocity  between the two frames. ONLY between relative rotational bodies!!!
   */
}

#endif /* _ROTATIONALSPRINGDAMPER_H_ */

