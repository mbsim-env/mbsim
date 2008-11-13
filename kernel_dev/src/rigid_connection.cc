/* Copyright (C) 2004-2008  Martin Förg
 
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
 * Contact:
 *   mfoerg@users.berlios.de
 *
 */

#define FMATVEC_NO_BOUNDS_CHECK
#include <config.h>
#include "rigid_connection.h"
#include "coordinate_system.h"
#include "multi_body_system.h"
#include "function.h"

namespace MBSim {

  RigidConnection::RigidConnection(const string &name) : Connection(name,true), ffl(0), fml(0), fifl(0), fiml(0) {
  }

  void RigidConnection::init() {

     Connection::init();

    }

   std::string RigidConnection::getTerminationInfo(double dt) {
    std::string s;
    int j=-1;
    s= Link::getTerminationInfo(dt);
    for (int i=0; i<gdn.size(); i++) {
      if (fabs(gdn(i)) > gdTol) j=i;
    }
    if (j>0) s = s + " gdn(" + numtostr(j) + ")= " + numtostr(gdn(j));
      s= s + " (gdTol= " + numtostr(gdTol) + " )";
    return s;
  }

}
