/* Copyright (C) 2004-2009 MBSim Development Team
 *
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
 * Contact: mfoerg@users.berlios.de
 */

#include <config.h>
#include "contact_kinematics.h"

using namespace fmatvec;

namespace MBSim {

  Vec computeTangential(const Vec &n) {
    Vec t(3,NONINIT);
    if(fabs(n(0))+fabs(n(1)) > 1e-12) {
      t(2)=0;
      double buf = pow(n(0),2)+pow(n(1),2);
      buf = 1.0/sqrt(buf);
      t(0) = n(1)*buf;
      t(1) = -n(0)*buf;
    } 
    else {
      t(0)=0;
      double buf = pow(n(1),2)+pow(n(2),2);
      buf = 1.0/sqrt(buf);
      t(1) = n(2)*buf;
      t(2) = -n(1)*buf;
    }
    return t;
  }

}

