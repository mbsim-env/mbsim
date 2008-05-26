/* Copyright (C) 2004-2006  Martin Förg
 
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
#include <config.h>
#define FMATVEC_NO_BOUNDS_CHECK
#include "contact_flexible_bilateral.h"
#include "multi_body_system.h"

namespace MBSim {

  ContactFlexibleBilateral::ContactFlexibleBilateral(const string &name) : ContactFlexible(name) {
    active = true;
  }

  void ContactFlexibleBilateral::updateKinetics(double t) {
    double mue0 = mue;

    la(0) = -c*g(0) - d*gd(0);

    for(int i=1; i<=nFric; i++) {
      if(fabs(gd(i)) < 0.01)
	la(i) = -fabs(la(0))*mue0*gd(i)/0.01;
      else
	la(i) = gd(i)>0?-la(0)*mue:fabs(la(0))*mue;
    }

    WF[0] = getContourPointData(0).Wn*la(0) + getContourPointData(0).Wt*la(iT);
    WF[1] = -WF[0];
  }

}
