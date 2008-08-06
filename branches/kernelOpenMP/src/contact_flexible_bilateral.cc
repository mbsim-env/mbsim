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
#include<config.h>
#define FMATVEC_NO_BOUNDS_CHECK
#include "contact_flexible_bilateral.h"
#include "multi_body_system.h"

namespace MBSim {

  ContactFlexibleBilateral::ContactFlexibleBilateral(const string &name) : ContactFlexible(name) {
    active = true;
  }

  void ContactFlexibleBilateral::updateKinetics(double t) {
  	static const double eps = epsroot;
  	if(flag_c) la(0) = ((*c_fun)(g(0)))(0) + ((*d_fun)(gd(0)))(0); // normal direction
	else la(0) = (((*V_fun)(g(0)))(0)-((*V_fun)(g(0)+eps))(0))/eps + ((*d_fun)(gd(0)))(0); // normal direction

    for(int i=1; i<=nFric; i++) { // tangential direction
      if(fabs(gd(i)) < gdT_grenz) la(i) = -fabs(la(0))*((*mue_fun)(gdT_grenz))(0)*gd(i)/gdT_grenz;
      else la(i) = gd(i)>0?-la(0)*((*mue_fun)(gd(i)))(0):fabs(la(0))*((*mue_fun)(-gd(i)))(0);
    }

    WF[0] = getContourPointData(0).Wn*la(0) + getContourPointData(0).Wt*la(iT);
    WF[1] = -WF[0];
  }
  
  double ContactFlexibleBilateral::computePotentialEnergy() {
    return ((*V_fun)(g(0)))(0);
  }

}
