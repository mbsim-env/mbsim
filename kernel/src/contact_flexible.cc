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
#include "contact_flexible.h"
#include "multi_body_system.h"

namespace MBSim {

  ContactFlexible::ContactFlexible(const string &name) : Contact(name,false), gdT_grenz(0.1) {
    active = false;
  }

  ContactFlexible::ContactFlexible(const ContactFlexible *master,const string &name_) : Contact(master,name_), gdT_grenz(0.1) {
    c = master->c;
    d = master->d;
    active = false;
  }

  void ContactFlexible::init() {
    Contact::init();
    for(int i=0; i<2 ; i++) {
      load.push_back(Vec(6));
      WF[i] >> load[i](Index(0,2));
      WM[i] >> load[i](Index(3,5));
    }
  }

  void ContactFlexible::updateKinetics(double t) {
    double mue0 = mue;

    if(gd(0)<0) 
      la(0) = -c*g(0) - d*gd(0);
    else
      la(0) = -c*g(0);

    if(nFric == 1) { 
      if(fabs(gd(1)) < gdT_grenz)
	la(1) = -la(0)*mue0*gd(1)/gdT_grenz;
      else
	la(1) = gd(1)>0?-la(0)*mue:la(0)*mue;
    } else if(nFric == 2) {
      double norm_gdT = nrm2(gd(1,2));
      if(norm_gdT < gdT_grenz)
	la(1,2) = gd(1,2)*(-la(0)*mue0/gdT_grenz);
      else
	la(1,2) = gd(1,2)*(-la(0)*mue/norm_gdT);
    }

    WF[0] = getContourPointData(0).Wn*la(0) + getContourPointData(0).Wt*la(iT);
    WF[1] = -WF[0];
  }

}
