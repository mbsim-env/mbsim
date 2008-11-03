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
#include "flexible_contact.h"
#include "multi_body_system.h"

namespace MBSim {

  FlexibleContact::FlexibleContact(const string &name) : Contact(name,false), fcl(0), ffl(0) {
    //active = false;
  }

  void FlexibleContact::init() {
    Contact::init();
  }

  void FlexibleContact::updateh(double t) {
    if(isActive()) {
      la(0) = (*fcl)(g(0),gd(0));
      if(ffl)
	la(1,getFrictionDirections()) = (*ffl)(gd(1,getFrictionDirections()),fabs(la(0)));

      WF[0] =  cosy[0]->getOrientation().col(1)*la(0);
      if(getFrictionDirections()) {
	WF[0] += cosy[0]->getOrientation().col(0)*la(1);
	if(getFrictionDirections() > 1)
	  WF[0] += cosy[0]->getOrientation().col(2)*la(2);
      }
      WF[1] = -WF[0];
      for(unsigned int i=0; i<contour.size(); i++)
	h[i] += trans(cosy[i]->getJacobianOfTranslation())*WF[i];
    }
  }

}
