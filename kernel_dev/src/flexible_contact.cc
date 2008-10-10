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
    active = false;
  }

  FlexibleContact::FlexibleContact(const FlexibleContact *master,const string &name_) : Contact(master,name_) {
    c = master->c;
    d = master->d;
    active = false;
  }

  void FlexibleContact::init() {
    Contact::init();
  }

  void FlexibleContact::calcSize() {
    if(ffl)
      nFric = ffl->getFrictionDirections();
    else
      nFric = 0;
    Contact::calcSize();
  }

  void FlexibleContact::updateKinetics(double t) {
    Vec WrPC[2];
    WrPC[0] = cpData[0].WrOC - contour[0]->getWrOP();
    WrPC[1] = cpData[1].WrOC - contour[1]->getWrOP();

    la(0) = (*fcl)(g(0),gd(0));
    if(ffl)
      la(1,nFric) = (*ffl)(gd(1,nFric),fabs(la(0)));
    
    WF[0] = getContourPointData(0).Wn*la(0) + getContourPointData(0).Wt*la(iT);
    WM[0] = crossProduct(WrPC[0],WF[0]);
    WF[1] = -WF[0];
    WM[1] = crossProduct(WrPC[1],WF[1]);
  }

  //void FlexibleContact::updateh(double t) {
  //  if(active) {
  //   h[0] += trans(contour[0]->getWJP())*WF[0] + trans(contour[0]->getWJR())*WM[0];
  //   h[1] += trans(contour[1]->getWJP())*WF[1] + trans(contour[1]->getWJR())*WM[1];
  //}
  //}

}
