/* Copyright (C) 2007  Martin Förg, Roland Zander
 
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
 *   rzander@users.berlios.de
 *
 */

#include <config.h> 
#include "circlesolid_circlehollow.h"
#include "contact.h"

namespace MBSim {

  void ContactKinematicsCircleSolidCircleHollow::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<CircleSolid*>(contour[0])) {
      icircle0 = 1; icircle1 = 0;
      circle0 = static_cast<CircleSolid*>(contour[0]);
      circle1 = static_cast<CircleHollow*>(contour[1]);
    } else {
      icircle0 = 0; icircle1 = 1;
      circle0 = static_cast<CircleSolid*>(contour[1]);
      circle1 = static_cast<CircleHollow*>(contour[0]);
    }
  }

  void ContactKinematicsCircleSolidCircleHollow::stage1(Vec &g, vector<ContourPointData> &cpData) {

    Vec WrD = circle1->getWrOP() - circle0->getWrOP();
    cpData[icircle1].Wn = WrD/nrm2(WrD);
    cpData[icircle0].Wn = -cpData[icircle1].Wn;
    g(0) = circle1->getRadius() - trans(cpData[icircle1].Wn)*WrD - circle0->getRadius();
  }

  void ContactKinematicsCircleSolidCircleHollow::stage2(const Vec& g, Vec &gd, vector<ContourPointData> &cpData) {

    Vec WrPC[2], WvC[2];

    WrPC[icircle1] = cpData[icircle1].Wn*circle1->getRadius();
    cpData[icircle1].WrOC = circle1->getWrOP()+WrPC[icircle1];

    WrPC[icircle0] = cpData[icircle0].Wn*(circle0->getRadius());
    cpData[icircle0].WrOC = circle0->getWrOP()+WrPC[icircle0];

    WvC[icircle0] = circle0->getWvP()+crossProduct(circle0->getWomegaC(),WrPC[icircle0]);
    WvC[icircle1] = circle1->getWvP()+crossProduct(circle1->getWomegaC(),WrPC[icircle1]);
    Vec WvD = WvC[icircle0] - WvC[icircle1];
    gd(0) = trans(cpData[icircle1].Wn)*WvD;
    if(cpData[icircle0].Wt.cols()) {
      cpData[icircle1].Wt = crossProduct(circle0->computeWb(),cpData[icircle1].Wn);
      cpData[icircle0].Wt = -cpData[icircle1].Wt;
      static Index iT(1,cpData[icircle0].Wt.cols());
      gd(iT) = trans(cpData[icircle1].Wt)*WvD;
    }
  }

}

