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
#include "circlesolid_circlesolid.h"
#include "contact.h"

namespace MBSim {

  void ContactKinematicsCircleSolidCircleSolid::assignContours(const vector<Contour*> &contour) {
    icircle0 = 0; icircle1 = 1;
    circle0 = static_cast<CircleSolid*>(contour[0]);
    circle1 = static_cast<CircleSolid*>(contour[1]);
  }

  void ContactKinematicsCircleSolidCircleSolid::stage1(Vec &g, vector<ContourPointData> &cpData) {

    Vec WrD = circle0->getWrOP() - circle1->getWrOP();
    cpData[icircle0].Wn = WrD/nrm2(WrD);
    cpData[icircle1].Wn = -cpData[icircle0].Wn;
    g(0) = trans(cpData[icircle0].Wn)*WrD - circle0->getRadius() - circle1->getRadius();
  }

  void ContactKinematicsCircleSolidCircleSolid::stage2(const Vec& g, Vec &gd, vector<ContourPointData> &cpData) {

    Vec WrOC[2], WrPC[2];
    WrPC[icircle1] = cpData[icircle0].Wn*circle1->getRadius();
    cpData[icircle1].WrOC = circle1->getWrOP()+WrPC[icircle1];

    WrPC[icircle0] = cpData[icircle1].Wn*(circle0->getRadius());
    cpData[icircle0].WrOC = circle0->getWrOP()+WrPC[icircle0];

    Vec WvC[2];
    WvC[icircle0] = circle0->getWvP()+crossProduct(circle0->getWomegaC(),WrPC[icircle0]);
    WvC[icircle1] = circle1->getWvP()+crossProduct(circle1->getWomegaC(),WrPC[icircle1]);
    Vec WvD = WvC[icircle0] - WvC[icircle1];

    gd(0) = trans(cpData[icircle0].Wn)*WvD;

    if(cpData[icircle0].Wt.cols()) {
      cpData[icircle0].Wt = crossProduct(circle0->computeWb(),cpData[icircle0].Wn);
      cpData[icircle1].Wt = -cpData[icircle0].Wt;
      static Index iT(1,cpData[icircle0].Wt.cols());
      gd(iT) = trans(cpData[icircle0].Wt)*WvD;
    }
  }

}
