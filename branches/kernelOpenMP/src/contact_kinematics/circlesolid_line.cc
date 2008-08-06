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
#include "circlesolid_line.h"
#include "contact.h"

namespace MBSim {

  void ContactKinematicsCircleSolidLine::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<CircleSolid*>(contour[0])) {
      icircle = 0; iline = 1;
      circlesolid = static_cast<CircleSolid*>(contour[0]);
      line = static_cast<Line*>(contour[1]);
    } else {
      icircle = 1; iline = 0;
      circlesolid = static_cast<CircleSolid*>(contour[1]);
      line = static_cast<Line*>(contour[0]);
    }
  }

  void ContactKinematicsCircleSolidLine::stage1(Vec &g, vector<ContourPointData> &cpData) {

    Vec Wd = line->getWrOP() - circlesolid->getWrOP();
    cpData[iline].Wn = line->computeWn();
    cpData[icircle].Wn = -cpData[iline].Wn;
    g(0) = trans(cpData[iline].Wn)*Wd - circlesolid->getRadius();
  }

  void ContactKinematicsCircleSolidLine::stage2(const Vec &g, Vec &gd, vector<ContourPointData> &cpData) {

    Vec WrPC[2], WvC[2];
    WrPC[icircle] = cpData[iline].Wn*circlesolid->getRadius();
    cpData[icircle].WrOC = circlesolid->getWrOP() + WrPC[icircle];
    cpData[iline].WrOC = cpData[icircle].WrOC + cpData[iline].Wn*g;
    WrPC[iline] = cpData[iline].WrOC - line->getWrOP();
    WvC[icircle] = circlesolid->getWvP()+crossProduct(circlesolid->getWomegaC(),WrPC[icircle]);
    WvC[iline] = line->getWvP()+crossProduct(line->getWomegaC(),WrPC[iline]);
    Vec WvD = WvC[iline] - WvC[icircle];
    gd(0) = trans(cpData[iline].Wn)*WvD;
    if(cpData[iline].Wt.cols()) {
      cpData[iline].Wt = crossProduct(line->computeWb(),cpData[iline].Wn);
      cpData[icircle].Wt = -cpData[iline].Wt;
      static const Index iT(1,cpData[iline].Wt.cols());
      gd(iT) = trans(cpData[iline].Wt)*WvD;
    }
  }

}

