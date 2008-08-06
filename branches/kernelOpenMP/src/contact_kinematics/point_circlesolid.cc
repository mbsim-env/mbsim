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
#include "point_circlesolid.h"
#include "contact.h"

namespace MBSim {

  void ContactKinematicsPointCircleSolid::assignContours(const vector<Contour*> &contour) {
	if(dynamic_cast<Point*>(contour[0])) {
	  ipoint = 0;
	  icircle = 1;
	  point = static_cast<Point*>(contour[0]);
	  circle = static_cast<CircleSolid*>(contour[1]);
	} else {
	  ipoint = 1;
	  icircle = 0;
	  point = static_cast<Point*>(contour[1]);
	  circle = static_cast<CircleSolid*>(contour[0]);
	}
  }

  void ContactKinematicsPointCircleSolid::stage1(Vec &g, vector<ContourPointData> &cpData) {

    Vec WrD = circle->getWrOP() - point->getWrOP();
    cpData[icircle].Wn = WrD/nrm2(WrD);
    cpData[ipoint] .Wn = -cpData[icircle].Wn;
    g(0) = trans(cpData[icircle].Wn)*WrD - circle->getRadius();
  }

  void ContactKinematicsPointCircleSolid::stage2(const Vec& g, Vec &gd, vector<ContourPointData> &cpData) {

    Vec WrPC_circle;
    WrPC_circle = cpData[ipoint].Wn*circle->getRadius();
    cpData[icircle].WrOC = circle->getWrOP()+WrPC_circle;

    cpData[ipoint].WrOC = point->getWrOP();

    Vec WvC[2];
    WvC[ipoint]  =  point->getWvP();
    WvC[icircle] = circle->getWvP()+crossProduct(circle->getWomegaC(),WrPC_circle);
    Vec WvD = WvC[icircle] - WvC[ipoint];

    gd(0) = trans(cpData[icircle].Wn)*WvD;

    if(cpData[icircle].Wt.cols()) {
      cpData[icircle].Wt = crossProduct(circle->computeWb(),cpData[icircle].Wn);
      cpData[ipoint].Wt  = -cpData[icircle].Wt;
      static const Index iT(1,cpData[icircle].Wt.cols());
      gd(iT) = trans(cpData[icircle].Wt)*WvD;
    }
  }

}
