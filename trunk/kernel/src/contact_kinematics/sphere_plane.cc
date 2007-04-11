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
#include "sphere_plane.h"
#include "contour.h"

namespace MBSim {

  void ContactKinematicsSpherePlane::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Sphere*>(contour[0])) {
      isphere = 0; iplane = 1;
      sphere = static_cast<Sphere*>(contour[0]);
      plane = static_cast<Plane*>(contour[1]);
    } else {
      isphere = 1; iplane = 0;
      sphere = static_cast<Sphere*>(contour[1]);
      plane = static_cast<Plane*>(contour[0]);
    }
  }

  void ContactKinematicsSpherePlane::stage1(Vec &g, vector<ContourPointData> &cpData) {

    Vec Wd = plane->getWrOP() - sphere->getWrOP();
    cpData[iplane].Wn = plane->computeWn();
    cpData[isphere].Wn = -cpData[iplane].Wn;
    g(0) = trans(cpData[iplane].Wn)*Wd - sphere->getRadius();
  }

  void ContactKinematicsSpherePlane::stage2(const Vec& g, Vec &gd, vector<ContourPointData> &cpData) {

    Vec WrPC[2], WvC[2];

    WrPC[isphere] = cpData[iplane].Wn*sphere->getRadius();
    cpData[isphere].WrOC = sphere->getWrOP()+WrPC[isphere];
    cpData[iplane].WrOC = cpData[isphere].WrOC+cpData[isphere].Wn*g;
    WrPC[iplane] = cpData[iplane].WrOC - plane->getWrOP();
    WvC[isphere] = sphere->getWvP()+crossProduct(sphere->getWomegaC(),WrPC[isphere]);
    WvC[iplane] = plane->getWvP()+crossProduct(plane->getWomegaC(),WrPC[iplane]);
    Vec WvD = WvC[iplane] - WvC[isphere]; 
    gd(0) = trans(cpData[iplane].Wn)*WvD;

    if(cpData[iplane].Wt.cols()) {
      Mat Wt[2];
      cpData[iplane].Wt.col(0) = computeTangential(cpData[iplane].Wn);
      if(cpData[iplane].Wt.cols()==2) 
	cpData[iplane].Wt.col(1) = crossProduct(cpData[iplane].Wn,cpData[iplane].Wt.col(0));
      cpData[isphere].Wt = -cpData[iplane].Wt; 
      static Index iT(1,cpData[iplane].Wt.cols());
      gd(iT) = trans(cpData[iplane].Wt)*WvD;
    }
  }

}
