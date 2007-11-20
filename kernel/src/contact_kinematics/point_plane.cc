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
#include "point_plane.h"
#include "contour.h"

namespace MBSim {

  void ContactKinematicsPointPlane::assignContours(const vector<Contour*> &contour)
  {
  	// ASSIGNCONTOURS treats the ordering of the bodies in connect-call
	// INPUT	contour	Vector of the two body contours
	
    if(dynamic_cast<Point*>(contour[0])) {
      ipoint = 0;
      iplane = 1;
      point = static_cast<Point*>(contour[0]);
      plane = static_cast<Plane*>(contour[1]);
    }
    else {
      ipoint = 1;
      iplane = 0;
      point = static_cast<Point*>(contour[1]);
      plane = static_cast<Plane*>(contour[0]);
    }
  }

  void ContactKinematicsPointPlane::stage1(Vec &g, vector<ContourPointData> &cpData)
  {
  	// STAGE1 computes normal distance in the possible contact point
	// INPUT	g		Normal distance (OUTPUT)
	//			cpData	Contact parameter (OUTPUT)
	
    Vec Wd; Wd = plane->getWrOP() - point->getWrOP();
    cpData[iplane].Wn = plane->computeWn();
    cpData[ipoint].Wn = -cpData[iplane].Wn;
    g(0) = trans(cpData[iplane].Wn)*Wd;
  }

  void ContactKinematicsPointPlane::stage2(const Vec& g, Vec &gd, vector<ContourPointData> &cpData)
  {
  	// STAGE2 computes tangential directions and normal velocities in contact point
	// INPUT	g		Normal distance
	//			gd		Normal velocity (OUTPUT)
	//			cpData	Contact parameter (OUTPUT)
	
    Vec WrPC[2], WvC[2];

    cpData[ipoint].WrOC = point->getWrOP();
    cpData[iplane].WrOC = cpData[ipoint].WrOC+cpData[iplane].Wn*g;
    WrPC[iplane] = cpData[iplane].WrOC - plane->getWrOP();
    WvC[ipoint] = point->getWvP();
    WvC[iplane] = plane->getWvP()+crossProduct(plane->getWomegaC(),WrPC[iplane]);
    Vec WvD; WvD = WvC[iplane] - WvC[ipoint]; 
    gd(0) = trans(cpData[iplane].Wn)*WvD;

    if(cpData[iplane].Wt.cols()) {
// ToDo: Pruefen: macht alles nur sinn bei exakt ZWEI Reibrichtungen!!!!!!!!!!!!
      cpData[iplane].Wt.col(0) = computeTangential(cpData[iplane].Wn);
	  cpData[iplane].Wt.col(1) = crossProduct(cpData[iplane].Wn,cpData[iplane].Wt.col(0));
	  cpData[ipoint].Wt= -cpData[iplane].Wt;
	  static Index iT(1,cpData[ipoint].Wt.cols());
	  gd(iT) = trans(cpData[iplane].Wt)*WvD;
    }
  } 

}
