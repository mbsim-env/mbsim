/* Copyright (C) 2004-2009 MBSim Development Team
 *
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
 * Contact: mfoerg@users.berlios.de
 *          rzander@users.berlios.de
 */

#include <config.h> 
#include "point_area.h"
#include "mbsim/contours/area.h"
#include "mbsim/contours/point.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsPointArea::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Point*>(contour[0])) {
      ipoint = 0; iarea = 1;
      point = static_cast<Point*>(contour[0]);
      area = static_cast<Area*>(contour[1]);
    } 
    else {
      ipoint = 1; iarea = 0;
      point = static_cast<Point*>(contour[1]);
      area = static_cast<Area*>(contour[0]);
    }
  }

  void ContactKinematicsPointArea::stage1(Vec &g, vector<ContourPointData> &cpData) {

//    Vec Wd = area->getWrOP() - point->getWrOP();
//    Vec Wd1 = area->computeWd1();
//    Vec Wd2 = area->computeWd2();
//    double a1 = -trans(Wd)*Wd1;
//    double a2 = -trans(Wd)*Wd2;
//    if(a1 > area->getLimit1() || a2 > area->getLimit2() ||
//	a1 < 0 || a2 < 0)
//      g(0) = 1;
//    else {
//      cpData[iarea].Wn  = area->computeWn();
//      cpData[ipoint].Wn = - cpData[iarea].Wn;
//      g(0) = trans(cpData[iarea].Wn)*Wd;
//      if(g(0) < -0.01)
//	g(0) = 1;
//    }
  }

  void ContactKinematicsPointArea::stage2(const Vec& g, Vec &gd, vector<ContourPointData> &cpData){

//    Vec WrPC[2], WvC[2];
//
//    cpData[ipoint].WrOC = point->getWrOP();
//    cpData[iarea].WrOC = cpData[ipoint].WrOC+cpData[iarea].Wn*g;
//    WrPC[iarea] = cpData[iarea].WrOC - area->getWrOP();
//    WvC[ipoint] = point->getWvP();
//    WvC[iarea] = area->getWvP()+crossProduct(area->getWomegaC(),WrPC[iarea]);
//    Vec WvD = WvC[iarea] - WvC[ipoint]; 
//    gd(0) = trans(cpData[iarea].Wn)*WvD;
//
//    if(cpData[ipoint].Wt.cols()) {
//      cpData[iarea].Wt.col(0) = computeTangential(cpData[iarea].Wn);
//      cpData[iarea].Wt.col(1) = crossProduct(cpData[iarea].Wn,cpData[iarea].Wt.col(0));
//      cpData[ipoint].Wt= -cpData[iarea].Wt;
//      static Index iT(1,cpData[ipoint].Wt.cols());
//      gd(iT) = trans(cpData[iarea].Wt)*WvD;
//    }
  }

}

