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
 *          thschindler@users.berlios.de
 */

#include <config.h> 
#include "point_frustum.h"
#include "mbsim/contour.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsPointFrustum::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Point*>(contour[0])) {
      ipoint = 0;
      ifrustum = 1;
      point = static_cast<Point*>(contour[0]);
      frustum = static_cast<Frustum*>(contour[1]);
    }
    else {
      ipoint = 1;
      ifrustum = 0;
      point = static_cast<Point*>(contour[1]);
      frustum = static_cast<Frustum*>(contour[0]);
    }
  }

  void ContactKinematicsPointFrustum::stage1(Vec &g, vector<ContourPointData> &cpData) {
//    double eps = 5.e-2; // tolerance for rough contact description
//    Vec Wd = point->getWrOP() - frustum->getWrOP(); // difference vector of Point and Frustum basis point in inertial FR
//    Vec Wa = frustum->getAWC()*frustum->getAxis(); // axis in inertial FR
//    Vec r = frustum->getRadii(); // radii of Frustum
//    double h = frustum->getHeight(); // height of Frustum	    
//    double s = trans(Wd)*Wa; // projection of difference vector on axis
//    double d = sqrt(pow(nrm2(Wd),2)-pow(s,2)); // distance Point to Frustum axis
//    if(h==0.) {
//      cout << "ERROR: Frustum with height = 0!" << endl;
//      throw(1);
//    }
//    double r_h = r(0) + (r(1)-r(0))/h * s; // radius of Frustum at s
//    bool outCont = frustum->getOutCont(); // contact on outer surface?
//
//    if(s<0 || s>h  || d < (r_h-eps) || d > (r_h+eps)) g(0) = 1.;		    
//    else {
//      if(outCont) { // contact on outer surface
//        double  phi = atan((r(1) - r(0))/h); // half cone angle
//        Vec b = Wd-s*Wa;
//        b /= d;
//        cpData[ifrustum].Wn = sin(phi)*Wa - cos(phi)*b;
//        cpData[ipoint].Wn  = -cpData[ifrustum].Wn;    
//        g(0) = (d-r_h)*cos(phi);
//      }
//      else { // contact on inner surface
//        double  phi = atan((r(1) - r(0))/h); // half cone angle
//        Vec b = Wd-s*Wa;
//        b /= d;
//        cpData[ifrustum].Wn = -(sin(phi)*Wa - cos(phi)*b);
//        cpData[ipoint].Wn  = -cpData[ifrustum].Wn;
//        g(0) = (r_h-d)*cos(phi);
//      }  
//    } 
  }

  void ContactKinematicsPointFrustum::stage2(const Vec& g, Vec &gd, vector<ContourPointData> &cpData) {
//    Vec WrPC[2], WvC[2];
//
//    cpData[ipoint].WrOC= point->getWrOP();
//    cpData[ifrustum].WrOC =  cpData[ipoint].WrOC - cpData[ipoint].Wn*g;
//    WrPC[ifrustum] = cpData[ifrustum].WrOC - frustum->getWrOP();
//    WvC[ipoint] = point->getWvP();
//    WvC[ifrustum] = frustum->getWvP()+crossProduct(frustum->getWomegaC(),WrPC[ifrustum]);
//    Vec WvD = WvC[ipoint] - WvC[ifrustum];
//    gd(0) = trans(cpData[ipoint].Wn)*WvD; // positive for enlarging the distance between Point and Frustum
//
//    if(cpData[ipoint].Wt.cols()) {
//      // If the Frustum does not degenerate to a Cylinder, the first column is the tangential direction
//      // and the second column the radial direction of the Frustum:
//      if(cpData[ipoint].Wt.cols() == 1) {
//        cout << "ERROR: Two tangential contact directions necessary for spatial contact!" << endl;
//        throw(1);
//      }
//      cpData[ipoint].Wt.col(0) = computeTangential(cpData[ipoint].Wn);
//      cpData[ipoint].Wt.col(1) = crossProduct(cpData[ipoint].Wn,cpData[ipoint].Wt.col(0));
//      cpData[ifrustum].Wt = -cpData[ipoint].Wt;
//      static Index iT(1,cpData[ipoint].Wt.cols());
//      gd(iT) = trans(cpData[ipoint].Wt)*WvD;
//    }
  }
}

