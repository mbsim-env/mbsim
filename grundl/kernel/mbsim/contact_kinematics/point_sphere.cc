/* Copyright (C) 2004-2013 MBSim Development Team
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
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h> 
#include "point_sphere.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/sphere.h"
#include "mbsim/utils/eps.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsPointSphere::assignContours(const vector<Contour*> &contour) {
    ipoint = 0; isphere = 1;
    point = static_cast<Point*>(contour[0]);
    sphere = static_cast<Sphere*>(contour[1]);
  }

  void ContactKinematicsPointSphere::updateg(Vec &g, ContourPointData *cpData, int index) {
    Vec3 Wd = sphere->getFrame()->getPosition() - point->getFrame()->getPosition();
    double l = nrm2(Wd);
    Wd = Wd/l;
    g(0) = l-sphere->getRadius();
    Vec3 t;
    if(fabs(Wd(0))<epsroot() && fabs(Wd(1))<epsroot()) {
      t(0) = 1.;
      t(1) = 0.;
      t(2) = 0.;
    }
    else {
      t(0) = -Wd(1);
      t(1) = Wd(0);
      t(2) = 0.0;
    }
    t = t/nrm2(t);
    cpData[ipoint].getFrameOfReference().getOrientation().set(0, Wd);
    cpData[isphere].getFrameOfReference().getOrientation().set(0, -cpData[ipoint].getFrameOfReference().getOrientation().col(0));
    cpData[ipoint].getFrameOfReference().getOrientation().set(1, t);
    cpData[isphere].getFrameOfReference().getOrientation().set(1, -cpData[ipoint].getFrameOfReference().getOrientation().col(1));
    cpData[ipoint].getFrameOfReference().getOrientation().set(2, crossProduct(Wd,t));
    cpData[isphere].getFrameOfReference().getOrientation().set(2, cpData[ipoint].getFrameOfReference().getOrientation().col(2));
    cpData[ipoint].getFrameOfReference().getPosition() = point->getFrame()->getPosition();
    cpData[isphere].getFrameOfReference().getPosition() = sphere->getFrame()->getPosition() - sphere->getRadius() * Wd;
  }

}


//  void ContactKinematicsPointSphere::stage2(const Vec& g, Vec &gd, vector<ContourPointData> &cpData) {

//    Vec WrPC[2], WvC[2];
//
//    WrPC[isphere] = cpData[ipoint].Wn*sphere->getRadius();
//    cpData[isphere].WrOC = sphere->getWrOP()+WrPC[isphere];
//    WrPC[ipoint] = cpData[ipoint].Wn*(-point->getRadius());
//    cpData[ipoint].WrOC = point->getWrOP()+WrPC[ipoint];
//    WvC[ipoint] = point->getWvP()+crossProduct(point->getWomegaC(),WrPC[ipoint]);
//    WvC[isphere] = sphere->getWvP()+crossProduct(sphere->getWomegaC(),WrPC[isphere]);
//    Vec WvD = WvC[ipoint] - WvC[isphere];
//    gd(0) = trans(cpData[ipoint].Wn)*WvD;
//
//    if(cpData[ipoint].Wt.cols()) {
//      cpData[ipoint].Wt.col(0) = computeTangential(cpData[ipoint].Wn);
//      if(cpData[ipoint].Wt.cols()==2) {
//        cpData[ipoint].Wt.col(1) = crossProduct(cpData[ipoint].Wn ,cpData[ipoint].Wt.col(0));
//        cpData[isphere].Wt  = -cpData[ipoint].Wt ; 
//        static Index iT(1,cpData[ipoint].Wt.cols());
//        gd(iT) = trans(cpData[ipoint].Wt)*WvD;
//      }
//    }
//  }
