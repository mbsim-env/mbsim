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
#include "point_frustum.h"
#include "contour.h"

namespace MBSim {

  void ContactKinematicsPointFrustum::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Point*>(contour[0])) {
      ipoint = 0; ifrustum = 1;
      point = static_cast<Point*>(contour[0]);
      frustum = static_cast<Frustum*>(contour[1]);
    } else {
      ipoint = 1; ifrustum = 0;
      point = static_cast<Point*>(contour[1]);
      frustum = static_cast<Frustum*>(contour[0]);
    }
  }

  void ContactKinematicsPointFrustum::stage1(Vec &g, vector<ContourPointData> &cpData) {

    Vec Wd = point->getWrOP() - frustum->getWrOP();
    Vec Wa = frustum->getAWC()*frustum->getAxis();
    double h = frustum->getHeight();
    double loc = trans(Wd)*Wa;
    // TODO Pruefen ob aussen oder innen
    if(loc<0 || loc>h) {
      g(0) = 1;
      //  active = false;
    } else {
      Vec r = frustum->getRadii();
      // Halber Oeffnungswinkel
      double  phi = atan((r(0) - r(1))/h);
      phi = M_PI*0.5 - phi;
      Vec Wrot = crossProduct(Wa,Wd);
      Wrot /= nrm2(Wrot);
      //Wn = -(cos(phi)*Wa + (1-cos(phi))*(trans(Wa)*Wrot)*Wrot + sin(phi)*crossProduct(Wrot,Wa));
      cpData[ifrustum].Wn =  cos(phi)*Wa + (1-cos(phi))*(trans(Wa)*Wrot)*Wrot + sin(phi)*crossProduct(Wrot,Wa);
      cpData[ipoint].Wn  = -cpData[ifrustum].Wn;

      double r_h = r(0) + (r(1)-r(0))/h * loc;
      Vec b = crossProduct(Wrot,Wa);
      double l = trans(Wd)*b;
      Vec c = (r_h - l)*b;

      g(0) = trans(cpData[ifrustum].Wn)*c;
    } 
  }

  void ContactKinematicsPointFrustum::stage2(const Vec& g, Vec &gd, vector<ContourPointData> &cpData) {

    Vec WrPC[2], WvC[2];

    cpData[ipoint].WrOC= point->getWrOP();
    cpData[ifrustum].WrOC =  cpData[ipoint].WrOC - cpData[ipoint].Wn*g;
    WrPC[ifrustum] = cpData[ifrustum].WrOC - frustum->getWrOP();
    WvC[ipoint] = point->getWvP();
    WvC[ifrustum] = frustum->getWvP()+crossProduct(frustum->getWomegaC(),WrPC[ifrustum]);
    Vec WvD = WvC[ipoint] - WvC[ifrustum];
    gd(0) = trans(cpData[ipoint].Wn)*WvD;

    if(cpData[ipoint].Wt.cols()) {
      cpData[ipoint].Wt.col(0) = computeTangential(cpData[ipoint].Wn);
      cpData[ipoint].Wt.col(1) = crossProduct(cpData[ipoint].Wn,cpData[ipoint].Wt.col(0));
      cpData[ifrustum].Wt = -cpData[ipoint].Wt;
      static Index iT(1,cpData[ipoint].Wt.cols());
      gd(iT) = trans(cpData[ipoint].Wt)*WvD;
    }
  }

}
