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
#include "sphere_frustum.h"
#include "contour.h"

namespace MBSim {

  void ContactKinematicsSphereFrustum::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Sphere*>(contour[0])) {
      isphere = 0; ifrustum = 1;
      sphere = static_cast<Sphere*>(contour[0]);
      frustum = static_cast<Frustum*>(contour[1]);
    } else {
      isphere = 1; ifrustum = 0;
      sphere = static_cast<Sphere*>(contour[1]);
      frustum = static_cast<Frustum*>(contour[0]);
    }
  }

  void ContactKinematicsSphereFrustum::stage1(Vec &g, vector<ContourPointData> &cpData){

    Vec Wd = sphere->getWrOP() - frustum->getWrOP();
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
      cpData[ifrustum].Wn = cos(phi)*Wa + (1-cos(phi))*(trans(Wa)*Wrot)*Wrot + sin(phi)*crossProduct(Wrot,Wa);
      cpData[isphere].Wn= -cpData[ifrustum].Wn;
      double r_h = r(0) + (r(1)-r(0))/h * loc;
      Vec b = crossProduct(Wrot,Wa);
      double l = trans(Wd)*b;
      Vec c = (r_h - l)*b;

      g(0) = trans(cpData[ifrustum].Wn)*c - sphere->getRadius();
    } 
  }

  void ContactKinematicsSphereFrustum::stage2(const Vec& g, Vec &gd, vector<ContourPointData> &cpData){

    Vec WrPC[2], WvC[2];

    WrPC[isphere] = cpData[ifrustum].Wn*sphere->getRadius();
    cpData[isphere].WrOC = sphere->getWrOP()+WrPC[isphere];
    cpData[ifrustum].WrOC = cpData[isphere].WrOC - cpData[isphere].Wn*g;
    WrPC[ifrustum] = cpData[ifrustum].WrOC - frustum->getWrOP();
    WvC[isphere] = sphere->getWvP()+crossProduct(sphere->getWomegaC(),WrPC[isphere]);
    WvC[ifrustum] = frustum->getWvP()+crossProduct(frustum->getWomegaC(),WrPC[ifrustum]);
    Vec WvD = WvC[isphere] - WvC[ifrustum];
    gd(0) = trans(cpData[isphere].Wn)*WvD;

    if(cpData[isphere].Wt.cols()) { 
      cpData[isphere].Wt.col(0) = computeTangential(cpData[isphere].Wn);
      if(cpData[isphere].Wt.cols()==2)
	cpData[isphere].Wt.col(1) = crossProduct(cpData[isphere].Wn,cpData[isphere].Wt.col(0));
      cpData[ifrustum].Wt = -cpData[isphere].Wt;
      static Index iT(1,cpData[isphere].Wt.cols());
      gd(iT) = trans(cpData[isphere].Wt)*WvD;
    }
  }

}
