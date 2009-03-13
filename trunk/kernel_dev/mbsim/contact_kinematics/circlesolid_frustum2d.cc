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
#include "circlesolid_frustum2d.h"
#include <mbsim/contact.h>

namespace MBSim {

  void ContactKinematicsCircleSolidFrustum2D::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<CircleSolid*>(contour[0])) {
      icircle = 0; ifrustum = 1;
      circle = static_cast<CircleSolid*>(contour[0]);
      frustum = static_cast<Frustum2D*>(contour[1]);
    } else {
      icircle = 1; ifrustum = 0;
      circle = static_cast<CircleSolid*>(contour[1]);
      frustum = static_cast<Frustum2D*>(contour[0]);
    }
  }

  void ContactKinematicsCircleSolidFrustum2D::stage1(Vec &g, vector<ContourPointData> &cpData) {

    Vec Wd = circle->getWrOP() - frustum->getWrOP();
    Vec Wa = frustum->getAWC()*frustum->getAxis();
    double h = frustum->getHeight();
    double loc = trans(Wd)*Wa;
    // TODO Pruefen ob aussen oder innen
    if(loc<0 || loc>h) {
      //active = false;
      g(0) = 1;
    } else {
      Vec r = frustum->getRadii();
      // Halber Oeffnungswinkel
      double  phi = atan((r(0) - r(1))/h);
      phi = M_PI*0.5 - phi;
      Vec Wrot = crossProduct(Wa,Wd);
      Wrot /= nrm2(Wrot);
      cpData[ifrustum].Wn = cos(phi)*Wa + (1-cos(phi))*(trans(Wa)*Wrot)*Wrot + sin(phi)*crossProduct(Wrot,Wa);
      cpData[icircle].Wn = -cpData[ifrustum].Wn;

      double r_h = r(0) + (r(1)-r(0))/h * loc;
      Vec b = crossProduct(Wrot,Wa);
      double l = trans(Wd)*b;
      Vec c = (r_h - l)*b;

      g(0) = trans(cpData[ifrustum].Wn)*c - circle->getRadius();
    }
  }

  void ContactKinematicsCircleSolidFrustum2D:: stage2(const Vec &g, Vec &gd, vector<ContourPointData> &cpData) {

    Vec WrPC[2], WvC[2];

    WrPC[icircle] = cpData[ifrustum].Wn*circle->getRadius();
    cpData[icircle].WrOC = circle->getWrOP() + WrPC[icircle];
    cpData[ifrustum].WrOC = cpData[icircle].WrOC + cpData[ifrustum].Wn*g;
    WrPC[ifrustum] = cpData[ifrustum].WrOC - frustum->getWrOP();
    WvC[icircle] = circle->getWvP()+crossProduct(circle->getWomegaC(),WrPC[icircle]);
    WvC[ifrustum] = frustum->getWvP()+crossProduct(frustum->getWomegaC(),WrPC[ifrustum]);
    Vec WvD = WvC[icircle] - WvC[ifrustum];
    gd(0) = trans(cpData[icircle].Wn)*WvD;

    if(cpData[icircle].Wt.cols()) {
      cpData[icircle].Wt = crossProduct(frustum->computeWb(),cpData[icircle].Wn);
      cpData[ifrustum].Wt = -cpData[icircle].Wt;
      static Index iT(1,cpData[icircle].Wt.cols());
      gd(iT) = trans(cpData[icircle].Wt)*WvD;
    }
  }

}
