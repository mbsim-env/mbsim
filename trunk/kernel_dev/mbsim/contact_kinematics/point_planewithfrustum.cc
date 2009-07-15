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
 */

#include <config.h> 
#include "point_planewithfrustum.h"
#include "mbsim/contour.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsPointPlaneWithFrustum::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Point*>(contour[0])) {
      ipoint = 0;
      iplane = 1;
      point = static_cast<Point*>(contour[0]);
      plane = static_cast<PlaneWithFrustum*>(contour[1]);
    }
    else {
      ipoint = 1;
      iplane = 0;
      point = static_cast<Point*>(contour[1]);
      plane = static_cast<PlaneWithFrustum*>(contour[0]);
    }
    h=plane->getFrustumHeight();
    rPlane=plane->getFrustumRadiusOnPlane();
    rTop=plane->getFrustumRadiusOnTop();
  }

  void ContactKinematicsPointPlaneWithFrustum::updateg(Vec &g, ContourPointData* cpData) {

    cpData[iplane].getFrameOfReference().setOrientation(plane->getFrame()->getOrientation()); // data of possible contact point

    Vec WrOPoint = point->getFrame()->getPosition();
    Vec WrOPlane = plane->getFrame()->getPosition();
    Vec Wn = cpData[iplane].getFrameOfReference().getOrientation().col(0);
    Vec Wd =  -WrOPlane + WrOPoint;
    double distanceToWn = nrm2(crossProduct(Wn, Wd));

    bool searchOnPlane = (distanceToWn>=rPlane);
    bool searchOnTop = (distanceToWn<=rTop);

    Vec nDir(3);
    Vec gDir(3);
    Vec g2Dir(3);
    if (searchOnPlane||searchOnTop) { // contact possible only with infinite plane or with plane on frustum top
      g(0) = trans(Wn)*Wd - (searchOnTop?h:0); // distance

      nDir = Wn;
      g2Dir = crossProduct(Wd, nDir);
      gDir = crossProduct(nDir, g2Dir);
      gDir /= nrm2(gDir);
      g2Dir = crossProduct(gDir, nDir);
      g2Dir /= nrm2(g2Dir);

      // if (searchOnPlane)
      //   cerr << "plane ";
      // else
      //   cerr << "top ";
    }
    else { // contact with frustum contour

      Vec WrONn = WrOPlane + trans(Wd)*Wn*Wn; // nearest point to Point on Wn

      Vec WrNnPoint = -WrONn + WrOPoint;
      Vec WrNnPointDir = WrNnPoint / nrm2(WrNnPoint);

      // defining a line g as a line of potential contact points
      Vec gPointPlane = WrOPlane + WrNnPointDir * rPlane;
      Vec gPointTop = WrOPlane + Wn * h + WrNnPointDir * rTop;
      gDir = -gPointPlane+gPointTop;
      gDir /= nrm2(gDir); // tangential direction of contact pair
      Vec WrONg = gPointPlane + trans(-gPointPlane + WrOPoint)*gDir*gDir; // nearest point to Point on g
      Vec WrNgPoint = -WrONg + WrOPoint;

      g2Dir = crossProduct(gDir, WrNnPointDir); // binormal direction
      nDir = (h>0?-1.:1.)*crossProduct(gDir, g2Dir); // normal direction
      nDir/=nrm2(nDir);
      g2Dir = crossProduct(gDir, nDir);
      g2Dir/=nrm2(g2Dir);

      g(0)=(trans(WrNgPoint)*Wn>0?1.:-1.)*nrm2(-WrONg + WrOPoint);

      // cerr << "frustum ";
    }
    cpData[ipoint].getFrameOfReference().getPosition() = WrOPoint; // possible contact locations
    cpData[iplane].getFrameOfReference().getPosition() =  WrOPoint - nDir * g(0);

    cpData[iplane].getFrameOfReference().getOrientation().col(0) = nDir;
    cpData[iplane].getFrameOfReference().getOrientation().col(1) = gDir;
    cpData[iplane].getFrameOfReference().getOrientation().col(2) = g2Dir;
    cpData[ipoint].getFrameOfReference().getOrientation().col(0) = -nDir;
    cpData[ipoint].getFrameOfReference().getOrientation().col(1) = -gDir;
    cpData[ipoint].getFrameOfReference().getOrientation().col(2) = -g2Dir;

    // cerr << " g=" << g(0);
    // cerr << " n=[" << nDir(0) << " " << nDir(1) << " " << nDir(2) << "]" << endl;
  }
}

