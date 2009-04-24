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
#include "circlesolid_plane.h"
#include "mbsim/contour.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsCircleSolidPlane::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<CircleSolid*>(contour[0])) {
      icircle = 0;
      iplane = 1;
      circlesolid = static_cast<CircleSolid*>(contour[0]);
      plane = static_cast<Plane*>(contour[1]);
    } 
    else {
      icircle = 1;
      iplane = 0;
      circlesolid = static_cast<CircleSolid*>(contour[1]);
      plane = static_cast<Plane*>(contour[0]);
    }
  }

  void ContactKinematicsCircleSolidPlane::updateg(Vec &g, ContourPointData *cpData) {
    cpData[iplane].getFrameOfReference().setOrientation(plane->getFrame()->getOrientation());
    cpData[icircle].getFrameOfReference().getOrientation().col(0) = -plane->getFrame()->getOrientation().col(0);
    cpData[icircle].getFrameOfReference().getOrientation().col(1) = -plane->getFrame()->getOrientation().col(1);
    cpData[icircle].getFrameOfReference().getOrientation().col(2) = plane->getFrame()->getOrientation().col(2);

    Vec Wd;
    Vec Wn = cpData[iplane].getFrameOfReference().getOrientation().col(0);
    Vec Wb = circlesolid->getFrame()->getOrientation().col(2);
    double t_EC = trans(Wn)*Wb;
    if(t_EC>0) {
      Wb *= -1.;
      t_EC *= -1;	
    }
    //cout << Wn << endl;
    //cout << Wb << endl;
    Vec z_EC = Wn - t_EC*Wb;
    double z_EC_nrm2 = nrm2(z_EC);
    if(z_EC_nrm2 <= 1e-8) { // infinite possible contact points
      Wd = circlesolid->getFrame()->getPosition() - plane->getFrame()->getPosition();
    } 
    else { // exactly one possible contact point
      Wd =  (circlesolid->getFrame()->getPosition() - (circlesolid->getRadius()/z_EC_nrm2)*z_EC) - plane->getFrame()->getPosition();
    }
    //cout << z_EC<<endl;
    //cout << Wd << endl;
    //cout << Wn << endl;
    g(0) = trans(Wn)*Wd;
    //cout << g(0) << endl;
    cpData[icircle].getFrameOfReference().setPosition(circlesolid->getFrame()->getPosition() - (circlesolid->getRadius()/z_EC_nrm2)*z_EC);
    cpData[iplane].getFrameOfReference().setPosition(cpData[icircle].getFrameOfReference().getPosition() - Wn*g(0));

  }

}

