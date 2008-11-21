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

  void ContactKinematicsPointPlane::updateg(Vec &g, ContourPointData* cpData)
  {
      cpData[iplane].cosy.setOrientation(plane->getCoordinateSystem()->getOrientation());
    cpData[ipoint].cosy.getOrientation().col(0) = -plane->getCoordinateSystem()->getOrientation().col(0);
    cpData[ipoint].cosy.getOrientation().col(1) = -plane->getCoordinateSystem()->getOrientation().col(1);
    cpData[ipoint].cosy.getOrientation().col(2) = plane->getCoordinateSystem()->getOrientation().col(2);
   Vec Wn = cpData[iplane].cosy.getOrientation().col(1);

    Vec Wd =  plane->getCoordinateSystem()->getPosition() - point->getCoordinateSystem()->getPosition();

    g(0) = trans(Wn)*Wd;

    cpData[ipoint].cosy.setPosition(point->getCoordinateSystem()->getPosition());
    cpData[iplane].cosy.setPosition(cpData[ipoint].cosy.getPosition() + Wn*g(0));
 }

  void ContactKinematicsPointPlane::updategd(const Vec& g, Vec &gd, ContourPointData *cpData) {}

}
