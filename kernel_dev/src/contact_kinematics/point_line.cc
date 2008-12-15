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
#include "point_line.h"
#include "contour.h"

namespace MBSim {

  void ContactKinematicsPointLine::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Point*>(contour[0])) {
      ipoint = 0; iline = 1;
      point = static_cast<Point*>(contour[0]);
      line = static_cast<Line*>(contour[1]);
    } else {
      ipoint = 1; iline = 0;
      point = static_cast<Point*>(contour[1]);
      line = static_cast<Line*>(contour[0]);
    }
  }

  void ContactKinematicsPointLine::updateg(Vec &g, ContourPointData *cpData) {

    cpData[iline].cosy.setOrientation(line->getCoordinateSystem()->getOrientation());
    cpData[ipoint].cosy.getOrientation().col(0) = -line->getCoordinateSystem()->getOrientation().col(0);
    cpData[ipoint].cosy.getOrientation().col(1) = -line->getCoordinateSystem()->getOrientation().col(1);
    cpData[ipoint].cosy.getOrientation().col(2) = line->getCoordinateSystem()->getOrientation().col(2);

    Vec Wn = cpData[iline].cosy.getOrientation().col(0);

    Vec Wd =  point->getCoordinateSystem()->getPosition() - line->getCoordinateSystem()->getPosition();

    g(0) = trans(Wn)*Wd;

    cpData[ipoint].cosy.setPosition(point->getCoordinateSystem()->getPosition());
    cpData[iline].cosy.setPosition(cpData[ipoint].cosy.getPosition() - Wn*g(0));
  }

  void ContactKinematicsPointLine::updategd(const Vec& g, Vec &gd, ContourPointData *cpData) {}

  void ContactKinematicsPointLine::updatewb(Vec &wb, const Vec &g, ContourPointData *cpData) {

    Vec n1 = cpData[iline].cosy.getOrientation().col(0);
    Vec u1 = cpData[iline].cosy.getOrientation().col(1);
    Vec vC1 = cpData[iline].cosy.getVelocity();
    Vec vC2 = cpData[ipoint].cosy.getVelocity();
    Vec Om1 = cpData[iline].cosy.getAngularVelocity();
    Vec Om2 = cpData[ipoint].cosy.getAngularVelocity();

    double sd1 = trans(u1)*(vC2 - vC1); 

    wb(0) += trans(n1)*(-crossProduct(Om1,vC2-vC1) - crossProduct(Om1,u1)*sd1);

    if(wb.size() > 1) 
      wb(1) += trans(u1)*(-crossProduct(Om1,vC2-vC1) - crossProduct(Om1,u1)*sd1);
  }
}

