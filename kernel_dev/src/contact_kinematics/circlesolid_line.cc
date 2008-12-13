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
#include "circlesolid_line.h"
#include "contact.h"

namespace MBSim {

  void ContactKinematicsCircleSolidLine::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<CircleSolid*>(contour[0])) {
      icircle = 0; iline = 1;
      circlesolid = static_cast<CircleSolid*>(contour[0]);
      line = static_cast<Line*>(contour[1]);
    } else {
      icircle = 1; iline = 0;
      circlesolid = static_cast<CircleSolid*>(contour[1]);
      line = static_cast<Line*>(contour[0]);
    }
  }

  void ContactKinematicsCircleSolidLine::updateg(Vec &g, ContourPointData *cpData) {

    cpData[iline].cosy.setOrientation(line->getCoordinateSystem()->getOrientation());
    cpData[icircle].cosy.getOrientation().col(0) = -line->getCoordinateSystem()->getOrientation().col(0);
    cpData[icircle].cosy.getOrientation().col(1) = -line->getCoordinateSystem()->getOrientation().col(1);
    cpData[icircle].cosy.getOrientation().col(2) = line->getCoordinateSystem()->getOrientation().col(2);

    Vec Wn = cpData[iline].cosy.getOrientation().col(1);

    Vec Wd = line->getCoordinateSystem()->getPosition() - circlesolid->getCoordinateSystem()->getPosition();

    g(0) = trans(Wn)*Wd - circlesolid->getRadius();

    cpData[icircle].cosy.setPosition(circlesolid->getCoordinateSystem()->getPosition() + Wn*circlesolid->getRadius());
    cpData[iline].cosy.setPosition(cpData[icircle].cosy.getPosition() + Wn*g(0));
  }

  void ContactKinematicsCircleSolidLine::updategd(const Vec &g, Vec &gd, ContourPointData *cpData) {}

  void ContactKinematicsCircleSolidLine::updatewb(Vec &wb, const Vec &g, ContourPointData *cpData) {

    Vec b1 = cpData[iline].cosy.getOrientation().col(2);
    Vec b2 = cpData[icircle].cosy.getOrientation().col(2);
    Vec n1 = cpData[iline].cosy.getOrientation().col(1);
    Vec n2 = cpData[icircle].cosy.getOrientation().col(1);
    Vec t1 = cpData[iline].cosy.getOrientation().col(0);
    Vec t2 = cpData[icircle].cosy.getOrientation().col(0);
    Vec vC1 = cpData[iline].cosy.getVelocity();
    Vec vC2 = cpData[icircle].cosy.getVelocity();
    Vec Om1 = cpData[iline].cosy.getAngularVelocity();
    Vec Om2 = cpData[icircle].cosy.getAngularVelocity();

    double ad2 = -trans(b2)*(Om2-Om1);
    double ad1 = trans(t1)*(vC2-vC1) - circlesolid->getRadius()*ad2;
    Vec s2 = t2*circlesolid->getRadius();

    //double kap1 = 0; // Gerade
    //double kap2 = 1.0/circlesolid->getRadius(); // Kreis
    //double sd1 = (kap2*trans(t1)*(vC2-vC1)+trans(b1)*(Om2-Om1))/(kap1+kap2);
    //double sd2 = (kap1*trans(t2)*(vC1-vC2)+trans(b2)*(Om1-Om2))/(kap1+kap2);

    wb(0) += -trans(n1)*(-crossProduct(Om1,vC2-vC1) - crossProduct(Om1,t1)*ad1 + crossProduct(Om2,s2)*ad2);
    //wb(0) += trans(n1)*(-crossProduct(Om1,vC1)) - kap1*sd1*trans(t1)*vC1 + sd1*trans(b1)*Om1 + trans(n2)*(-crossProduct(Om2,vC2)) - kap2*sd2*trans(t2)*vC2 + sd2*trans(b1)*Om2;
    
    if(wb.size() > 1) 
      wb(1) += -trans(t1)*(-crossProduct(Om1,vC2-vC1) - crossProduct(Om1,t1)*ad1 + crossProduct(Om2,s2)*ad2);
     // wb(1) += trans(t1)*(-crossProduct(Om1,vC1)) + kap1*sd1*trans(n1)*vC1 + trans(t2)*(-crossProduct(Om2,vC2)) + kap2*sd2*trans(n2)*vC2;
    
  }

}

