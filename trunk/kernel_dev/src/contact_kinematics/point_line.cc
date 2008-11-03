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

  void ContactKinematicsPointLine::updateg(Vec &g, vector<CoordinateSystem*> &cosy) {
    Vec Wn;

    cosy[iline]->setOrientation(line->getCoordinateSystem()->getOrientation());
    cosy[ipoint]->setOrientation(-line->getCoordinateSystem()->getOrientation());

    Wn = cosy[iline]->getOrientation().col(1);

    Vec Wd =  line->getCoordinateSystem()->getPosition() - point->getCoordinateSystem()->getPosition();

    g(0) = trans(Wn)*Wd;

    cosy[ipoint]->setPosition(point->getCoordinateSystem()->getPosition());
    cosy[iline]->setPosition(cosy[ipoint]->getPosition() + Wn*g(0));
  }

  void ContactKinematicsPointLine::updategd(const Vec& g, Vec &gd, vector<CoordinateSystem*> &cosy) {
    Vec WrPC[2];
    Vec Wn = cosy[iline]->getOrientation().col(1);

    WrPC[iline] = cosy[iline]->getPosition() - line->getCoordinateSystem()->getPosition();
    WrPC[ipoint] = Vec(3);

    cosy[ipoint]->setVelocity(point->getCoordinateSystem()->getVelocity());
    cosy[iline]->setVelocity(line->getCoordinateSystem()->getVelocity() + crossProduct(line->getCoordinateSystem()->getAngularVelocity(),WrPC[iline]));
   cosy[iline]->setAngularVelocity(line->getCoordinateSystem()->getAngularVelocity());
   cosy[ipoint]->setAngularVelocity(point->getCoordinateSystem()->getAngularVelocity());

    Vec WvD = cosy[iline]->getVelocity() - cosy[ipoint]->getVelocity();

    gd(0) = trans(Wn)*WvD;

    if(gd.size()>1) {
      Mat Wt(3,gd.size()-1);
      Wt.col(0) = cosy[iline]->getOrientation().col(0);
      if(gd.size() > 2)
	Wt.col(1) = cosy[iline]->getOrientation().col(2);

      gd(1,gd.size()-1) = trans(Wt)*WvD;
    }

   Mat tWrPC[2];
   tWrPC[iline] = tilde(WrPC[iline]);
   tWrPC[ipoint] = tilde(WrPC[ipoint]);

   cosy[iline]->setJacobianOfTranslation(line->getCoordinateSystem()->getJacobianOfTranslation() - tWrPC[iline]*line->getCoordinateSystem()->getJacobianOfRotation());
   cosy[iline]->setJacobianOfRotation(line->getCoordinateSystem()->getJacobianOfRotation());
   cosy[iline]->setGyroscopicAccelerationOfTranslation(line->getCoordinateSystem()->getGyroscopicAccelerationOfTranslation() - tWrPC[iline]*line->getCoordinateSystem()->getGyroscopicAccelerationOfRotation() + crossProduct(line->getCoordinateSystem()->getAngularVelocity(),crossProduct(line->getCoordinateSystem()->getAngularVelocity(),WrPC[iline])));
   cosy[iline]->setGyroscopicAccelerationOfRotation(line->getCoordinateSystem()->getGyroscopicAccelerationOfRotation());

   cosy[ipoint]->setJacobianOfTranslation(point->getCoordinateSystem()->getJacobianOfTranslation() - tWrPC[ipoint]*point->getCoordinateSystem()->getJacobianOfRotation());
   cosy[ipoint]->setJacobianOfRotation(point->getCoordinateSystem()->getJacobianOfRotation());
   cosy[ipoint]->setGyroscopicAccelerationOfTranslation(point->getCoordinateSystem()->getGyroscopicAccelerationOfTranslation() - tWrPC[ipoint]*point->getCoordinateSystem()->getGyroscopicAccelerationOfRotation() + crossProduct(point->getCoordinateSystem()->getAngularVelocity(),crossProduct(point->getCoordinateSystem()->getAngularVelocity(),WrPC[ipoint])));
   cosy[ipoint]->setGyroscopicAccelerationOfRotation(point->getCoordinateSystem()->getGyroscopicAccelerationOfRotation());
  }
  
  void ContactKinematicsPointLine::updatewb(Vec &wb, const Vec &g, vector<CoordinateSystem*> &cosy) {

    Vec b1 = cosy[iline]->getOrientation().col(2);
    Vec Wn1 = cosy[iline]->getOrientation().col(1);
    Vec Wn2 = cosy[ipoint]->getOrientation().col(1);
    Vec Wt1 = cosy[iline]->getOrientation().col(0);
    Vec Wt2 = cosy[ipoint]->getOrientation().col(0);
    Vec vC1 = cosy[iline]->getVelocity();
    Vec vC2 = cosy[ipoint]->getVelocity();
    Vec Om1 = cosy[iline]->getAngularVelocity();
    Vec Om2 = cosy[ipoint]->getAngularVelocity();

    double kappa_sd2 = -trans(b1)*(Om2 - Om1);
    double sd1 = trans(Wt1)*(vC2 - vC1) - g(0)*trans(b1)*Om1;
    wb(0) += trans(Wn1)*(-crossProduct(Om1,vC1)) + sd1 * trans(b1) * Om1
      + trans(Wn2)*(-crossProduct(Om2,vC2)) - kappa_sd2*trans(Wt2)*vC2;
    if(wb.size() > 1)
      wb(1) += trans(Wt1)*(-crossProduct(Om1,vC1)) + trans(Wt2)*(-crossProduct(Om2,vC2)) + kappa_sd2*trans(Wn2)*vC2;
  }
}

