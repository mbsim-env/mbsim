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
 * Contact: martin.o.foerg@googlemail.com
 */

#include<config.h>
#include "mbsim/contours/area.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/grid.h>
#endif

using namespace std;
using namespace fmatvec;

namespace MBSim {

  Area::Area(const string &name) :
      Plane(name), limy(1), limz(1), RrA(0), RrB(0), RrC(0), RrD(0) {
  }

  void Area::init(InitStage stage) {
    if(stage == calculateLocalInitialValues)
      setVertices();

    Plane::init(stage);
  }

  void Area::setVertices() {
    {
      //coordinates of the vertices under reference frame of the area
      RrA(0) = 0.;
      RrA(1) = limy / 2;
      RrA(2) = limz / 2;

      RrB(0) = 0.;
      RrB(1) = -limy / 2;
      RrB(2) = limz / 2;

      RrC(0) = 0.;
      RrC(1) = -limy / 2;
      RrC(2) = -limz / 2;

      RrD(0) = 0.;
      RrD(1) = limy / 2;
      RrD(2) = -limz / 2;
    }

  }

  bool Area::PointInArea(const fmatvec::Vec3& Point) {
    //point vector in coordinates of reference frame of the area
    Vec3 P_inA = this->getFrame()->getOrientation().T() * (Point - this->getFrame()->getPosition());

    // Here we suppose that the area in our codes has axis x parallel to its normal
    double y = this->limy, z = this->limz;
    if ((P_inA(1) <= y / 2) && (P_inA(1) >= (-y / 2)) && (P_inA(2) <= z / 2) && (P_inA(2) >= (-z / 2))) {
      return true;
    }
    return false;
  }

  bool Area::PointInCircle(const fmatvec::Vec3& Point, const fmatvec::Vec3& CenCir, const double & radius) {
    double dis = nrm2(Point - CenCir);
    if (dis > radius) {
      return false;
    }
    return true;
  }

  // this algorithm comes from http://doswa.com/2009/07/13/circle-segment-intersectioncollision.html
  fmatvec::Vec3 Area::Point_closest_toCircle_onLineseg(const fmatvec::Vec3 & EndP1, const fmatvec::Vec3 & EndP2, const fmatvec::Vec3& CenCir) {
    //vector of the line segment
    Vec3 SegV = EndP2 - EndP1;
    //vector from end point 1 to center of the circle
    Vec3 CP1V = CenCir - EndP1;
    //unit vector along SegV
    Vec3 SegV_unit = SegV / nrm2(SegV);
    //projection of CP1V on SegV
    double proj = CP1V.T() * SegV_unit;
    if (proj <= 0.) {
      return EndP1;
    }
    if (proj >= nrm2(SegV)) {
      return EndP2;
    }
    Vec3 Vproj = SegV_unit * proj;
    Vec3 Closest = Vproj + EndP1;
    return Closest;
  }

  bool Area::Intersect_Circle(const double & radi, const fmatvec::Vec3& CenCir) {
    //vertices under inertial frame
    Vec3 IrA(NONINIT), IrB(NONINIT), IrC(NONINIT), IrD(NONINIT);
    //closest point on the line segment
    Vec3 Closest(NONINIT);
    //center of area
    Vec3 C_A = this->getFrame()->getPosition();
    //orientation matrix
    SqrMat3 OriMat = this->getFrame()->getOrientation();

    //translate the coordinates under the inertial frame
    IrA = C_A + OriMat * RrA;
    IrB = C_A + OriMat * RrB;
    IrC = C_A + OriMat * RrC;
    IrD = C_A + OriMat * RrD;

    //if CenCir lies in the square, intersect
    if (PointInArea(CenCir)) {
      return true;
    }

    //if any of the vertices lies in the circle, intersect
    if (PointInCircle(IrA, CenCir, radi)) {
      return true;
    }
    if (PointInCircle(IrB, CenCir, radi)) {
      return true;
    }
    if (PointInCircle(IrC, CenCir, radi)) {
      return true;
    }
    if (PointInCircle(IrD, CenCir, radi)) {
      return true;
    }

    //check if any of the 4 edges of the area(AB,BC,CD,DA) intersect with the square
    //firstly select the nearest point to the circle on the line segment
    Closest = Point_closest_toCircle_onLineseg(IrA, IrB, CenCir);
    if (PointInCircle(Closest, CenCir, radi)) {
      return true;
    }
    Closest = Point_closest_toCircle_onLineseg(IrB, IrC, CenCir);
    if (PointInCircle(Closest, CenCir, radi)) {
      return true;
    }
    Closest = Point_closest_toCircle_onLineseg(IrC, IrD, CenCir);
    if (PointInCircle(Closest, CenCir, radi)) {
      return true;
    }
    Closest = Point_closest_toCircle_onLineseg(IrD, IrA, CenCir);
    if (PointInCircle(Closest, CenCir, radi)) {
      return true;
    }

    return false;
  }

#ifdef HAVE_OPENMBVCPPINTERFACE
  void Area::enableOpenMBV(bool enable, int number) {
    if (enable) {
      openMBVRigidBody = new OpenMBV::Grid;
      ((OpenMBV::Grid*) openMBVRigidBody)->setXSize(limy);
      ((OpenMBV::Grid*) openMBVRigidBody)->setYSize(limz);
      ((OpenMBV::Grid*) openMBVRigidBody)->setXNumber(number);
      ((OpenMBV::Grid*) openMBVRigidBody)->setYNumber(number);
      ((OpenMBV::Grid*) openMBVRigidBody)->setInitialRotation(0., M_PI / 2., 0.);
    }
    else
      openMBVRigidBody = 0;
  }
#endif

}
