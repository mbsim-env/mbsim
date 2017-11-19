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

#include <config.h>
#include "mbsim/contours/plate.h"
#include "mbsim/frames/frame.h"

#include <openmbvcppinterface/grid.h>

using namespace std;
using namespace fmatvec;

namespace MBSim {

  Plate::Plate(const string &name, Frame *R) : Plane(name,R),  RrA(0), RrB(0), RrC(0), RrD(0) {
  }

  Plate::Plate(const string &name, double yL, double zL, Frame *R) : Plane(name,R), yLength(yL), zLength(zL), RrA(0), RrB(0), RrC(0), RrD(0) {
  }

  Plate::Plate(const string &name, double yL, double zL, double t, Frame *R) : Plane(name,R), yLength(yL), zLength(zL), RrA(0), RrB(0), RrC(0), RrD(0) {
    thickness = t;
  }

  void Plate::init(InitStage stage, const InitConfigSet &config) {
    if (stage == preInit)
      setVertices();
    else if(stage==plotting) {
      if(plotFeature[openMBV] && openMBVRigidBody)
        if(openMBVRigidBody) static_pointer_cast<OpenMBV::Cuboid>(openMBVRigidBody)->setLength(0,yLength,zLength);
    }
    Plane::init(stage, config);
  }

  void Plate::setVertices() {
    {
      //coordinates of the vertices under reference frame of the plate
      RrA(0) = 0.;
      RrA(1) = yLength / 2;
      RrA(2) = zLength / 2;

      RrB(0) = 0.;
      RrB(1) = -yLength / 2;
      RrB(2) = zLength / 2;

      RrC(0) = 0.;
      RrC(1) = -yLength / 2;
      RrC(2) = -zLength / 2;

      RrD(0) = 0.;
      RrD(1) = yLength / 2;
      RrD(2) = -zLength / 2;
    }

  }

  bool Plate::PointInPlate(const fmatvec::Vec3& Point) {
    //point vector in coordinates of reference frame of the plate
    Vec3 P_inA = this->getFrame()->getOrientation().T() * (Point - this->getFrame()->getPosition());

    // Here we suppose that the plate in our codes has axis x parallel to its normal
    return (P_inA(1) <= yLength / 2) && (P_inA(1) >= (-yLength / 2)) && (P_inA(2) <= zLength / 2) && (P_inA(2) >= (-zLength / 2));
  }

  bool Plate::PointInCircle(const fmatvec::Vec3& Point, const fmatvec::Vec3& CenCir, const double & radius) {
    double dis = nrm2(Point - CenCir);
    return dis <= radius;
  }

  // this algorithm comes from http://doswa.com/2009/07/13/circle-segment-intersectioncollision.html
  Vec3 Plate::Point_closest_toCircle_onLineseg(const Vec3 & EndP1, const Vec3 & EndP2, const Vec3& CenCir) {
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
    return EndP1 + proj * SegV_unit;
  }

  bool Plate::Intersect_Circle(const double & radi, const Vec3& CenCir) {
    //if CenCir lies in the square, intersect
    if (PointInPlate(CenCir)) {
      return true;
    }

    //center of plate
    Vec3 C_A = this->getFrame()->getPosition();
    //orientation matrix
    SqrMat3 OriMat = this->getFrame()->getOrientation();

    //vertices in world frame
    Vec3 IrA = C_A + OriMat * RrA;
    Vec3 IrB = C_A + OriMat * RrB;
    Vec3 IrC = C_A + OriMat * RrC;
    Vec3 IrD = C_A + OriMat * RrD;

    //closest point on the line segment
    Vec3 Closest(NONINIT);

//    //if any of the vertices lies in the circle, intersect --> Check is done due to Closest point on linesegment check already!
//    if (PointInCircle(IrA, CenCir, radi)) {
//      return true;
//    }
//    if (PointInCircle(IrB, CenCir, radi)) {
//      return true;
//    }
//    if (PointInCircle(IrC, CenCir, radi)) {
//      return true;
//    }
//    if (PointInCircle(IrD, CenCir, radi)) {
//      return true;
//    }

//check if any of the 4 edges of the plate(AB,BC,CD,DA) intersect with the square
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
    return PointInCircle(Closest, CenCir, radi);
  }

}
