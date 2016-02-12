/* Copyright (C) 2004-2015 MBSim Development Team
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
 * Contact: thorsten.schindler@mytum.de
 */

#include <config.h>

#include "contour1s_flexible.h"
#include "mbsimFlexibleBody/floating_frame.h"
#include "mbsimFlexibleBody/flexible_body.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  Contour1sFlexible::Contour1sFlexible(const string & name) : Contour1s(name), neutral(0) { }

  Frame* Contour1sFlexible::createContourFrame(const string &name) {
    return new FloatingFrame(name,this);
  }

  void Contour1sFlexible::updatePositions(double t, ContourFrame *frame) {
    static_cast<FlexibleBody*>(parent)->updatePositions(t,frame);
  }

  void Contour1sFlexible::updateVelocities(double t, ContourFrame *frame) {
    static_cast<FlexibleBody*>(parent)->updateVelocities(t,frame);
  }

  void Contour1sFlexible::updateAccelerations(double t, ContourFrame *frame) {
    static_cast<FlexibleBody*>(parent)->updateAccelerations(t,frame);
  }

  void Contour1sFlexible::updateJacobians(double t, ContourFrame *frame, int j) {
    static_cast<FlexibleBody*>(parent)->updateJacobians(t,frame,j);
  }

  void Contour1sFlexible::updateGyroscopicAccelerations(double t, ContourFrame *frame) {
    static_cast<FlexibleBody*>(parent)->updateGyroscopicAccelerations(t,frame);
  }

  Vec3 Contour1sFlexible::getWu(double t, const fmatvec::Vec2 &zeta) {
    return static_cast<FlexibleBody*>(parent)->getWu(t,zeta);
  }

}
