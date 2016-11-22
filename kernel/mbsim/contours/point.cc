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
#include "mbsim/contours/point.h"
#include <fmatvec/fmatvec.h>

#include <openmbvcppinterface/grid.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, Point)

  Vec3 Point::evalParDer1Ku(const fmatvec::Vec2 &zeta) {
    Vec3 parDer1Ku(NONINIT);
    double a = zeta(0);
    parDer1Ku(0) = -cos(a);
    parDer1Ku(1) = -sin(a);
    parDer1Ku(2) = 0;
    return parDer1Ku;
  }

  Vec3 Point::evalParDer2Ku(const fmatvec::Vec2 &zeta) {
    static Vec3 parDer2Ku;
    return parDer2Ku;
  }

  Vec3 Point::evalParDer1Kv(const fmatvec::Vec2 &zeta) {
    Vec3 parDer1Kv(NONINIT);
    double a = zeta(0);
    double b = zeta(1);
    parDer1Kv(0) = sin(a)*sin(b);
    parDer1Kv(1) = -cos(a)*sin(b);
    parDer1Kv(2) = 0;
    return parDer1Kv;
  }

  Vec3 Point::evalParDer2Kv(const fmatvec::Vec2 &zeta) {
    Vec3 parDer2Kv(NONINIT);
    double a = zeta(0);
    double b = zeta(1);
    parDer2Kv(0) = -cos(a)*cos(b);
    parDer2Kv(1) = -sin(a)*cos(b);
    parDer2Kv(2) = -sin(b);
    return parDer2Kv;
  }

  Vec3 Point::evalParDer1Wn(const Vec2 &zeta) {
    Vec3 parDer1Wn(NONINIT);
    double a = zeta(0);
    double b = zeta(1);
    parDer1Wn(0) = -sin(a)*cos(b);
    parDer1Wn(1) = cos(a)*cos(b);
    parDer1Wn(2) = 0;
    return parDer1Wn;
  }

  Vec3 Point::evalParDer2Wn(const Vec2 &zeta) {
    Vec3 parDer2Wn(NONINIT);
    double a = zeta(0);
    double b = zeta(1);
    parDer2Wn(0) = -cos(a)*sin(b);
    parDer2Wn(1) = -sin(a)*sin(b);
    parDer2Wn(2) = cos(b);
    return parDer2Wn;
  }

  void Point::initializeUsingXML(DOMElement *element) {
    RigidContour::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(e) {
      OpenMBVSphere ombv(0.001,"[-1;1;1]",0,"size");
      openMBVRigidBody=ombv.createOpenMBV(e); 
    }
  }

}
