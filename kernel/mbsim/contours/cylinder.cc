/* Copyright (C) 2004-2022 MBSim Development Team
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
#include "mbsim/contours/cylinder.h"
#include "mbsim/frames/frame.h"
#include <mbsim/utils/utils.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, Cylinder)

  void Cylinder::init(InitStage stage, const InitConfigSet &config) {
    if(stage==plotting) {
      if(plotFeature[openMBV] && openMBVRigidBody) {
        static_pointer_cast<OpenMBV::Cylinder>(openMBVRigidBody)->setInitialRotation(M_PI/2,0,0);
        static_pointer_cast<OpenMBV::Cylinder>(openMBVRigidBody)->setRadius(r);
        static_pointer_cast<OpenMBV::Cylinder>(openMBVRigidBody)->setHeight(h);
      }
    }
    RigidContour::init(stage, config);
  }

  Vec3 Cylinder::evalKrPS(const Vec2 &zeta) {
    static Vec3 Kr;
    double a = zeta(0);
    double z = zeta(1);
    Kr(0) = r*cos(a);
    Kr(1) = r*sin(a);
    Kr(2) = z;
    return Kr;
  }

  Vec3 Cylinder::evalKs(const fmatvec::Vec2 &zeta) {
    static Vec3 Ks;
    double a = zeta(0);
    Ks(0) = -r*sin(a);
    Ks(1) = r*cos(a);
    return Ks;
  }

  Vec3 Cylinder::evalKt(const fmatvec::Vec2 &zeta) {
    static Vec3 Kt;
    Kt(2) = 1;
    return Kt;
  }

  Vec3 Cylinder::evalKu(const fmatvec::Vec2 &zeta) {
    static Vec3 Ku;
    double a = zeta(0);
    Ku(0) = -sin(a);
    Ku(1) = cos(a);
    return Ku;
  }

  Vec3 Cylinder::evalKv(const fmatvec::Vec2 &zeta) {
    static Vec3 Kv;
    Kv(2) = 1;
    return Kv;
  }

  Vec3 Cylinder::evalKn(const fmatvec::Vec2 &zeta) {
    static Vec3 Kn;
    double a = zeta(0);
    Kn(0) = cos(a);
    Kn(1) = sin(a);
    return Kn;
  }

  Vec3 Cylinder::evalParDer1Ku(const fmatvec::Vec2 &zeta) {
    static Vec3 parDer1Ku;
    double a = zeta(0);
    parDer1Ku(0) = -cos(a);
    parDer1Ku(1) = -sin(a);
    return parDer1Ku;
  }

  Vec3 Cylinder::evalParDer1Kv(const fmatvec::Vec2 &zeta) {
    static Vec3 parDer1Kv;
    return parDer1Kv;
  }

  Vec3 Cylinder::evalParDer2Kv(const fmatvec::Vec2 &zeta) {
    static Vec3 parDer2Kv;
    return parDer2Kv;
  }

  Vec3 Cylinder::evalParDer1Kn(const Vec2 &zeta) {
    static Vec3 parDer1Kn;
    double a = zeta(0);
    parDer1Kn(0) = -sin(a);
    parDer1Kn(1) = cos(a);
    return parDer1Kn;
  }

  Vec3 Cylinder::evalParDer2Kn(const Vec2 &zeta) {
    static Vec3 parDer2Kn;
    return parDer2Kn;
  }

  void Cylinder::initializeUsingXML(DOMElement *element) {
    RigidContour::initializeUsingXML(element);
    DOMElement* e;
    e=E(element)->getFirstElementChildNamed(MBSIM%"radius");
    setRadius(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"height");
    setHeight(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"solid");
    if(e) setSolid(E(e)->getText<bool>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(e) {
      OpenMBVColoredBody ombv;
      ombv.initializeUsingXML(e);
      openMBVRigidBody=ombv.createOpenMBV<OpenMBV::Cylinder>();
    }
  }

}
