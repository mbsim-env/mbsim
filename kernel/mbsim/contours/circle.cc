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
#include "mbsim/frames/frame.h"
#include "mbsim/contours/circle.h"
#include "mbsim/utils/utils.h"

#include <openmbvcppinterface/frustum.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, Circle)

  Vec3 Circle::evalKs(const fmatvec::Vec2 &zeta) {
    static Vec3 Ks;
    double a = zeta(0);
    Ks(0)=-r*sin(a);
    Ks(1)=r*cos(a);
    return Ks;
  }

  Vec3 Circle::evalKt(const fmatvec::Vec2 &zeta) {
    static Vec3 Kt;
    return Kt;
  }

  Vec3 Circle::evalKu(const fmatvec::Vec2 &zeta) {
    static Vec3 Ku;
    double a = zeta(0);
    Ku(0)=-sin(a);
    Ku(1)=cos(a);
    return Ku;
  }

  Vec3 Circle::evalKv(const fmatvec::Vec2 &zeta) {
    static Vec3 Kv;
    double a = zeta(0);
    double b = zeta(1);
    Kv(0)=-cos(a)*sin(b);
    Kv(1)=-sin(a)*sin(b);
    Kv(2)=cos(b);
    return Kv;
  }

  Vec3 Circle::evalKn(const fmatvec::Vec2 &zeta) {
    static Vec3 Kn;
    double a = zeta(0);
    double b = zeta(1);
    Kn(0)=cos(a)*cos(b);
    Kn(1)=sin(a)*cos(b);
    Kn(2)=sin(b);
    return Kn;
  }

  Vec3 Circle::evalWu(const Vec2 &zeta) {
    return R->evalOrientation()*evalKu(zeta);
  }

  Vec3 Circle::evalWv(const Vec2 &zeta) {
    return R->evalOrientation()*evalKv(zeta);
  }

  Vec3 Circle::evalWn(const Vec2 &zeta) {
    return R->evalOrientation()*evalKn(zeta);
  }

  Vec3 Circle::evalParDer1Ku(const fmatvec::Vec2 &zeta) {
    static Vec3 parDer1Ku;
    double a = zeta(0);
    parDer1Ku(0)=-cos(a);
    parDer1Ku(1)=-sin(a);
    return parDer1Ku;
  }

  Vec3 Circle::evalParDer2Ku(const fmatvec::Vec2 &zeta) {
    static Vec3 parDer2Ku;
    return parDer2Ku;
  }

  Vec3 Circle::evalParDer1Kv(const fmatvec::Vec2 &zeta) {
    Vec3 parDer1Kv(NONINIT);
    double a = zeta(0);
    double b = zeta(1);
    parDer1Kv(0) = sin(a)*sin(b);
    parDer1Kv(1) = -cos(a)*sin(b);
    parDer1Kv(2) = 0;
    return parDer1Kv;
  }

  Vec3 Circle::evalParDer2Kv(const fmatvec::Vec2 &zeta) {
    Vec3 parDer2Kv(NONINIT);
    double a = zeta(0);
    double b = zeta(1);
    parDer2Kv(0) = -cos(a)*cos(b);
    parDer2Kv(1) = -sin(a)*cos(b);
    parDer2Kv(2) = -sin(b);
    return parDer2Kv;
  }

  Vec3 Circle::evalParDer1Kn(const fmatvec::Vec2 &zeta) {
    static Vec3 parDer1Kn;
    double a = zeta(0);
    double b = zeta(1);
    parDer1Kn(0)=-sign*sin(a)*cos(b);
    parDer1Kn(1)=sign*cos(a)*cos(b);
    return parDer1Kn;
  }

  Vec3 Circle::evalParDer2Kn(const fmatvec::Vec2 &zeta) {
    static Vec3 parDer2Kn;
    double a = zeta(0);
    double b = zeta(1);
    parDer2Kn(0)=-sign*cos(a)*sin(b);
    parDer2Kn(1)=-sign*sin(a)*sin(b);
    parDer2Kn(2)=sign*cos(b);
    return parDer2Kn;
  }

  Vec2 Circle::evalZeta(const Vec3& WrPoint) {
    Vec2 zeta;

    Vec3 CrPoint = WrPoint;

    CrPoint -= R->evalPosition();
    CrPoint = R->getOrientation().T() * CrPoint; // position in moving frame of reference

    zeta(0) = ArcTan(CrPoint(0), CrPoint(1));

    return zeta;
  }

  void Circle::init(InitStage stage, const InitConfigSet &config) {
    if(stage==preInit) {
      sign = solid?1:-1;
      Kt(2) = sign;
    }
    else if(stage==plotting) {
      if(plotFeature[openMBV] && openMBVRigidBody) {
        static_pointer_cast<OpenMBV::Frustum>(openMBVRigidBody)->setBaseRadius(r);
        static_pointer_cast<OpenMBV::Frustum>(openMBVRigidBody)->setTopRadius(r);
        static_pointer_cast<OpenMBV::Frustum>(openMBVRigidBody)->setHeight(0);
      }
    }
    RigidContour::init(stage, config);
  }

  void Circle::initializeUsingXML(DOMElement *element) {
    RigidContour::initializeUsingXML(element);
    DOMElement* e;
    e=E(element)->getFirstElementChildNamed(MBSIM%"radius");
    setRadius(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"solid");
    if(e) setSolid(E(e)->getText<bool>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(e) {
      OpenMBVColoredBody ombv;
      ombv.initializeUsingXML(e);
      openMBVRigidBody=ombv.createOpenMBV<OpenMBV::Frustum>();
    }
  }

}
