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
#include "mbsim/contours/sphere.h"
#include "mbsim/frames/frame.h"

#include <openmbvcppinterface/sphere.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, Sphere)

  void Sphere::init(InitStage stage, const InitConfigSet &config) {
    if(stage==plotting) {
      if(plotFeature[openMBV] && openMBVRigidBody)
        static_pointer_cast<OpenMBV::Sphere>(openMBVRigidBody)->setRadius(r);
    }
    RigidContour::init(stage, config);
  }

  Vec3 Sphere::evalKrPS(const Vec2 &zeta) {
    static Vec3 Kr(NONINIT);
    double a = zeta(0);
    double b = zeta(1);
    Kr(0) = r*cos(a)*cos(b);
    Kr(1) = r*sin(a)*cos(b);
    Kr(2) = r*sin(b);
    return Kr;
  }

  Vec3 Sphere::evalKs(const fmatvec::Vec2 &zeta) {
    static Vec3 Ks;
    double a = zeta(0);
    double b = zeta(1);
    Ks(0) = -r*sin(a)*cos(b);
    Ks(1) = r*cos(a)*cos(b);
    return Ks;
  }

  Vec3 Sphere::evalKt(const fmatvec::Vec2 &zeta) {
    static Vec3 Kt(NONINIT);
    double a = zeta(0);
    double b = zeta(1);
    Kt(0) = -r*cos(a)*sin(b);
    Kt(1) = -r*sin(a)*sin(b);
    Kt(2) = r*cos(b);
    return Kt;
  }

  Vec3 Sphere::evalKu(const fmatvec::Vec2 &zeta) {
    static Vec3 Ku;
    double a = zeta(0);
    Ku(0) = -sin(a);
    Ku(1) = cos(a);
    return Ku;
  }

  Vec3 Sphere::evalKv(const fmatvec::Vec2 &zeta) {
    static Vec3 Kv(NONINIT);
    double a = zeta(0);
    double b = zeta(1);
    Kv(0) = -cos(a)*sin(b);
    Kv(1) = -sin(a)*sin(b);
    Kv(2) = cos(b);
    return Kv;
  }

  Vec3 Sphere::evalKn(const fmatvec::Vec2 &zeta) {
    static Vec3 Kn(NONINIT);
    double a = zeta(0);
    double b = zeta(1);
    Kn(0) = cos(a)*cos(b);
    Kn(1) = sin(a)*cos(b);
    Kn(2) = sin(b);
    return Kn;
  }

  Vec3 Sphere::evalParDer1Ku(const fmatvec::Vec2 &zeta) {
    static Vec3 parDer1Ku;
    double a = zeta(0);
    parDer1Ku(0) = -cos(a);
    parDer1Ku(1) = -sin(a);
    return parDer1Ku;
  }

  Vec3 Sphere::evalParDer1Kv(const fmatvec::Vec2 &zeta) {
    static Vec3 parDer1Kv;
    double a = zeta(0);
    double b = zeta(1);
    parDer1Kv(0) = sin(a)*sin(b);
    parDer1Kv(1) = -cos(a)*sin(b);
    return parDer1Kv;
  }

  Vec3 Sphere::evalParDer2Kv(const fmatvec::Vec2 &zeta) {
    static Vec3 parDer2Kv(NONINIT);
    double a = zeta(0);
    double b = zeta(1);
    parDer2Kv(0) = -cos(a)*cos(b);
    parDer2Kv(1) = -sin(a)*cos(b);
    parDer2Kv(2) = -sin(b);
    return parDer2Kv;
  }

  Vec3 Sphere::evalParDer1Kn(const Vec2 &zeta) {
    static Vec3 parDer1Kn;
    double a = zeta(0);
    double b = zeta(1);
    parDer1Kn(0) = -sin(a)*cos(b);
    parDer1Kn(1) = cos(a)*cos(b);
    return parDer1Kn;
  }

  Vec3 Sphere::evalParDer2Kn(const Vec2 &zeta) {
    static Vec3 parDer2Kn(NONINIT);
    double a = zeta(0);
    double b = zeta(1);
    parDer2Kn(0) = -cos(a)*sin(b);
    parDer2Kn(1) = -sin(a)*sin(b);
    parDer2Kn(2) = cos(b);
    return parDer2Kn;
  }

  Vec2 Sphere::evalZeta(const fmatvec::Vec3 &WrPoint) {
    Vec3 SrPoint = R->evalOrientation().T() * (WrPoint - R->evalPosition());
    static Vec2 zeta(NONINIT);
    double r = nrm2(SrPoint);
    zeta(0) = acos(SrPoint(2) / r); // inclination
    zeta(1) = atan2(SrPoint(1), SrPoint(0)); // azimuth
    return zeta;
  }

  void Sphere::initializeUsingXML(DOMElement *element) {
    RigidContour::initializeUsingXML(element);
    DOMElement* e;
    e=E(element)->getFirstElementChildNamed(MBSIM%"radius");
    setRadius(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(e) {
      OpenMBVColoredBody ombv;
      ombv.initializeUsingXML(e);
      openMBVRigidBody=ombv.createOpenMBV<OpenMBV::Sphere>();
    }
  }

}
