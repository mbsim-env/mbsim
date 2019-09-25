/* Copyright (C) 2004-2019 MBSim Development Team
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
#include "mbsim/contours/rack.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, Rack)

  Vec3 Rack::evalKrPS(const Vec2 &zeta) {
    static Vec3 KrPS(NONINIT);
    static Vec3 d("[1;0;0]");
    double eta = zeta(0);
    double xi = zeta(1);
    KrPS(0) = -eta*sin(al)*cos(be)+xi*sin(be);
    KrPS(1) = signi*eta*cos(al);
    KrPS(2) = eta*sin(al)*sin(be)+xi*cos(be);
    return d*(k*M_PI*m/cos(be)+signi*s0h)+KrPS;
  }

  Vec3 Rack::evalKs(const Vec2 &zeta) {
    static Vec3 Ks(NONINIT);
    Ks(0) = -sin(al)*cos(be);
    Ks(1) = signi*cos(al);
    Ks(2) = sin(al)*sin(be);
    return Ks;
  }

  Vec3 Rack::evalKt(const Vec2 &zeta) {
    static Vec3 Kt;
    Kt(0) = sin(be);
    Kt(2) = cos(be);
    return Kt;
  }

  void Rack::init(InitStage stage, const InitConfigSet &config) {
    if(stage==preInit) {
      s0h = (m/cos(be)*M_PI/2-b)/2;
    }
    else if(stage==plotting) {
      if(plotFeature[openMBV] && openMBVRigidBody) {
        static_pointer_cast<OpenMBV::Rack>(openMBVRigidBody)->setNumberOfTeeth(N);
        static_pointer_cast<OpenMBV::Rack>(openMBVRigidBody)->setHeight(h);
        static_pointer_cast<OpenMBV::Rack>(openMBVRigidBody)->setWidth(w);
        static_pointer_cast<OpenMBV::Rack>(openMBVRigidBody)->setHelixAngle(be);
        static_pointer_cast<OpenMBV::Rack>(openMBVRigidBody)->setModule(m);
        static_pointer_cast<OpenMBV::Rack>(openMBVRigidBody)->setPressureAngle(al);
        static_pointer_cast<OpenMBV::Rack>(openMBVRigidBody)->setBacklash(b);
      }
    }
    RigidContour::init(stage, config);
  }

  void Rack::initializeUsingXML(DOMElement *element) {
    RigidContour::initializeUsingXML(element);
    DOMElement* e;
    e=E(element)->getFirstElementChildNamed(MBSIM%"numberOfTeeth");
    setNumberOfTeeth(E(e)->getText<int>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"height");
    setHeight(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"width");
    setWidth(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"helixAngle");
    if(e) setHelixAngle(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"module");
    if(e) setModule(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"pressureAngle");
    if(e) setPressureAngle(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"backlash");
    if(e) setBacklash(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(e) {
      OpenMBVColoredBody ombv;
      ombv.initializeUsingXML(e);
      openMBVRigidBody=ombv.createOpenMBV<OpenMBV::Rack>();
    }
  }

}
