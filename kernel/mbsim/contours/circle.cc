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

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/frustum.h>
#endif

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;
using namespace boost;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Circle, MBSIM%"Circle")

  Vec3 Circle::evalKs(const fmatvec::Vec2 &zeta) {
    Vec3 Ks(NONINIT);
    double a = zeta(0);
    Ks(0)=-r*sin(a);
    Ks(1)=r*cos(a);
    Ks(2)=0;
    return Ks;
  }

  Vec3 Circle::evalParDer1Kn(const fmatvec::Vec2 &zeta) {
    static Vec3 parDer1Kn;
    double a = zeta(0);
    parDer1Kn(0)=-sign*sin(a);
    parDer1Kn(1)=sign*cos(a);
    return parDer1Kn;
  }

  Vec3 Circle::evalParDer1Ku(const fmatvec::Vec2 &zeta) {
    static Vec3 parDer1Ku;
    double a = zeta(0);
    parDer1Ku(0)=-cos(a);
    parDer1Ku(1)=-sin(a);
    return parDer1Ku;
  }

  Vec2 Circle::evalZeta(const Vec3& WrPoint) {
    Vec2 zeta;

    Vec3 CrPoint = WrPoint;

    CrPoint -= R->evalPosition();
    CrPoint = R->getOrientation().T() * CrPoint; // position in moving frame of reference

    zeta(0) = ArcTan(CrPoint(0), CrPoint(1));

    return zeta;
  }

  void Circle::init(InitStage stage) {
    if(stage==preInit) {
      sign = solid?1:-1;
      Kt(2) = sign;
      RigidContour::init(stage);
    }
    else if(stage==plotting) {
      updatePlotFeatures();

      if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        if(getPlotFeature(openMBV)==enabled && openMBVRigidBody) {
          static_pointer_cast<OpenMBV::Frustum>(openMBVRigidBody)->setBaseRadius(r);
          static_pointer_cast<OpenMBV::Frustum>(openMBVRigidBody)->setTopRadius(r);
          static_pointer_cast<OpenMBV::Frustum>(openMBVRigidBody)->setHeight(0);
        }
#endif
        RigidContour::init(stage);
      }
    }
    else
      RigidContour::init(stage);
  }

  void Circle::initializeUsingXML(DOMElement *element) {
    RigidContour::initializeUsingXML(element);
    DOMElement* e;
    e=E(element)->getFirstElementChildNamed(MBSIM%"radius");
    setRadius(getDouble(e));
    e=E(element)->getFirstElementChildNamed(MBSIM%"solid");
    if(e) setSolid(getBool(e));
#ifdef HAVE_OPENMBVCPPINTERFACE
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(e) {
      OpenMBVCircle ombv;
      openMBVRigidBody=ombv.createOpenMBV(e); 
    }
#endif
  }

  DOMElement* Circle::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Contour::writeXMLFile(parent);
//    addElementText(ele0,MBSIM%"radius",r);
//#ifdef HAVE_OPENMBVCPPINTERFACE
//    if(openMBVRigidBody)
//      ele0->LinkEndChild(new DOMElement(MBSIM%"enableOpenMBV"));
//#endif
    return ele0;
  }

}

