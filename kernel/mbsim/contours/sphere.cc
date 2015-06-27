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
#include "mbsim/contours/sphere.h"
#include "mbsim/utils/utils.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/sphere.h>
#endif

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;
using namespace boost;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Sphere, MBSIM%"Sphere")

  void Sphere::init(InitStage stage) {
    if(stage==plotting) {
      updatePlotFeatures();

      if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        if(getPlotFeature(openMBV)==enabled && openMBVRigidBody) {
          if(openMBVRigidBody) static_pointer_cast<OpenMBV::Sphere>(openMBVRigidBody)->setRadius(r);
        }
#endif
        RigidContour::init(stage);
      }
    }
    else
      RigidContour::init(stage);
  }

  Vec3 Sphere::getKs(ContourPointData &cp) {
    Vec3 Ks(NONINIT);
    double a = cp.getLagrangeParameterPosition()(0);
    double b = cp.getLagrangeParameterPosition()(1);
    Ks(0) = -r*sin(a)*cos(b);
    Ks(1) = r*cos(a)*cos(b);
    Ks(2) = 0;
    return Ks;
  }

  Vec3 Sphere::getKt(ContourPointData &cp) {
    Vec3 Kt(NONINIT);
    double a = cp.getLagrangeParameterPosition()(0);
    double b = cp.getLagrangeParameterPosition()(1);
    Kt(0) = -r*cos(a)*sin(b);
    Kt(1) = -r*sin(a)*sin(b);
    Kt(2) = r*cos(b);
    return Kt;
  }

  Vec3 Sphere::getParDer1Ks(ContourPointData &cp) {
    Vec3 parDer1Ks(NONINIT);
    double a = cp.getLagrangeParameterPosition()(0);
    parDer1Ks(0) = -cos(a);
    parDer1Ks(1) = -sin(a);
    parDer1Ks(2) = 0;
    return parDer1Ks;
  }

  Vec2 Sphere::getLagrangeParameter(const fmatvec::Vec3 &WrPoint) {
    Vec3 SrPoint = R->getOrientation().T() * (WrPoint - R->getPosition());
    Vec2 alpha;
    double r = nrm2(SrPoint);
    alpha(0) = acos(SrPoint(2) / r); // inclination
    alpha(1) = atan2(SrPoint(1), SrPoint(0)); // azimuth
    return alpha;
  }

  void Sphere::initializeUsingXML(DOMElement *element) {
    RigidContour::initializeUsingXML(element);
    DOMElement* e;
    e=E(element)->getFirstElementChildNamed(MBSIM%"radius");
    setRadius(getDouble(e));
    e=e->getNextElementSibling();
#ifdef HAVE_OPENMBVCPPINTERFACE
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(e) {
      OpenMBVSphere ombv;
      openMBVRigidBody=ombv.createOpenMBV(e); 
    }
#endif
  }

  DOMElement* Sphere::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Contour::writeXMLFile(parent);
//    addElementText(ele0,MBSIM%"radius",r);
//#ifdef HAVE_OPENMBVCPPINTERFACE
//    if(openMBVRigidBody)
//      ele0->LinkEndChild(new DOMElement(MBSIM%"enableOpenMBV"));
//#endif
    return ele0;
  }


}

