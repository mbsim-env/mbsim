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
 * Contact: thschindler@users.berlios.de
 */

#include<config.h>
#include "mbsim/contours/circle.h"
#include "mbsim/contours/circle_hollow.h"
#include "mbsim/contours/circle_solid.h"

#include <mbsim/utils/contact_utils.h>
#include <mbsim/utils/utils.h>

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/frustum.h>
#endif

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;
using namespace boost;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(CircleHollow, MBSIM%"CircleHollow")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(CircleSolid, MBSIM%"CircleSolid")

  Circle::Circle(const string& name, Frame *R) : RigidContour(name,R),r(0.),curvature(0),outCont(false) {}
 
  Circle::Circle(const string& name, bool outCont_, Frame *R) : RigidContour(name,R),r(0.),curvature(0),outCont(outCont_) {}

  Circle::Circle(const string& name, double r_, bool outCont_, Frame *R) : RigidContour(name,R),r(r_),curvature(outCont_ ? 1./r_ : -1./r_),outCont(outCont_) {}

  Circle::~Circle() {}

  Vec2 Circle::getLagrangeParameter(const Vec3& WrPoint) {
    Vec2 LagrangeParameter;

    Vec3 CrPoint = WrPoint;

    CrPoint -= R->getPosition();
    CrPoint = R->getOrientation().T() * CrPoint; // position in moving frame of reference

    LagrangeParameter(0) = ArcTan(CrPoint(0), CrPoint(1));

    return LagrangeParameter;
  }

  void Circle::init(InitStage stage) {
    if(stage==plotting) {
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
    e=e->getNextElementSibling();
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

