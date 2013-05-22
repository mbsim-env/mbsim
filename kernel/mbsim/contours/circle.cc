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

#include <mbsim/utils/contact_utils.h>
#include <mbsim/utils/utils.h>

#include <fmatvec.h>

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/frustum.h>
#endif

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;

namespace MBSim {
  Circle::Circle(const string& name) : RigidContour(name),r(0.),curvature(0),outCont(false) {}
 
  Circle::Circle(const string& name, bool outCont_) : RigidContour(name),r(0.),curvature(0),outCont(outCont_) {}

  Circle::Circle(const string& name, double r_, bool outCont_) : RigidContour(name),r(r_),curvature(outCont_ ? 1./r_ : -1./r_),outCont(outCont_) {}

  Circle::~Circle() {}

  Vec2 Circle::computeLagrangeParameter(const Vec3& WrPoint) {
    Vec2 LagrangeParameter;

    Vec3 CrPoint = WrPoint;

    CrPoint -= R->getPosition();
    CrPoint = R->getOrientation().T() * CrPoint; // position in moving frame of reference

    LagrangeParameter(0) = ArcTan(CrPoint(0), CrPoint(1));

    return LagrangeParameter;
  }

  void Circle::init(InitStage stage) {
    if(stage==MBSim::plot) {
      updatePlotFeatures();

      if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        if(getPlotFeature(openMBV)==enabled && openMBVRigidBody) {
          ((OpenMBV::Frustum*)openMBVRigidBody)->setBaseRadius(r);
          ((OpenMBV::Frustum*)openMBVRigidBody)->setTopRadius(r);
        }
#endif
        RigidContour::init(stage);
      }
    }
    else
      RigidContour::init(stage);
  }

#ifdef HAVE_OPENMBVCPPINTERFACE
  void Circle::enableOpenMBV(bool enable) {
    if(enable) {
      openMBVRigidBody=new OpenMBV::Frustum;
      ((OpenMBV::Frustum*)openMBVRigidBody)->setHeight(0);
    }
    else openMBVRigidBody=0;
  }
#endif

  void Circle::initializeUsingXML(TiXmlElement *element) {
    RigidContour::initializeUsingXML(element);
    TiXmlElement* e;
    e=element->FirstChildElement(MBSIMNS"radius");
    setRadius(getDouble(e));
    e=e->NextSiblingElement();
#ifdef HAVE_OPENMBVCPPINTERFACE
    if(e && e->ValueStr()==MBSIMNS"enableOpenMBV")
      enableOpenMBV();
#endif
  }

  TiXmlElement* Circle::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = Contour::writeXMLFile(parent);
    addElementText(ele0,MBSIMNS"radius",r);
#ifdef HAVE_OPENMBVCPPINTERFACE
    if(openMBVRigidBody)
      ele0->LinkEndChild(new TiXmlElement(MBSIMNS"enableOpenMBV"));
#endif
    return ele0;
  }

}

