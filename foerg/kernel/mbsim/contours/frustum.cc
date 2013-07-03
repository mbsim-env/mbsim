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
#include "mbsim/contours/frustum.h"
#include <mbsim/utils/utils.h>


#include <fmatvec.h>

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/frustum.h>
#endif


using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, Frustum, MBSIMNS"Frustum")

  void Frustum::init(InitStage stage) {
    if(stage==MBSim::plot) {
      updatePlotFeatures();
  
      if(getPlotFeature(plotRecursive)==enabled) {
  #ifdef HAVE_OPENMBVCPPINTERFACE
        if(getPlotFeature(openMBV)==enabled && openMBVRigidBody) {
          ((OpenMBV::Frustum*)openMBVRigidBody)->setInitialTranslation(0.,h,0.);
          ((OpenMBV::Frustum*)openMBVRigidBody)->setInitialRotation(3./2.*M_PI,0,0.);
          ((OpenMBV::Frustum*)openMBVRigidBody)->setBaseRadius(r(0));
          ((OpenMBV::Frustum*)openMBVRigidBody)->setTopRadius(r(1));
          ((OpenMBV::Frustum*)openMBVRigidBody)->setHeight(h);
        }
  #endif
        RigidContour::init(stage);
      }
    }
    else
      RigidContour::init(stage);
  }

  //TODO: Same function as in flexible_body_2s_13_mfr_mindlin (transformCW) --> this is just the transformation into cylindrical coordinates --> get it into utils?
  Vec2 Frustum::computeLagrangeParameter(const Vec3 &WrPoint) {
    Vec3 CrPoint = WrPoint;

    CrPoint -= R->getPosition();
    CrPoint = R->getOrientation().T() * CrPoint; // position in moving frame of reference

    const double xt = CrPoint(0);
    const double yt = CrPoint(2);

    Vec2 p;

    p(0) = sqrt(xt * xt + yt * yt);
    p(1) = ArcTan(yt, xt);

    return p;
  }

#ifdef HAVE_OPENMBVCPPINTERFACE
  void Frustum::enableOpenMBV(bool enable) {
    if(enable) {
      openMBVRigidBody=new OpenMBV::Frustum;
      ((OpenMBV::Frustum*)openMBVRigidBody)->setStaticColor(0.5);
      ((OpenMBV::Frustum*)openMBVRigidBody)->setInitialRotation(-M_PI*0.5,0.,0.);
    }
    else openMBVRigidBody=0;
  }
#endif

  void Frustum::initializeUsingXML(TiXmlElement *element) {
    RigidContour::initializeUsingXML(element);
    TiXmlElement* e;
    e=element->FirstChildElement(MBSIMNS"baseRadius");
    r(0)=getDouble(e);
    e=element->FirstChildElement(MBSIMNS"topRadius");
    r(1)=getDouble(e);
    e=element->FirstChildElement(MBSIMNS"height");
    h=getDouble(e);
    if (element->FirstChildElement(MBSIMNS"solid"))
      outCont=true;
    else
      outCont=false;
#ifdef HAVE_OPENMBVCPPINTERFACE
    if(element->FirstChildElement(MBSIMNS"enableOpenMBV"))
      enableOpenMBV();
#endif
  }

  TiXmlElement* Frustum::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = Contour::writeXMLFile(parent);
    addElementText(ele0,MBSIMNS"baseRadius",r(0));
    addElementText(ele0,MBSIMNS"topRadius",r(1));
    addElementText(ele0,MBSIMNS"height",h);
    addElementText(ele0,MBSIMNS"solid",outCont);
    if(openMBVRigidBody)
      ele0->LinkEndChild(new TiXmlElement(MBSIMNS"enableOpenMBV"));
    return ele0;
  }

}
