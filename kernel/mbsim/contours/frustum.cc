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
#include "mbsim/frame.h"
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

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Frustum, MBSIM%"Frustum")

  void Frustum::init(InitStage stage) {
    if(stage==plotting) {
      updatePlotFeatures();
  
      if(getPlotFeature(plotRecursive)==enabled) {
  #ifdef HAVE_OPENMBVCPPINTERFACE
        if(getPlotFeature(openMBV)==enabled && openMBVRigidBody) {
          static_pointer_cast<OpenMBV::Frustum>(openMBVRigidBody)->setInitialTranslation(0.,h,0.);
          static_pointer_cast<OpenMBV::Frustum>(openMBVRigidBody)->setInitialRotation(3./2.*M_PI,0,0.);
          static_pointer_cast<OpenMBV::Frustum>(openMBVRigidBody)->setBaseRadius(r(0));
          static_pointer_cast<OpenMBV::Frustum>(openMBVRigidBody)->setTopRadius(r(1));
          static_pointer_cast<OpenMBV::Frustum>(openMBVRigidBody)->setHeight(h);
        }
  #endif
        RigidContour::init(stage);
      }
    }
    else
      RigidContour::init(stage);
  }

  //TODO: Same function as in flexible_body_2s_13_mfr_mindlin (transformCW) --> this is just the transformation into cylindrical coordinates --> get it into utils?
  Vec2 Frustum::getContourParameters(double t, const Vec3 &WrPoint) {
    Vec3 CrPoint = WrPoint;

    CrPoint -= R->getPosition(t);
    CrPoint = R->getOrientation().T() * CrPoint; // position in moving frame of reference

    const double xt = CrPoint(0);
    const double yt = CrPoint(2);

    Vec2 p;

    p(0) = sqrt(xt * xt + yt * yt);
    p(1) = ArcTan(yt, xt);

    return p;
  }

  void Frustum::initializeUsingXML(DOMElement *element) {
    RigidContour::initializeUsingXML(element);
    DOMElement* e;
    e=E(element)->getFirstElementChildNamed(MBSIM%"baseRadius");
    r(0)=getDouble(e);
    e=E(element)->getFirstElementChildNamed(MBSIM%"topRadius");
    r(1)=getDouble(e);
    e=E(element)->getFirstElementChildNamed(MBSIM%"height");
    h=getDouble(e);
    if (E(element)->getFirstElementChildNamed(MBSIM%"solid"))
      outCont=true;
    else
      outCont=false;
#ifdef HAVE_OPENMBVCPPINTERFACE
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(e) {
      OpenMBVFrustum ombv;
      openMBVRigidBody=ombv.createOpenMBV(e); 
    }
#endif
  }

  DOMElement* Frustum::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Contour::writeXMLFile(parent);
 //   addElementText(ele0,MBSIM%"baseRadius",r(0));
 //   addElementText(ele0,MBSIM%"topRadius",r(1));
 //   addElementText(ele0,MBSIM%"height",h);
 //   addElementText(ele0,MBSIM%"solid",outCont);
 //   if(openMBVRigidBody)
 //     ele0->LinkEndChild(new DOMElement(MBSIM%"enableOpenMBV"));
    return ele0;
  }

}
