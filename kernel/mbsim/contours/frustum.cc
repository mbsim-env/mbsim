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
#include "mbsim/frames/frame.h"
#include <mbsim/utils/utils.h>

#include <openmbvcppinterface/frustum.h>


using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, Frustum)

  void Frustum::init(InitStage stage, const InitConfigSet &config) {
    if(stage==plotting) {
      if(plotFeature[openMBV] && openMBVRigidBody) {
        static_pointer_cast<OpenMBV::Frustum>(openMBVRigidBody)->setInitialTranslation(0.,h,0.);
        static_pointer_cast<OpenMBV::Frustum>(openMBVRigidBody)->setInitialRotation(3./2.*M_PI,0,0.);
        static_pointer_cast<OpenMBV::Frustum>(openMBVRigidBody)->setBaseRadius(r(0));
        static_pointer_cast<OpenMBV::Frustum>(openMBVRigidBody)->setTopRadius(r(1));
        static_pointer_cast<OpenMBV::Frustum>(openMBVRigidBody)->setHeight(h);
      }
    }
    RigidContour::init(stage, config);
  }

  //TODO: Same function as in flexible_body_2s_13_mfr_mindlin (transformCW) --> this is just the transformation into cylindrical coordinates --> get it into utils?
  Vec2 Frustum::evalZeta(const Vec3 &WrPoint) {
    Vec3 CrPoint = WrPoint;

    CrPoint -= R->evalPosition();
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
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(e) {
      OpenMBVFrustum ombv;
      openMBVRigidBody=ombv.createOpenMBV(e); 
    }
  }

}
