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
 * Contact: schneidm@users.berlios.de
 */

#include<config.h>
#include "mbsim/contours/planewithfrustum.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/frustum.h>
#endif

using namespace std;
using namespace fmatvec;

namespace MBSim {

#ifdef HAVE_OPENMBVCPPINTERFACE
  void PlaneWithFrustum::enableOpenMBV(bool enable) {
    if(enable) {
      openMBVRigidBody=new OpenMBV::Frustum;
      if (hFrustum<0) {
        ((OpenMBV::Frustum*)openMBVRigidBody)->setBaseRadius(rFrustumOnPlane);
        ((OpenMBV::Frustum*)openMBVRigidBody)->setInnerBaseRadius(rFrustumOnPlane);
        ((OpenMBV::Frustum*)openMBVRigidBody)->setTopRadius(rFrustumOnTop);
        ((OpenMBV::Frustum*)openMBVRigidBody)->setInnerTopRadius(rFrustumOnTop);
      }
      else {
        ((OpenMBV::Frustum*)openMBVRigidBody)->setBaseRadius(rFrustumOnPlane);
        ((OpenMBV::Frustum*)openMBVRigidBody)->setTopRadius(rFrustumOnTop);
      }
      ((OpenMBV::Frustum*)openMBVRigidBody)->setHeight(hFrustum);
      ((OpenMBV::Frustum*)openMBVRigidBody)->setInitialRotation(0, M_PI/2., 0);
      ((OpenMBV::Frustum*)openMBVRigidBody)->setInitialTranslation(hFrustum, 0, 0);
    }
    else openMBVRigidBody=0;
  }
#endif

  void PlaneWithFrustum::initializeUsingXML(TiXmlElement *element) {
    RigidContour::initializeUsingXML(element);
    TiXmlElement* e;
    e=element->FirstChildElement(MBSIMNS"baseRadius");
    rFrustumOnPlane=getDouble(e);
    e=element->FirstChildElement(MBSIMNS"topRadius");
    rFrustumOnTop=getDouble(e);
    e=element->FirstChildElement(MBSIMNS"height");
    hFrustum=getDouble(e);
    e=element->FirstChildElement(MBSIMNS"roundingRadius");
    rho=getDouble(e);
    e=e->NextSiblingElement();
#ifdef HAVE_OPENMBVCPPINTERFACE
    if(element->FirstChildElement(MBSIMNS"enableOpenMBV"))
      enableOpenMBV();
#endif
    checkInput();
  }

}

