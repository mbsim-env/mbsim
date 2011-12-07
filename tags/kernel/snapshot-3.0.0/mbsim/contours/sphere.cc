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
 * Contact: mfoerg@users.berlios.de
 */

#include<config.h>
#include "mbsim/contours/sphere.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/sphere.h>
#endif

using namespace std;

namespace MBSim {

  #ifdef HAVE_OPENMBVCPPINTERFACE
  void Sphere::enableOpenMBV(bool enable) {
    if(enable) {
      openMBVRigidBody=new OpenMBV::Sphere;
      ((OpenMBV::Sphere*)openMBVRigidBody)->setRadius(r);
    }
    else openMBVRigidBody=0;
  }
#endif

  void Sphere::initializeUsingXML(TiXmlElement *element) {
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

}

