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
#include "mbsim/contours/plane.h"
#include "mbsim/utils/utils.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/grid.h>
#endif

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, Plane, MBSIMNS"Plane")

  Vec2 Plane::computeLagrangeParameter(const fmatvec::Vec3 &WrPoint) {
    return (R->getOrientation().T() *(WrPoint - R->getPosition()) )(Range<Fixed<1>,Fixed<2> >());
  }

 void Plane::initializeUsingXML(TiXmlElement *element) {
    RigidContour::initializeUsingXML(element);
#ifdef HAVE_OPENMBVCPPINTERFACE
    TiXmlElement *e=element->FirstChildElement(MBSIMNS"enableOpenMBV");
    if(e) {
      OpenMBVPlane ombv;
      openMBVRigidBody=ombv.createOpenMBV(e); 
    }
#endif
  }

  TiXmlElement* Plane::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = Contour::writeXMLFile(parent);
#ifdef HAVE_OPENMBVCPPINTERFACE
    if(openMBVRigidBody) {
      TiXmlElement *ele1 = new TiXmlElement(MBSIMNS"enableOpenMBV");
      addElementText(ele1,MBSIMNS"size",static_cast<OpenMBV::Grid*>(openMBVRigidBody)->getXSize());
      addElementText(ele1,MBSIMNS"numberOfLines",static_cast<OpenMBV::Grid*>(openMBVRigidBody)->getXNumber());
      ele0->LinkEndChild(ele1);
    }
#endif
    return ele0;
  }

}

