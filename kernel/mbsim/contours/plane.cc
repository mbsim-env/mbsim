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
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;
using namespace boost;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Plane, MBSIM%"Plane")

  Vec2 Plane::getLagrangeParameter(const fmatvec::Vec3 &WrPoint) {
    return (R->getOrientation().T() *(WrPoint - R->getPosition()) )(Range<Fixed<1>,Fixed<2> >());
  }

 void Plane::initializeUsingXML(DOMElement *element) {
    RigidContour::initializeUsingXML(element);
#ifdef HAVE_OPENMBVCPPINTERFACE
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(e) {
      OpenMBVPlane ombv;
      openMBVRigidBody=ombv.createOpenMBV(e); 
    }
#endif
  }

  DOMElement* Plane::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Contour::writeXMLFile(parent);
//#ifdef HAVE_OPENMBVCPPINTERFACE
//    if(openMBVRigidBody) {
//      DOMElement *ele1 = new DOMElement(MBSIM%"enableOpenMBV");
//      addElementText(ele1,MBSIM%"size",static_pointer_cast<OpenMBV::Grid>(openMBVRigidBody)->getXSize());
//      addElementText(ele1,MBSIM%"numberOfLines",static_pointer_cast<OpenMBV::Grid>(openMBVRigidBody)->getXNumber());
//      ele0->LinkEndChild(ele1);
//    }
//#endif
    return ele0;
  }

}

