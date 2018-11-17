/* Copyright (C) 2004-2018 MBSim Development Team
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

#include <config.h>
#include "mbsim/contours/disk.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, Disk)

  void Disk::init(InitStage stage, const InitConfigSet &config) {
    if(stage==plotting) {
      if(plotFeature[openMBV] && openMBVRigidBody) {
        static_pointer_cast<OpenMBV::Frustum>(openMBVRigidBody)->setBaseRadius(rO);
        static_pointer_cast<OpenMBV::Frustum>(openMBVRigidBody)->setTopRadius(rO);
        static_pointer_cast<OpenMBV::Frustum>(openMBVRigidBody)->setInnerBaseRadius(rI);
        static_pointer_cast<OpenMBV::Frustum>(openMBVRigidBody)->setInnerTopRadius(rI);
        static_pointer_cast<OpenMBV::Frustum>(openMBVRigidBody)->setHeight(w);
        static_pointer_cast<OpenMBV::Frustum>(openMBVRigidBody)->setInitialTranslation(0,0,w/2);
      }
    }
    RigidContour::init(stage, config);
  }

  void Disk::initializeUsingXML(DOMElement *element) {
    RigidContour::initializeUsingXML(element);
    DOMElement* e;
    e=E(element)->getFirstElementChildNamed(MBSIM%"outerRadius");
    setOuterRadius(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"width");
    setWidth(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"innerRadius");
    if(e) setInnerRadius(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(e) {
      OpenMBVCircle ombv;
      ombv.initializeUsingXML(e);
      openMBVRigidBody=ombv.createOpenMBV(); 
    }
  }

}
