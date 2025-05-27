/* Copyright (C) 2004-2009 MBSim Development Team
 * 
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 * 
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details. 
 *
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: friedrich.at.gc@googlemail.com
 */

#include <config.h>
#include "mbsim/environment.h"
#include "mbsim/utils/utils.h"
#include "mbsim/objectfactory.h"
#include "mbsim/namespace.h"
#include <openmbvcppinterface/objectfactory.h>
#include <openmbvcppinterface/object.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, MBSimEnvironment)

  void MBSimEnvironment::initializeUsingXML(DOMElement *element) {
    Environment::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIM%"accelerationOfGravity");
    setAccelerationOfGravity(E(e)->getText<Vec3>());

    e=E(element)->getFirstElementChildNamed(MBSIM%"openMBVObject");
    if(e)
      for(DOMElement *ee=e->getFirstElementChild(); ee!=nullptr; ee=ee->getNextElementSibling()) {
        auto object=OpenMBV::ObjectFactory::create<OpenMBV::Object>(ee);
        addOpenMBVObject(object);
        object->initializeUsingXML(ee);
      }
  }

  void MBSimEnvironment::addOpenMBVObject(const std::shared_ptr<OpenMBV::Object> &object) {
    openMBVObject.emplace_back(object);
  }

  std::vector<std::shared_ptr<OpenMBV::Object>> MBSimEnvironment::getOpenMBVObjects() {
    return openMBVObject;
  }

}
