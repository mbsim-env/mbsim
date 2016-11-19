/* Copyright (C) 2004-2016 MBSim Development Team
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
 * Contact: martin.o.foerg@gmail.com
 */

#include <config.h>
#include "mbsimFlexibleBody/utils/openmbv_utils.h"
#include "mbsimFlexibleBody/namespace.h"
#include "mbsim/element.h"

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;
using namespace MBSim;
using namespace xercesc;

#ifdef HAVE_OPENMBVCPPINTERFACE

namespace MBSimFlexibleBody {

  void OpenMBVDynamicIndexedFaceSet::initializeUsingXML(DOMElement *e) {
    OpenMBVBody::initializeUsingXML(e);
    DOMElement *ee;
    ee = E(e)->getFirstElementChildNamed(MBSIMFLEX%"minimalColorValue");
    if(ee) minCol = Element::getDouble(ee);
    ee = E(e)->getFirstElementChildNamed(MBSIMFLEX%"maximalColorValue");
    if(ee) maxCol = Element::getDouble(ee);
  }

  void OpenMBVDynamicIndexedFaceSet::initializeObject(const shared_ptr<OpenMBV::DynamicIndexedFaceSet> &object) {
    OpenMBVBody::initializeObject(object);
    object->setMinimalColorValue(minCol);
    object->setMaximalColorValue(maxCol);
  }

  shared_ptr<OpenMBV::DynamicIndexedFaceSet> OpenMBVDynamicIndexedFaceSet::createOpenMBV(DOMElement *e) {
    shared_ptr<OpenMBV::DynamicIndexedFaceSet> object = OpenMBV::ObjectFactory::create<OpenMBV::DynamicIndexedFaceSet>();
    if(e) initializeUsingXML(e);
    initializeObject(object);
    return object;
  }

}

#endif
