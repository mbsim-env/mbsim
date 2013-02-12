/* Copyright (C) 2004-2011 MBSim Development Team
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
 * Contact: markus.ms.schneider@googlemail.com
 */

#include "config.h"
#include "mbsimFlexibleBody/objectfactory.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_23_bta.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_33_rcm.h"

using namespace std;
using namespace MBSim;

namespace MBSimFlexibleBody {

  ObjectFactory *ObjectFactory::instance=NULL;

  void ObjectFactory::initialize() {
    if(instance==NULL) {
      instance=new ObjectFactory;
      MBSim::ObjectFactory::getInstance()->registerObjectFactory(instance);
    }
  }

  Object * ObjectFactory::createObject(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMFLEXIBLEBODYNS"FlexibleBody1s23BTA")
      return new FlexibleBody1s23BTA(element->Attribute("name"));
    if(element->ValueStr()==MBSIMFLEXIBLEBODYNS"FlexibleBody1s33RCMCantilever")
      return new FlexibleBody1s33RCM(element->Attribute("name"), true);
    if(element->ValueStr()==MBSIMFLEXIBLEBODYNS"FlexibleBody1s33RCMRing")
      return new FlexibleBody1s33RCM(element->Attribute("name"), false);
    return 0;
  }

  MBSim::ObjectFactoryBase::MM_PRINSPRE& ObjectFactory::getPriorityNamespacePrefix() {
    static MBSim::ObjectFactoryBase::MM_PRINSPRE priorityNamespacePrefix;

    if(priorityNamespacePrefix.empty()) {
      priorityNamespacePrefix.insert(P_PRINSPRE( 50, P_NSPRE(MBSIMFLEXIBLEBODYNS_, "")));
      priorityNamespacePrefix.insert(P_PRINSPRE( 40, P_NSPRE(MBSIMFLEXIBLEBODYNS_, "flex")));
      priorityNamespacePrefix.insert(P_PRINSPRE( 30, P_NSPRE(MBSIMFLEXIBLEBODYNS_, "flexbody")));
      priorityNamespacePrefix.insert(P_PRINSPRE( 20, P_NSPRE(MBSIMFLEXIBLEBODYNS_, "mbsimflexbody")));
    }

    return priorityNamespacePrefix;
  }

}

