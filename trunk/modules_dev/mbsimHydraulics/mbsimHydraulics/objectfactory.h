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

#ifndef _MBSIMHYDRAULICS_OBJECTFACTORY_H_
#define _MBSIMHYDRAULICS_OBJECTFACTORY_H_

#include "mbsim/objectfactory.h"
#include "mbsimtinyxml/tinyxml-src/tinyxml.h"
#include "mbsim/utils/function.h"

#define MBSIMHYDRAULICSNS "{http://mbsim.berlios.de/MBSimHydraulics}"

namespace MBSim {

  class HydraulicsObjectFactory : protected ObjectFactory {
    private:
      static HydraulicsObjectFactory *instance;
      HydraulicsObjectFactory() {}
    public:
      // This static function must be called before the ObjectFactory is usend to create
      // objects from MBSimObjectFactory
      static void initialize();
    protected:
      Group* createGroup(TiXmlElement *element);
      Object* createObject(TiXmlElement *element);
      Link* createLink(TiXmlElement *element);
      Environment *getEnvironment(TiXmlElement *element);
      Function1<double, double> * createFunction1_SS(TiXmlElement *element);
  };

}

#endif
