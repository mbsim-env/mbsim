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

#ifndef _MBSIMFLEXIBLEBODY_OBJECTFACTORY_H_
#define _MBSIMFLEXIBLEBODY_OBJECTFACTORY_H_

#include "mbsim/objectfactory.h"
#include "mbsimtinyxml/tinyxml-src/tinyxml.h"

#define MBSIMFLEXIBLEBODYNS "{http://mbsim.berlios.de/MBSimFlexibleBody}"

namespace MBSim {
  class Object;
}

namespace MBSimFlexibleBody {

  /**
   * \brief object factory for XML modeling
   * \author Markus Schneider
   * \date 2011-10-16 some comments (Thorsten Schindler)
   */
  class ObjectFactory : protected MBSim::ObjectFactoryBase {
    public:
      /**
       * \brief create and register new ObjectFactory
       */
      static void initialize();
    
    protected:
      /**
       * \brief create objects with registered ObjectFactory
       * \param XML object
       * \return analogous MBSim::Object
       */
      MBSim::Object* createObject(TiXmlElement *element);
    
    private:
      /**
       * \brief instance of ObjectFactory
       */
      static ObjectFactory *instance;
      
      ObjectFactory() {} // standard constructor
      ObjectFactory(const ObjectFactory&); // copy constructor
      ObjectFactory& operator=(const ObjectFactory&); // assignment operator
  };

}

#endif /* _MBSIMFLEXIBLEBODY_OBJECTFACTORY_H_ */
