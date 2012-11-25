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

#ifndef _MBSIM_ENVIRONMENT_H_
#define _MBSIM_ENVIRONMENT_H_

#include "mbxmlutilstinyxml/tinyxml.h"
#include "fmatvec.h"

namespace MBSim {

  /**
   * \brief basic singleton (see GAMMA et al.) class to capsulate environment variables for XML
   * \author Markus Friedrich
   * \date 2009-07-28 some comments (Thorsten Schindler)
   */
  class Environment {
    public:
      /* INTERFACE FOR DERIVED CLASSES */
      /**
       * \brief initializes environment variables by XML element
       * \param XML element
       */
      virtual void initializeUsingXML(TiXmlElement *element)=0;
      virtual TiXmlElement* writeXMLFile(TiXmlNode *parent)=0;
      /***************************************************/
    
    protected:
      /**
       * \brief constructor
       */
      Environment() {};

      /**
       * \brief destructor
       */
      virtual ~Environment() {};
  };

  /**
   * \brief singleton class (see GAMMA et al.) to capsulate environment variables for XML multibody systems
   * \author Markus Friedrich
   * \date 2009-07-28 some comments (Thorsten Schindler)
   */
  class MBSimEnvironment : public Environment {
    public:
      /* INHERITED INTERFACE */
      virtual void initializeUsingXML(TiXmlElement *element);
      virtual TiXmlElement* writeXMLFile(TiXmlNode *parent);
      /***************************************************/

      /* GETTER / SETTER */
      static MBSimEnvironment *getInstance() { return instance?instance:(instance=new MBSimEnvironment); }
      void setAccelerationOfGravity(const fmatvec::Vec3 &grav_) { grav=grav_; }
      const fmatvec::Vec3& getAccelerationOfGravity() const { return grav; }
      /***************************************************/
    
    protected:
      /**
       * class pointer to ensure singleton status
       */
      static MBSimEnvironment *instance;
      
      /**
       * \brief constructor
       */
      MBSimEnvironment() : Environment() {}

      /**
       * \brief acceleration of gravity
       */
      fmatvec::Vec3 grav;
  };

}

#endif /* _MBSIM_ENVIRONMENT_H_ */

