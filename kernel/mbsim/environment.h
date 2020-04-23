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

#include "fmatvec/fmatvec.h"
#include "fmatvec/atom.h"
#include <xercesc/dom/DOMElement.hpp>
#include <xercesc/dom/DOMNode.hpp>

namespace OpenMBV {
  class Object;
}

namespace MBSim {

  /**
   * Base class for all environment objects
   */
  class Environment : virtual public fmatvec::Atom {
    public:
      /* INTERFACE FOR DERIVED CLASSES */
      /**
       * \brief initializes environment variables by XML element
       * \param XML element
       */
      virtual void initializeUsingXML(xercesc::DOMElement *element) {}
      /***************************************************/

      ~Environment() override = default;

    protected:
      /**
       * \brief constructor
       */
      Environment()  {};
  };

  /**
   * Environment object for mechanical systems.
   */
  class MBSimEnvironment : public Environment {
    public:
      /* INHERITED INTERFACE */
      void initializeUsingXML(xercesc::DOMElement *element) override;
      /***************************************************/

      /* GETTER / SETTER */
      void setAccelerationOfGravity(const fmatvec::Vec3 &grav_) { grav=grav_; }
      const fmatvec::Vec3& getAccelerationOfGravity() const { return grav; }

      void addOpenMBVObject(const std::shared_ptr<OpenMBV::Object> &object);
      std::vector<std::shared_ptr<OpenMBV::Object>> getOpenMBVObjects();
      /***************************************************/

      /**
       * \brief constructor
       */
      MBSimEnvironment()  {}
    
    protected:

      /**
       * \brief acceleration of gravity
       */
      fmatvec::Vec3 grav;

      std::vector<std::shared_ptr<OpenMBV::Object>> openMBVObject;
  };

}

#endif /* _MBSIM_ENVIRONMENT_H_ */

