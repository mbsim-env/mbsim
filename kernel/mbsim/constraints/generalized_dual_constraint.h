/* Copyright (C) 2004-2016 MBSim Development Team
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

#ifndef _GENERALIZED_DUAL_CONSTRAINT_H
#define _GENERALIZED_DUAL_CONSTRAINT_H

#include "generalized_constraint.h"

namespace MBSim {

  class RigidBody;

  /** 
   * \brief Class for dual generalized constraints
   * \author Martin Foerg
   */
  class GeneralizedDualConstraint : public GeneralizedConstraint {
    public:
      GeneralizedDualConstraint(const std::string &name) : GeneralizedConstraint(name), bd(NULL), bi(NULL) { }

      void init(InitStage stage, const InitConfigSet &config);

      void setDependentRigidBody(RigidBody* body_) { bd=body_; }
      void setIndependentRigidBody(RigidBody* body_) { bi=body_; }

      void initializeUsingXML(xercesc::DOMElement * element);

    protected:
      RigidBody *bd, *bi;

      std::string saved_IndependentBody, saved_DependentBody;
  };

}

#endif
