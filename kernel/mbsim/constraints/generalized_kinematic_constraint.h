/* Copyright (C) 2004-2009 MBSim Development Team
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

#ifndef _GENERALIZED_KINEMATIC_CONSTRAINT_H
#define _GENERALIZED_KINEMATIC_CONSTRAINT_H

#include "mbsim/constraints/generalized_constraint.h"

namespace MBSim {

  class RigidBody;

  class GeneralizedKinematicConstraint : public GeneralizedConstraint {

    public:
      GeneralizedKinematicConstraint(const std::string &name="") : GeneralizedConstraint(name), bd(0), saved_DependentBody("") { }

      void setDependentRigidBody(RigidBody* body) {bd=body; }

      void init(InitStage stage);

      void initializeUsingXML(xercesc::DOMElement * element);

    protected:
      RigidBody *bd;

      std::string saved_DependentBody;
  };

}

#endif
