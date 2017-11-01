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

#ifndef _GENERALIZED_GEAR_CONSTRAINT_H
#define _GENERALIZED_GEAR_CONSTRAINT_H

#include "mbsim/constraints/generalized_constraint.h"

namespace MBSim {

  class RigidBody;

  class GeneralizedGearConstraint : public GeneralizedConstraint {

    public:
      GeneralizedGearConstraint(const std::string &name="") : GeneralizedConstraint(name), bd(NULL) { }

      void init(InitStage stage, const InitConfigSet &config);

      void setDependentRigidBody(RigidBody* body_) { bd=body_; }
      void addIndependentRigidBody(RigidBody *body, double ratio);

      void updateGeneralizedCoordinates();
      void updateGeneralizedJacobians(int j=0);
      void setUpInverseKinetics();

      void initializeUsingXML(xercesc::DOMElement * element);

    private:
      std::vector<RigidBody*> bi;
      RigidBody *bd;
      std::vector<double> ratio;

      std::string saved_DependentBody;
      std::vector<std::string> saved_IndependentBody;
  };

}

#endif
