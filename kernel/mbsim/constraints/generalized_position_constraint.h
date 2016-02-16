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

#ifndef _GENERALIZED_POSITION_CONSTRAINT_H
#define _GENERALIZED_POSITION_CONSTRAINT_H

#include "mbsim/constraints/kinematic_constraint.h"
#include "mbsim/functions/function.h"

namespace MBSim {

  class GeneralizedPositionConstraint : public KinematicConstraint {

    public:
      GeneralizedPositionConstraint(const std::string &name="") : KinematicConstraint(name), f(NULL) {}
      ~GeneralizedPositionConstraint() { delete f; }

      void init(Element::InitStage stage);

      void setConstraintFunction(Function<fmatvec::VecV(double)>* f_) {
        f = f_;
        f->setParent(this);
        f->setName("Constraint");
      }

      void setUpInverseKinetics();

      void updateGeneralizedCoordinates(double t);
      void updateGeneralizedJacobians(double t, int j=0);

      void initializeUsingXML(xercesc::DOMElement * element);

      virtual std::string getType() const { return "GeneralizedPositionConstraint"; }

    private:
      Function<fmatvec::VecV(double)> *f;
  };

}

#endif
