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

#ifndef _GENERALIZED_VELOCITY_CONSTRAINT_H
#define _GENERALIZED_VELOCITY_CONSTRAINT_H

#include "mbsim/constraints/generalized_dual_constraint.h"
#include "mbsim/functions/time_dependent_function.h"
#include "mbsim/functions/state_dependent_function.h"

namespace MBSim {

  class GeneralizedVelocityConstraint : public GeneralizedDualConstraint {

    public:
      GeneralizedVelocityConstraint(const std::string &name="") : GeneralizedDualConstraint(name), f(NULL) { }
      ~GeneralizedVelocityConstraint() { delete f; }

      void init(InitStage stage, const InitConfigSet &config);

      void calcxSize();

      // NOTE: we can not use a overloaded setConstraintFunction here due to restrictions in XML but define them for convinience in c++
      void setGeneralConstraintFunction(Function<fmatvec::VecV(fmatvec::VecV,double)>* f_) {
        f = f_;
        f->setParent(this);
        f->setName("Constraint");
      }
      void setTimeDependentConstraintFunction(Function<fmatvec::VecV(double)>* f_) {
        setGeneralConstraintFunction(new TimeDependentFunction<fmatvec::VecV>(f_));
      }
      void setStateDependentConstraintFunction(Function<fmatvec::VecV(fmatvec::VecV)>* f_) {
        setGeneralConstraintFunction(new StateDependentFunction<fmatvec::VecV>(f_));
      }
      void setConstraintFunction(Function<fmatvec::VecV(fmatvec::VecV,double)>* f_) { setGeneralConstraintFunction(f_); }
      void setConstraintFunction(Function<fmatvec::VecV(double)>* f_) { setTimeDependentConstraintFunction(f_); }
      void setConstraintFunction(Function<fmatvec::VecV(fmatvec::VecV)>* f_) { setStateDependentConstraintFunction(f_); }

      virtual void setUpInverseKinetics();

      void updatexd();
      void updateGeneralizedCoordinates();
      void updateGeneralizedJacobians(int j=0);

      void initializeUsingXML(xercesc::DOMElement * element);

    private:
      Function<fmatvec::VecV(fmatvec::VecV,double)> *f;
  };

}

#endif
