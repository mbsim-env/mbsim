/* Copyright (C) 2004-2015 MBSim Development Team
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
 * Contact: martin.o.foerg@gmail.com
 */

#ifndef _GENERALIZED_VELOCITY_EXCITATION_H_
#define _GENERALIZED_VELOCITY_EXCITATION_H_

#include "mbsim/links/generalized_kinematic_excitation.h"
#include "mbsim/functions/time_dependent_function.h"
#include "mbsim/functions/state_dependent_function.h"

namespace MBSim {

  class GeneralizedVelocityExcitation : public GeneralizedKinematicExcitation {
    friend class GeneralizedVelocityConstraint;
    protected:
      Function<fmatvec::VecV(fmatvec::VecV,double)> *f;
    public:
      GeneralizedVelocityExcitation(const std::string &name) : GeneralizedKinematicExcitation(name) { }

      void calcxSize() override;
      void init(InitStage stage, const InitConfigSet &config) override;

      void updateGeneralizedPositions() override;
      void updateGeneralizedVelocities() override;
      void updatexd() override;
      void updatewb() override;

      void setExcitationFunction(Function<fmatvec::VecV(fmatvec::VecV,double)>* f_) { 
        f = f_;
        f->setParent(this);
        f->setName("Excitation");
      }
      void setExcitationFunction(Function<fmatvec::VecV(fmatvec::VecV)>* f_) {
        setExcitationFunction(new StateDependentFunction<fmatvec::VecV>(f_));
      }
      void setExcitationFunction(Function<fmatvec::VecV(double)>* f_) { 
        setExcitationFunction(new TimeDependentFunction<fmatvec::VecV>(f_));
      }
  };

}

#endif
