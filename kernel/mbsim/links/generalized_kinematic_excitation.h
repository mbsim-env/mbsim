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

#ifndef _GENERALIZED_KINEMATIC_EXCITATION_H_
#define _GENERALIZED_KINEMATIC_EXCITATION_H_

#include "mbsim/links/dual_rigid_body_link.h"

namespace MBSim {

  class GeneralizedForceLaw;
  class GeneralizedImpactLaw;

  class GeneralizedKinematicExcitation : public DualRigidBodyLink {
    protected:
      GeneralizedForceLaw *fl;
      GeneralizedImpactLaw *il;
    public:
      GeneralizedKinematicExcitation(const std::string &name);
      ~GeneralizedKinematicExcitation() override;

      void updateGeneralizedForces() override;

      bool isActive() const override { return true; }
      bool gActiveChanged() override { return false; }
      void init(InitStage stage, const InitConfigSet &config) override;
      void calclaSize(int j) override;
      void calcgSize(int j) override;
      void calcgdSize(int j) override;
      bool isSetValued() const override;
      bool isSingleValued() const override { return not(isSetValued()); }

      void setGeneralizedForceLaw(GeneralizedForceLaw * fl_);
  };

}

#endif
