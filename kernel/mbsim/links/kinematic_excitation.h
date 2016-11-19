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

#ifndef _KINEMATIC_EXCITATION_H_
#define _KINEMATIC_EXCITATION_H_

#include "mbsim/links/rigid_body_link.h"

namespace MBSim {

  class GeneralizedForceLaw;
  class GeneralizedImpactLaw;
  class RigidBody;

  class KinematicExcitation : public RigidBodyLink {
    protected:
      GeneralizedForceLaw *fl;
      GeneralizedImpactLaw *il;
      std::string saved_DependentBody;
    public:
      KinematicExcitation(const std::string &name);
      ~KinematicExcitation();

      void updateGeneralizedForces();
      void setDependentRigidBody(RigidBody* body_) { body[0] = body_; }

      bool isActive() const { return true; }
      bool gActiveChanged() { return false; }
      void init(InitStage stage);
      void calclaSize(int j);
      void calcgSize(int j);
      void calcgdSize(int j);
      bool isSetValued() const;
      bool isSingleValued() const { return not(isSetValued()); }

      void setGeneralizedForceLaw(GeneralizedForceLaw * fl_);
  };

}

#endif
