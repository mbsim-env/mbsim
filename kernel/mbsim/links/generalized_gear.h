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

#ifndef _GENERALIZED_GEAR_H_
#define _GENERALIZED_GEAR_H_

#include "mbsim/links/rigid_body_link.h"

namespace MBSim {

  class GeneralizedForceLaw;
  class GeneralizedImpactLaw;

  class GeneralizedGear : public RigidBodyLink {
    friend class GeneralizedGearConstraint;
    protected:
      GeneralizedForceLaw *fl{nullptr};
      GeneralizedImpactLaw *il{nullptr};
      std::string saved_gearOutput;
      std::vector<std::string> saved_gearInput;
    public:
      GeneralizedGear(const std::string &name="") : RigidBodyLink(name) { body.resize(1); ratio.resize(1); }
      ~GeneralizedGear() override;
      void updateGeneralizedForces() override;
      void setGearOutput(RigidBody* body_) { body[0] = body_; ratio[0] = -1; }
      void addGearInput(RigidBody* body_, double ratio_) { body.push_back(body_); ratio.push_back(ratio_); }

      bool isActive() const override { return true; }
      bool gActiveChanged() override { return false; }
      void init(InitStage stage, const InitConfigSet &config) override;
      bool isSetValued() const override;
      bool isSingleValued() const override { return not(isSetValued()); }

      void setGeneralizedForceLaw(GeneralizedForceLaw * fl_);

      void initializeUsingXML(xercesc::DOMElement * element) override;
  };

}

#endif
