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

#ifndef _GEAR_H_
#define _GEAR_H_

#include "mbsim/links/rigid_body_link.h"

namespace MBSim {

  class GeneralizedForceLaw;
  class GeneralizedImpactLaw;

  class Gear : public RigidBodyLink {
    protected:
      GeneralizedForceLaw *fl;
      GeneralizedImpactLaw *il;
      std::string saved_gearOutput;
      std::vector<std::string> saved_gearInput;
    public:
      Gear(const std::string &name="") : RigidBodyLink(name), fl(NULL), il(NULL) { body.resize(1); ratio.resize(1); }
      ~Gear();
      void updateGeneralizedForces();
      void setGearOutput(RigidBody* body_) { body[0] = body_; ratio[0] = -1; }
      void addGearInput(RigidBody* body_, double ratio_) { body.push_back(body_); ratio.push_back(ratio_); }

      bool isActive() const { return true; }
      bool gActiveChanged() { return false; }
      std::string getType() const { return "Gear"; }
      void init(InitStage stage);
      bool isSetValued() const;
      bool isSingleValued() const { return not(isSetValued()); }

      void setGeneralizedForceLaw(GeneralizedForceLaw * fl_);

      void initializeUsingXML(xercesc::DOMElement * element);
  };

}

#endif
