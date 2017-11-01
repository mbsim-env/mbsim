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

#ifndef _GENERALIZED_FRICTION_H_
#define _GENERALIZED_FRICTION_H_

#include "mbsim/links/dual_rigid_body_link.h"
#include "mbsim/functions/function.h"

namespace MBSim {

  class RigidBody;
  class FrictionForceLaw;

  class GeneralizedFriction : public DualRigidBodyLink {
    protected:
      FrictionForceLaw *func;
      Function<double(double)> *laN;
    public:
      GeneralizedFriction(const std::string &name="") : DualRigidBodyLink(name), func(NULL), laN(0) { }
      ~GeneralizedFriction();
      void updateGeneralizedForces();

      bool isActive() const { return true; }
      bool gActiveChanged() { return false; }
      virtual bool isSingleValued() const { return true; }
      void init(InitStage stage, const InitConfigSet &config);

      void setGeneralizedFrictionForceLaw(FrictionForceLaw *func_);
      void setGeneralizedNormalForceFunction(Function<double(double)> *laN_) { 
        laN = laN_; 
        laN->setParent(this);
      }

      void initializeUsingXML(xercesc::DOMElement *element);
  };

}

#endif 

