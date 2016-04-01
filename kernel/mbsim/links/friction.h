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

#ifndef _FRICTION_H_
#define _FRICTION_H_

#include "mbsim/links/rigid_body_link.h"
#include "mbsim/functions/function.h"

namespace MBSim {

  class RigidBody;
  class FrictionForceLaw;

  class GeneralizedFriction : public RigidBodyLink {
    protected:
      FrictionForceLaw *func;
      Function<double(double)> *laN;
      RigidBody *body[2];
    public:
      GeneralizedFriction(const std::string &name="");
      ~GeneralizedFriction();
      void updateGeneralizedForces();

      bool isActive() const { return true; }
      bool gActiveChanged() { return false; }
      virtual bool isSingleValued() const { return true; }
      std::string getType() const { return "GeneralizedFriction"; }
      void init(InitStage stage);

      void setGeneralizedFrictionForceLaw(FrictionForceLaw *func_);
      void setGeneralizedNormalForceFunction(Function<double(double)> *laN_) { 
        laN = laN_; 
        laN->setParent(this);
      }

      void setRigidBodyFirstSide(RigidBody* body_) { body[0] = body_; }
      void setRigidBodySecondSide(RigidBody* body_) { body[1] = body_; }

      void initializeUsingXML(xercesc::DOMElement *element);

    private:
      std::string saved_body1, saved_body2;
  };

}

#endif 

