/* Copyright (C) 2004-2016 MBSim Development Team
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

#ifndef _GENERALIZED_ELASTIC_CONNECTION_H_
#define _GENERALIZED_ELASTIC_CONNECTION_H_

#include "mbsim/links/dual_rigid_body_link.h"
#include "mbsim/functions/function.h"

#include "mbsim/utils/boost_parameters.h"
#include "mbsim/utils/openmbv_utils.h"

namespace MBSim {

  class GeneralizedElasticConnection : public DualRigidBodyLink {
    protected:
      Function<fmatvec::VecV(fmatvec::VecV,fmatvec::VecV)> *func;
    public:
      GeneralizedElasticConnection(const std::string &name="") : DualRigidBodyLink(name), func(nullptr) { }
      ~GeneralizedElasticConnection() override;

      void updateGeneralizedForces() override;

      bool isActive() const override { return true; }
      bool gActiveChanged() override { return false; }
      bool isSingleValued() const override { return true; }
      void init(InitStage stage, const InitConfigSet &config) override;

      void setGeneralizedForceFunction(Function<fmatvec::VecV(fmatvec::VecV,fmatvec::VecV)> *func_) {
        func=func_;
        func->setParent(this);
        func->setName("GeneralizedForce");
      }

      void initializeUsingXML(xercesc::DOMElement *element) override;

    private:
      std::string saved_body1, saved_body2;
  };

}

#endif
