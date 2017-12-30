/* Copyright (C) 2004-2017 MBSim Development Team
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

#ifndef _GENERALIZED_ELASTIC_STRUCTURE_H_
#define _GENERALIZED_ELASTIC_STRUCTURE_H_

#include "mbsim/links/rigid_body_link.h"
#include "mbsim/functions/function.h"

namespace MBSim {

  class GeneralizedElasticStructure : public RigidBodyLink {
    protected:
      Function<fmatvec::VecV(fmatvec::VecV,fmatvec::VecV)> *func;
      std::vector<fmatvec::RangeV> ila;
      std::vector<std::string> saved_body;
    public:
      GeneralizedElasticStructure(const std::string &name="") : RigidBodyLink(name) { }
      ~GeneralizedElasticStructure() override;
      void updateGeneralizedForces() override;
      void updateGeneralizedPositions() override;
      void updateGeneralizedVelocities() override;
      void updateForce() override;
      void updateMoment() override;
      void updateR() override;
      void updateh(int i=0) override;
      void addRigidBody(RigidBody* body_) { body.push_back(body_); }

      void calcSize() override;

      bool isActive() const override { return true; }
      bool gActiveChanged() override { return false; }
      bool isSingleValued() const override { return true; }
      void init(InitStage stage, const InitConfigSet &config) override;

      void setGeneralizedForceFunction(Function<fmatvec::VecV(fmatvec::VecV,fmatvec::VecV)> *func_) {
        func=func_;
        func->setParent(this);
        func->setName("GeneralizedForce");
      }

      void initializeUsingXML(xercesc::DOMElement * element) override;
  };

}

#endif
