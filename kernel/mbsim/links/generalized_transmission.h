/* Copyright (C) 2004-2026 MBSim Development Team
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

#ifndef _GENERALIZED_TRANSMISSION_H_
#define _GENERALIZED_TRANSMISSION_H_

#include "mbsim/links/dual_rigid_body_link.h"
#include "mbsim/functions/function.h"

namespace MBSim {

  class FrictionForceLaw;
  class FrictionImpactLaw;

class GeneralizedTransmission : public DualRigidBodyLink {
    protected:
      Function<double(double)> *i{nullptr};
      double sForce{1};
      bool iSync{false};
      unsigned int active{1};
      fmatvec::Vec gdn, gdd;
      double i0{0};
      int gdDir{1};
      int rootID{0};
    public:
      GeneralizedTransmission(const std::string &name="") : DualRigidBodyLink(name), gdn(1), gdd(1) { }
      ~GeneralizedTransmission() override;
      void updateGeneralizedPositions() override;
      void updateGeneralizedVelocities() override;
      void updateForce() override;
      void updateMoment() override;
      void updateR() override;
      void updateGeneralizedForces() override;
      void updateh(int i=0) override;
      void updateW(int i=0) override;
      void updatewb() override;
      void updateg() override { }
      void updategd() override;
      const double& evalgdn();
      const double& evalgdd();

      bool isActive() const override { return true; }
      bool gActiveChanged() override { return false; }
      bool isSetValued() const override { return true; }
      bool isSingleValued() const override { return true; }
      void plot() override;
      void init(InitStage stage, const InitConfigSet &config) override;

      void setTransmissionFunction(Function<double(double)> *i_) { 
        i = i_; 
        i->setParent(this);
      }
      void setGeneralizedSynchronizationForce(double sForce_) { sForce = sForce_; }
      void setImpulsiveSynchronization(bool iSync_) { iSync = iSync_; }

      void updateStopVector() override;
      void calclaSize(int j) override;
      void calcgSize(int j) override;
      void calcgdSize(int j) override;
      void calcrFactorSize(int j) override;
      void calcsvSize() override;
      void checkActive(int j) override;
      void calccorrSize(int j) override;
      void updatecorr(int j) override;
      void checkRoot() override;
      void updaterFactors() override;
      void solveConstraintsFixpointSingle() override;
      void solveImpactsFixpointSingle() override;
      void solveImpactsGaussSeidel() override;
      void checkConstraintsForTermination() override;
      void checkImpactsForTermination() override;

      void initializeUsingXML(xercesc::DOMElement *element) override;
  };

}

#endif 
