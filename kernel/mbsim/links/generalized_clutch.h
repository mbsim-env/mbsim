/* Copyright (C) 2004-2021 MBSim Development Team
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

#ifndef _GENERALIZED_CLUTCH_H_
#define _GENERALIZED_CLUTCH_H_

#include "mbsim/links/dual_rigid_body_link.h"
#include "mbsim/functions/function.h"

namespace MBSim {

  class FrictionForceLaw;
  class FrictionImpactLaw;

class GeneralizedClutch : public DualRigidBodyLink {
    protected:
      FrictionForceLaw *laT{nullptr};
      FrictionImpactLaw *LaT{nullptr};
      Function<double(double)> *e{nullptr};
      Function<double(double)> *laN{nullptr};
      unsigned int gActive{1}, gActive0{0};
      unsigned int gdActive{1};
      unsigned int gddActive{1};
      fmatvec::Vec gdn, gdd;
      int gdDir;
      int rootID{0};
    public:
      GeneralizedClutch(const std::string &name="") : DualRigidBodyLink(name), gdn(1), gdd(1) { }
      ~GeneralizedClutch() override;
      void updateGeneralizedForces() override;
      void updateh(int i=0) override;
      void updateW(int i=0) override;
      const double& evalgdn();
      const double& evalgdd();

      bool isActive() const override;
      bool gActiveChanged() override;
      bool isSetValued() const override;
      bool isSingleValued() const override { return true; }
      void plot() override;
      void init(InitStage stage, const InitConfigSet &config) override;

      void setGeneralizedFrictionForceLaw(FrictionForceLaw *laT_);
      void setGeneralizedFrictionImpactLaw(FrictionImpactLaw *LaT_);
      void setEngagementFunction(Function<double(double)> *e_) { 
        e = e_; 
        e->setParent(this);
      }
      void setGeneralizedNormalForceFunction(Function<double(double)> *laN_) { 
        laN = laN_; 
        laN->setParent(this);
      }

      void updateg() override { }
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
      void checkConstraintsForTermination() override;
      void checkImpactsForTermination() override;

      void initializeUsingXML(xercesc::DOMElement *element) override;
  };

}

#endif 
