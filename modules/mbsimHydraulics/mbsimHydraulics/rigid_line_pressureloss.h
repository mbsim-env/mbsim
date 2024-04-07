/* Copyright (C) 2004-2009 MBSim Development Team
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
 * Contact: markus.ms.schneider@gmail.com
 */

#ifndef  _RIGID_LINE_PRESSURELOSS_H_
#define  _RIGID_LINE_PRESSURELOSS_H_

#include "mbsim/links/link.h"

namespace MBSim {
  class GeneralizedForceLaw;
  class GeneralizedImpactLaw;
}

namespace MBSimHydraulics {

  class RigidHLine; 
  class PressureLoss;
  class LinePressureLoss;
  class ClosablePressureLoss;
  class LeakagePressureLoss;
  class UnidirectionalPressureLoss;

  /*! RigidLinePressureLoss */
  class RigidLinePressureLoss : public MBSim::Link {
    public:
      RigidLinePressureLoss(const std::string &name, RigidHLine * line_, PressureLoss * pressureLoss, bool bilateral_=false, bool unilateral_=false);
      ~RigidLinePressureLoss() override;
      void calcSize() override { nla = 1; updSize = false; }
      void plot() override;

      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      // ================================
      // Methods from init-Process
      // ================================
      bool isSetValued() const override {return (unilateral or bilateral); }
      bool isSingleValued() const override { return (not isSetValued()) or (bilateral or (unilateral and (fabs(dpMin)>1e-6))); }
      void calcgdSize(int j) override {gdSize=active?1:0; }
      void calcsvSize() override {svSize=isSetValued()?1:0; }
      void calclaSize(int j) override;
      void calccorrSize(int j) override;
      void calcrFactorSize(int j) override {rFactorSize=active?1:0; }
      //void calcgdSizeActive() {gdSize=1; }
      //void calclaSizeForActiveg() {laSize=0; } // event-driven
      void updatehRef(fmatvec::Vec& hRef, int i=0) override;
      void updaterRef(fmatvec::Vec& rRef, int i=0) override;
      void updatedhdqRef(fmatvec::Mat& dhdqRef, int i=0) {};
      void updatedhduRef(fmatvec::SqrMat& dhduRef, int i=0);
      void updatedhdtRef(fmatvec::Vec& dhdtRef, int i=0);
      void updaterRef(fmatvec::Vec& rRef);
      void updateWRef(fmatvec::Mat& WRef, int i=0) override;
      void updateVRef(fmatvec::Mat& VRef, int i=0) override;
      // ==== END Methods for init-Process ===

      // ================================
      // Methods for update-Process
      // ================================
      void checkActive(int j) override; /* update */
      //void checkActivegdn(); // event-driven
      bool gActiveChanged() override; /* update */
      bool isActive() const override {return active; }
      void updateGeneralizedForces() override;
      void updategd() override; /* zdotStandard */
      void updateStopVector() override; // event-driven
      void updateh(int j) override; /* zdotStandard */
      void updateW(int j) override; /* zdotStandard */
      void updatedhdz();
      // ==== END Methods for update-Process ===

      // ================================
      // Methods for solve-Process
      // ================================
      void checkRoot() override;
      void updaterFactors() override;
      void solveImpactsFixpointSingle() override;
      void solveConstraintsFixpointSingle() override;
      void solveImpactsGaussSeidel() override;
      void solveConstraintsGaussSeidel() override;
      void solveImpactsRootFinding() override;
      void solveConstraintsRootFinding() override;
      void jacobianImpacts() override;
      void jacobianConstraints() override;
      void checkImpactsForTermination() override;
      void checkConstraintsForTermination() override;
      // ==== END Methods for solve-Process ===

      void setDynamicSystemSolver(MBSim::DynamicSystemSolver *ds) override;

    private:
      RigidHLine * line;
      bool active, active0;
      bool unilateral, bilateral;
      double gdn, gdd;
      double dpMin;

      LinePressureLoss * linePressureLoss;
      ClosablePressureLoss * closablePressureLoss;
      LeakagePressureLoss * leakagePressureLoss;
      UnidirectionalPressureLoss * unidirectionalPressureLoss;

      MBSim::GeneralizedForceLaw * gfl;
      MBSim::GeneralizedImpactLaw * gil;
  };

}

#endif   /* ----- #ifndef _HYDLINE_PRESSURELOSS_H_  ----- */

