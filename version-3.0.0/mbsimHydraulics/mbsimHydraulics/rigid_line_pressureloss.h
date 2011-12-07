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

#include "mbsim/link.h"

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
      ~RigidLinePressureLoss();
      virtual std::string getType() const { return "RigidLinePressureLoss"; }
      void plot(double t, double dt);

      bool hasSmoothPart() const {return (bilateral || (unilateral && (fabs(dpMin)>1e-6))); }

      void init(MBSim::InitStage stage);
      // ================================
      // Methods from init-Process
      // ================================
      bool isSetValued() const {return (unilateral || bilateral); }
      void calcgdSize() {gdSize=1; }
      void calcsvSize() {svSize=isSetValued()?1:0; }
      void updatehRef(const fmatvec::Vec& hRef, int i=0);
      void updaterRef(const fmatvec::Vec& rRef, int i=0);
      void updatedhdqRef(const fmatvec::Mat& dhdqRef, int i=0) {};
      void updatedhduRef(const fmatvec::SqrMat& dhduRef, int i=0);
      void updatedhdtRef(const fmatvec::Vec& dhdtRef, int i=0);
      void updaterRef(const fmatvec::Vec& rRef);
      void updateWRef(const fmatvec::Mat& WRef, int i=0);
      void updateVRef(const fmatvec::Mat& VRef, int i=0);
      // ==== END Methods for init-Process ===

      // ================================
      // Methods for update-Process
      // ================================
      void updateg(double t); /* zdotStandard */
      void checkActiveg(); /* update */
      void checkActivegdn(); // event-driven
      bool gActiveChanged(); /* update */
      bool isActive() const {return active; }
      void calcgdSizeActive() {gdSize=1; }
      void calclaSize() {laSize=1; }
      void calclaSizeForActiveg() {laSize=0; } // event-driven
      void calcrFactorSize() {rFactorSize=1; }
      void updategd(double t); /* zdotStandard */
      void updateStopVector(double t); // event-driven
      void updateh(double t, int i=0); /* zdotStandard */
      void updateW(double t, int i=0); /* zdotStandard */
      // ==== END Methods for update-Process ===

      // ================================
      // Methods for solve-Process
      // ================================
      void updateCondition();
      void updaterFactors();
      void solveImpactsFixpointSingle(double dt);
      void solveConstraintsFixpointSingle();
      void solveImpactsGaussSeidel(double dt);
      void solveConstraintsGaussSeidel();
      void solveImpactsRootFinding(double dt);
      void solveConstraintsRootFinding();
      void jacobianImpacts();
      void jacobianConstraints();
      void checkImpactsForTermination(double dt);
      void checkConstraintsForTermination();
      // ==== END Methods for solve-Process ===


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

