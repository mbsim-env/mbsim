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
 * Contact: schneidm@users.berlios.de
 */

#ifndef  _RIGID_LINE_PRESSURELOSS_H_
#define  _RIGID_LINE_PRESSURELOSS_H_

#include "mbsim/link.h"

namespace MBSim {

  class RigidHLine; 
  class PressureLoss;
  class LinePressureLoss;
  class ClosablePressureLoss;
  class LeakagePressureLoss;

  /*! RigidLinePressureLoss */
  class RigidLinePressureLoss : public Link {
    public:
      RigidLinePressureLoss(const std::string &name, RigidHLine * line_, PressureLoss * pressureLoss, bool bilateral_=false, bool unilateral_=false);
      virtual std::string getType() const { return "RigidLinePressureLoss"; }
      void plot(double t, double dt);

      void init(InitStage stage);
      // ================================
      // Methods from init-Process
      // ================================
      bool isSetValued() const {return (unilateral || bilateral); }
      void calcgdSize() {gdSize=1; }
      // void updatelaRef(); // TODO
      // void updategdRef(); // TODO
      void updatehRef(const fmatvec::Vec& hRef, const fmatvec::Vec& hLinkRef, int i=0);
      void updatedhdqRef(const fmatvec::Mat& dhdqRef, int i=0) {};
      void updatedhduRef(const fmatvec::SqrMat& dhduRef, int i=0);
      void updatedhdtRef(const fmatvec::Vec& dhdtRef, int i=0);
      void updaterRef(const fmatvec::Vec& rRef);
      void updateWRef(const fmatvec::Mat& WRef, int i=0);
      void updateVRef(const fmatvec::Mat& VRef, int i=0);
      void updatewbRef(const fmatvec::Vec& wbRef);
      // void updateresRef(); // TODO
      // void updaterFactorRef(); // TODO
      // ==== END Methods for init-Process ===

      // ================================
      // Methods for update-Process
      // ================================
      void updateg(double t); /* zdotStandard */
      void checkActiveg(); /* update */
      bool gActiveChanged(); /* update */
      bool isActive() const {return ((unilateral || bilateral)?active:false); } 
      void calcgdSizeActive() {gdSize=1; }
      void calclaSize() {laSize=1; }
      void calcrFactorSize() {rFactorSize=1; }
      void updategd(double t); /* zdotStandard */
      void updateh(double t); /* zdotStandard */
      void updateW(double t); /* zdotStandard */
      // ==== END Methods for update-Process ===

      // ================================
      // Methods for solve-Process
      // ================================
      void updaterFactors();
      void solveImpactsFixpointSingle();
      void solveImpactsGaussSeidel();
      void checkImpactsForTermination();
      // ==== END Methods for solve-Process ===

      // ================================
      // not needed methods
      // ================================
      void calclaSizeForActiveg() {laSize=1; }
      // ==== END not needed methods ===

    private:
      RigidHLine * line;
      bool isActive0;
      fmatvec::Vec gdn;
      bool unilateral, bilateral;
      bool active;
      double pLoss;
      LinePressureLoss * linePressureLoss;
      ClosablePressureLoss * closablePressureLoss;
      LeakagePressureLoss * leakagePressureLoss;
  };

}

#endif   /* ----- #ifndef _HYDLINE_PRESSURELOSS_H_  ----- */

