/* Copyright (C) 2004-2006  Martin Förg
 
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
 * Contact:
 *   martin.o.foerg@googlemail.com
 *
 */

#ifndef _ODEX_INTEGRATOR_H_
#define _ODEX_INTEGRATOR_H_

#include "integrator.h"

namespace MBSimIntegrator {

  /** \brief ODE-Integrator ODEX
  */

  class ODEXIntegrator : public Integrator {

    private:

      static void fzdot(int* n, double* t, double* z, double* zd, double* rpar, int* ipar);
      static void plot(int* nr, double* told, double* t,double* z, int* n, double* con, int *ncon, int* icomp, int* nd, double* rpar, int* ipar, int* irtrn);

      bool signChangedWRTsvLast(const fmatvec::Vec &svStepEnd) const;

      double tPlot{0};
      double dtOut{0};
      std::ofstream integPlot;
      double s0; 
      double time{0};

      /** Absolute Toleranz */
      fmatvec::Vec aTol;
      /** Relative Toleranz */
      fmatvec::Vec rTol;
      /** Step size for the first step */
      double dt0{0};
      /** Maximal number of allowed steps */
      int maxSteps{2000000000};
      /** maximale step size */
      double dtMax{0};

      bool plotOnRoot{false};

       /** tolerance for position constraints */
      double gMax{-1};
      /** tolerance for velocity constraints */
      double gdMax{-1};

      fmatvec::Vec svLast;
      bool shift{false};

    public:

      ~ODEXIntegrator() override = default;

      const fmatvec::Vec& getAbsoluteTolerance() const { return aTol; }
      const fmatvec::Vec& getRelativeTolerance() const { return rTol; }
      double getInitialStepSize() const { return dt0; }
      int getStepLimit() const { return maxSteps; }
      double getMaximumStepSize() const { return dtMax; }

      double getToleranceForPositionConstraints() { return gMax; }
      double getToleranceForVelocityConstraints() { return gdMax; }

      void setAbsoluteTolerance(const fmatvec::Vec &aTol_) { aTol = aTol_; }
      void setAbsoluteTolerance(double aTol_) { aTol = fmatvec::Vec(1,fmatvec::INIT,aTol_); }
      void setRelativeTolerance(const fmatvec::Vec &rTol_) { rTol = rTol_; }
      void setRelativeTolerance(double rTol_) { rTol = fmatvec::Vec(1,fmatvec::INIT,rTol_); }
      void setInitialStepSize(double dt0_) { dt0 = dt0_; }
      void setStepLimit(int maxSteps_) { maxSteps = maxSteps_; }
      void setMaximumStepSize(double dtMax_) { dtMax = dtMax_; }

      void setPlotOnRoot(bool b) { plotOnRoot = b; }

      void setToleranceForPositionConstraints(double gMax_) { gMax = gMax_; }
      void setToleranceForVelocityConstraints(double gdMax_) { gdMax = gdMax_; }

      using Integrator::integrate;
      void integrate() override;

      void initializeUsingXML(xercesc::DOMElement *element) override;
  };

}

#endif
