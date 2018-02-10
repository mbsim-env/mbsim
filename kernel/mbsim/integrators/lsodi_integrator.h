/* Copyright (C) 2004-2018  Martin Förg
 
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

#ifndef _LSODI_INTEGRATOR_H_
#define _LSODI_INTEGRATOR_H_

#include "integrator.h"

namespace MBSimIntegrator {

  extern bool odePackInUse;

  /** \brief Hindmarsh’s ODE solver LSODI
   *
   * Livermore Solver for Ordinary Differential Equations (Implicit form).
   * LSODI solves the initial-value problem for differential-algebraic systems
   * of index 2.
   * This integrator uses ODEPACK (http://www.netlib.org/odepack).
   */
  class LSODIIntegrator : public Integrator {

    private:

      static void res(int* neq, double* t, double* z_, double* zd_, double* res_, int* ires);

      static void adda(int *neq, double* t, double* z_, int* ml, int* mu, double* P, int* nrowp);

      /** maximal step size */
      double dtMax{0};
      /** minimal step size */
      double dtMin{0};
      /** Absolute Toleranz */
      fmatvec::Vec aTol;
      /** Relative Toleranz */
      double rTol{1e-6};
      /** step size for the first step */
      double dt0{0};
      /**  maximum number of steps allowed during one call to the solver. (default 10000) */
      int maxSteps{10000};

       /** tolerance for position constraints */
      double gMax{-1};
      /** tolerance for velocity constraints */
      double gdMax{-1};

    public:

      void setMaximumStepSize(double dtMax_) { dtMax = dtMax_; }
      void setMinimumStepSize(double dtMin_) { dtMin = dtMin_; }
      void setRelativeTolerance(double rTol_) { rTol = rTol_; }
      void setAbsoluteTolerance(const fmatvec::Vec &aTol_) { aTol = aTol_; }
      void setAbsoluteTolerance(double aTol_) { aTol = fmatvec::Vec(1,fmatvec::INIT,aTol_); }
      void setInitialStepSize(double dt0_) { dt0 = dt0_; }
      void setStepLimit(int maxSteps_) { maxSteps = maxSteps_; }

      void setToleranceForPositionConstraints(double gMax_) { gMax = gMax_; }
      void setToleranceForVelocityConstraints(double gdMax_) { gdMax = gdMax_; }

      using Integrator::integrate;
      void integrate();
      
      virtual void initializeUsingXML(xercesc::DOMElement *element);
  };

}

#endif
