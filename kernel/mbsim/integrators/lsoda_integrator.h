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

#ifndef _LSODA_INTEGRATOR_H_
#define _LSODA_INTEGRATOR_H_

#include "implicit_integrator.h"

namespace MBSim {

  extern bool odePackInUse;

  /** \brief Hindmarsh’s ODE solver LSODA
   *
   * Livermore Solver for Ordinary Differential Equations, with
   * Automatic method switching for stiff and nonstiff problems.
   * LSODA solves the initial-value problem for stiff or
   * nonstiff systems of first-order ODE's.
   * This integrator uses ODEPACK (http://www.netlib.org/odepack).
   */
  class LSODAIntegrator : public ImplicitIntegrator {

    private:
      double delta(int i, double z) const override;
      static void fzdot(int* neq, double* t, double* z_, double* zd_);
      static void jac(int *neq, double* t, double* z_, int* ml, int* mu, double* J_, int* nrowp);

      void init() override;

      /** maximal step size */
      double dtMax{0};
      /** minimal step size */
      double dtMin{0};
      /** Absolute Toleranz */
      fmatvec::Vec aTol;
      /** Relative Toleranz */
      fmatvec::Vec rTol;
      /** step size for the first step */
      double dt0{0};
      /**  maximum number of steps allowed during one call to the solver. */
      int maxSteps{std::numeric_limits<int>::max()};

      fmatvec::Vec rWork;
      int lewt;
      double r0;

      std::exception_ptr exception;

    public:

      void setMaximumStepSize(double dtMax_) { dtMax = dtMax_; }
      void setMinimumStepSize(double dtMin_) { dtMin = dtMin_; }
      void setAbsoluteTolerance(const fmatvec::Vec &aTol_) { aTol <<= aTol_; }
      void setAbsoluteTolerance(double aTol_) { aTol.resize(1,fmatvec::INIT,aTol_); }
      void setRelativeTolerance(const fmatvec::Vec &rTol_) { rTol <<= rTol_; }
      void setRelativeTolerance(double rTol_) { rTol.resize(1,fmatvec::INIT,rTol_); }
      void setInitialStepSize(double dt0_) { dt0 = dt0_; }
      void setStepLimit(int maxSteps_) { maxSteps = maxSteps_; }

      using Integrator::integrate;
      void integrate();
      
      virtual void initializeUsingXML(xercesc::DOMElement *element);
  };

}

#endif
