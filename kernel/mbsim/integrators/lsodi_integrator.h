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

#include "dae_integrator.h"

namespace MBSim {

  /** \brief Hindmarsh’s ODE solver LSODI
   *
   * Livermore Solver for Ordinary Differential Equations (Implicit form).
   * LSODI solves the initial-value problem for differential-algebraic systems
   * of index 2.
   * This integrator uses ODEPACK (http://www.netlib.org/odepack).
   */
  class LSODIIntegrator : public DAEIntegrator {

    private:
      double delta(int i, double z) const override;
      typedef void (*Res)(int* neq, double* t, double* y_, double* yd_, double* res_, int* ires);
      typedef void (*Jac)(int *neq, double* t, double* y_, double *yd_, int* ml, int* mu, double* J_, int* nrowp);
      static Res res[5];
      static Jac jac[5];
      static void resODE(int* neq, double* t, double* z_, double* zd_, double* res_, int* ires);
      static void resDAE2(int* neq, double* t, double* y_, double* yd_, double* res_, int* ires);
      static void resGGL(int* neq, double* t, double* y_, double* yd_, double* res_, int* ires);
      static void adda(int *neq, double* t, double* y_, int* ml, int* mu, double* P, int* nrowp);
      static void jacODE(int *neq, double* t, double* y_, double *yd_, int* ml, int* mu, double* J_, int* nrowp);
      static void jacDAE2(int *neq, double* t, double* y_, double *yd_, int* ml, int* mu, double* J_, int* nrowp);
      static void jacGGL(int *neq, double* t, double* y_, double *yd_, int* ml, int* mu, double* J_, int* nrowp);

      void init() override;
      void reinit() override;

      /** maximal step size */
      double dtMax{0};
      /** minimal step size */
      double dtMin{0};
      /** step size for the first step */
      double dt0{0};
      /**  maximum number of steps allowed during one call to the solver. */
      int maxSteps{std::numeric_limits<int>::max()};
      /** exclude algebraic variables from error test **/
      bool excludeAlgebraicVariables{true};

      fmatvec::VecInt neq_;
      fmatvec::Vec rWork;
      int lewt;

      std::exception_ptr exception;

    public:
      void setMaximumStepSize(double dtMax_) { dtMax = dtMax_; }
      void setMinimumStepSize(double dtMin_) { dtMin = dtMin_; }
      void setInitialStepSize(double dt0_) { dt0 = dt0_; }
      void setStepLimit(int maxSteps_) { maxSteps = maxSteps_; }
      void setExcludeAlgebraicVariablesFromErrorTest(bool excludeAlgebraicVariables_) { excludeAlgebraicVariables = excludeAlgebraicVariables_; }

      using Integrator::integrate;
      void integrate();
      
      virtual void initializeUsingXML(xercesc::DOMElement *element);
  };

}

#endif
