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

#include "root_finding_integrator.h"

namespace MBSimIntegrator {

  extern bool odePackInUse;

  /** \brief Hindmarsh’s ODE solver LSODI
   *
   * Livermore Solver for Ordinary Differential Equations (Implicit form).
   * LSODI solves the initial-value problem for differential-algebraic systems
   * of index 2.
   * This integrator uses ODEPACK (http://www.netlib.org/odepack).
   */
  class LSODIIntegrator : public RootFindingIntegrator {

    public:
      enum Formalism {
        ODE=0,
        DAE2,
        GGL,
        unknown
      };

    private:
      typedef void (*Res)(int* neq, double* t, double* z_, double* zd_, double* res_, int* ires);
      static Res res[3];
      static void resODE(int* neq, double* t, double* z_, double* zd_, double* res_, int* ires);
      static void resDAE2(int* neq, double* t, double* y_, double* yd_, double* res_, int* ires);
      static void resGGL(int* neq, double* t, double* y_, double* yd_, double* res_, int* ires);
      static void adda(int *neq, double* t, double* y_, int* ml, int* mu, double* P, int* nrowp);

      void calcSize();

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
      /**  maximum number of steps allowed during one call to the solver. (default 10000) */
      int maxSteps{10000};
      /** formalism **/
      Formalism formalism{DAE2};
      /** exclude algebraic variables from error test **/
      bool excludeAlgebraicVariables{true};

      int N;

    public:
      void setMaximumStepSize(double dtMax_) { dtMax = dtMax_; }
      void setMinimumStepSize(double dtMin_) { dtMin = dtMin_; }
      void setAbsoluteTolerance(const fmatvec::Vec &aTol_) { aTol = aTol_; }
      void setAbsoluteTolerance(double aTol_) { aTol = fmatvec::Vec(1,fmatvec::INIT,aTol_); }
      void setRelativeTolerance(const fmatvec::Vec &rTol_) { rTol = rTol_; }
      void setRelativeTolerance(double rTol_) { rTol = fmatvec::Vec(1,fmatvec::INIT,rTol_); }
      void setInitialStepSize(double dt0_) { dt0 = dt0_; }
      void setStepLimit(int maxSteps_) { maxSteps = maxSteps_; }
      void setFormalism(Formalism formalism_) { formalism = formalism_; }
      void setExcludeAlgebraicVariablesFromErrorTest(bool excludeAlgebraicVariables_) { excludeAlgebraicVariables = excludeAlgebraicVariables_; }

      using Integrator::integrate;
      void integrate();
      
      virtual void initializeUsingXML(xercesc::DOMElement *element);
  };

}

#endif
