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

#ifndef _DASPK_INTEGRATOR_H_
#define _DASPK_INTEGRATOR_H_

#include "dae_integrator.h"

namespace MBSim {

  /** \brief Petzold’s DAE solver DASPK
   *
   * This integrator uses DASPK (http://www.netlib.org/ode).
   */
  class DASPKIntegrator : public DAEIntegrator {

    private:
      double delta(int i, double z) const override;
      typedef void (*Delta)(double* t, double* y_, double* yd_, double* cj, double* delta_, int *ires, double* rpar, int* ipar);
      typedef void (*Jac)(double* t, double* y_, double* yd_, double* J_, double* cj, double* rpar, int* ipar);
      static Delta delt[5];
      static Jac jac[5];
      static void deltaODE(double* t, double* z_, double* zd_, double* cj, double* delta_, int *ires, double* rpar, int* ipar);
      static void deltaDAE1(double* t, double* y_, double* yd_, double* cj, double* delta_, int *ires, double* rpar, int* ipar);
      static void deltaDAE2(double* t, double* y_, double* yd_, double* cj, double* delta_, int *ires, double* rpar, int* ipar);
      static void deltaGGL(double* t, double* y_, double* yd_, double* cj, double* delta_, int *ires, double* rpar, int* ipar);
      static void jacODE(double* t, double* z_, double* zd_, double* J_, double* cj, double* rpar, int* ipar);
      static void jacDAE1(double* t, double* y_, double* yd_, double* J_, double* cj, double* rpar, int* ipar);
      static void jacDAE2(double* t, double* y_, double* yd_, double* J_, double* cj, double* rpar, int* ipar);
      static void jacGGL(double* t, double* y_, double* yd_, double* J_, double* cj, double* rpar, int* ipar);

      void init() override;
      void reinit() override;

      /** maximal step size */
      double dtMax{0};
      /** step size for the first step */
      double dt0{0};
      /** exclude algebraic variables from error test **/
      bool excludeAlgebraicVariables{true};

      fmatvec::VecInt info, iPar, iWork;
      fmatvec::Vec work;
      double h;
      int lphi, lewt;
      double *yd;

      std::exception_ptr exception;

    public:
      void setMaximumStepSize(double dtMax_) { dtMax = dtMax_; }
      void setInitialStepSize(double dt0_) { dt0 = dt0_; }
      void setExcludeAlgebraicVariablesFromErrorTest(bool excludeAlgebraicVariables_) { excludeAlgebraicVariables = excludeAlgebraicVariables_; }

      using Integrator::integrate;
      void integrate();

      virtual void initializeUsingXML(xercesc::DOMElement *element);
  };

}

#endif
