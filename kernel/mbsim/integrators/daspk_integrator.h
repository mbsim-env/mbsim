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

#include "integrator.h"

namespace MBSimIntegrator {

  /** \brief Petzold’s DAE solver DASPK
   *
   * This integrator uses DASPK (http://www.netlib.org/ode).
   */
  class DASPKIntegrator : public Integrator {

    public:
      enum Formalism {
        ODE=0,
        DAE1,
        DAE2,
        GGL,
        unknown
      };

    private:
      typedef void (*Delta)(double* t, double* y_, double* yd_, double* cj, double* delta_, int *ires, double* rpar, int* ipar);
      static Delta delta[4];
      static void deltaODE(double* t, double* y_, double* yd_, double* cj, double* delta_, int *ires, double* rpar, int* ipar);
      static void deltaDAE1(double* t, double* y_, double* yd_, double* cj, double* delta_, int *ires, double* rpar, int* ipar);
      static void deltaDAE2(double* t, double* y_, double* yd_, double* cj, double* delta_, int *ires, double* rpar, int* ipar);
      static void deltaGGL(double* t, double* y_, double* yd_, double* cj, double* delta_, int *ires, double* rpar, int* ipar);

      /** maximal step size */
      double dtMax{0};
      /** Absolute Toleranz */
      fmatvec::Vec aTol;
      /** Relative Toleranz */
      fmatvec::Vec rTol;
      /** step size for the first step */
      double dt0{0};
      /** formalism **/
      Formalism formalism{DAE2};
      /** exclude algebraic variables from error test **/
      bool excludeAlgebraicVariables{true};

       /** tolerance for position constraints */
      double gMax{-1};
      /** tolerance for velocity constraints */
      double gdMax{-1};

    public:
      void setMaximumStepSize(double dtMax_) { dtMax = dtMax_; }
      void setAbsoluteTolerance(const fmatvec::Vec &aTol_) { aTol = aTol_; }
      void setAbsoluteTolerance(double aTol_) { aTol = fmatvec::Vec(1,fmatvec::INIT,aTol_); }
      void setRelativeTolerance(const fmatvec::Vec &rTol_) { rTol = rTol_; }
      void setRelativeTolerance(double rTol_) { rTol = fmatvec::Vec(1,fmatvec::INIT,rTol_); }
      void setInitialStepSize(double dt0_) { dt0 = dt0_; }
      void setFormalism(Formalism formalism_) { formalism = formalism_; }
      void setExcludeAlgebraicVariablesFromErrorTest(bool excludeAlgebraicVariables_) { excludeAlgebraicVariables = excludeAlgebraicVariables_; }

      void setToleranceForPositionConstraints(double gMax_) { gMax = gMax_; }
      void setToleranceForVelocityConstraints(double gdMax_) { gdMax = gdMax_; }

      using Integrator::integrate;
      void integrate();

      virtual void initializeUsingXML(xercesc::DOMElement *element);
  };

}

#endif
