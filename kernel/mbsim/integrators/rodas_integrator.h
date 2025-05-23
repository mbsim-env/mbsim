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

#ifndef _RODAS_INTEGRATOR_H_
#define _RODAS_INTEGRATOR_H_

#include "dae_integrator.h"

namespace MBSim {

  /** \brief DAE-Integrator RODAS
  */
  class RODASIntegrator : public DAEIntegrator {

    private:
      double delta(int i, double z) const override;
      void par_ud_xd_par_t(fmatvec::Vec &pt);
      void par_ud_xd_gdd_par_t(fmatvec::Vec &pt);
      typedef void (*Fzdot)(int* n, double* t, double* y, double* yd, double* rpar, int* ipar);
      typedef void (*Jac)(int* n, double *t, double *y, double *J, int *nn, double *rpar, int *ipar);
      typedef void (*Pt)(int* n, double* t, double* y, double* pt, double* rpar, int* ipar);
      typedef void (*Mass)(int* n, double* m, int* lmas, double* rpar, int* ipar);
      static Fzdot fzdot[2];
      static Jac jac[2];
      static Pt pt[2];
      static Mass mass[2];
      static void fzdotODE(int* n, double* t, double* z, double* zd, double* rpar, int* ipar);
      static void fzdotDAE1(int* n, double* t, double* y, double* yd, double* rpar, int* ipar);
      static void jacODE(int* n, double *t, double *z, double *J, int *nn, double *rpar, int *ipar);
      static void jacDAE1(int* n, double *t, double *y, double *J, int *nn, double *rpar, int *ipar);
      static void ptODE(int* n, double* t, double* z, double* pt, double* rpar, int* ipar);
      static void ptDAE1(int* n, double* t, double* y, double* pt, double* rpar, int* ipar);
      static void massFull(int* n, double* m, int* lmas, double* rpar, int* ipar);
      static void massReduced(int* n, double* m, int* lmas, double* rpar, int* ipar);
      static void plot(int* nr, double* told, double* t, double* y, double* cont, int* lrc, int* n, double* rpar, int* ipar, int* irtrn);

      void reinit() override;

      double tPlot{0};
      double dtOut{0};
      double s0; 
      double time{0}; 

      /** step size for the first step */
      double dt0{0};
      /** maximum number of steps */
      int maxSteps{std::numeric_limits<int>::max()};
      /** maximal step size */
      double dtMax{0};
      /** autonomous system **/
      bool autonom{false};

      int mlJac, muJac;
      fmatvec::VecInt iWork;
      fmatvec::Vec work;

      bool drift { false };

      std::exception_ptr exception;

    public:
      ~RODASIntegrator() override = default;

      void setInitialStepSize(double dt0_) { dt0 = dt0_; }
      void setMaximumStepSize(double dtMax_) { dtMax = dtMax_; }
      void setStepLimit(int maxSteps_) { maxSteps = maxSteps_; }
      void setFormalism(Formalism formalism_) { formalism = formalism_; }
      void setReducedForm(bool reduced_) { reduced = reduced_; }
      void setAutonomousSystem(bool autonom_) { autonom = autonom_; }

      using Integrator::integrate;
      void integrate() override;

      void initializeUsingXML(xercesc::DOMElement *element) override;
  };

}

#endif
