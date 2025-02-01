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

#ifndef _RADAU_INTEGRATOR_H_
#define _RADAU_INTEGRATOR_H_

#include "dae_integrator.h"

namespace MBSim {

  /** \brief DAE-Integrator RADAU
  */
  class RADAUIntegrator : public DAEIntegrator {

    public:
      enum class StepSizeControl {
        ModPred = 1,
        Classic = 2,
      };

    private:
      typedef void (*Fzdot)(int* n, double* t, double* y, double* yd, double* rpar, int* ipar);
      typedef void (*Jac)(int* n, double *t, double *y, double *J, int *nn, double *rpar, int *ipar);
      typedef void (*Mass)(int* n, double* m, int* lmas, double* rpar, int* ipar);
      static Fzdot fzdot[5];
      static Jac jac[5];
      static Mass mass[2];
      static void fzdotODE(int* n, double* t, double* z, double* zd, double* rpar, int* ipar);
      static void fzdotDAE1(int* n, double* t, double* y, double* yd, double* rpar, int* ipar);
      static void fzdotDAE2(int* n, double* t, double* y, double* yd, double* rpar, int* ipar);
      static void fzdotDAE3(int* n, double* t, double* y, double* yd, double* rpar, int* ipar);
      static void jacODE(int* n, double *t, double *z, double *J, int *nn, double *rpar, int *ipar);
      static void jacDAE1(int* n, double *t, double *y, double *J, int *nn, double *rpar, int *ipar);
      static void jacDAE2(int* n, double *t, double *y, double *J, int *nn, double *rpar, int *ipar);
      static void jacDAE3(int* n, double *t, double *y, double *J, int *nn, double *rpar, int *ipar);
      static void jacGGL(int* n, double *t, double *y, double *J, int *nn, double *rpar, int *ipar);
      static void fzdotGGL(int* n, double* t, double* y, double* yd, double* rpar, int* ipar);
      static void massFull(int* n, double* m, int* lmas, double* rpar, int* ipar);
      static void massReduced(int* n, double* m, int* lmas, double* rpar, int* ipar);
      static void plot(int* nr, double* told, double* t, double* y, double* cont, int* lrc, int* n, double* rpar, int* ipar, int* irtrn);

      void reinit() override;

      double tPlot{0};
      double dtOut{0};
      double s0; 
      double time{0}; 

      /** Absolute Toleranz */
      fmatvec::Vec aTol;
      /** Relative Toleranz */
      fmatvec::Vec rTol;
      /** step size for the first step */
      double dt0{0};
      /** maximum number of steps */
      int maxSteps{std::numeric_limits<int>::max()};
      /** maximal step size */
      double dtMax{0};

      int mlJac, muJac;
      std::vector<int> iWorkExtended; int *iWork;
      fmatvec::Vec work;

      int maxNewtonIter { 0 };
      double newtonIterTol { 0 };
      double jacobianRecomputation { 0 };
      bool jacobianRecomputationAtRejectedSteps { true };
      bool drift { false };
      StepSizeControl stepSizeControl { StepSizeControl::ModPred };
      double stepSizeSaftyFactor { 0.9 };

      std::exception_ptr exception;

    public:
      ~RADAUIntegrator() override = default;

      void setAbsoluteTolerance(const fmatvec::Vec &aTol_) { aTol <<= aTol_; }
      void setAbsoluteTolerance(double aTol_) { aTol.resize(1,fmatvec::INIT,aTol_); }
      void setRelativeTolerance(const fmatvec::Vec &rTol_) { rTol <<= rTol_; }
      void setRelativeTolerance(double rTol_) { rTol.resize(1,fmatvec::INIT,rTol_); }
      void setInitialStepSize(double dt0_) { dt0 = dt0_; }
      void setMaximumStepSize(double dtMax_) { dtMax = dtMax_; }
      void setStepLimit(int maxSteps_) { maxSteps = maxSteps_; }
      void setMaximalNumberOfNewtonIterations(int iter) { maxNewtonIter = iter; }
      void setNewtonIterationTolerance(double tol) { newtonIterTol = tol; }
      void setJacobianRecomputation(double value) { jacobianRecomputation = value; }
      void setJacobianRecomputationAtRejectedSteps(bool recomp) { jacobianRecomputationAtRejectedSteps = recomp; }
      void setStepSizeControl(StepSizeControl ssc) { stepSizeControl = ssc; }
      void setStepSizeSaftyFactor(double fac) { stepSizeSaftyFactor = fac; }

      using Integrator::integrate;
      void integrate() override;

      void initializeUsingXML(xercesc::DOMElement *element) override;
  };

}

#endif
