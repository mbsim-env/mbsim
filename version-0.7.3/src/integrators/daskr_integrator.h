/* Copyright (C) 2007 Robert Huber
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
 * Contact:
 *   rhuber@users.berlios.de
 *
 */

#ifndef _DASKR_INTEGRATOR_H_
#define _DASKR_INTEGRATOR_H_

using namespace fmatvec;

#include "integrator.h"
#include "stopwatch.h"

namespace MBSim {

  /** \brief DAE-Integrator DASKR (up to index 2) with root function
   */
  class DASKRIntegrator : public Integrator {

    friend class DAETSIntegrator;
 
    private:
      static void residuum(double* t, double* Y_, double* Ydot_, double* CJ, double* res_, int* ires, double* rpar, int* ipar);
      static void jac(double* t, double* Y_, double* Ydot_, double* PD, double* CJ, double* rpar, int* ipar); 
      static void rt(int* neq, double* t, double* Y_, double* Ydot_, int* nrt, double *rval, double* rpar, int* ipar);
      /** Absolute Toleranz */
      fmatvec::Vec aTol;
      /** Relative Toleranz */
      fmatvec::Vec rTol;
      /** step size for the first step */
      double dt0;
      /** maximum number of steps */
      int maxSteps;
      /** maximal step size */
      double dtMax;
      /** index of DAE (default 2)
	possible values:  2, 21 (Index 2 formulation with Gear Gupta Leimkuhler Stabilisation (GGL)) or 1
       */
      int DAEIndex;
      /** maximum method order (one to five)*/
      int MaxOrder;
      /** include or exclude algebraic variables or scale variables with stepsize */
      int FlagErrorTest;
      /** use external jacobian A=dG/dY + CJ*dG/dY' (true) or numerical calculation by integrator (false=default) */
      bool useExternalJac;
      /** time (current value of the independent variable t */
      double t;
      /** solution components and derivatives at t */
      Vec Y;
      Vec Ydot;
      Vec z;
      /** integer array to communicate with integrator */
      Vector<int> info;
      /** a real work array */
      Vec rwork;
      /** a integer work array */
      Vector<int> iwork;
      /** real parameter array, which can be used for communication  between the calling program and daskr subroutines */
      Vec rPar;
      /** integer parameter array, which can be used for communication  between the calling program and daskr subroutines */
      Vector<int> iPar;
      /** for monitoring daskr */
      int idid;			
      /** integer array for output to indicate where one or more roots were found */
      Vector<int> jroot;
      /** computational time for integration */
      double time;
      /** filestream for integrator info at each step and flag */
      ofstream integPlot;
      bool FlagPlotIntegrator;
      /** Flag: write integration summary into a file */
      bool FlagPlotIntegrationSum;
      /** Flag: write integration info (Index, status of calc. init. values, integ. summary) to cout */
      bool FlagCoutInfo;
      /** for internal use (start clock, and system sizes) */
      int YSize;
      int zSize;
      int LRW;
      int LIW;
      int nrt;
      StopWatch Timer;
      int IntSteps, RefusedSteps, FuncEvals, JacEval, RootEvals;

    public:

      DASKRIntegrator();
      ~DASKRIntegrator() {}

      void setaTol(const fmatvec::Vec &aTol_) {aTol.resize() = aTol_;}
      void setaTol(double aTol_) {aTol.resize() = Vec(1,INIT,aTol_);}
      void setrTol(const fmatvec::Vec &rTol_) {rTol.resize() = rTol_;}
      void setrTol(double rTol_) {rTol.resize() = Vec(1,INIT,rTol_);}
      void setdt0(double dt0_) {dt0 = dt0_;}
      void setdtMax(double dtMax_) {dtMax = dtMax_;}
      void setMaxStepNumber(int maxSteps_) {maxSteps = maxSteps_;}
      void setDAEIndex(int index);
      void setMaxOrder(int order_);
      void setFlagErrorTest(int Flag);
      void useExternalJacobian(bool flag=true) {useExternalJac = flag;}
      void setGGLStabilisation() {DAEIndex=21;};
      void integrate(MultiBodySystem& system);

      /** subroutines used by integrate */
      void initIntegrator(MultiBodySystem &system_);
      int computeInitialConditions(bool FlagPlot0, bool FlagCoutInfo_);
      void refreshIntegratorSetups();
      void IntegrationStep();
      void closeIntegrator();


  };

}

#endif


