/* Copyright (C) 2004-2007  Robert Huber

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

#ifndef _TIME_STEPPING_SSC_INTEGRATOR_H_ 
#define _TIME_STEPPING_SSC_INTEGRATOR_H_

#include<fmatvec.h>
#include "integrator.h"

namespace MBSim {

  /*! brief Half-explicit time-stepping integrator of first order with StepSize Control (SSC)*/
  class TimeSteppingSSCIntegrator : public Integrator { 

    friend class DAETSIntegrator;

    private:
      double dt, dtOld;
      double dtMin, dtMax;
      bool driftCompensation;
      double t, ti, tPlot;
      int zSize;
      Vec z, zi, z1e, z2e;
      Vec laiBi, laiUni, la1eBi, la1eUni;
      Vec q;
      Vec u;
      Vec x;
      int StepsWithUnchangedConstraints;
      /** Flag if stepsize control is used */
      bool FlagSSC;
      /** Absolute Toleranz */
      fmatvec::Vec aTol;
      /** Relative Toleranz */
      fmatvec::Vec rTol;
      /** Tolernaz for closing gaps */
      double gapTol;
      /** maximal gain factor for increasing dt by stepsize control (default 2.5; maxGain * safetyFactor must be GT 1)*/
      double maxGainSSC;
      /** safety factor for stepsize estimation: dt = dt_estimate * safetyFactorSSC (]0;1]; default 0.6)*/
      double safetyFactorSSC; 
      /** filestream for integrator info at each step */
      ofstream integPlot;
      /** Flag: write integrator info at each step to a file (default true)*/
      bool FlagPlotIntegrator;
      /** Flag: write integration summary to a file (default true)*/
      bool FlagPlotIntegrationSum;
      /** Flag: write integration info (integ. summary) to cout  (default true)*/
      bool FlagCoutInfo;
      /** every successful integration step is ploted (set dtPlot=0 to activate plotEveryStep) (default false)*/
      bool plotEveryStep;
      /** Flag interpolate z and la for plotting (default true)*/
      bool outputInterpolation;
      /** dt is optimised to minimize penetration; no test with gapTol is applied (set gapTol>,0 to ensure gapTol) default false*/
      bool optimisedtforgaps;
      /** Safety factor dt (calculated by gapControl) is multiplied with (>=1.0 ... 1.1) */
      double safetyFactorGapControl;
      /** computaional time*/
      double time;
      /** for internal use (start clock, integration info ...) */
      double s0;
      int iter, maxIter, sumIter;
      int integrationSteps, refusedSteps;
      double maxdtUsed, mindtUsed;      

    public:
      /*! Constructor with \default dt(1e-5), \default driftCompensation(false) */
      TimeSteppingSSCIntegrator();
      /*! Destructor */
      ~TimeSteppingSSCIntegrator() {}
      /*! Set start step size */
      void setdt(double dt_) {dt = dt_;}
      /*! Set maximal step size */
      void setdtMax(double dtMax_) {dtMax = dtMax_;}
      /*! Set maximal gain for increasing dt by stepsize control */
      void setmaxGainSSC(double maxGain) {maxGainSSC = maxGain;}
      /*! safety factor for stepsize estimation: dt = dt_estimate * safetyFactorSSC (]0;1]; default 0.6)*/
      void setsafetyFactorSSC(double sfactor) {safetyFactorSSC=sfactor;}
      /*! Deactivate stepsize control */
      void deactivateSSC(bool flag=false) {FlagSSC = flag;}
      /*! Set Flag for output interpolation */
      void setOutputInterpolation(bool flag=true) {outputInterpolation = flag;}
      /*! Set Flag to plot every successful integration step*/
      void setplotEveryStep(bool flag=true) {plotEveryStep = flag;}
      /*! Set Flag to optimise dt for minmal penetration of unilateral links*/
      void optimiseDtForGaps(bool flag=true) {optimisedtforgaps=flag;}
      /*! Set drift compensation */
      void setDriftCompensation(bool dc) {driftCompensation = dc;}

      /*! Start the integration */
      void integrate(MultiBodySystem& system);

      void setaTol(const fmatvec::Vec &aTol_) {aTol.resize() = aTol_;}
      void setaTol(double aTol_) {aTol.resize() = Vec(1,INIT,aTol_);}
      void setrTol(const fmatvec::Vec &rTol_) {rTol.resize() = rTol_;}
      void setrTol(double rTol_) {rTol.resize() = Vec(1,INIT,rTol_);}
      void setgapTol(double gTol) {gapTol = gTol;}

      /** subroutines for integrate function */
      void initIntegrator(MultiBodySystem &system_);
      void IntegrationStep();
      void closeIntegrator();
      bool testTolerances();
      bool gapControl(const Vec &g);
      void plot();

  };

}

#endif 
