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

#ifndef _TIME_STEPPING_SSC_INTEGRATOR4_H_ 
#define _TIME_STEPPING_SSC_INTEGRATOR4_H_

#include<fmatvec.h>
#include "integrator.h"
#include "stopwatch.h"

namespace MBSim {

  /*! brief Half-explicit time-stepping integrator of first or 2nd order with StepSize Control (SSC)*/
  class TimeSteppingSSCIntegrator : public Integrator { 

    friend class DAETSIntegrator;

    protected:
      MultiBodySystem* systemT1;
      MultiBodySystem* systemT2;
      MultiBodySystem* systemT3;

      double dt, dtOld, dte;
      double dtMin, dtMax;
      bool driftCompensation;
      double t, tPlot;
      int qSize, xSize, zSize;
      Vec ze, zi, zT1, zT2, zT3, z1d, z2d, z2dRE, z4dRE, z2b, z4b;
      Vector<int> LS, LSe, LSA, LSB1, LSB2, LSC1, LSC2, LSC3, LSC4;
      Vec laBi, laUni, laeBi, laeUni, la1dBi, la1dUni, la2bBi, la2bUni;
      Vec qT1, qT2, qT3;
      Vec uT1, uT2, uT3;
      Vec xT1, xT2, xT3;
      int StepsWithUnchangedConstraints;
      /** include (0) or exclude (3) variable u or scale (2) with stepsize for error test*/
      int FlagErrorTest;
      /** Absolute Toleranz */
      fmatvec::Vec aTol;
      /** Relative Toleranz */
      fmatvec::Vec rTol;
      /** activate step size control*/
      bool FlagSSC;
      /** maximum order of integration scheme (1 or 2) */
      int maxOrder;
      /** Toleranz for closing gaps */
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
      /** every successful integration step is ploted (set dtPlot=0 to activate FlagPlotEveryStep) (default false)*/
      bool FlagPlotEveryStep;
      /** Flag interpolate z and la for plotting (default true)*/
      bool outputInterpolation;
      /** dt is optimised to minimize penetration; no test with gapTol is applied (set gapTol>,0 to ensure gapTol) default false*/
      bool optimisedtforgaps;
      /** Safety factor dt (calculated by gapControl) is multiplied with (>=1.0 ... 1.1) */
      double safetyFactorGapControl;
      /** computaional time*/
      double time;
      /** for internal use (start clock, integration info ...) */
      StopWatch Timer;
      int iter, iterA, iterB1, iterB2, iterC1, iterC2, iterC3, iterC4, iterB2RE, maxIterUsed, maxIter, sumIter;
      int integrationSteps, integrationStepswithChange, refusedSteps, refusedStepsWithImpact;
      double maxdtUsed, mindtUsed;      
      bool IterConvergence;
      bool ConstraintsChanged, ConstraintsChangedBlock1, ConstraintsChangedBlock2;
      int integrationStepsOrder1;
      int integrationStepsOrder2;
      int order;
      int StepTrials;

    public:
      /*! Constructor with \default dt(1e-5), \default driftCompensation(false) */
      TimeSteppingSSCIntegrator();
      /*! Destructor */
      ~TimeSteppingSSCIntegrator() {}
      /*! Set start step size */
      void setdt(double dt_) {dt = dt_;}
      /*! Set maximal step size */
      void setdtMax(double dtMax_) {dtMax = dtMax_;}
      /*! Set minimal step size (default 2*maxOrder*epsroot() */
      void setdtMin(double dtMin_) {dtMin = dtMin_;}
      /*! Set maximal gain for increasing dt by stepsize control */
      void setmaxGainSSC(double maxGain) {maxGainSSC = maxGain;}
      /*! safety factor for stepsize estimation: dt = dt_estimate * safetyFactorSSC (]0;1]; default 0.6)*/
      void setsafetyFactorSSC(double sfactor) {safetyFactorSSC=sfactor;}
      /*! Set Flag for output interpolation */
      void setOutputInterpolation(bool flag=true) {outputInterpolation = flag;}
      /*! Set Flag to plot every successful integration step*/
      void plotEveryStep() {dtPlot =0;}
      /*! set Flag for  writing integrator info at each step to a file (default true)*/
      void setFlagPlotIntegrator(bool flag=true) {FlagPlotIntegrator = flag;}
      /*! Set Flag to optimise dt for minmal penetration of unilateral links*/
      void optimiseDtForGaps(bool flag=true) {optimisedtforgaps=flag;}
      /*! Set drift compensation */
      void setDriftCompensation(bool dc) {driftCompensation = dc;}
      /*! set maximum order of integration scheme (1 or 2)*/  
      void setMaxOrder(int order_);
      /* activate step size control */
      void deactivateSSC(bool flag=false) {FlagSSC=flag;}
      /*! Set Flag vor ErrorTest (default 0: all variables are tested;  2: u is scaled with dt;  3: exclude u*/
      void setFlagErrorTest(int Flag);


      /*! Start the integration */
      void integrate(MultiBodySystem& system_);
      void integrate(MultiBodySystem& systemT1_, MultiBodySystem& systemT2_, MultiBodySystem& systemT3_);

      void setaTol(const fmatvec::Vec &aTol_) {aTol.resize() = aTol_;}
      void setaTol(double aTol_) {aTol.resize() = Vec(1,INIT,aTol_);}
      void setrTol(const fmatvec::Vec &rTol_) {rTol.resize() = rTol_;}
      void setrTol(double rTol_) {rTol.resize() = Vec(1,INIT,rTol_);}
      void setgapTol(double gTol) {gapTol = gTol;}

      /** subroutines for integrate function */
      void initIntegrator(MultiBodySystem& system_);
      void initIntegrator(MultiBodySystem& systemT1_, MultiBodySystem& systemT2_, MultiBodySystem& systemT3_);
      void IntegrationStep();
      void closeIntegrator();
      bool testTolerances();
      double calculatedtNewRel(const fmatvec::Vec &ErrorLocal, double H);
      void plot();

  };

}

#endif 
